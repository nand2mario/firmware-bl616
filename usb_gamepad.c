// USB gamepad
// Mostly based on FPGA-Companion by Till Harbaum
#include "usbh_core.h"
#include "usb_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "usb_gamepad.h"

#include "hidparser.h"

void overlay_status(const char *fmt, ...);
#define DEBUG(fmt, ...) overlay_status(fmt, ##__VA_ARGS__)

#define MAX_REPORT_SIZE   8
#define XBOX_REPORT_SIZE 20

#define STATE_NONE      0 
#define STATE_DETECTED  1 
#define STATE_RUNNING   2
#define STATE_FAILED    3

#define XINPUT_GAMEPAD_DPAD_UP 0x0001
#define XINPUT_GAMEPAD_DPAD_DOWN 0x0002
#define XINPUT_GAMEPAD_DPAD_LEFT 0x0004
#define XINPUT_GAMEPAD_DPAD_RIGHT 0x0008
#define XINPUT_GAMEPAD_START 0x0010
#define XINPUT_GAMEPAD_BACK 0x0020
#define XINPUT_GAMEPAD_LEFT_SHOULDER 0x0100
#define XINPUT_GAMEPAD_RIGHT_SHOULDER 0x0200

static struct usb_config {
    struct xbox_info_S {
        int index;
        int state;
        struct usbh_hid *class;     // USB host Xbox class
        uint8_t *buffer;
        int nbytes;
        struct usb_config *usb;
        SemaphoreHandle_t sem;
        TaskHandle_t task_handle;    
        unsigned char last_state;
        unsigned char js_index;
        unsigned char last_state_btn_extra;
        int16_t last_state_x;
        int16_t last_state_y;    
    } xbox_info[CONFIG_USBHOST_MAX_XBOX_CLASS];
    struct hid_info_S {
        int index;
        int state;
        struct usbh_hid *class;     // USB host HID class
        uint8_t *buffer;            // URB transfer buffer
        int nbytes;                 // number of bytes in the buffer
        hid_report_t report;        // parsed HID report descriptor
        struct usb_config *usb;
        SemaphoreHandle_t sem;
        TaskHandle_t task_handle;    
        hid_state_t hid_state;
    } hid_info[CONFIG_USBHOST_MAX_HID_CLASS];
} usb_config;

TaskHandle_t usb_handle;
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t hid_buffer[CONFIG_USBHOST_MAX_HID_CLASS][MAX_REPORT_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t xbox_buffer[CONFIG_USBHOST_MAX_XBOX_CLASS][XBOX_REPORT_SIZE];

// keep a map of joysticks to be able to report
// them individually
static uint8_t joystick_map = 0;

uint8_t hid_allocate_joystick(void) {
    uint8_t idx;
    for(idx=0;joystick_map & (1<<idx);idx++);
    joystick_map |= (1<<idx);
    DEBUG("Allocating joystick %d (map = %02x)", idx, joystick_map);
    return idx;
}

void hid_release_joystick(uint8_t idx) {
    joystick_map &= ~(1<<idx);
    DEBUG("Releasing joystick %d (map = %02x)", idx, joystick_map);
}

uint8_t byteScaleAnalog(int16_t xbox_val)
{
    // Scale the xbox value from [-32768, 32767] to [1, 255]
    // Offset by 32768 to get in range [0, 65536], then divide by 256 to get in range [1, 255]
    uint8_t scale_val = (xbox_val + 32768) / 256;
    if (scale_val == 0) return 1;
    return scale_val;
}

void usbh_hid_callback(void *arg, int nbytes) {
    struct hid_info_S *hid = (struct hid_info_S *)arg;

    xSemaphoreGiveFromISR(hid->sem, NULL);
    hid->nbytes = nbytes;
}  

void usbh_xbox_callback(void *arg, int nbytes) {
    struct xbox_info_S *xbox = (struct xbox_info_S *)arg;
    xSemaphoreGiveFromISR(xbox->sem, NULL);
    xbox->nbytes = nbytes;
}

#define print_usb_class_info(cls) \
    DEBUG("Interval: %d, ", cls->hport->config.intf[i].altsetting[0].ep[0].ep_desc.bInterval); \
    DEBUG("Interface %d, ", cls->intf); \
    DEBUG("  class %d, ", cls->hport->config.intf[i].altsetting[0].intf_desc.bInterfaceClass); \
    DEBUG("  subclass %d, ", cls->hport->config.intf[i].altsetting[0].intf_desc.bInterfaceSubClass); \
    DEBUG("  protocol %d\n", cls->hport->config.intf[i].altsetting[0].intf_desc.bInterfaceProtocol);

uint8_t joy_driver_map = 0;

static void usbh_update(struct usb_config *usb) {
    // check for active hid devices
    for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
        char *dev_str = "/dev/inputX";
        dev_str[10] = '0' + i;
        usb->hid_info[i].class = (struct usbh_hid *)usbh_find_class_instance(dev_str);
        if (usb->hid_info[i].class) {
            joy_driver_map |= (1 << i);
        } else {
            joy_driver_map &= ~(1 << i);
        }
        
        if(usb->hid_info[i].class && usb->hid_info[i].state == STATE_NONE) {
            DEBUG("NEW HID %d\n", i);
            print_usb_class_info(usb->hid_info[i].class);

            // parse report descriptor ...
            DEBUG("report descriptor: %p", usb->hid_info[i].class->report_desc);
            
            if(!parse_report_descriptor(usb->hid_info[i].class->report_desc, 128, &usb->hid_info[i].report, NULL)) {
                usb->hid_info[i].state = STATE_FAILED;   // parsing failed, don't use
                return;
            }
            
            usb->hid_info[i].state = STATE_DETECTED;
        }
        
        else if(!usb->hid_info[i].class && usb->hid_info[i].state != STATE_NONE) {
            DEBUG("HID %d LOST\n", i);
            vTaskDelete( usb->hid_info[i].task_handle );
            DEBUG("HID %d task deleted\n", i);
            usb->hid_info[i].state = STATE_NONE;

            if(usb->hid_info[i].report.type == REPORT_TYPE_JOYSTICK) {
                DEBUG("Joystick %d gone", usb->hid_info[i].hid_state.joystick.js_index);
                hid_release_joystick(usb->hid_info[i].hid_state.joystick.js_index);
            }
        }
    }

    // check for active xbox devices
    for(int i=0;i<CONFIG_USBHOST_MAX_XBOX_CLASS;i++) {
        char *dev_str = "/dev/xboxX";
        dev_str[9] = '0' + i;
        usb->xbox_info[i].class = (struct usbh_hid *)usbh_find_class_instance(dev_str);
        if (usb->xbox_info[i].class) {
            joy_driver_map |= (1 << (i + 2));
        } else {
            joy_driver_map &= ~(1 << (i + 2));
        }
        
        if(usb->xbox_info[i].class && usb->xbox_info[i].state == STATE_NONE) {
            DEBUG("NEW XBOX %d\n", i);
            print_usb_class_info(usb->xbox_info[i].class);
            usb->xbox_info[i].state = STATE_DETECTED;
        }
        
        else if(!usb->xbox_info[i].class && usb->xbox_info[i].state != STATE_NONE) {
            DEBUG("XBOX %d LOST\n", i);
            vTaskDelete( usb->xbox_info[i].task_handle );
            DEBUG("XBOX %d task deleted\n", i);
            usb->xbox_info[i].state = STATE_NONE;

            DEBUG("Joystick %d is gone", usb->xbox_info[i].js_index);
            hid_release_joystick(usb->xbox_info[i].js_index);
        }
    }
}

static void xbox_parse(struct xbox_info_S *xbox) {
    // verify length field
    if(xbox->buffer[0] != 0 || xbox->buffer[1] != 20) {
        DEBUG("XBOX Joy%d: wrong length field %02x %02x\n", xbox->index, xbox->buffer[0], xbox->buffer[1]);
        return;
    }

    uint16_t wButtons = xbox->buffer[3] << 8 | xbox->buffer[2]; // Xbox: Y X B A == SNES: X Y A B

    // build new state
    unsigned char state =
        ((wButtons & XINPUT_GAMEPAD_DPAD_UP   )?0x08:0x00) |
        ((wButtons & XINPUT_GAMEPAD_DPAD_DOWN )?0x04:0x00) |
        ((wButtons & XINPUT_GAMEPAD_DPAD_LEFT )?0x02:0x00) |
        ((wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)?0x01:0x00) |
        ((wButtons & 0xf000) >> 8); // Y, X, B, A

    // build extra button new state
    unsigned char state_btn_extra =
        ((wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER  )?0x01:0x00) |
        ((wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER )?0x02:0x00) |
        ((wButtons & XINPUT_GAMEPAD_BACK           )?0x10:0x00) | // Rumblepad 2 / Dual Action compatibility
        ((wButtons & XINPUT_GAMEPAD_START          )?0x20:0x00);

    // build analog stick x,y state
    int16_t sThumbLX = xbox->buffer[7] << 8 | xbox->buffer[6];
    int16_t sThumbLY = xbox->buffer[9] << 8 | xbox->buffer[8];
    uint8_t ax = byteScaleAnalog(sThumbLX);
    uint8_t ay = ~byteScaleAnalog(sThumbLY);

    // map analog stick directions to digital
    if(ax > (uint8_t) 0xc0) state |= 0x01;
    if(ax < (uint8_t) 0x40) state |= 0x02;
    if(ay > (uint8_t) 0xc0) state |= 0x04;
    if(ay < (uint8_t) 0x40) state |= 0x08;    

    // submit if state has changed
    if(state != xbox->last_state ||
        state_btn_extra != xbox->last_state_btn_extra ||
        sThumbLX != xbox->last_state_x ||
        sThumbLY != xbox->last_state_y) {

        xbox->last_state = state;
        xbox->last_state_btn_extra = state_btn_extra;
        xbox->last_state_x = sThumbLX;
        xbox->last_state_y = sThumbLY;
        DEBUG("XBOX Joy%d: B %02x EB %02x X %02x Y %02x", xbox->js_index, state, state_btn_extra, byteScaleAnalog(sThumbLX), byteScaleAnalog(sThumbLY));
    }
}

// each HID client gets itws own thread which submits urbs
// and waits for the interrupt to succeed
static void usbh_hid_client_thread(void *arg) {
    struct hid_info_S *hid = (struct hid_info_S *)arg;

    DEBUG("HID client #%d: thread started\n", hid->index);

    while(1) {
        int ret = usbh_submit_urb(&hid->class->intin_urb);
        if (ret < 0)
            DEBUG("HID client #%d: submit failed %d\n", hid->index, ret);
        else {
            // Wait for result
            xSemaphoreTake(hid->sem, 0xffffffffUL);
            if(hid->nbytes > 0) {       // use first two joysticks
                hid_parse(&hid->report, &hid->hid_state, hid->buffer, hid->nbytes);
                if (hid->hid_state.joystick.js_index < 2) {
                    xSemaphoreTake(state_mutex, portMAX_DELAY);
                    volatile uint16_t *snes = hid->hid_state.joystick.js_index == 0 ? &hid1_state : &hid2_state;
                    uint8_t hid_state = hid->hid_state.joystick.last_state;
                    uint8_t hid_extra = hid->hid_state.joystick.last_state_btn_extra;
                    //       11 10 9 8 7  6  5  4  3  2  1  0
                    // SNES: R  L  X A RT LT DN UP ST SE Y  B
                    // HID:            Y  B  A  X  UP DN LT RT                     
                    // EXTRA:                ST SE       R  L
                    *snes = (hid_state & 1) << 7 | (hid_state & 2) << 5 | (hid_state & 4) << 3 | (hid_state & 8) << 1 |
                            (hid_state & 0x10) << 5 | (hid_state & 0x20) << 3| (hid_state & 0x40) >> 6 | (hid_state & 0x80) >> 6 |
                            (hid_extra & 0x01) << 10 | (hid_extra & 0x02) << 10 | (hid_extra & 0x10) >> 2 | (hid_extra & 0x20) >> 2;
                    xSemaphoreGive(state_mutex);
                }
            }
            
            hid->nbytes = 0;
        }      
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// from fixcontroler.py: https://gist.github.com/adnanh/f60f069fc9185a48b73db9987b9e9108
struct usb_setup_packet xbox360_init_packet1 = {
    0xC1, 0x01, 0x0100, 0x00, 0x14
};

static void usbh_xbox_client_thread(void *arg) {
    struct xbox_info_S *xbox = (struct xbox_info_S *)arg;

    DEBUG("XBOX client #%d: thread started\n", xbox->index);

    // 8bitdo SN30pro requires getting the string descriptor for initialization
    uint8_t *str = malloc(256);
    if (!str) {
        DEBUG("XBOX: failed to allocate string descriptor buffer");
        goto free_str;
    }
    if (usbh_get_string_desc(xbox->class->hport, 2, str) < 0)
        DEBUG("XBOX: getting string descriptor failed");

    // A lot of xbox 360 compatible controllers require the following initialization packet
    usbh_control_transfer(xbox->class->hport, &xbox360_init_packet1, str);

free_str:
    free(str);

    while(1) {
        int ret = usbh_submit_urb(&xbox->class->intin_urb);
        if (ret < 0)
            DEBUG("XBOX client #%d: submit failed %d\n", xbox->index, ret);
        else {
            // Wait for result
            xSemaphoreTake(xbox->sem, 0xffffffffUL);
            // DEBUG("XBOX client #%d: nbytes %d\n", xbox->index, xbox->nbytes);
            if (xbox->nbytes == XBOX_REPORT_SIZE) {
                xbox_parse(xbox);

                if (xbox->js_index < 2) {
                    xSemaphoreTake(state_mutex, portMAX_DELAY);
                    volatile uint16_t *snes = xbox->js_index == 0 ? &hid1_state : &hid2_state;  
                    uint8_t state = xbox->last_state;
                    uint8_t state_extra = xbox->last_state_btn_extra;
                    // SNES: R  L  X A RT LT DN UP ST SE Y  B
                    // XBOX:           X  Y  A  B  UP DN LT RT
                    // EXTRA:                ST SE       R  L
                    *snes = (state & 1) << 7 | (state & 2) << 5 | (state & 4) << 3 | (state & 8) << 1 |
                            (state & 0x10) >> 4 | (state & 0x20) << 3| (state & 0x40) >> 5 | (state & 0x80) << 2 |
                            (state_extra & 0x01) << 10 | (state_extra & 0x02) << 10 | (state_extra & 0x10) >> 2 | (state_extra & 0x20) >> 2;
                    xSemaphoreGive(state_mutex);
                }
            }
            xbox->nbytes = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static const char *state2str(int state) {
    switch (state) {
        case STATE_NONE: return "none";
        case STATE_DETECTED: return "detected";
        case STATE_RUNNING: return "running";
        case STATE_FAILED: return "failed";
    }
    return "unknown";
}

static void usbh_hid_thread(void *argument) {
    DEBUG("Starting usb gamepad host task...");
    uint64_t last_status = 0;

    struct usb_config *usb = (struct usb_config *)argument;

    while (1) {
        // DEBUG("calling usbh_update()\n");
        usbh_update(usb);

        for (int i = 0; i < CONFIG_USBHOST_MAX_HID_CLASS; i++) {
            if (usb->hid_info[i].state == STATE_DETECTED) {
                DEBUG("HID device %d is detected\n", i);
                usb->hid_info[i].state = STATE_RUNNING;

                // allocate a joystick index
                usb->hid_info[i].hid_state.joystick.js_index = hid_allocate_joystick();
                DEBUG("  -> joystick %d", usb->hid_info[i].hid_state.joystick.js_index);

                // setup urb
                usbh_int_urb_fill(&usb->hid_info[i].class->intin_urb,
                        usb->hid_info[i].class->hport,
                        usb->hid_info[i].class->intin, usb->hid_info[i].buffer,
                        usb->hid_info[i].report.report_size + (usb->hid_info[i].report.report_id_present ? 1:0),
                        0, usbh_hid_callback, &usb->hid_info[i]);     

                xTaskCreate(usbh_hid_client_thread, (char *)"hid_client_task", 1024,
                        &usb->hid_info[i], configMAX_PRIORITIES-3, &usb->hid_info[i].task_handle );
            }
        }

        // check for active xbox pads
        for (int i = 0; i < CONFIG_USBHOST_MAX_XBOX_CLASS; i++) {
            if (usb->xbox_info[i].state == STATE_DETECTED) {
                DEBUG("XBOX device %d is detected\n", i);
                usb->xbox_info[i].state = STATE_RUNNING;
                usb->xbox_info[i].js_index = i;

                // allocate a joystick index
                usb->xbox_info[i].js_index = hid_allocate_joystick();
                DEBUG("  -> joystick %d", usb->xbox_info[i].js_index);

                // setup urb
                usbh_int_urb_fill(&usb->xbox_info[i].class->intin_urb,
                        usb->xbox_info[i].class->hport,
                        usb->xbox_info[i].class->intin, usb->xbox_info[i].buffer,
                        XBOX_REPORT_SIZE,
                        0, usbh_xbox_callback, &usb->xbox_info[i]);

                xTaskCreate(usbh_xbox_client_thread, (char *)"xbox_client_task", 2048,
                        &usb->xbox_info[i], configMAX_PRIORITIES-3, &usb->xbox_info[i].task_handle );
            }
        }

        uint64_t now = bflb_mtimer_get_time_ms();
        if (now - last_status > 5000) {
            last_status = now;
            DEBUG("hid1: %s %s, hid2: %s %s, xbox1: %s %s, xbox2: %s %s", 
                joy_driver_map & 1 ? "on" : "off", state2str(usb->hid_info[0].state),
                joy_driver_map & 2 ? "on" : "off", state2str(usb->hid_info[1].state),
                joy_driver_map & 4 ? "on" : "off", state2str(usb->xbox_info[0].state),
                joy_driver_map & 8 ? "on" : "off", state2str(usb->xbox_info[1].state)
            );
        }

        // wait for 100ms as this thread only deals with device detection and leaving
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void usb_gamepad_init(void) {

    for (int i = 0; i < CONFIG_USBHOST_MAX_HID_CLASS; i++) {
        usb_config.hid_info[i].index = i;
        usb_config.hid_info[i].state = 0;
        usb_config.hid_info[i].buffer = hid_buffer[i];
        usb_config.hid_info[i].usb = &usb_config;
        usb_config.hid_info[i].sem = xSemaphoreCreateBinary();
    }

    for (int i = 0; i < CONFIG_USBHOST_MAX_XBOX_CLASS; i++) {
        usb_config.xbox_info[i].index = i;
        usb_config.xbox_info[i].state = 0;
        usb_config.xbox_info[i].buffer = xbox_buffer[i];
        usb_config.xbox_info[i].usb = &usb_config;
        usb_config.xbox_info[i].sem = xSemaphoreCreateBinary();
    }

    xTaskCreate(usbh_hid_thread, (char *)"usbh_hid_task", 2048, &usb_config, configMAX_PRIORITIES-3, &usb_handle);
}
