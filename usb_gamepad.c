// USB gamepad
// Mostly based on FPGA-Companion by Till Harbaum
#include "usbh_core.h"
#include "usb_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "usb_gamepad.h"

#include "hidparser.h"

#define DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__)

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


static void usbh_update(struct usb_config *usb) {
    // check for active hid devices
    for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
        char *dev_str = "/dev/inputX";
        dev_str[10] = '0' + i;
        usb->hid_info[i].class = (struct usbh_hid *)usbh_find_class_instance(dev_str);
        
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
        }
    }

    // check for active xbox devices
    for(int i=0;i<CONFIG_USBHOST_MAX_XBOX_CLASS;i++) {
        char *dev_str = "/dev/xboxX";
        dev_str[9] = '0' + i;
        usb->xbox_info[i].class = (struct usbh_hid *)usbh_find_class_instance(dev_str);
        
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
        }
    }
}

static void xbox_parse(struct xbox_info_S *xbox) {
    // verify length field
    if(xbox->buffer[0] != 0 || xbox->buffer[1] != 20) {
        DEBUG("XBOX Joy%d: wrong length field %02x %02x\n", xbox->index, xbox->buffer[0], xbox->buffer[1]);
        return;
    }

    uint16_t wButtons = xbox->buffer[3] << 8 | xbox->buffer[2];

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
    hid->hid_state.joystick.js_index = hid->index;

    while(1) {
        int ret = usbh_submit_urb(&hid->class->intin_urb);
        if (ret < 0)
            DEBUG("HID client #%d: submit failed %d\n", hid->index, ret);
        else {
            // Wait for result
            xSemaphoreTake(hid->sem, 0xffffffffUL);
            if(hid->nbytes > 0) {
                hid_parse(&hid->report, &hid->hid_state, hid->buffer, hid->nbytes);
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                volatile uint16_t *snes = hid->index == 0 ? &hid1_state : &hid2_state;
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
            
            hid->nbytes = 0;
        }      
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void usbh_xbox_client_thread(void *arg) {
    struct xbox_info_S *xbox = (struct xbox_info_S *)arg;
    uint8_t xbox360_wired_led[] = {0x01, 0x03, 0x0A};       // command, length, data: 0 is off, 1 is all blinking, 2 is 1 flash then on
                                                            //                        6 is 1 on, 7 is 2 on, 0xA is rotating

    DEBUG("XBOX client #%d: thread started\n", xbox->index);

    usbh_bulk_urb_fill(&xbox->class->intout_urb,
        xbox->class->hport,
        xbox->class->intout,
        xbox360_wired_led,
        sizeof(xbox360_wired_led),
        0xfffffff, 
        NULL,
        NULL);

    int ret = usbh_submit_urb(&xbox->class->intout_urb);
    if (ret < 0)
        DEBUG("xbox set_led failed %d\n", ret);
    else
        DEBUG("xbox set_led sucess\n");

    while(1) {
        int ret = usbh_submit_urb(&xbox->class->intin_urb);
        if (ret < 0)
            DEBUG("XBOX client #%d: submit failed %d\n", xbox->index, ret);
        else {
            // Wait for result
            xSemaphoreTake(xbox->sem, 0xffffffffUL);
            DEBUG("XBOX client #%d: nbytes %d\n", xbox->index, xbox->nbytes);
            if (xbox->nbytes == XBOX_REPORT_SIZE)
                xbox_parse(xbox);
            xbox->nbytes = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void usbh_hid_thread(void *argument) {
    DEBUG("Starting usb gamepad host task...");

    struct usb_config *usb = (struct usb_config *)argument;

    while (1) {
        // DEBUG("calling usbh_update()\n");
        usbh_update(usb);

        for (int i = 0; i < CONFIG_USBHOST_MAX_HID_CLASS; i++) {
            if (usb->hid_info[i].state == STATE_DETECTED) {
                DEBUG("HID device %d is detected\n", i);
                usb->hid_info[i].state = STATE_RUNNING;

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
/*
        for (int i = 0; i < CONFIG_USBHOST_MAX_XBOX_CLASS; i++) {
            if (usb->xbox_info[i].state == STATE_DETECTED) {
                DEBUG("XBOX device %d is detected\n", i);
                usb->xbox_info[i].state = STATE_RUNNING;
                usb->xbox_info[i].js_index = i;

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

*/
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
