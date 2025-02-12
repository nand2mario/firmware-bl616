#include "usbh_core.h"
#include "usbh_cdc_acm.h"
#include "usbh_hid.h"
#include "usbh_msc.h"
#include "usbh_video.h"
#include "usbh_audio.h"

#define TEST_USBH_CDC_ACM   0
#define TEST_USBH_HID       0
#define TEST_USBH_MSC       1
#define TEST_USBH_MSC_FATFS 1
#define TEST_USBH_AUDIO     0
#define TEST_USBH_VIDEO     0

#if TEST_USBH_HID
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t hid_buffer[128];

struct usbh_urb hid_intin_urb;

void usbh_hid_callback(void *arg, int nbytes)
{
    //struct usbh_hid *hid_class = (struct usbh_hid *)arg;

    if (nbytes > 0) {
        for (size_t i = 0; i < nbytes; i++) {
            USB_LOG_RAW("0x%02x ", hid_buffer[i]);
        }
        USB_LOG_RAW("nbytes:%d\r\n", nbytes);
        usbh_submit_urb(&hid_intin_urb);
    }
}

static void usbh_hid_thread(void *argument)
{
    int ret;
    struct usbh_hid *hid_class;

    while (1) {
        // clang-format off
find_class:
        // clang-format on
        hid_class = (struct usbh_hid *)usbh_find_class_instance("/dev/input0");
        if (hid_class == NULL) {
            USB_LOG_RAW("do not find /dev/input0\r\n");
            usb_osal_msleep(1500);
            continue;
        }
        usbh_int_urb_fill(&hid_intin_urb, hid_class->intin, hid_buffer, 8, 0, usbh_hid_callback, hid_class);
        ret = usbh_submit_urb(&hid_intin_urb);
        if (ret < 0) {
            usb_osal_msleep(1500);
            goto find_class;
        }

        while (1) {
            hid_class = (struct usbh_hid *)usbh_find_class_instance("/dev/input0");
            if (hid_class == NULL) {
                goto find_class;
            }
            usb_osal_msleep(1500);
        }
    }
}
#endif

#if TEST_USBH_MSC

#if TEST_USBH_MSC_FATFS
#include "ff.h"

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_write_buffer[25 * 100];

USB_NOCACHE_RAM_SECTION FATFS fs;
USB_NOCACHE_RAM_SECTION FIL fnew;
UINT fnum;
FRESULT res_sd = 0;

// in diskio.h, DEV_USB is 4

int usb_msc_fatfs_test()
{
    const char *tmp_data = "cherryusb fatfs demo...\r\n";

    USB_LOG_RAW("data len:%d\r\n", strlen(tmp_data));
    for (uint32_t i = 0; i < 100; i++) {
        memcpy(&read_write_buffer[i * 25], tmp_data, strlen(tmp_data));
    }

    res_sd = f_mount(&fs, "usb:", 1);
    if (res_sd != FR_OK) {
        USB_LOG_RAW("mount fail, res:%d\r\n", res_sd);
        return -1;
    }

    USB_LOG_RAW("test fatfs write\r\n");
    res_sd = f_open(&fnew, "usb:test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res_sd == FR_OK) {
        res_sd = f_write(&fnew, read_write_buffer, sizeof(read_write_buffer), &fnum);
        if (res_sd == FR_OK) {
            USB_LOG_RAW("write success, write len：%d\n", fnum);
        } else {
            USB_LOG_RAW("write fail\r\n");
            goto unmount;
        }
        f_close(&fnew);
    } else {
        USB_LOG_RAW("open fail\r\n");
        goto unmount;
    }
    USB_LOG_RAW("test fatfs read\r\n");

    res_sd = f_open(&fnew, "usb:test.txt", FA_OPEN_EXISTING | FA_READ);
    if (res_sd == FR_OK) {
        res_sd = f_read(&fnew, read_write_buffer, sizeof(read_write_buffer), &fnum);
        if (res_sd == FR_OK) {
            USB_LOG_RAW("read success, read len：%d\n", fnum);
        } else {
            USB_LOG_RAW("read fail\r\n");
            goto unmount;
        }
        f_close(&fnew);
    } else {
        USB_LOG_RAW("open fail\r\n");
        goto unmount;
    }
    f_mount(NULL, "", 1);
    return 0;
unmount:
    f_mount(NULL, "", 1);
    return -1;
}
#endif

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t partition_table[512];

static void usbh_msc_thread(void *argument)
{
    int ret;
    struct usbh_msc *msc_class;

    while (1) {
find_class:
        msc_class = (struct usbh_msc *)usbh_find_class_instance("/dev/sda");
        if (msc_class == NULL) {
            USB_LOG_RAW("do not find /dev/sda\r\n");
            usb_osal_msleep(2000);
            continue;
        }

#if 1
        /* get the partition table */
        ret = usbh_msc_scsi_read10(msc_class, 0, partition_table, 1);
        if (ret < 0) {
            USB_LOG_RAW("scsi_read10 error,ret:%d\r\n", ret);
            usb_osal_msleep(2000);
            goto find_class;
        }
        for (uint32_t i = 0; i < 512; i++) {
            if (i % 16 == 0) {
                USB_LOG_RAW("\r\n");
            }
            USB_LOG_RAW("%02x ", partition_table[i]);
        }
        USB_LOG_RAW("\r\n");
#endif

#if TEST_USBH_MSC_FATFS
        extern void fatfs_usbh_driver_register(void);
        fatfs_usbh_driver_register();       // register the USB disk driver to FatFS

        usb_msc_fatfs_test();
#endif
        while (1) {
            msc_class = (struct usbh_msc *)usbh_find_class_instance("/dev/sda");
            if (msc_class == NULL) {
                goto find_class;
            }
            usb_osal_msleep(2000);
        }
    }
}
#endif

void usbh_cdc_acm_run(struct usbh_cdc_acm *cdc_acm_class)
{
}

void usbh_cdc_acm_stop(struct usbh_cdc_acm *cdc_acm_class)
{
}

void usbh_hid_run(struct usbh_hid *hid_class)
{
}

void usbh_hid_stop(struct usbh_hid *hid_class)
{
}

void usbh_msc_run(struct usbh_msc *msc_class)
{
}

void usbh_msc_stop(struct usbh_msc *msc_class)
{
}

void usbh_audio_run(struct usbh_audio *audio_class)
{
}

void usbh_audio_stop(struct usbh_audio *audio_class)
{
}

void usbh_video_run(struct usbh_video *video_class)
{
}

void usbh_video_stop(struct usbh_video *video_class)
{
}

void usbh_class_test(void)
{
#if TEST_USBH_CDC_ACM
    usb_osal_thread_create("usbh_cdc", 2048, CONFIG_USBHOST_PSC_PRIO + 1, usbh_cdc_acm_thread, NULL);
#endif
#if TEST_USBH_HID
    usb_osal_thread_create("usbh_hid", 2048, CONFIG_USBHOST_PSC_PRIO + 1, usbh_hid_thread, NULL);
#endif
#if TEST_USBH_MSC
    usb_osal_thread_create("usbh_msc", 2048, CONFIG_USBHOST_PSC_PRIO + 1, usbh_msc_thread, NULL);
#endif
#if TEST_USBH_AUDIO
    extern void usbh_audio_test();
    usbh_audio_test();
#endif
#if TEST_USBH_VIDEO
    extern void usbh_video_test();
    usbh_video_test();
#endif
}