#pragma once

#include "FreeRTOS.h"
#include "semphr.h"

#include "usbh_core.h"
#include "usbh_hid.h"

// Start USB gamepad tasks
void usb_gamepad_init(void);

// state will be written here by the USB gamepad task
extern volatile uint16_t hid1_state;               // SNES-format gamepad state
extern volatile uint16_t hid2_state;               // SNES-format gamepad state
extern SemaphoreHandle_t state_mutex;              // for all global state access
