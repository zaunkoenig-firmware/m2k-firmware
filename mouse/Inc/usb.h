#pragma once

#include "usbd_def.h"

extern USBD_HandleTypeDef USBD_Device;

void usb_init(int hs_usb);

void usb_wait_configured(void);
