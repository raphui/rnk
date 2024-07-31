#ifndef USB_STM32_H
#define USB_STM32_H

#include <kernel/ksem.h>
#include <stdint.h>

#include <usb.h>
#include <usb_cdc.h>
#include <usb_std.h>
#include <usbd_core.h>



struct usb_pdata {
	usbd_device usbd;
	uint32_t ubuf[0x20];
};

#endif /* USB_STM32_H */
