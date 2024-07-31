#ifndef __USB_H__
#define __USB_H__

#include <mach/usb-stm32.h>
#include <board.h>
#include <arch/system.h>

#include <stdint.h>

#include "os.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

typedef struct __device {
	void *priv;
} device;

#endif // __USB_H__
