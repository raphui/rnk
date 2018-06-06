/*
 * Copyright (C) 2018  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef USB_STM32_H
#define USB_STM32_H

#include <ksem.h>
#include <stdint.h>

#define __IO volatile
#define USE_USB_OTG_FS

#include <usbd_cdc_core.h>
#include <usbd_usr.h>
#include <usbd_desc.h>

struct usb_pdata {
	USB_OTG_CORE_HANDLE USB_OTG_dev;
	unsigned char *user_buffer;
	struct semaphore acknoledge;
	struct semaphore device_addressed;
	unsigned int len;
	
	unsigned char CmdBuff[CDC_CMD_PACKET_SZE];
	unsigned int APP_Rx_ptr_out;
	unsigned int APP_Rx_length;
	unsigned char USB_Tx_State;
	unsigned int cdcCmd;
	unsigned int cdcLen;	
	volatile unsigned int  usbd_cdc_AltSet;
	unsigned int USBD_ep_status; 
	unsigned int USBD_default_cfg;
	unsigned int USBD_cfg_status;
	unsigned char cfgidx;
	
	unsigned char abort;
};

#endif /* USB_STM32_H */
