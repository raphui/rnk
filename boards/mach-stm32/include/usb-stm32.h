#ifndef USB_STM32_H
#define USB_STM32_H

#include <kernel/ksem.h>
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
