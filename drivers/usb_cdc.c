#include <fdtparse.h>
#include <drv/device.h>
#include <drv/usb.h>
#include <drv/usb_cdc.h>
#include <init.h>
#include <string.h>
#include <kernel/printk.h>
#include <kernel/ksem.h>
#include <kernel/spinlock.h>
#include <drv/irq.h>
#include <mm/mm.h>
#include <ioctl.h>
#include <errno.h>

#define CDC_EP0_SIZE    0x08
#define CDC_RXD_EP      0x01
#define CDC_TXD_EP      0x81
#define CDC_DATA_SZ     0x40
#define CDC_NTF_EP      0x82
#define CDC_NTF_SZ      0x08
#define HID_RIN_EP      0x83
#define HID_RIN_SZ      0x10

#define CDC_PROTOCOL USB_PROTO_NONE

/* Declaration of the report descriptor */
struct cdc_config {
	struct usb_config_descriptor        config;
	struct usb_iad_descriptor           comm_iad;
	struct usb_interface_descriptor     comm;
	struct usb_cdc_header_desc          cdc_hdr;
	struct usb_cdc_call_mgmt_desc       cdc_mgmt;
	struct usb_cdc_acm_desc             cdc_acm;
	struct usb_cdc_union_desc           cdc_union;
	struct usb_endpoint_descriptor      comm_ep;
	struct usb_interface_descriptor     data;
	struct usb_endpoint_descriptor      data_eprx;
	struct usb_endpoint_descriptor      data_eptx;
} __attribute__((packed));


/* Device descriptor */
static const struct usb_device_descriptor device_desc = {
	.bLength = sizeof(struct usb_device_descriptor),
		.bDescriptorType = USB_DTYPE_DEVICE,
		.bcdUSB = VERSION_BCD(2, 0, 0),
		.bDeviceClass = USB_CLASS_IAD,
		.bDeviceSubClass = USB_SUBCLASS_IAD,
		.bDeviceProtocol = USB_PROTO_IAD,
		.bMaxPacketSize0 = CDC_EP0_SIZE,
		.idVendor = 0x0483,
		.idProduct = 0x5740,
		.bcdDevice = VERSION_BCD(1, 0, 0),
		.iManufacturer = 1,
		.iProduct = 2,
		.iSerialNumber = INTSERIALNO_DESCRIPTOR,
		.bNumConfigurations = 1,
	};

/* Device configuration descriptor */
static const struct cdc_config config_desc = {
	.config =
	{
		.bLength = sizeof(struct usb_config_descriptor),
			.bDescriptorType = USB_DTYPE_CONFIGURATION,
			.wTotalLength = sizeof(struct cdc_config),
				.bNumInterfaces = 2,
				.bConfigurationValue = 1,
				.iConfiguration = NO_DESCRIPTOR,
				.bmAttributes = USB_CFG_ATTR_RESERVED | USB_CFG_ATTR_SELFPOWERED,
				.bMaxPower = USB_CFG_POWER_MA(100),
			},
	.comm_iad = {
		.bLength = sizeof(struct usb_iad_descriptor),
			.bDescriptorType = USB_DTYPE_INTERFASEASSOC,
			.bFirstInterface = 0,
			.bInterfaceCount = 2,
			.bFunctionClass = USB_CLASS_CDC,
			.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
			.bFunctionProtocol = CDC_PROTOCOL,
			.iFunction = NO_DESCRIPTOR,
		},
	.comm = {
		.bLength = sizeof(struct usb_interface_descriptor),
			.bDescriptorType = USB_DTYPE_INTERFACE,
			.bInterfaceNumber = 0,
			.bAlternateSetting = 0,
			.bNumEndpoints = 1,
			.bInterfaceClass = USB_CLASS_CDC,
			.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
			.bInterfaceProtocol = CDC_PROTOCOL,
			.iInterface = NO_DESCRIPTOR,
		},
	.cdc_hdr = {
		.bFunctionLength = sizeof(struct usb_cdc_header_desc),
			.bDescriptorType = USB_DTYPE_CS_INTERFACE,
			.bDescriptorSubType = USB_DTYPE_CDC_HEADER,
			.bcdCDC = VERSION_BCD(1, 1, 0),
		},
	.cdc_mgmt = {
		.bFunctionLength = sizeof(struct usb_cdc_call_mgmt_desc),
			.bDescriptorType = USB_DTYPE_CS_INTERFACE,
			.bDescriptorSubType = USB_DTYPE_CDC_CALL_MANAGEMENT,
			.bmCapabilities = 0,
			.bDataInterface = 1,

		},
	.cdc_acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_desc),
			.bDescriptorType = USB_DTYPE_CS_INTERFACE,
			.bDescriptorSubType = USB_DTYPE_CDC_ACM,
			.bmCapabilities = 0,
		},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_desc),
			.bDescriptorType = USB_DTYPE_CS_INTERFACE,
			.bDescriptorSubType = USB_DTYPE_CDC_UNION,
			.bMasterInterface0 = 0,
			.bSlaveInterface0 = 1,
		},
	.comm_ep = {
		.bLength = sizeof(struct usb_endpoint_descriptor),
			.bDescriptorType = USB_DTYPE_ENDPOINT,
			.bEndpointAddress = CDC_NTF_EP,
			.bmAttributes = USB_EPTYPE_INTERRUPT,
			.wMaxPacketSize = CDC_NTF_SZ,
			.bInterval = 0xFF,
		},
	.data = {
		.bLength = sizeof(struct usb_interface_descriptor),
			.bDescriptorType = USB_DTYPE_INTERFACE,
			.bInterfaceNumber = 1,
			.bAlternateSetting = 0,
			.bNumEndpoints = 2,
			.bInterfaceClass = USB_CLASS_CDC_DATA,
			.bInterfaceSubClass = USB_SUBCLASS_NONE,
			.bInterfaceProtocol = USB_PROTO_NONE,
			.iInterface = NO_DESCRIPTOR,
		},
	.data_eprx = {
		.bLength = sizeof(struct usb_endpoint_descriptor),
			.bDescriptorType = USB_DTYPE_ENDPOINT,
			.bEndpointAddress = CDC_RXD_EP,
			.bmAttributes = USB_EPTYPE_BULK,
			.wMaxPacketSize = CDC_DATA_SZ,
			.bInterval = 0x01,
		},
	.data_eptx = {
		.bLength = sizeof(struct usb_endpoint_descriptor),
			.bDescriptorType = USB_DTYPE_ENDPOINT,
			.bEndpointAddress = CDC_TXD_EP,
			.bmAttributes = USB_EPTYPE_BULK,
			.wMaxPacketSize = CDC_DATA_SZ,
			.bInterval = 0x01,
		},

};

static const struct usb_string_descriptor lang_desc     = USB_ARRAY_DESC(USB_LANGID_ENG_US);
static const struct usb_string_descriptor manuf_desc_en = USB_STRING_DESC("Open source USB stack for STM32");
static const struct usb_string_descriptor prod_desc_en  = USB_STRING_DESC("CDC Loopback demo");
static const struct usb_string_descriptor *const dtable[] = {
	&lang_desc,
	&manuf_desc_en,
	&prod_desc_en,
};


static struct usb_cdc_line_coding cdc_line = {
	.dwDTERate = 38400,
	.bCharFormat = USB_CDC_1_STOP_BITS,
	.bParityType = USB_CDC_NO_PARITY,
	.bDataBits = 8,
};

static usbd_respond cdc_getdesc(usbd_ctlreq *req, void **address, uint16_t *length)
{
	const uint8_t dtype = req->wValue >> 8;
	const uint8_t dnumber = req->wValue & 0xFF;
	const void* desc;
	uint16_t len = 0;
	switch (dtype) {
	case USB_DTYPE_DEVICE:
		desc = &device_desc;
		break;
	case USB_DTYPE_CONFIGURATION:
		desc = &config_desc;
		len = sizeof(config_desc);
		break;
	case USB_DTYPE_STRING:
		if (dnumber < 3) {
			desc = dtable[dnumber];
		} else {
			return usbd_fail;
		}
		break;
	default:
		return usbd_fail;
	}
	if (len == 0) {
		len = ((struct usb_header_descriptor*)desc)->bLength;
	}
	*address = (void*)desc;
	*length = len;
	return usbd_ack;
}


static usbd_respond cdc_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback)
{
	if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_INTERFACE | USB_REQ_CLASS)
	    && req->wIndex == 0) {
		switch (req->bRequest) {
		case USB_CDC_SET_CONTROL_LINE_STATE:
			return usbd_ack;
		case USB_CDC_SET_LINE_CODING:
			memcpy(&cdc_line, req->data, sizeof(cdc_line));
			return usbd_ack;
		case USB_CDC_GET_LINE_CODING:
			dev->status.data_ptr = &cdc_line;
			dev->status.data_count = sizeof(cdc_line);
			return usbd_ack;
		default:
			return usbd_fail;
		}
	}

	return usbd_fail;
}

static void cdc_rxonly(usbd_device *dev, uint8_t event, uint8_t ep)
{
	if(event == usbd_evt_eptx)
		return;

	int read;
	struct usb_cdc *cdc = dev->class_priv;

	/*
	 * 2 ways of dealing with received data:
	 *    - User has provided a buffer and an excepted len because he was ready to read - so we use these infos.
	 *    - User was not ready to receive, we use internal buffer.
	 */
	if (cdc->len) {
		read = usbd_ep_read(dev, ep, cdc->user_buffer + cdc->offset, cdc->len - cdc->offset);
		cdc->offset += read;

		if (cdc->offset == cdc->len) {
			cdc->len = 0;
			cdc->offset = 0;
			ksem_post_isr(&cdc->read_sem);
		}
	} else {
		read = usbd_ep_read(dev, ep, cdc->internal_buff + cdc->internal_offset, sizeof(cdc->internal_buff) - cdc->internal_offset);
		cdc->internal_offset = read;
	}
}

static usbd_respond cdc_setconf(usbd_device *dev, uint8_t cfg)
{
	switch (cfg) {
	case 0:
		/* deconfiguring device */
		usbd_ep_deconfig(dev, CDC_NTF_EP);
		usbd_ep_deconfig(dev, CDC_TXD_EP);
		usbd_ep_deconfig(dev, CDC_RXD_EP);
		usbd_reg_endpoint(dev, CDC_RXD_EP, 0);
		usbd_reg_endpoint(dev, CDC_TXD_EP, 0);
		return usbd_ack;
	case 1:
		/* configuring device */
		usbd_ep_config(dev, CDC_RXD_EP, USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/, CDC_DATA_SZ);
		usbd_ep_config(dev, CDC_TXD_EP, USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/, CDC_DATA_SZ);
		usbd_ep_config(dev, CDC_NTF_EP, USB_EPTYPE_INTERRUPT, CDC_NTF_SZ);
		usbd_reg_endpoint(dev, CDC_RXD_EP, cdc_rxonly);
		usbd_ep_write(dev, CDC_TXD_EP, 0, 0);
		return usbd_ack;
	default:
		return usbd_fail;
	}
}

static int cdc_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct usb_cdc *usb = container_of(dev, struct usb_cdc, dev);

#ifdef TRACE_EXCHANGE
	printk("=> %04d [ ", size);
	for(int i = 0; i < size; i++)
		printk("%02x ", buff[i]);
	printk("]\n");
#endif

	usbd_ep_write( usb->usb, CDC_TXD_EP, buff, size );

	return size;
}

static int cdc_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	struct usb_cdc *usb = container_of(dev, struct usb_cdc, dev);

	if (usb->internal_offset) {
		thread_lock(state);

		memcpy(buff, usb->internal_buff, usb->internal_offset);
		buff += usb->internal_offset;
		size -= usb->internal_offset;
		usb->internal_offset = 0;

		thread_unlock(state);
	} else {
		usb->user_buffer = buff;
		usb->len = size;

		ksem_wait( &usb->read_sem );
	}

#ifdef TRACE_EXCHANGE
	printk("<= %04d [ ", size);
	for(int i = 0; i < size; i++ )
		printk("%02x ", buff[i]);
	printk("]\n");
#endif

	return size;
}

static int cdc_ioctl(struct device *dev, int request, char *arg)
{
	int ret = 0;
	struct usb_cdc *usb = container_of(dev, struct usb_cdc, dev);

	switch (request) {
	case IOCTL_USB_ENUMERATE:
		ksem_wait( &usb->usb->addressed );
		break;

	default :
		ret = -EINVAL ;
		break ;
	}

	return ret ;
}

int usbcdc_init(struct device *dev)
{
	int offset;
	int ret = 0;
	int parent_offset;
	char *fdt_path = NULL;
	struct usb_cdc *cdc = NULL;
	struct device *usbdev;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, dev->of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto err;
	}

	parent_offset = fdt_parent_offset(fdt_blob, offset);
	if (parent_offset < 0) {
		error_printk("failed to retrive usb cdc parent\n");
		goto err;
	}

	fdt_path = fdtparse_get_path(parent_offset);
	if (!fdt_path) {
		error_printk("failed to fdt path of usb device parent node\n");
		goto err;
	}

	usbdev = device_from_of_path((const char *)fdt_path);
	if (!usbdev) {
		error_printk("failed to find device with fdt path: %s\n", fdt_path);
		goto err;
	}

	cdc = kmalloc(sizeof(*cdc));
	if (!cdc) {
		error_printk("failed to alloc cdc struct\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&cdc->dev, dev, sizeof(struct device));

	cdc->dev.read = cdc_read;
	cdc->dev.write = cdc_write;
	cdc->dev.ioctl = cdc_ioctl;

	cdc->usb_device = usbdev;
	cdc->offset = 0;
	cdc->internal_offset = 0;

	cdc->usb = (usbd_device *)cdc->usb_device->ioctl(cdc->usb_device, IOCTL_INIT, (char *)CDC_EP0_SIZE);

	ksem_init(&cdc->read_sem, 1);
	ksem_init(&cdc->usb->addressed, 1);

	snprintf(cdc->dev.name, sizeof(cdc->dev.name), "/dev/ttyACM0");

	cdc->usb->class_priv = cdc;

	usbd_reg_config(cdc->usb, cdc_setconf);
	usbd_reg_control(cdc->usb, cdc_control);
	usbd_reg_descr(cdc->usb, cdc_getdesc);
	usbd_enable(cdc->usb, true);
	usbd_connect(cdc->usb, true);

	ret = device_register(&cdc->dev);
	if (ret < 0) {
		error_printk("failed to register cdc device\n");
		goto free_cdc;
	}

	return 0;

free_cdc:
	kfree(cdc);
err:
	return ret;
}


struct device usbcdc_driver = {
	.of_compat = "usb,usb_cdc",
	.probe = usbcdc_init,
};

static int usbcdc_register(void)
{
	int ret = 0;

	ret = device_of_register(&usbcdc_driver);
	if (ret < 0)
		error_printk("failed to register usbcdc device\n");
	return ret;
}
coredevice_initcall(usbcdc_register);
