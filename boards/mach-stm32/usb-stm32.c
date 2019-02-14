#include <board.h>
#include <armv7m/nvic.h>
#include <mach/rcc-stm32.h>
#include <mach/pio-stm32.h>
#include <mach/usb-stm32.h>
#include <errno.h>
#include <fdtparse.h>
#include <drv/device.h>
#include <init.h>
#include <string.h>
#include <kernel/printk.h>
#include <kernel/ksem.h>
#include <drv/irq.h>
#include <mm/mm.h>
#include <drv/usb.h>

extern uint32_t USBD_OTG_ISR_Handler(USB_OTG_CORE_HANDLE *pdev);

static int stm32_usb_write(struct usb_device *usbdev, unsigned char *buff, unsigned int len)
{
	int ret = len;

	struct usb_pdata *pdata = usbdev->priv;

	DCD_EP_Tx(&pdata->USB_OTG_dev, CDC_IN_EP, buff, len);

	return ret;
}

static int stm32_usb_read(struct usb_device *usbdev, unsigned char *buff, unsigned int len)
{
	int ret = 0;
	struct usb_pdata *pdata = usbdev->priv;

	pdata->user_buffer = buff;

	DCD_EP_PrepareRx(&pdata->USB_OTG_dev, CDC_OUT_EP, pdata->user_buffer, len);

	ksem_wait(&pdata->acknoledge);
		
	if(pdata->abort)
	{
		ret = 0;
	}
	else
	{
		ret = pdata->len;
		pdata->len = 0;
	}
	
	pdata->abort = 0;

	return ret;
}

static void stm32_usb_isr(void *arg)
{
	struct usb_pdata *pdata = (struct usb_pdata *)arg;

	USBD_OTG_ISR_Handler(&pdata->USB_OTG_dev);
}

unsigned short VCP_DataRx(struct usb_pdata *pdata, unsigned char *buff, unsigned int len)
{
	if(pdata->user_buffer) {
		pdata->user_buffer = NULL;
		pdata->len = len;
		ksem_post_isr(&pdata->acknoledge);
	}

	return USBD_OK;
}

struct usb_operations usb_ops = {
	.write = stm32_usb_write,
	.read = stm32_usb_read,
};

static int stm32_usb_of_init(struct usb_device *usb)
{
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, usb->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, usb->dev.of_compat);
	if (ret < 0)
		goto out;

	usb->base_reg = (unsigned int)fdtparse_get_addr32(offset, "reg");
	if (!usb->base_reg) {
		error_printk("failed to retrieve usb base reg from fdt\n");
		ret = -ENOENT;
		goto out;
	}

	ret = stm32_pio_of_configure(offset);
	if (ret < 0) {
		error_printk("failed to configure usb gpio\n");
		goto out;
	}

	ret = fdtparse_get_int(offset, "interrupts", (int *)&usb->irq);
	if (ret < 0) {
		error_printk("failed to retrieve usb irq\n");
		ret = -EIO;
		goto out;
	}

	ret = stm32_rcc_of_enable_clk(offset, &usb->clock);
	if (ret < 0) {
		error_printk("failed to retrieve usb clock\n");
		ret = -EIO;
		goto out;
	}

out:
	return ret;
}

static int stm32_usb_init(struct device *device)
{
	int ret = 0;
	struct usb_device *usb = NULL;
	struct usb_pdata *pdata = NULL;

	pdata = (struct usb_pdata *)kmalloc(sizeof(struct usb_pdata));
	if (!pdata) {
		error_printk("failed to allocate usb pdata\n");
		ret = -ENOMEM;
		goto err;
	}

	usb = usb_new_device();
	if (!usb) {
		error_printk("failed to retrieve new usb device\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&usb->dev, device, sizeof(struct device));

	ret = stm32_usb_of_init(usb);
	if (ret < 0) {
		error_printk("failed to init usb with fdt data\n");
		goto err;
	}

	memset(pdata, 0, sizeof(struct usb_pdata));

	pdata->cdcCmd = 0xFF;
	pdata->USB_OTG_dev.priv = pdata;

	ksem_init(&pdata->acknoledge, 1);

	usb->usb_ops = &usb_ops;
	usb->priv = pdata;

	USBD_Init(&pdata->USB_OTG_dev, USB_OTG_FS_CORE_ID, (USBD_DEVICE *)&USR_desc, 0, 0);

	ret = irq_request(usb->irq, stm32_usb_isr, pdata);
	if (ret < 0) {
		error_printk("failed to request usb irq\n");
		goto disable_clk;
	}

	nvic_enable_interrupt(usb->irq);

	ret = usb_register_device(usb);
	if (ret < 0) {
		error_printk("failed to register usb device\n");
		goto disable_clk;
	}

	return 0;

disable_clk:
	stm32_rcc_disable_clk(usb->clock.gated, usb->clock.id);
err:
	return ret;
}

struct device stm32_usb_driver = {
	.of_compat = "st,stm32f4xx-usb",
	.probe = stm32_usb_init,
};

static int stm32_usb_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_usb_driver);
	if (ret < 0)
		error_printk("failed to register stm32_usb device\n");
	return ret;
}
postarch_initcall(stm32_usb_register);
