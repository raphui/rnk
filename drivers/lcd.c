#include <board.h>
#include <lcd.h>
#include <printk.h>
#include <errno.h>
#include <string.h>
#include <mm.h>
#include <init.h>

static int dev_count = 0;
static char dev_prefix[10] = "/dev/fb";
static struct list_node lcd_device_list;

struct lcd *lcd_get_device(void)
{
	struct lcd *lcddev = NULL;

	lcddev = list_peek_head_type(&lcd_device_list, struct lcd, node);

	return lcddev;
}

int lcd_configure_device(struct lcd *lcd)
{
	return lcd->lcd_ops->configure(lcd);
}

struct lcd *lcd_new_device(void)
{
	struct lcd *lcddev = NULL;

	lcddev = (struct lcd *)kmalloc(sizeof(struct lcd));
	if (!lcddev) {
		error_printk("cannot allocate lcd device\n");
		return NULL;
	}

	dev_count++;

	return lcddev;
}

int lcd_remove_device(struct lcd *lcd)
{
	int ret = 0;
	struct lcd *lcddev = NULL;

	list_for_every_entry(&lcd_device_list, lcddev, struct lcd, node)
		if (lcddev == lcd)
			break;

	if (lcddev) {
		list_delete(&lcddev->node);
		kfree(lcddev);
	}
	else
		ret = -ENOENT;


	dev_count--;

	return ret;
}

int lcd_register_device(struct lcd *lcd)
{
	int ret = 0;
	char tmp[10] = {0};

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));

	/* XXX: ascii 0 start at 0x30
	 *	and (dev_count - 1) to start at 0
	 */
	tmp[8] = 0x30 + (dev_count - 1);

	memcpy(lcd->dev.name, tmp, sizeof(tmp));

	ret = device_register(&lcd->dev);
	if (ret < 0) {
		error_printk("failed to register device\n");
		ret = -ENOMEM;
		goto failed_out;
	}

	list_add_tail(&lcd_device_list, &lcd->node);

	return ret;

failed_out:
	/* XXX: deallocate here ? */
	kfree(lcd);
	return ret;
}

int lcd_init(void)
{
	int ret = 0;

	list_initialize(&lcd_device_list);

	return ret;
}
coredevice_initcall(lcd_init);
