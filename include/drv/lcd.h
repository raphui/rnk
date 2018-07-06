#ifndef LCD_H
#define LCD_H

#include <drv/device.h>
#include <list.h>

struct lcd {
	unsigned int base_reg;
	unsigned short hsync;
	unsigned short vsync;
	unsigned short hbp;
	unsigned short hfp;
	unsigned short vbp;
	unsigned short vfp;
	unsigned int width;
	unsigned int height;
	unsigned char bpp;
	unsigned int fb_addr;
	struct lcd_operations *lcd_ops;
	struct list_node node;
	struct device dev;
};

struct lcd_bus {
	struct device dev;
};

struct lcd_operations
{
	int (*configure)(struct lcd *lcd);
};

struct lcd *lcd_get_device(void);
int lcd_configure_device(struct lcd *lcd);
struct lcd *lcd_new_device(void);
int lcd_remove_device(struct lcd *lcd);
int lcd_register_device(struct lcd *lcd);
void lcd_init_gpio(void);
int lcd_init(void);

#endif /* LCD_H */
