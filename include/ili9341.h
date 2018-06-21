#ifndef ILI9341_H
#define ILI9341_H

#include <device.h>
#include <spi.h>
#include <lcd.h>

struct ili9341_device {
	unsigned int fb_addr;
	struct spi_device *spi;
	struct lcd *lcd;
	struct device dev;
};

/* LCD settings */
#define ILI9341_WIDTH				240
#define ILI9341_HEIGHT				320
#define ILI9341_HSYNC				16
#define ILI9341_VSYNC				2
#define ILI9341_HBP				40
#define ILI9341_HFP				10
#define ILI9341_VBP				2
#define ILI9341_VFP				4
#define ILI9341_BPP				2
#define ILI9341_PIXEL				76800

/* Commands */
#define ILI9341_RESET				0x01
#define ILI9341_SLEEP_OUT			0x11
#define ILI9341_GAMMA				0x26
#define ILI9341_DISPLAY_OFF			0x28
#define ILI9341_DISPLAY_ON			0x29
#define ILI9341_COLUMN_ADDR			0x2A
#define ILI9341_PAGE_ADDR			0x2B
#define ILI9341_GRAM				0x2C
#define ILI9341_MAC				0x36
#define ILI9341_PIXEL_FORMAT			0x3A
#define ILI9341_WDB				0x51
#define ILI9341_WCD				0x53
#define ILI9341_RGB_INTERFACE			0xB0
#define ILI9341_FRC				0xB1
#define ILI9341_BPC				0xB5
#define ILI9341_DFC				0xB6
#define ILI9341_POWER1				0xC0
#define ILI9341_POWER2				0xC1
#define ILI9341_VCOM1				0xC5
#define ILI9341_VCOM2				0xC7
#define ILI9341_POWERA				0xCB
#define ILI9341_POWERB				0xCF
#define ILI9341_PGAMMA				0xE0
#define ILI9341_NGAMMA				0xE1
#define ILI9341_DTCA				0xE8
#define ILI9341_DTCB				0xEA
#define ILI9341_POWER_SEQ			0xED
#define ILI9341_3GAMMA_EN			0xF2
#define ILI9341_INTERFACE			0xF6
#define ILI9341_PRC				0xF7

#endif /* ILI9341_H */
