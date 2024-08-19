#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <gpiolib.h>
#include <errno.h>
#include <time.h>
#include "main.h"
#include "lr1110_update.h"

#define USB_PACKET	4192
#define ANSWER_SIZE	32

/* PA15 */
#define BUSY_PIN	16
/* PB4 */
#define RADIO_EVENT_PIN	21

#define SPI_DEVICE	"/dev/spi1"
#define USB_DEVICE	"/dev/ttyACM0"

static char usb_buff[USB_PACKET];

struct command {
	int (*cmd)(struct update *upd);
};

struct command commands[] = {
	{ update_start },
	{ update_details },
	{ update_data },
	{ update_hash },
	{ update_end },
};

int errno_to_com_err(int errno)
{
	int ret;

	if (!errno)
		ret = 0x9000;
	else
		ret = (0x6000 - errno);

	return ret;
}

int usb_read(struct bootloader *bootloader)
{
	int size, n;
	unsigned char ack = 0x15;
	struct command_header *cmd_header = (struct command_header *)bootloader->com_buff;
	char *cmd_data = bootloader->com_buff + sizeof(struct command_header);

	printf("reading on usb\n");

	read(bootloader->usb_fd, cmd_header, sizeof(struct command_header));

	printf("size: %x, ordinal: %x, options: %x\n", cmd_header->size, cmd_header->ordinal, cmd_header->options);

	write(bootloader->usb_fd, &ack, sizeof(unsigned char));

	size = cmd_header->size - sizeof(struct command_header);

	while (size) {
		n = (size > ANSWER_SIZE) ? ANSWER_SIZE : size;

		read(bootloader->usb_fd, cmd_data, n);
		write(bootloader->usb_fd, &ack, sizeof(unsigned char));

		size -= n;
		cmd_data += n;
	}

	return cmd_header->size;
}

void usb_write(struct bootloader *bootloader, int cmd_size)
{
	int size, n;
	unsigned char ack;
	struct reply_header *reply_header = (struct reply_header *)bootloader->com_buff;
	char *reply_data = bootloader->com_buff + sizeof(struct reply_header);

	printf("writing on usb\n");

	write(bootloader->usb_fd, reply_header, sizeof(struct reply_header));
	read(bootloader->usb_fd, &ack, sizeof(unsigned char));

	size = reply_header->size - sizeof(struct reply_header);

	while (size) {
		n = (size > ANSWER_SIZE) ? ANSWER_SIZE : size;

		write(bootloader->usb_fd, reply_data, n);
		read(bootloader->usb_fd, &ack, sizeof(unsigned char));

		size -= n;
		reply_data += n;
	}
}

int main(void)
{
	int ret, n;
	lr11xx_fw_update_status_t status;
	struct bootloader *bootloader;
	struct lr1110_update *lr1110_update;
	struct command_header *cmd_header;
	struct reply_header *reply_header;

	printf("bootloader\n");

	bootloader = malloc(sizeof(*bootloader));
	if (!bootloader) {
		printf("cannot allocate bootloader\n");
		return -ENOMEM;
	}

	bootloader->upd.lr1110_update = malloc(sizeof(*lr1110_update));
	if (!bootloader->upd.lr1110_update) {
		printf("cannot allocate lr1110_update\n");
		free(bootloader);
		return -ENOMEM;
	}

	bootloader->com_buff = usb_buff;
	cmd_header = (struct command_header *)bootloader->com_buff;
	reply_header = (struct reply_header *)bootloader->com_buff;
	lr1110_update = bootloader->upd.lr1110_update;

	lr1110_update->spi_fd = open(SPI_DEVICE, O_RDWR);
	if (lr1110_update->spi_fd < 0) {
		printf("failed to open spi device\n");
		free(lr1110_update);
		return -EIO;
	}

	bootloader->usb_fd = open(USB_DEVICE, O_RDWR);
	if (bootloader->usb_fd < 0) {
		printf("failed to open usb device\n");
		free(lr1110_update);
		free(bootloader);
		return -EIO;
	}

	lr1110_update->busy = gpiolib_export(BUSY_PIN);
	lr1110_update->radio_event = gpiolib_export(RADIO_EVENT_PIN);

	gpiolib_set_input(lr1110_update->busy);
	gpiolib_set_output(lr1110_update->radio_event, 1);

	time_usleep(2000 * 1000);

	printf("waiting for usb to enumerate...");

	ioctl(bootloader->usb_fd, IOCTL_USB_ENUMERATE, 0);

	printf("OK\n");

	while (1) {
		printf("waiting usb command...\n");
		bootloader->com_buff = usb_buff;

		n = usb_read(bootloader);

		bootloader->upd.buff = bootloader->com_buff;
		bootloader->upd.size = n;

		ret = commands[cmd_header->ordinal].cmd(&bootloader->upd);

		reply_header->code = errno_to_com_err(ret);
		reply_header->size = bootloader->upd.size;

		bootloader->com_buff = bootloader->upd.buff;

		usb_write(bootloader, bootloader->upd.size);
	}
}
