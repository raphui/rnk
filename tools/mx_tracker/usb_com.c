#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include "usb_com.h"

#define MAX_SIZE	4192
#define ANSWER_SIZE	32

struct usb_com {
	int fd;
};

static struct usb_com usb_priv;

int errno_to_com_err(int errno)
{
	int ret;

	if (!errno)
		ret = 0x9000;	
	else
		ret = (0x6000 - errno);

	return ret;
}

int com_err_to_errno(int com_err)
{
	int ret;

	if (com_err == 0x9000)
		ret = 0;
	else
		ret = -(com_err - 0x6000);

	return ret;
}


static int usb_read(struct reply_header *reply_hdr, char **buff)
{
	int n, size;
	int ret = 0;
	unsigned char ack = 0x15;
	char *reply_data;

	read(usb_priv.fd, reply_hdr, sizeof(struct reply_header));

	write(usb_priv.fd, &ack, sizeof(unsigned char));

	size = reply_hdr->size - sizeof(struct reply_header);

	if (size) {
		reply_data = malloc(size);
		*buff = reply_data;
	}

	while (size) {
		n = (size > ANSWER_SIZE) ? ANSWER_SIZE : size;

		read(usb_priv.fd, reply_data, n);
		write(usb_priv.fd, &ack, sizeof(unsigned char));

		size -= n;
		reply_data += n;
	}

	ret = com_err_to_errno(reply_hdr->code);

	return ret;
}

static int usb_write(struct command_header *cmd_hdr, char *buff)
{
	int n, size;
	int ret = 0;
	unsigned char ack = 0x15;
	char *cmd_data = buff;

	write(usb_priv.fd, cmd_hdr, sizeof(struct command_header));

	read(usb_priv.fd, &ack, sizeof(unsigned char));

	size = cmd_hdr->size - sizeof(struct command_header);

	while (size) {
		n = (size > ANSWER_SIZE) ? ANSWER_SIZE : size;

		write(usb_priv.fd, cmd_data, n);
		read(usb_priv.fd, &ack, sizeof(unsigned char));

		size -= n;
		cmd_data += n;
	}

	return ret;
}

int usb_send_command(int ordinal, char *buff, int size)
{
	int n, ret, n_sent = 0;
	char *data = NULL;
	struct command_header cmd_hdr;
	struct reply_header reply_hdr;

	cmd_hdr.ordinal = ordinal;
	cmd_hdr.options = 0;

	if ((size + sizeof(struct command_header)) <= MAX_SIZE) {
		cmd_hdr.size = size + sizeof(struct command_header);
		
		data = malloc(size);

		memcpy(data, buff, size);

		ret = usb_write(&cmd_hdr, data);

		free(data);
		data = NULL;

		ret = usb_read(&reply_hdr, &data);
		if (ret) {
			goto err;
		}

		free(data);
	} else {
		cmd_hdr.options = MESSAGE_CHUNK;

		do {
			n = (size > MAX_SIZE) ? MAX_SIZE : size;

			if (n == size) {
				cmd_hdr.options |= MESSAGE_LAST;
			}

			cmd_hdr.size = n + sizeof(struct command_header);

			data = malloc(size);

			memcpy(data, buff + n_sent, n);

			ret = usb_write(&cmd_hdr, data);
			if (ret) {
				free(data);
				goto err;
			}

			free(data);
			data = NULL;

			ret = usb_read(&reply_hdr, &data);
			if (ret) {
				goto err;
			}
			free(data);

			size -= n;
			n_sent += n;
		} while (size);
	}

err:
	return ret;
}

int usb_open(char *port)
{
	int ret = 0;
	struct termios termios;

	usb_priv.fd = open(port, O_RDWR);
	if (usb_priv.fd < 0) {
		printf("fail to open %s\n", port);
		ret = usb_priv.fd;
		goto err;
	}

	tcgetattr(usb_priv.fd, &termios);
	cfmakeraw(&termios);
	termios.c_cc[VMIN]  = 128;
	termios.c_cc[VTIME] = 250;
	tcsetattr(usb_priv.fd, TCSANOW, &termios);

err:
	return ret;
}


