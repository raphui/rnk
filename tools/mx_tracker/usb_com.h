#ifndef USB_COM_H
#define USB_COM_H

#define MESSAGE_CHUNK	(1 << 6)
#define MESSAGE_LAST	(1 << 7)

struct __attribute__((__packed__)) command_header {
	unsigned short size;
	unsigned char ordinal;
	unsigned char options;
};

struct __attribute__((__packed__)) reply_header {
	unsigned short size;
	unsigned short code;
};

int usb_open(char *port);
int usb_send_command(int ordinal, char *buff, int size);

#endif /* USB_COM_H */
