#ifndef MAIN_H
#define MAIN_H

#define LR11XX_FIRMWARE_VERSION 0x04010107 //FIXME: this should come from the upd_details

#define UPD_APP_FW	(1 << 0)
#define UPD_LORA_FW	(1 << 1)

#define MESSAGE_CHUNK	(1 << 6)
#define MESSAGE_LAST	(1 << 7)

struct update {
	char *buff;
	int size;
	int chunks;
	int type;
	int offset;
	int fw_size;
	struct lr1110_update *lr1110_update;
};

struct bootloader {
	int usb_fd;
	char *com_buff;
	struct update upd;
};

struct __attribute__((__packed__)) command_header {
	unsigned short size;
	unsigned char ordinal;
	unsigned char options;
};

struct __attribute__((__packed__)) reply_header {
	unsigned short size;
	unsigned short code;
};

struct __attribute__((__packed__)) upd_details {
	unsigned int app_id;
	unsigned int size;
	unsigned char flags;
};

int update_start(struct update *upd);
int update_details(struct update *upd);
int update_data(struct update *upd);
int update_hash(struct update *upd);
int update_end(struct update *upd);

#endif /* MAIN_H */
