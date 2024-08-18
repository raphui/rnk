#ifndef UPDATE_H
#define UPDATE_H

#define UPD_APP_FW	(1 << 0)
#define UPD_LORA_FW	(1 << 1)

struct __attribute__((__packed__)) upd_details {
	unsigned int app_id;
	unsigned int size;
	unsigned char flags;
};

int update_fw(int type, char *fw, int size);

#endif /* UPDATE_H */
