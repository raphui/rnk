#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <gpiolib.h>
#include <errno.h>
#include <time.h>

#include "main.h"
#include "lr1110_update.h"

int update_data(struct update *upd)
{
	int size;
	int ret = 0;
	char *data;
	lr11xx_status_t lr11xx_status;
	struct command_header *cmd_hdr = (struct command_header *)upd->buff;

	data = upd->buff + sizeof(*cmd_hdr);
	size = cmd_hdr->size - sizeof(*cmd_hdr);

	switch (upd->type) {
	case UPD_APP_FW:
		break;
	case UPD_LORA_FW:
		printf("flashing - offset=%d size=%d remaining=%d (%d/%d)\n", upd->offset, size, upd->fw_size - upd->offset, upd->offset, upd->fw_size);
		lr11xx_status = lr11xx_bootloader_write_flash_encrypted_full(upd->lr1110_update, upd->offset, (const uint32_t *)data, size);
		if (lr11xx_status != LR11XX_STATUS_ERROR) {
			ret = -EIO;
			goto err;	
		}

		upd->offset += size;
		break;
	}

	upd->size = sizeof(struct reply_header);

err:
	return ret;
}
