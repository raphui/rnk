#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <gpiolib.h>
#include <errno.h>
#include <time.h>

#include "main.h"
#include "lr1110_update.h"

int update_details(struct update *upd)
{
	int ret = 0;
	struct upd_details *upd_details = (struct upd_details *)(upd->buff + sizeof(struct command_header));

	upd->type = upd_details->flags;
	upd->fw_size = upd_details->size;
	upd->offset = 0;

	switch (upd->type) {
	case UPD_APP_FW:
		printf("update app fw\n");
		break;
	case UPD_LORA_FW:
		printf("update lora fw\n");
		ret = lr11xx_update_init(upd->lr1110_update);
		break;
	}

	upd->size = sizeof(struct reply_header);

	return ret;
}
