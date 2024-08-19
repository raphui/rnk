#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <gpiolib.h>
#include <errno.h>
#include <time.h>

#include "main.h"
#include "lr1110_update.h"

int update_hash(struct update *upd)
{
	int ret = 0;

	switch (upd->type) {
	case UPD_APP_FW:
		break;
	case UPD_LORA_FW:
		ret = lr11xx_update_finalize(upd->lr1110_update, LR11XX_FIRMWARE_VERSION);
		break;
	}

	upd->size = sizeof(struct reply_header);

	return ret;
}
