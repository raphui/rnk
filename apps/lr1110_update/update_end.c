#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <gpiolib.h>
#include <errno.h>
#include <time.h>
#include "main.h"

int update_end(struct update *upd)
{
	int ret = 0;

	upd->size = sizeof(struct reply_header);

	return ret;
}
