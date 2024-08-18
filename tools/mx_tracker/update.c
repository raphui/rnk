#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include "main.h"
#include "update.h"
#include "usb_com.h"

enum {
	UPDATE_START,
	UPDATE_DETAILS,
	UPDATE_DATA,
	UPDATE_HASH,
	UPDATE_END,
};

int update_fw(int type, char *fw, int size)
{
	int ret = 0;
	struct upd_details upd_details;

	ret = usb_send_command(UPDATE_START, NULL, 0);
	if (ret) {
		printf("UPDATE_START failed with %d\n", ret);
		goto err;
	}

	upd_details.flags = type;
	upd_details.size = size;

	ret = usb_send_command(UPDATE_DETAILS, (char *)&upd_details, sizeof(struct upd_details));
	if (ret) {
		printf("UPDATE_DETAILS failed with %d\n", ret);
		goto err;
	}


	ret = usb_send_command(UPDATE_DATA, fw, size);
	if (ret) {
		printf("UPDATE_DATA failed with %d\n", ret);
		goto err;
	}

	ret = usb_send_command(UPDATE_HASH, NULL, 0);
	if (ret) {
		printf("UPDATE_HASH failed with %d\n", ret);
		goto err;
	}

	ret = usb_send_command(UPDATE_END, NULL, 0);
	if (ret) {
		printf("UPDATE_END failed with %d\n", ret);
		goto err;
	}
err:
	return ret;
}
