#include <stdio.h>
#include <string.h>
#include <errno.h>

#define PATH	"/home/raphio/test_semihosting"

int main(void)
{
	int ret = 0;
	char buff[1024];
	FILE *f;

	f = fopen(PATH, "rb");
	if (!f) {
		error_printk("failed to open: %s\n", PATH);
		ret = -ENOENT;
		goto out;
	}

	memset(buff, 0, 1024);

	ret = fread(buff, 1024, sizeof(char), f);
	if (ret < 0) {
		error_printk("failed to read from: %s\n", PATH);
		ret = -EIO;
		goto out;
	}

	printk("content from [%s]: %s\n", PATH, buff);

out:
	return ret;
}
