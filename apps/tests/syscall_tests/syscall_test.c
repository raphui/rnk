#include <stdio.h>
#include <unistd.h>
#include <time.h>

int main(void)
{
	int fd;
	int ret;

	printf("Starting syscall tests\n");

	printf("Testing privilege operation return code: ");
	fd = open("/dev/dummy", O_RDWR);
	if (fd < 0)
		printf("SUCCESS (%d)\n", fd);
	else
		printf("FAIL\n");

	printf("Testing privilege elevation return code: ");
	ret = write(fd, NULL, 0);
	if (ret < 0)
		printf("SUCCESS (%d)\n", ret);
	else
		printf("FAIL\n");
}
