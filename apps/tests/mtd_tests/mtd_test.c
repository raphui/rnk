#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

#define MTD_DEVICE	"/dev/mtd1"

static int mtd_buff[] = {0xDEADBEEF,
			0xBABEFACE,
			0x12345678,
			0xCAFEBABE,
			0xDEADBABE,
			0xDEADC0DE};

static int mtd_buff2[] = {0x00000000,
			0x00000000,
			0x00000000,
			0x00000000,
			0x00000000,
			0x00000000};

static int mtd_buff3[] = {0xDEADBEEF,
			0xBABEFACE,
			0x12345678,
			0xDEADBABE};

static pthread_t thr_a;

static int mtd_test(int fd, int offset, char *buff, int size)
{
	int i;
	int ret;
	unsigned int val;
	unsigned int *p = (unsigned int *)buff;

	lseek(fd, offset, SEEK_SET);

	ret = write(fd, buff, size);
	if (ret != size) {
		printf("failed to write mtd buff\n");
		goto err;
	}

	lseek(fd, offset, SEEK_SET);

	for (i = 0; i < size / sizeof(unsigned int); i++) {
		ret = read(fd, &val, sizeof(unsigned int));
		if (ret != sizeof(unsigned int)) {
			printf("failed to read mtd buff\n");
			break;
		}

		if (val != p[i])
			printf("invalid read value: 0x%x != 0x%x\n", val, p[i]);
	}

err:
	return ret;
}

void thread_a(void *arg)
{
	int fd;
	int ret;

	printf("Starting mtd tests\n");

	fd = open(MTD_DEVICE, O_RDWR);
	if (fd < 0) {
		printf("failed to open: %s\n", MTD_DEVICE);
		goto err;
	}

	printf("Testing aligned write\n");
	printf("---------------------\n");

	ret = mtd_test(fd, 0x30000, (char *)mtd_buff, sizeof(mtd_buff));
	if (ret < 0)
		printf("fail, reason: %d\n", ret);

	printf("---------------------\n");

	printf("Testing aligned write with erase\n");
	printf("---------------------\n");

	ret = mtd_test(fd, 0x30000, (char *)mtd_buff2, sizeof(mtd_buff));
	if (ret < 0)
		printf("fail, reason: %d\n", ret);

	printf("---------------------\n");

	printf("Testing non-aligned write\n");
	printf("---------------------\n");

	ret = mtd_test(fd, 0x30000, (char *)mtd_buff2, 13);
	if (ret < 0)
		printf("fail, reason: %d\n", ret);

	printf("---------------------\n");

	printf("Testing cross page write\n");
	printf("---------------------\n");

	ret = mtd_test(fd, 0x37FF8, (char *)mtd_buff3, sizeof(mtd_buff3));
	if (ret < 0)
		printf("fail, reason: %d\n", ret);

	printf("---------------------\n");

	close(fd);
err:
	return;
}

int main(void)
{
	printf("Starting mtd driver tests\n");

	pthread_create(&thr_a, &thread_a, NULL, DEFAULT_PRIORITY);

	return 0;

}
