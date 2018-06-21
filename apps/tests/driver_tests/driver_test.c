#include <stdio.h>
#include <pthread.h>
#include <board.h>
#include <unistd.h>

#define SPI_DEVICE	"/dev/spi1"
#define MTD_DEVICE	"/dev/mtd1"

#define MTD_TEST_DATA_SIZE	0x200

#ifdef CONFIG_STM32F429

#define SDRAM_ADDRESS		(unsigned int *)0xC0000000
#define SDRAM_TEST_PATTERN	0xDEADBEEF

#endif /* CONFIG_STM32F429 */

static char spi_buff[16] = {0x1, 0x2, 0x3, 0x4,
			0x5, 0x6, 0x7, 0x8,
			0x9, 0xA, 0xB, 0xC,
			0xD, 0xE, 0xF, 0x10};


void thread_a(void *arg)
{
	int i;
	int fd;
	int ret;

	while (1) {
#ifdef CONFIG_STM32F429
		printf("Starting SDRAM tests\n");

		printf("Writing: 0x%x at 0x%x\n", SDRAM_TEST_PATTERN, SDRAM_ADDRESS);

		*(SDRAM_ADDRESS) = SDRAM_TEST_PATTERN;

		printf("Reading content at 0x%x\n", SDRAM_ADDRESS);

		ret = *(SDRAM_ADDRESS);

		printf("SDRAM test: ");

		if (ret == SDRAM_TEST_PATTERN)
			printf("OK\n");
		else
			printf("NOK\n");
#endif /* CONFIG_STM32F429 */

		printf("Starting spi tests\n");
		printf("Validate using your logical analyser\n");

		fd = open(SPI_DEVICE, O_RDWR);
		if (fd < 0) {
			printf("failed to open: %s\n", SPI_DEVICE);
			break;
		}

		ret = write(fd, spi_buff, 16);
		if (ret != 16) {
			printf("failed to write spi buff\n");
			break;
		}

		ret = read(fd, spi_buff, 16);
		if (ret != 16) {
			printf("failed to read spi buff\n");
			break;
		}

		for (i = 0; i < 16; i++)
			printf("spi_read[%d]: 0x%x\n", spi_buff[i]);
	}
}

int main(void)
{
	int size;
	int ret;
	int fd;
	unsigned char *p1, *p2;
	int mtd_ok = 1;

	printf("Starting driver tests\n");

	fd = open(MTD_DEVICE, O_RDWR);
	if (fd < 0) {
		printf("failed to open: %s\n", MTD_DEVICE);
		goto skip_mtd;
	}

	ret = lseek(fd, 0x20000, SEEK_SET);
	if (ret < 0) {
		printf("failed to lseek\n");
		goto skip_mtd;
	}

	ret = write(fd, (unsigned char *)0x08000000, MTD_TEST_DATA_SIZE);
	if (ret < 0) {
		printf("failed to write\n");
	}

	size = MTD_TEST_DATA_SIZE;

	p1 = (unsigned char *)0x08000000;
	p2 = (unsigned char *)0x08020000;

	while (size--) {
		if (*p1++ != *p2++) {
			printf("failed to check flash write data\n");
			mtd_ok = 0;
			break;
		}
	}

	if (mtd_ok)
		printf("mtd tests: OK\n");

skip_mtd:
	pthread_create(&thread_a, NULL, 2);

	return 0;

}
