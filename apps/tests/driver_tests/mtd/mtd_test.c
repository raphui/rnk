#include <stdio.h>
#include <thread.h>
#include <board.h>
#include <mtd.h>
#include <unistd.h>

#define MTD_DEVICE	"/dev/mtd"

static int mtd_buff[] = {0xDEADBEEF,
			0xBABEFACE,
			0x12345678,
			0xCAFEBABE,
			0xDEADBABE,
			0xDEADC0DE};


void thread_a(void)
{
	int i;
	int fd;
	int ret;
	unsigned int val;

	while (1) {
		printk("Starting mtd tests\n");

		fd = open(MTD_DEVICE, O_RDWR);
		if (fd < 0) {
			error_printk("failed to open: %s\n", SPI_DEVICE);
			break;
		}

		ret = write(fd, mtd_buff, sizeof(mtd_buff));
		if (ret != 16) {
			error_printk("failed to write mtd buff\n");
			break;
		}

		lseek(fd, 0, SEEK_SET);

		for (i = 0; i < sizeof(mtd_buff); i++) {
			ret = read(fd, &val, sizeof(unsigned int));
			if (ret != sizeof(unsigned int)) {
				error_printk("failed to read mtd buff\n");
				break;
			}

			if (val != mtd_buff[i])
				error_printk("invalid read value: 0x%x != 0x%x\n", val, mtd_buff[i]);
		}

	}
}

int main(void)
{
	printk("Starting mtd driver tests\n");

	add_thread(&thread_a, DEFAULT_PRIORITY);

	return 0;

}
