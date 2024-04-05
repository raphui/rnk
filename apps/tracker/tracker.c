#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#define SPI_DEVICE	"/dev/spi1"

static int fd_spi;


int main(void)
{
	int ret;
	unsigned char buff[16];

	printf("Starting tracker app\n");

	fd_spi = open(SPI_DEVICE, O_RDWR);
	if (fd_spi < 0) {
		printf("failed to open: %s\n", SPI_DEVICE);
		goto err;
	}

	while (1) {
		time_usleep(1000000);

		ret = read(fd_spi, buff, sizeof(buff));
		if (ret < 0) {
			printf("failed to read tracker: %d\n", ret);
			goto err_close;
		}

		time_usleep(1000000);

		ret = write(fd_spi, buff, sizeof(buff));
		if (ret < 0) {
			printf("failed to write tracker: %d\n", ret);
			goto err_close;
		}

	}

err_close:
	close(fd_spi);
err:
	return 0;
}
