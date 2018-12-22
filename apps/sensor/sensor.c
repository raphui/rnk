#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#define SPI_DEVICE	"/dev/spi1"

static int fd_spi;

struct sensor_data {
	unsigned int humidity;
	unsigned int temperature;
	unsigned int pressure;
};


int sensor_read(struct sensor_data *data)
{
	return read(fd_spi, data, sizeof(*data));
}

int main(void)
{
	int ret;
	struct sensor_data data;

	printf("Starting sensor app\n");

	fd_spi = open(SPI_DEVICE, O_RDWR);
	if (fd_spi < 0) {
		printf("failed to open: %s\n", SPI_DEVICE);
		goto err;
	}

	while (1) {
		ret = sensor_read(&data);
		if (ret < 0) {
			printf("failed to read sensor data: %d\n", ret);
			goto err_close;
		}

		printf("Sensor datas:\n");
		printf("\tPressure: %d\n", data.pressure);
		printf("\tTemperature: %d\n", data.temperature);
		printf("\tHumidity: %d\n", data.humidity);
		printf("\033[4A");
		printf("\033[K");
		printf("\033[K");
		printf("\033[K");
	}

err_close:
	close(fd_spi);
err:
	return 0;
}
