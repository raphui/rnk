#include <errno.h>
#include <fdtparse.h>
#include <init.h>
#include <string.h>
#include <drv/irq.h>
#include <drv/pio.h>
#include <drv/spi.h>
#include <kernel/ksem.h>
#include <kernel/printk.h>
#include <mm/mm.h>

#define BME280_REG_TEMP_PRES_CAL	0x88
#define BME280_REG_HUM_CAL		0xE1
#define BME280_REG_ID			0xD0
#define BME280_REG_RESET		0xE0
#define BME280_REG_CTRL_HUM		0xF2
#define BME280_REG_STATUS		0xF3
#define BME280_REG_CTRL_MEAS		0xF4
#define BME280_REG_CONFIG		0xF5
#define BME280_REG_DATA			0xF7

#define BME280_REG_TEMP_PRES_CAL_SIZE	26
#define BME280_REG_HUM_CAL_SIZE		7
#define BME280_REG_DATA_SIZE		8

#define BME280_MAX_BURST		30

struct bme280_calib {
	unsigned short dig_T1;
	short dig_T2;
	short dig_T3;
	unsigned short dig_P1;
	short dig_P2;
	short dig_P3;
	short dig_P4;
	short dig_P5;
	short dig_P6;
	short dig_P7;
	short dig_P8;
	short dig_P9;
	unsigned short dig_H1;
	short dig_H2;
	short dig_H3;
	short dig_H4;
	short dig_H5;
	short dig_H6;
	int t_fine;

};

struct bme280_uncomp_data {
	unsigned int pressure;
	unsigned int temperature;
	unsigned int humidity;
};

struct bme280_comp_data {
	unsigned int pressure;
	unsigned int temperature;
	unsigned int humidity;
};

struct bme280_priv {
	unsigned int param;
	struct bme280_calib calib;
	struct bme280_uncomp_data uncomp;
	struct bme280_comp_data comp;
};

static int bme280_compensate_temperature(struct bme280_priv *priv)
{
	int var1;
	int var2;
	int temperature;
	int temperature_min = -4000;
	int temperature_max = 8500;

	var1 = (int)((priv->uncomp.temperature / 8) - ((int)priv->calib.dig_T1 * 2));
	var1 = (var1 * ((int)priv->calib.dig_T2)) / 2048;
	var2 = (int)((priv->uncomp.temperature / 16) - ((int)priv->calib.dig_T1));
	var2 = (((var2 * var2) / 4096) * ((int)priv->calib.dig_T3)) / 16384;
	priv->calib.t_fine = var1 + var2;
	temperature = (priv->calib.t_fine * 5 + 128) / 256;

	if (temperature < temperature_min)
		temperature = temperature_min;
	else if (temperature > temperature_max)
		temperature = temperature_max;

	return temperature;
}

static int bme280_compensate_pressure(struct bme280_priv *priv)
{
	int var1;
	int var2;
	int var3;
	int var4;
	unsigned int var5;
	unsigned int pressure;
	unsigned int pressure_min = 30000;
	unsigned int pressure_max = 110000;

	var1 = (((int)priv->calib.t_fine) / 2) - (int)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int)priv->calib.dig_P6);
	var2 = var2 + ((var1 * ((int)priv->calib.dig_P5)) * 2);
	var2 = (var2 / 4) + (((int)priv->calib.dig_P4) * 65536);
	var3 = (priv->calib.dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int)priv->calib.dig_P2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int)priv->calib.dig_P1)) / 32768;
	 /* avoid exception caused by division by zero */
	if (var1) {
		var5 = (unsigned int)((unsigned int)1048576) - priv->uncomp.pressure;
		pressure = ((unsigned int)(var5 - (unsigned int)(var2 / 4096))) * 3125;
		if (pressure < 0x80000000)
			pressure = (pressure << 1) / ((unsigned int)var1);
		else
			pressure = (pressure / (unsigned int)var1) * 2;

		var1 = (((int)priv->calib.dig_P9) * ((int)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
		var2 = (((int)(pressure / 4)) * ((int)priv->calib.dig_P8)) / 8192;
		pressure = (unsigned int)((int)pressure + ((var1 + var2 + priv->calib.dig_P7) / 16));

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else {
		pressure = pressure_min;
	}

	return pressure;
}

static int bme280_compensate_humidity(struct bme280_priv *priv)
{
	int var1;
	int var2;
	int var3;
	int var4;
	int var5;
	unsigned int humidity;
	unsigned int humidity_max = 102400;

	var1 = priv->calib.t_fine - ((int)76800);
	var2 = (int)(priv->uncomp.humidity * 16384);
	var3 = (int)(((int)priv->calib.dig_H4) * 1048576);
	var4 = ((int)priv->calib.dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int)16384) / 32768;
	var2 = (var1 * ((int)priv->calib.dig_H6)) / 1024;
	var3 = (var1 * ((int)priv->calib.dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (int)32768)) / 1024) + (int)2097152;
	var2 = ((var4 * ((int)priv->calib.dig_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int)priv->calib.dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humidity = (unsigned int)(var5 / 4096);

	if (humidity > humidity_max)
		humidity = humidity_max;

	return humidity;
}

static int bme280_read_reg(struct device *dev, int reg, unsigned char *buffer, unsigned int size)
{
	int n;
	int ret = 0;
	unsigned char data[BME280_MAX_BURST];
	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	reg |= 0x80;

	n = (size < BME280_MAX_BURST) ? size : BME280_MAX_BURST;

	data[0] = reg;

	memcpy(&data[1], buffer, n);

	spi_transfer(spi, data, n);

	return ret;
}

static int bme280_write_reg(struct device *dev, unsigned char reg, unsigned char *buffer, unsigned int size)
{
	int n;
	int ret = 0;
	unsigned char data[BME280_MAX_BURST];
	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	reg &= ~0x80;

	n = (size < BME280_MAX_BURST) ? size : BME280_MAX_BURST;

	data[0] = reg;

	memcpy(&data[1], buffer, n);

	spi_transfer(spi, data, n);

	return ret;
}

static int bme280_read_cal(struct device *dev)
{
	int ret = 0;
	unsigned char calib_data[BME280_REG_TEMP_PRES_CAL_SIZE];
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct bme280_priv *priv = (struct bme280_priv *)spi->priv;

	ret = bme280_read_reg(dev, BME280_REG_TEMP_PRES_CAL, calib_data, BME280_REG_TEMP_PRES_CAL_SIZE);
	if (ret < 0)
		goto err;

	priv->calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];
	priv->calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];
	priv->calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];
	priv->calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];
	priv->calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];
	priv->calib.dig_P3 = (calib_data[11] << 8) | calib_data[10];
	priv->calib.dig_P4 = (calib_data[13] << 8) | calib_data[12];
	priv->calib.dig_P5 = (calib_data[15] << 8) | calib_data[14];
	priv->calib.dig_P6 = (calib_data[17] << 8) | calib_data[16];
	priv->calib.dig_P7 = (calib_data[19] << 8) | calib_data[18];
	priv->calib.dig_P8 = (calib_data[21] << 8) | calib_data[20];
	priv->calib.dig_P9 = (calib_data[23] << 8) | calib_data[22];
	priv->calib.dig_H1 = calib_data[25];

	ret = bme280_read_reg(dev, BME280_REG_HUM_CAL, calib_data, BME280_REG_HUM_CAL_SIZE);
	if (ret < 0)
		goto err;

	priv->calib.dig_H2 = (calib_data[1] << 8) | calib_data[0];
	priv->calib.dig_H3 = calib_data[2];
	priv->calib.dig_H4 = (calib_data[3] << 4) | (calib_data[4] & 0x0F);
	priv->calib.dig_H5 = (calib_data[5] << 4) | (calib_data[4] >> 4);
	priv->calib.dig_H3 = calib_data[6];

err:
	return ret;
}

static int bme280_read(struct device *dev, unsigned char *buffer, unsigned int size)
{
	int ret = 0;
	unsigned int *data = (unsigned int *)buffer;
	unsigned char sensor_data[BME280_REG_DATA_SIZE];
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct bme280_priv *priv = (struct bme280_priv *)spi->priv;

	if (!buffer)
		return -EINVAL;

	if (size < (3 * sizeof(unsigned int))) /* ensure enough space to return pressure/temp/humidity */
		return -EINVAL;

	ret = bme280_read_reg(dev, BME280_REG_DATA, sensor_data, BME280_REG_DATA_SIZE);
	if (ret < 0)
		goto err;

	priv->uncomp.pressure = (sensor_data[0] << 12) | (sensor_data[1] << 4) | (sensor_data[2] >> 4);
	priv->uncomp.temperature = (sensor_data[3] << 12) | (sensor_data[4] << 4) | (sensor_data[5] >> 4);
	priv->uncomp.humidity = (sensor_data[6] << 8) | sensor_data[7];

	priv->comp.pressure = bme280_compensate_pressure(priv);
	priv->comp.temperature = bme280_compensate_temperature(priv);
	priv->comp.humidity = bme280_compensate_humidity(priv);

	data[0] = priv->comp.humidity;
	data[1] = priv->comp.temperature;
	data[2] = priv->comp.pressure;

err:
	return ret;
}

static int bme280_write(struct device *dev, unsigned char *buffer, unsigned int size)
{
	if (!buffer)
		return -EINVAL;

	return bme280_write_reg(dev, buffer[0], &buffer[1], size - 1);
}

static int bme280_of_init(struct spi_device *spi, int offset)
{
	int ret = 0;

	ret = pio_of_configure(offset);
	if (ret < 0) {
		error_printk("failed to init gpio skipping error\n");
		ret = 0;
	}

	ret = pio_of_configure_name(offset, "cs-gpio");
	if (ret < 0)
		goto out;

	ret = pio_of_get(offset, "cs-gpio", &spi->cs_port, &spi->cs_pin);
	if (ret < 0) {
		error_printk("failed to init cs gpio\n");
		goto out;
	}

out:
	return ret;
}

struct device_operations dev_ops = {
	.read = bme280_read,
	.write = bme280_write,
};

int bme280_init(struct device *dev)
{
	int offset;
	int ret = 0;
	struct bme280_priv *priv = NULL;
	struct spi_device *spi = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, dev->of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto err;
	}

	priv = (struct bme280_priv *)kmalloc(sizeof(*priv));
	if (!priv) {
		error_printk("failed to allocate bme280 priv struct\n");
		ret = -ENOMEM;
		goto err;
	}

	spi = spi_new_device_with_master(offset);
	if (!spi) {
		error_printk("failed to retrieve new spi device\n");
		ret = -ENOMEM;
		goto err_free_priv;
	}

	memcpy(&spi->dev, dev, sizeof(*dev));

	spi->priv = priv;
	spi->use_cs = 1;

	ret = bme280_of_init(spi, offset);
	if (ret < 0) {
		error_printk("failed to init fdt data\n");
		goto err_free_spi;
	}

	ret = spi_register_device(spi, &dev_ops);
	if (ret < 0) {
		error_printk("failed to register spi device\n");
		goto err_free_spi;
	}

	ret = bme280_read_cal(dev);
	if (ret < 0) {
		error_printk("failed to read cal from sensor\n");
		goto err_free_spi;
	}

	return 0;

err_free_priv:
	kfree(priv);
err_free_spi:
	kfree(spi);
err:
	return ret;
}

struct device bme280_driver = {
	.of_compat = "bosch,bme280",
	.probe = bme280_init,
};

static int bme280_register(void)
{
	int ret = 0;

	ret = device_of_register(&bme280_driver);
	if (ret < 0)
		error_printk("failed to register bme280 driver\n");
	return ret;
}
coredevice_initcall(bme280_register);
