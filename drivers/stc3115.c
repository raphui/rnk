#include <kernel/printk.h>
#include <drv/i2c.h>
#include <mm/mm.h>
#include <errno.h>
#include <string.h>
#include <utils.h>
#include <init.h>
#include <fdtparse.h>
#include <drv/pio.h>
#include <ioctl.h>
#include <drv/stc3115.h>

/* OCVTAB (Open Circuit Voltage curve, ie when the battery is relaxed (no charge or discharge) */
#define OCV_OFFSET_TAB	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

struct stc3115_config {
	int vmode;       		/* 1=Voltage mode, 0=mixed mode	*/
	int alm_soc;     		/* SOC alarm level in %	*/
	int alm_vbat;    		/* Vbat alarm level in mV */
	int cc_cnf;      		/* nominal battery CC_cnf */
	int vm_cnf;      		/* nominal battery VM cnf */
	int cnom;        		/* nominal battery capacity in mAh */
	int rsense;      		/* sense resistor in mOhms */
	int relaxcurrent;		/* relaxation current(< C/20) in mA */
	unsigned char ocvoffset[16];    /* OCV curve adjustment in 0.55mV */
};

struct stc3115_battery_data {
	int statusword;		/* STC3115 status registers */
	int hrsoc;		/* battery relative SOC (%) in 1/512% */
	int soc;            	/* battery relative SOC (%) in 0.1% */
	int voltage;        	/* battery voltage in mV */
	int current;        	/* battery current in mA */
	int temperature;    	/* battery temperature in 0.1°C */
	int convcounter;	/* STC3115 convertion counter in 0.5s */
	int ocv;		/* battery relax voltage in mV	*/
	int presence;		/* battery presence */
	int chargevalue;    	/* battery remaining capacity in mAh */
	int remtime;        	/* battery remaining operating time during discharge */
};

union internal_ram {
	unsigned char db[STC3115_RAM_SIZE];  /* last byte holds the CRC */
	struct {
		short testword;     /* 0-1 RAM test word */
		short hrsoc;        /* 2-3 SOC backup in (1/512%) */
		short cc_cnf;       /* 4-5 current cc_cnf */
		short vm_cnf;       /* 6-7 current vm_cnf */
		char soc;           /* 8  SOC (in %) */
		char state;	/* 9  STC3115 working state */
		char unused1;   /* 10  -Bytes upto ..STC3115_RAM_SIZE-2 are free */
		char unused2;   /* 11  -Bytes upto ..STC3115_RAM_SIZE-2 are free */
		char unused3;   /* 12  -Bytes upto ..STC3115_RAM_SIZE-2 are free */
		char unused4;   /* 13  -Bytes upto ..STC3115_RAM_SIZE-2 are free */
		char unused5;   /* 14  -Bytes upto ..STC3115_RAM_SIZE-2 are free */
		char CRC;       /* 15  last byte STC3115_RAM_SIZE-1 is the CRC */
	} reg;
};

struct stc3115_priv {
	int mode;
	int rsense;
	int battery_capacity;
	int battery_impedance;
	int alarm_enable;
	int alarm_soc;
	int alarm_mv;
	int address;
	struct i2c_msg curr_msg;
	struct stc3115_config config;
	struct stc3115_battery_data battery_data;
	union internal_ram ram_data;
};

static unsigned char stc3115_crc8(unsigned char *buff, int size)
{
	int crc = 0;

	for (int i = 0; i < size; i++)
	{
		crc ^= buff[i];
		for (int j = 0; j < 8; j++) 
		{
			crc <<= 1;
			if (crc & 0x100)
				crc ^= 7;
		}
	}
	return (crc & 0xFF);
}

static int stc3115_reg_read(struct device *dev, int reg, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;

	priv->curr_msg.reg = reg;
	priv->curr_msg.buff = buff;
	priv->curr_msg.size = size;

	i2c->master->i2c_ops->ioctl(i2c, IOCTL_SET_ADDRESS, (char *)priv->address);

	ret = i2c_transfer(i2c, &priv->curr_msg, I2C_TRANSFER_READ);

	return ret;
}

static int stc3115_reg_write(struct device *dev, int reg, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;

	priv->curr_msg.reg = reg;
	priv->curr_msg.buff = buff;
	priv->curr_msg.size = size;

	i2c->master->i2c_ops->ioctl(i2c, IOCTL_SET_ADDRESS, (char *)priv->address);

	ret = i2c_transfer(i2c, &priv->curr_msg, I2C_TRANSFER_WRITE);

	return ret;
}

static int stc3115_reg_write_word(struct device *dev, int reg, uint16_t value)
{
	int ret = 0;
	uint8_t data[2];
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;

	data[0]= value & 0xFF;
	data[1]= (value >> 8) & 0xFF;

	priv->curr_msg.reg = reg;
	priv->curr_msg.buff = data;
	priv->curr_msg.size = sizeof(data);

	i2c->master->i2c_ops->ioctl(i2c, IOCTL_SET_ADDRESS, (char *)priv->address);

	ret = i2c_transfer(i2c, &priv->curr_msg, I2C_TRANSFER_WRITE);

	return ret;
}

static int stc3115_reg_read_word(struct device *dev, int reg)
{
	int ret = 0;
	uint8_t data[2];
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;

	priv->curr_msg.reg = reg;
	priv->curr_msg.buff = data;
	priv->curr_msg.size = sizeof(data);

	i2c->master->i2c_ops->ioctl(i2c, IOCTL_SET_ADDRESS, (char *)priv->address);

	ret = i2c_transfer(i2c, &priv->curr_msg, I2C_TRANSFER_READ);

	ret = data[1];
	ret = (ret << 8) | data[0];

	return ret;
}

static int stc3115_enable_powersaving(struct device *dev)
{
	int ret = 0;
	unsigned char data;

	stc3115_reg_read(dev, STC3115_REG_CTRL, &data, sizeof(data));

	data |= STC3115_VMODE;

	stc3115_reg_write(dev, STC3115_REG_CTRL, &data, sizeof(data));

	return ret;
}

static int stc3115_disable_powersaving(struct device *dev)
{
	int ret = 0;
	unsigned char data;

	stc3115_reg_read(dev, STC3115_REG_CTRL, &data, sizeof(data));

	data &= ~(STC3115_VMODE);

	stc3115_reg_write(dev, STC3115_REG_CTRL, &data, sizeof(data));

	return ret;
}

static void stc3115_init_ram_data(struct stc3115_config *config, union internal_ram *ram)
{
	memset(ram->db, 0, STC3115_RAM_SIZE);

	ram->reg.testword = RAM_TESTWORD;
	ram->reg.cc_cnf = config->cc_cnf;
	ram->reg.vm_cnf = config->vm_cnf;

	ram->db[STC3115_RAM_SIZE - 1] = stc3115_crc8(ram->db, STC3115_RAM_SIZE - 1);
}

static int stc3115_get_status(struct device *dev)
{
	uint16_t status;

	status = stc3115_reg_read_word(dev, STC3115_REG_MODE);
	status &= 0x7FFF;

	return status;
}

static int stc3115_conv(short value, unsigned short factor)
{
	int v;

	v= ((long)value * factor) >> 11;
	v= (v + 1) / 2;

	return v;
}

static int stc3115_set_param_and_run(struct device *dev)
{
	int ret = 0;
	uint8_t data;
	uint16_t tmp;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;
	struct stc3115_config *config = &priv->config;

	/* set GG_RUN=0 before changing algo parameters */
	data = STC3115_REGMODE_DEFAULT_STANDBY;

	stc3115_reg_write(dev, STC3115_REG_MODE, &data, sizeof(data));

	/* init OCV curve */
	stc3115_reg_write(dev, STC3115_REG_OCVTAB, (unsigned char *)&config->ocvoffset, STC3115_OCVTAB_SIZE);

	/* set alm level if different from default */
	if (config->alm_soc != 0) {
		data = config->alm_soc * 2;
		stc3115_reg_write(dev, STC3115_REG_ALARM_SOC,&data, sizeof(data));
	}

	if (config->alm_vbat != 0) {
		data = ((long)(config->alm_vbat << 9) / VOLTAGE_FACTOR); /* LSB=8*2.2mV */
		stc3115_reg_write(dev, STC3115_REG_ALARM_VOLTAGE, &data, sizeof(data));
	}

	/* relaxation timer */
	if (config->rsense != 0) {
		data = ((long)(config->relaxcurrent << 9) / (CURRENT_FACTOR / config->rsense));   /* LSB=8*5.88uV */
		stc3115_reg_write(dev, STC3115_REG_CURRENT_THRES, &data, sizeof(data));
	}

	/* set parameters VM_CNF and CC_CNF */
	if (config->cc_cnf != 0) {
		stc3115_reg_write_word(dev, STC3115_REG_CC_CNF, config->cc_cnf);
	} else {
		/* force writing a default value at startup */
		tmp = 395;
		stc3115_reg_write_word(dev, STC3115_REG_CC_CNF, tmp);
	}

	if (config->vm_cnf != 0)
		stc3115_reg_write_word(dev, STC3115_REG_VM_CNF, config->vm_cnf);
	else {

		tmp = 321;
		stc3115_reg_write_word(dev, STC3115_REG_VM_CNF, tmp);

	}

	/*   clear PORDET, BATFAIL, free ALM pin, reset conv counter */
	data = 0x03;
	stc3115_reg_write(dev, STC3115_REG_CTRL, &data, sizeof(data));

	/* set GG_RUN=1, set mode, set alm enable */
	data = STC3115_GG_RUN | (STC3115_VMODE * config->vmode) | (STC3115_ALM_ENA * priv->alarm_enable);
	stc3115_reg_write(dev, STC3115_REG_MODE, &data, sizeof(data));

	return ret;
}

static int stc3115_startup(struct device *dev)
{
	int ret = 0;
	uint16_t status;
	uint16_t hrsoc;
	int16_t ocv;
	int ocv_min;
	int ocvoffset[16] = OCV_OFFSET_TAB;

	status = stc3115_get_status(dev);
	if (status < 0) {
		error_printk("incorrect status for STC3115\n");
		ret = -EIO;
		goto err;
	}

	/* read OCV */
	stc3115_reg_read(dev, STC3115_REG_OCV, (unsigned char *)&ocv, sizeof(ocv));

	/* Check OCV integrity after reset: it must be above or equal to OCV min = 3300 (mV) + OCVOffset[0] (0.55mV)  */
	ocv_min = 6000 + ocvoffset[0];
	if (ocv <= ocv_min) {
		hrsoc = 0;

		stc3115_reg_write_word(dev, STC3115_REG_SOC, hrsoc);
		stc3115_set_param_and_run(dev);  /* set STC3115 parameters and run it  */
	} else {
		stc3115_set_param_and_run(dev);  /* set STC3115 parameters and run it  */

		/* rewrite ocv to start SOC with updated OCV curve */
		stc3115_reg_write_word(dev, STC3115_REG_OCV, ocv);
	}

err:
	return ret;
}

static int stc3115_reset(struct device *dev)
{
	int ret = 0;
	uint8_t data;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;
	union internal_ram *ram = &priv->ram_data;

	/* reset RAM */
	ram->reg.testword = 0;
	ram->reg.state = STC3115_UNINIT;
	ret = stc3115_reg_write(dev, STC3115_REG_RAM, ram->db, STC3115_RAM_SIZE);
	if (ret < 0)
		goto err;

	/* reset STC3115*/
	data = STC3115_PORDET;
	ret = stc3115_reg_write(dev, STC3115_REG_CTRL, &data, sizeof(data));

err:
	return ret;
}

int stc3115_restore_from_ram(struct device *dev)
{
	int status;
	int ret = 0;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;
	union internal_ram *ram = &priv->ram_data;

	status = stc3115_get_status(dev);
	if (status < 0) {
		error_printk("incorrect status for STC3115\n");
		ret = -EIO;
		goto err;
	}

	stc3115_set_param_and_run(dev);

	if ((ram->reg.soc != 0) && (ram->reg.hrsoc != 0)) {
		ret = stc3115_reg_write(dev, STC3115_REG_SOC, (unsigned char *)&ram->reg.hrsoc, sizeof(ram->reg.hrsoc));
	}

err:
	return ret;
}

static int stc3115_read_battery_data(struct device *dev)
{
	int value;
	int ret = 0;
	unsigned char data[16];
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;
	struct stc3115_battery_data *battery = &priv->battery_data;

	/* read STC3115 registers 0 to 14 */
	ret = stc3115_reg_read(dev, 0, data, 15);
	if (ret < 0) {
		error_printk("fail to read all 15 registers\n");
		goto err;
	}

	/* fill the battery status data */
	/* SOC */
	value = data[3];
	value = (value << 8) + data[2];
	/* result in 1/512% */
	battery->hrsoc = value;
	/* result in 0.1% */
	battery->soc = (value * 10 + 256) / 512;

	/* conversion counter */
	value = data[5];
	value = (value << 8) + data[4];
	battery->convcounter = value;

	/* current */
	value = data[7];
	value = (value << 8) + data[6];
	value &= 0x3fff;   /* mask unused bits */
	if (value >= 0x2000)
		value = value - 0x4000;  /* convert to signed value */
	battery->current = stc3115_conv(value, CURRENT_FACTOR / priv->rsense);  /* result in mA */

	/* voltage */
	value = data[9];
       	value = (value << 8) + data[8];
	value &= 0x0fff; /* mask unused bits */
	if (value >= 0x0800)
		value -= 0x1000;  /* convert to signed value */
	value = stc3115_conv(value, VOLTAGE_FACTOR);  /* result in mV */
	battery->voltage = value;  /* result in mV */

	/* temperature */
	value = data[10]; 
	if (value >= 0x80)
		value -= 0x100;  /* convert to signed value */
	battery->temperature = value * 10;  /* result in 0.1°C */

	/* OCV */
	value = data[14];
	value = (value << 8) + data[13];
	value &= 0x3fff; /* mask unused bits */
	if (value >= 0x02000)
		value -= 0x4000;  /* convert to signed value */
	value = stc3115_conv(value, VOLTAGE_FACTOR);  
	value = (value + 2) / 4;  /* divide by 4 with rounding */
	battery->ocv = value;  /* result in mV */

err:
	return ret;
}

int stc3115_update_battery_data(struct device *dev)
{
	int crc, status;
	int ret = 0;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;
	struct stc3115_config *config = &priv->config;
	struct stc3115_battery_data *battery = &priv->battery_data;
	union internal_ram *ram = &priv->ram_data;

	status = stc3115_get_status(dev);
	if (status < 0) {
		error_printk("incorrect status for STC3115\n");
		ret = -EIO;
		goto err;
	}

	battery->statusword = status;

	/* check STC3115 RAM status (battery has not been changed) */
	stc3115_reg_read(dev, STC3115_REG_RAM, ram->db, STC3115_RAM_SIZE);

	crc = stc3115_crc8(ram->db, STC3115_RAM_SIZE);

	if ((ram->reg.testword != RAM_TESTWORD) || (crc != 0)) {
		/* if RAM non ok, reset it and set init state */
		stc3115_init_ram_data(config, ram);
		ram->reg.state = STC3115_INIT;
	}  

	/* check battery presence status */
	if ((battery->statusword & ((int)STC3115_BATFAIL<<8)) != 0) {
		error_printk("battery is disconnected\n");

		/* BATD pin level is over 1.61 or Vcc is below 2.7V */
		battery->presence = 0;

		/*HW and SW state machine reset*/
		stc3115_reset(dev);

		ret = -EIO;
		goto err;
	}

	/* check STC3115 running mode*/
	/* Gas gauge no more running (in Standby mode) */
	if ((battery->statusword & STC3115_GG_RUN) == 0) {
		if ((ram->reg.state == STC3115_RUNNING) || (ram->reg.state == STC3115_POWERDN)) {
			/* if RUNNING state, restore STC3115 with latest good SoC value for better accuracy */
			stc3115_restore_from_ram(dev);
		} else {
			/* if INIT state, initialize STC3115 */
			stc3115_startup(dev);
		}

		ram->reg.state = STC3115_INIT;
	}

	/* --------------------------------- Read battery data ------------------------------- */

	ret = stc3115_read_battery_data(dev);  
	if (ret < 0) {
		error_printk("failed to read battery data\n");
		goto err;
	}

	/* ------------------------------- battery data report ------------------------------- */
	/* check INIT state */
	if (ram->reg.state == STC3115_INIT) {
		/* INIT state, wait for current & temperature value available: */
		if (battery->convcounter > VCOUNT) {
			ram->reg.state = STC3115_RUNNING;
			/*Battery is connected*/
			battery->presence = 1;
		}
	}

	/* not running : data partially available*/
	if (ram->reg.state != STC3115_RUNNING) {
		battery->chargevalue = config->cnom * battery->soc / MAX_SOC;
		battery->current = 0;
		battery->temperature = 250;
		battery->remtime = -1;
	} else {
		/* STC3115 running */

		/* ---------- process SW algorithms -------- */

		/* early empty compensation */
		if (battery->voltage < APP_CUTOFF_VOLTAGE) {
			battery->soc = 0;
		} else if (battery->voltage < (APP_CUTOFF_VOLTAGE + VOLTAGE_SECURITY_RANGE)) {
			// Recommended software security: scaling down the SOC if voltage is considered too close to the cutoff voltage. (no accuracy effect) 
			battery->soc = battery->soc * (battery->voltage - APP_CUTOFF_VOLTAGE) / VOLTAGE_SECURITY_RANGE;   
		}

		/* Battery charge value calculation */
		battery->chargevalue = config->cnom * battery->soc / MAX_SOC;

		/* mixed mode only */
		if ((battery->statusword & STC3115_VMODE) == 0) {
			/*Lately fully compensation*/
			if (battery->current > APP_EOC_CURRENT && battery->soc > 990) {
				battery->soc = 990;
				stc3115_reg_write_word(dev, STC3115_REG_SOC, 99*512); //99% (99*512= 50688) //force a new SoC displayed by fuel gauge
			}

			/*Remaining time calculation*/
			if (battery->current < 0) {
				battery->remtime = (battery->remtime * 4 + battery->chargevalue / battery->current * 60 ) / 5;
				if (battery->remtime  < 0)
					battery->remtime = -1; /* means no estimated time available */
			}
			else
				battery->remtime = -1; /* means no estimated time available */

		} else {
			/* voltage mode only */
			battery->current = 0;
			battery->remtime = -1;
		}

		//SOC min/max clamping
		if (battery->soc > 1000)
			battery->soc = MAX_SOC;

		if (battery->soc < 0)
			battery->soc = 0;
	}

	/* save periodically the last valid SOC to internal RAM (in case of future Restore process) */
	ram->reg.hrsoc = battery->hrsoc;
	ram->reg.soc = (battery->soc + 5) / 10;

	ram->db[STC3115_RAM_SIZE - 1] = stc3115_crc8(ram->db, STC3115_RAM_SIZE - 1);
	stc3115_reg_write(dev, STC3115_REG_RAM, ram->db, STC3115_RAM_SIZE);

	/* only SOC, OCV and voltage are valid */
	if (ram->reg.state != STC3115_RUNNING)
		ret = -EINVAL;

err:
	return ret;
}

int stc3115_init_battery_management(struct device *dev)
{
	int crc;
	int ret = 0;
	int ocvoffset[16] = OCV_OFFSET_TAB;
	unsigned char data;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;
	struct stc3115_config *config = &priv->config;
	struct stc3115_battery_data *battery = &priv->battery_data;
	union internal_ram *ram = &priv->ram_data;

	config->vmode = priv->mode;
	config->rsense = priv->rsense;
	config->cc_cnf = (priv->battery_capacity * config->rsense * 250 + 6194) / 12389;

	if (priv->battery_impedance)
		config->vm_cnf = (priv->battery_impedance * priv->battery_impedance * 50 + 24444) / 48889;
	else
		config->vm_cnf = (priv->battery_capacity * 200 * 50 + 24444) / 48889; /* default value */

	for (int i = 0; i < 16; i++) {
		if (ocvoffset[i] > 127)
			ocvoffset[i] = 127;

		if (ocvoffset[i] < -127)
			ocvoffset[i] = -127;

		config->ocvoffset[i] = ocvoffset[i];
	}

	config->cnom = priv->battery_capacity; 
	config->relaxcurrent = priv->battery_capacity / 20;

	config->alm_soc = priv->alarm_soc;
	config->alm_vbat = priv->alarm_mv;

	battery->presence = 1;

	stc3115_reg_read(dev, STC3115_REG_RAM, ram->db, STC3115_RAM_SIZE);

	crc = stc3115_crc8(ram->db, STC3115_RAM_SIZE);

	/* RAM invalid */
	if ((ram->reg.testword != RAM_TESTWORD) || (crc !=0)) {
		/*
		 * RAM is empty (Fuel gauge first power-up)
		 * or RAM not yet initialized by this Driver (no TESTWORD)
		 * or RAM corrupted (bad CRC)
		 * => Full initialisation:  STC3115 init + RAM init
		 *    e.g. New battery plugged-in, using the initial battery model.
		 */
		stc3115_init_ram_data(config, ram);

		ret = stc3115_startup(dev);
	}
	/* RAM valid (i.e initialization process started again, battery has not been removed) */
	else {
		stc3115_reg_read(dev, STC3115_REG_CTRL, &data, sizeof(data));

		/* check STC3115 status */
		if ((data & (STC3115_BATFAIL | STC3115_PORDET)) != 0) {
			/*
			 * PORDET detected (Power On Reset occurred after Device powered-On or a Soft-reset)
			 * or Error occured: BATFAIL (battery disconnected or Undervoltage UVLO)
			 * => Standard initialisation: STC3115 init (without RAM init)
			 *    e.g. Battery has not been removed, but restoration from internal RAM not possible
			 */

			ret = stc3115_startup(dev);
		}
		else {
			/*
			 * Restoration OK, The battery has not been removed since the last application switch off.
			 * (no specific event occured, restore the latest good SoC value for better accuracy)
			 */
			ret = stc3115_restore_from_ram(dev);
		}
	}

	/* Update RAM state flag to INIT state */
	ram->reg.state = STC3115_INIT;
	ram->db[STC3115_RAM_SIZE - 1] = stc3115_crc8(ram->db, STC3115_RAM_SIZE - 1);
	stc3115_reg_write(dev, STC3115_REG_RAM, ram->db, STC3115_RAM_SIZE);

	return ret;
}

static int stc3115_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;
	struct stc3115_battery_data *battery = &priv->battery_data;

	ret = stc3115_update_battery_data(dev);
	if (ret < 0) {
		error_printk("failed to update battery data\n");
		goto err;
	}

	if (size < sizeof(*battery)) {
		error_printk("not enough space to copy battery data\n");
		ret = - ENOSPC;
		goto err;
	}


	memcpy(buff, battery, sizeof(*battery));

	ret = size;
err:
	return ret;
}

static int stc3115_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;

	return ret;
}

static int stc3115_ioctl(struct device *dev, int request, char *arg)
{
	int ret = 0;

	switch (request) {
	case IOCTL_INIT:
		ret = stc3115_init_battery_management(dev);
		break;
	case IOCTL_PM:
		if (arg)
			ret = stc3115_enable_powersaving(dev);
		else
			ret = stc3115_disable_powersaving(dev);
		break;
	}

	return ret;
}

static int stc3115_of_init(struct i2c_device *i2c, int offset)
{
	int ret = 0;
	struct stc3115_priv *priv = (struct stc3115_priv *)i2c->priv;

	ret = fdtparse_get_int(offset, "mode", (int *)&priv->mode);
	if (ret < 0) {
		error_printk("failed to retrieve mode\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "rsense", (int *)&priv->rsense);
	if (ret < 0) {
		error_printk("failed to retrieve rsense\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "battery-capacity", (int *)&priv->battery_capacity);
	if (ret < 0) {
		error_printk("failed to retrieve battery capacity\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "battery-impedance", (int *)&priv->battery_impedance);
	if (ret < 0) {
		error_printk("failed to retrieve battery impedance\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "alarm-enable", (int *)&priv->alarm_enable);
	if (ret < 0) {
		error_printk("failed to retrieve alarm enable\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "alarm-soc", (int *)&priv->alarm_soc);
	if (ret < 0) {
		error_printk("failed to retrieve alarm SOC %%\n");
		ret = -EIO;
		goto out;
	}

	ret = fdtparse_get_int(offset, "alarm-mv", (int *)&priv->alarm_mv);
	if (ret < 0) {
		error_printk("failed to retrieve alarm mV\n");
		ret = -EIO;
		goto out;
	}

out:
	return ret;
}

static struct device_operations dev_ops = {
	.read = stc3115_read,
	.write = stc3115_write,
	.ioctl = stc3115_ioctl,
};

int stc3115_init(struct device *dev)
{
	int offset;
	int ret = 0;
	unsigned char id;
	struct stc3115_priv *priv = NULL;
	struct i2c_device *i2c = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, dev->of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto err;
	}

	priv = (struct stc3115_priv *)kmalloc(sizeof(struct stc3115_priv));
	if (!priv) {
		error_printk("failed to allocate stc3115 private struct\n");
		ret = -ENOMEM;
		goto err;
	}

	i2c = i2c_new_device_with_master(offset);
	if (!i2c) {
		error_printk("failed to retrive new i2c device\n");
		ret = -EIO;
		goto free_i2c;
	}

	memcpy(&i2c->dev, dev, sizeof(struct device));

	i2c->priv = priv;

	ret = stc3115_of_init(i2c, offset);
	if (ret < 0) {
		error_printk("failed to init fdt data\n");
		goto free_i2c;
	}

	ret = i2c_register_device(i2c, &dev_ops);
	if (ret < 0) {
		error_printk("failed to register i2c device\n");
		goto free_i2c;
	}

	priv->address = STC3115_SLAVE_ADDRESS;
#if 0

	/* Check STC3115 ID register */
	ret = stc3115_reg_read(dev, STC3115_REG_ID, &id, sizeof(id));
	if (ret < 0) {
		error_printk("failed to check STC3115 ID over I2C\n");
		goto unregister_i2c;
	}

	if (id != STC3115_ID) {
		error_printk("STC3115 is replying with wrong ID (0x%02x vs 0x%02x)\n", id, STC3115_ID);
		ret = -EINVAL;
		goto unregister_i2c;
	}
#endif

	return 0;

unregister_i2c:
	i2c_remove_device(i2c);
free_i2c:
	kfree(i2c);
err:
	return ret;
}

struct device stc3115_driver = {
	.of_compat = "i2c,stc3115",
	.probe = stc3115_init,
};

static int stc3115_register(void)
{
	int ret = 0;

	ret = device_of_register(&stc3115_driver);
	if (ret < 0)
		error_printk("failed to register stc3115 device\n");
	return ret;
}
coredevice_initcall(stc3115_register);
