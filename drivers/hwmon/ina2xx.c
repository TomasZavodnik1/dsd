// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Texas Instruments INA219, INA226 power monitor chips
 *
 * INA219:
 * Zero Drift Bi-Directional Current/Power Monitor with I2C Interface
 * Datasheet: https://www.ti.com/product/ina219
 *
 * INA220:
 * Bi-Directional Current/Power Monitor with I2C Interface
 * Datasheet: https://www.ti.com/product/ina220
 *
 * INA226:
 * Bi-Directional Current/Power Monitor with I2C Interface
 * Datasheet: https://www.ti.com/product/ina226
 *
 * INA230:
 * Bi-directional Current/Power Monitor with I2C Interface
 * Datasheet: https://www.ti.com/product/ina230
 *
 * Copyright (C) 2012 Lothar Felten <lothar.felten@gmail.com>
 * Thanks to Jan Volkering
 *
 * AND
 *
 * Driver for the Texas Instruments / Burr Brown INA209
 * Bidirectional Current/Power Monitor
 *
 * Copyright (C) 2012 Guenter Roeck <linux@roeck-us.net>
 *
 * Derived from Ira W. Snyder's original driver submission
 *	Copyright (C) 2008 Paul Hays <Paul.Hays@cattail.ca>
 *	Copyright (C) 2008-2009 Ira W. Snyder <iws@ovro.caltech.edu>
 *
 * Aligned with ina2xx driver
 *	Copyright (C) 2012 Lothar Felten <l-felten@ti.com>
 *	Thanks to Jan Volkering
 *
 * Datasheet:
 * https://www.ti.com/lit/gpn/ina209
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                           *
 * Combined with INA209 driver ina209.c to able same driver to be used with  *
 * INA209 and INA231, so same device tree can be used for different HWs.     *
 * If using for INA209, define it as "ina231" in the device tree.            *
 *                                                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/util_macros.h>
#include <linux/regmap.h>

#include <linux/platform_data/ina2xx.h>


/* common register definitions */
#define INA2XX_CONFIG			0x00
#define INA2XX_SHUNT_VOLTAGE		0x01 /* readonly */
#define INA2XX_BUS_VOLTAGE		0x02 /* readonly */
#define INA2XX_POWER			0x03 /* readonly */
#define INA2XX_CURRENT			0x04 /* readonly */
#define INA2XX_CALIBRATION		0x05

/* INA226 register definitions */
#define INA226_MASK_ENABLE		0x06
#define INA226_ALERT_LIMIT		0x07
#define INA226_DIE_ID			0xFF

/* register count */
#define INA219_REGISTERS		6
#define INA226_REGISTERS		8

#define INA2XX_MAX_REGISTERS		8

/* settings - depend on use case */
#define INA219_CONFIG_DEFAULT		0x399F	/* PGA=8 */
#define INA226_CONFIG_DEFAULT		0x4527	/* averages=16 */

/* worst case is 68.10 ms (~14.6Hz, ina219) */
#define INA2XX_CONVERSION_RATE		15
#define INA2XX_MAX_DELAY		69 /* worst case delay in ms */

#define INA2XX_RSHUNT_DEFAULT		10000

/* bit mask for reading the averaging setting in the configuration register */
#define INA226_AVG_RD_MASK		0x0E00

#define INA226_READ_AVG(reg)		(((reg) & INA226_AVG_RD_MASK) >> 9)
#define INA226_SHIFT_AVG(val)		((val) << 9)

/* bit number of alert functions in Mask/Enable Register */
#define INA226_SHUNT_OVER_VOLTAGE_BIT	15
#define INA226_SHUNT_UNDER_VOLTAGE_BIT	14
#define INA226_BUS_OVER_VOLTAGE_BIT	13
#define INA226_BUS_UNDER_VOLTAGE_BIT	12
#define INA226_POWER_OVER_LIMIT_BIT	11

/* bit mask for alert config bits of Mask/Enable Register */
#define INA226_ALERT_CONFIG_MASK	0xFC00
#define INA226_ALERT_FUNCTION_FLAG	BIT(4)

/* common attrs, ina226 attrs and NULL */
#define INA2XX_MAX_ATTRIBUTE_GROUPS	3

/*
 * Both bus voltage and shunt voltage conversion times for ina226 are set
 * to 0b0100 on POR, which translates to 2200 microseconds in total.
 */
#define INA226_TOTAL_CONV_TIME_DEFAULT	2200

/* INA209 register definitions */
#define INA209_CONFIGURATION		0x00
#define INA209_STATUS			0x01
#define INA209_STATUS_MASK		0x02
#define INA209_SHUNT_VOLTAGE		0x03
#define INA209_BUS_VOLTAGE		0x04
#define INA209_POWER			0x05
#define INA209_CURRENT			0x06
#define INA209_SHUNT_VOLTAGE_POS_PEAK	0x07
#define INA209_SHUNT_VOLTAGE_NEG_PEAK	0x08
#define INA209_BUS_VOLTAGE_MAX_PEAK	0x09
#define INA209_BUS_VOLTAGE_MIN_PEAK	0x0a
#define INA209_POWER_PEAK		0x0b
#define INA209_SHUNT_VOLTAGE_POS_WARN	0x0c
#define INA209_SHUNT_VOLTAGE_NEG_WARN	0x0d
#define INA209_POWER_WARN		0x0e
#define INA209_BUS_VOLTAGE_OVER_WARN	0x0f
#define INA209_BUS_VOLTAGE_UNDER_WARN	0x10
#define INA209_POWER_OVER_LIMIT		0x11
#define INA209_BUS_VOLTAGE_OVER_LIMIT	0x12
#define INA209_BUS_VOLTAGE_UNDER_LIMIT	0x13
#define INA209_CRITICAL_DAC_POS		0x14
#define INA209_CRITICAL_DAC_NEG		0x15
#define INA209_CALIBRATION		0x16

#define INA209_REGISTERS		0x17

#define INA209_CONFIG_DEFAULT		0x3c47	/* PGA=8, full range */
#define INA209_SHUNT_DEFAULT		10000	/* uOhm */

static struct regmap_config ina2xx_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
};

enum ina2xx_ids { ina219, ina226 };

struct ina2xx_config {
	u16 config_default;
	int calibration_value;
	int registers;
	int shunt_div;
	int bus_voltage_shift;
	int bus_voltage_lsb;	/* uV */
	int power_lsb_factor;
};

struct ina2xx_data {
	const struct ina2xx_config *config;

	long rshunt;
	long current_lsb_uA;
	long power_lsb_uW;
	struct mutex config_lock;
	struct regmap *regmap;

	const struct attribute_group *groups[INA2XX_MAX_ATTRIBUTE_GROUPS];

	bool is_ina209;

	/* INA209 specific data: */
	struct i2c_client *client;

	bool valid;
	unsigned long last_updated;	/* in jiffies */

	u16 regs[INA209_REGISTERS];	/* All chip registers */

	u16 config_orig;		/* Original configuration */
	u16 calibration_orig;		/* Original calibration */
	u16 update_interval;
};

static const struct ina2xx_config ina2xx_config[] = {
	[ina219] = {
		.config_default = INA219_CONFIG_DEFAULT,
		.calibration_value = 4096,
		.registers = INA219_REGISTERS,
		.shunt_div = 100,
		.bus_voltage_shift = 3,
		.bus_voltage_lsb = 4000,
		.power_lsb_factor = 20,
	},
	[ina226] = {
		.config_default = INA226_CONFIG_DEFAULT,
		.calibration_value = 2048,
		.registers = INA226_REGISTERS,
		.shunt_div = 400,
		.bus_voltage_shift = 0,
		.bus_voltage_lsb = 1250,
		.power_lsb_factor = 25,
	},
};

/* Handle overlapping registers of INA2xx and INA209 */
static u8 ina2xx_reg_to_ina209(u8 reg)
{
	switch(reg) {
	case INA2XX_CURRENT:
		return INA209_CURRENT;
	case INA2XX_SHUNT_VOLTAGE:
		return INA209_SHUNT_VOLTAGE;
	case INA2XX_BUS_VOLTAGE:
		return INA209_BUS_VOLTAGE;
	case INA2XX_POWER:
		return INA209_POWER;
	case INA226_POWER_OVER_LIMIT_BIT:
		return INA209_POWER_OVER_LIMIT;
	}
	return reg;
}

static struct ina2xx_data *ina209_update_device(struct device *dev)
{
	struct ina2xx_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	struct ina2xx_data *ret = data;
	s32 val;
	int i;

	mutex_lock(&data->config_lock);

	if (!data->valid ||
	    time_after(jiffies, data->last_updated + data->update_interval)) {
		for (i = 0; i < ARRAY_SIZE(data->regs); i++) {
			val = i2c_smbus_read_word_swapped(client, i);
			if (val < 0) {
				ret = ERR_PTR(val);
				goto abort;
			}
			data->regs[i] = val;
		}
		data->last_updated = jiffies;
		data->valid = true;
	}
abort:
	mutex_unlock(&data->config_lock);
	return ret;
}

/*
 * Read a value from a device register and convert it to the
 * appropriate sysfs units
 */
static long ina209_from_reg(const u8 reg, const u16 val)
{
	switch (reg) {
	case INA209_SHUNT_VOLTAGE:
	case INA209_SHUNT_VOLTAGE_POS_PEAK:
	case INA209_SHUNT_VOLTAGE_NEG_PEAK:
	case INA209_SHUNT_VOLTAGE_POS_WARN:
	case INA209_SHUNT_VOLTAGE_NEG_WARN:
		/* LSB=10 uV. Convert to mV. */
		return DIV_ROUND_CLOSEST((s16)val, 100);

	case INA209_BUS_VOLTAGE:
	case INA209_BUS_VOLTAGE_MAX_PEAK:
	case INA209_BUS_VOLTAGE_MIN_PEAK:
	case INA209_BUS_VOLTAGE_OVER_WARN:
	case INA209_BUS_VOLTAGE_UNDER_WARN:
	case INA209_BUS_VOLTAGE_OVER_LIMIT:
	case INA209_BUS_VOLTAGE_UNDER_LIMIT:
		/* LSB=4 mV, last 3 bits unused */
		return (val >> 3) * 4;

	case INA209_CRITICAL_DAC_POS:
		/* LSB=1 mV, in the upper 8 bits */
		return val >> 8;

	case INA209_CRITICAL_DAC_NEG:
		/* LSB=1 mV, in the upper 8 bits */
		return -1 * (val >> 8);

	case INA209_POWER:
	case INA209_POWER_PEAK:
	case INA209_POWER_WARN:
	case INA209_POWER_OVER_LIMIT:
		/* LSB=20 mW. Convert to uW */
		return val * 20 * 1000L;

	case INA209_CURRENT:
		/* LSB=1 mA (selected). Is in mA */
		return (s16)val;
	}

	/* programmer goofed */
	WARN_ON_ONCE(1);
	return 0;
}

/*
 * Take a value and convert it to register format, clamping the value
 * to the appropriate range.
 */
static int ina209_to_reg(u8 reg, u16 old, long val)
{
	switch (reg) {
	case INA209_SHUNT_VOLTAGE_POS_WARN:
	case INA209_SHUNT_VOLTAGE_NEG_WARN:
		/* Limit to +- 320 mV, 10 uV LSB */
		return clamp_val(val, -320, 320) * 100;

	case INA209_BUS_VOLTAGE_OVER_WARN:
	case INA209_BUS_VOLTAGE_UNDER_WARN:
	case INA209_BUS_VOLTAGE_OVER_LIMIT:
	case INA209_BUS_VOLTAGE_UNDER_LIMIT:
		/*
		 * Limit to 0-32000 mV, 4 mV LSB
		 *
		 * The last three bits aren't part of the value, but we'll
		 * preserve them in their original state.
		 */
		return (DIV_ROUND_CLOSEST(clamp_val(val, 0, 32000), 4) << 3)
		  | (old & 0x7);

	case INA209_CRITICAL_DAC_NEG:
		/*
		 * Limit to -255-0 mV, 1 mV LSB
		 * Convert the value to a positive value for the register
		 *
		 * The value lives in the top 8 bits only, be careful
		 * and keep original value of other bits.
		 */
		return (clamp_val(-val, 0, 255) << 8) | (old & 0xff);

	case INA209_CRITICAL_DAC_POS:
		/*
		 * Limit to 0-255 mV, 1 mV LSB
		 *
		 * The value lives in the top 8 bits only, be careful
		 * and keep original value of other bits.
		 */
		return (clamp_val(val, 0, 255) << 8) | (old & 0xff);

	case INA209_POWER_WARN:
	case INA209_POWER_OVER_LIMIT:
		/* 20 mW LSB */
		return DIV_ROUND_CLOSEST(val, 20 * 1000);
	}

	/* Other registers are read-only, return access error */
	return -EACCES;
}

static int ina209_interval_from_reg(u16 reg)
{
	return 68 >> (15 - ((reg >> 3) & 0x0f));
}

static u16 ina209_reg_from_interval(u16 config, long interval)
{
	int i, adc;

	if (interval <= 0) {
		adc = 8;
	} else {
		adc = 15;
		for (i = 34 + 34 / 2; i; i >>= 1) {
			if (i < interval)
				break;
			adc--;
		}
	}
	return (config & 0xf807) | (adc << 3) | (adc << 7);
}

static ssize_t ina209_interval_store(struct device *dev,
				     struct device_attribute *da,
				     const char *buf, size_t count)
{
	struct ina2xx_data *data = ina209_update_device(dev);
	long val;
	u16 regval;
	int ret;

	if (IS_ERR(data))
		return PTR_ERR(data);

	ret = kstrtol(buf, 10, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&data->config_lock);
	regval = ina209_reg_from_interval(data->regs[INA209_CONFIGURATION],
					  val);
	i2c_smbus_write_word_swapped(data->client, INA209_CONFIGURATION,
				     regval);
	data->regs[INA209_CONFIGURATION] = regval;
	data->update_interval = ina209_interval_from_reg(regval);
	mutex_unlock(&data->config_lock);
	return count;
}

static ssize_t ina209_interval_show(struct device *dev,
				    struct device_attribute *da, char *buf)
{
	struct ina2xx_data *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", data->update_interval);
}

/*
 * History is reset by writing 1 into bit 0 of the respective peak register.
 * Since more than one peak register may be affected by the scope of a
 * reset_history attribute write, use a bit mask in attr->index to identify
 * which registers are affected.
 */
static u16 ina209_reset_history_regs[] = {
	INA209_SHUNT_VOLTAGE_POS_PEAK,
	INA209_SHUNT_VOLTAGE_NEG_PEAK,
	INA209_BUS_VOLTAGE_MAX_PEAK,
	INA209_BUS_VOLTAGE_MIN_PEAK,
	INA209_POWER_PEAK
};

static ssize_t ina209_history_store(struct device *dev,
				    struct device_attribute *da,
				    const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina2xx_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u32 mask = attr->index;
	long val;
	int i, ret;

	ret = kstrtol(buf, 10, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&data->config_lock);
	for (i = 0; i < ARRAY_SIZE(ina209_reset_history_regs); i++) {
		if (mask & (1 << i))
			i2c_smbus_write_word_swapped(client,
					ina209_reset_history_regs[i], 1);
	}
	data->valid = false;
	mutex_unlock(&data->config_lock);
	return count;
}

static ssize_t ina209_value_store(struct device *dev,
				  struct device_attribute *da,
				  const char *buf, size_t count)
{
	struct ina2xx_data *data = ina209_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int reg = ina2xx_reg_to_ina209(attr->index);
	long val;
	int ret;

	if (IS_ERR(data))
		return PTR_ERR(data);

	ret = kstrtol(buf, 10, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&data->config_lock);
	ret = ina209_to_reg(reg, data->regs[reg], val);
	if (ret < 0) {
		count = ret;
		goto abort;
	}
	i2c_smbus_write_word_swapped(data->client, reg, ret);
	data->regs[reg] = ret;
abort:
	mutex_unlock(&data->config_lock);
	return count;
}

static ssize_t ina209_value_show(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina2xx_data *data = ina209_update_device(dev);
	long val;
	u8 index = ina2xx_reg_to_ina209(attr->index);

	if (IS_ERR(data))
		return PTR_ERR(data);

	val = ina209_from_reg(index, data->regs[index]);
	return sysfs_emit(buf, "%ld\n", val);
}

static ssize_t ina209_alarm_show(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina2xx_data *data = ina209_update_device(dev);
	const unsigned int mask = attr->index;
	u16 status;

	if (IS_ERR(data))
		return PTR_ERR(data);

	status = data->regs[INA209_STATUS];

	/*
	 * All alarms are in the INA209_STATUS register. To avoid a long
	 * switch statement, the mask is passed in attr->index
	 */
	return sysfs_emit(buf, "%u\n", !!(status & mask));
}

/*
 * Available averaging rates for ina226. The indices correspond with
 * the bit values expected by the chip (according to the ina226 datasheet,
 * table 3 AVG bit settings, found at
 * https://www.ti.com/lit/ds/symlink/ina226.pdf.
 */
static const int ina226_avg_tab[] = { 1, 4, 16, 64, 128, 256, 512, 1024 };

static int ina226_reg_to_interval(u16 config)
{
	int avg = ina226_avg_tab[INA226_READ_AVG(config)];

	/*
	 * Multiply the total conversion time by the number of averages.
	 * Return the result in milliseconds.
	 */
	return DIV_ROUND_CLOSEST(avg * INA226_TOTAL_CONV_TIME_DEFAULT, 1000);
}

/*
 * Return the new, shifted AVG field value of CONFIG register,
 * to use with regmap_update_bits
 */
static u16 ina226_interval_to_reg(int interval)
{
	int avg, avg_bits;

	avg = DIV_ROUND_CLOSEST(interval * 1000,
				INA226_TOTAL_CONV_TIME_DEFAULT);
	avg_bits = find_closest(avg, ina226_avg_tab,
				ARRAY_SIZE(ina226_avg_tab));

	return INA226_SHIFT_AVG(avg_bits);
}

/*
 * Calibration register is set to the best value, which eliminates
 * truncation errors on calculating current register in hardware.
 * According to datasheet (eq. 3) the best values are 2048 for
 * ina226 and 4096 for ina219. They are hardcoded as calibration_value.
 */
static int ina2xx_calibrate(struct ina2xx_data *data)
{
	return regmap_write(data->regmap, INA2XX_CALIBRATION,
			    data->config->calibration_value);
}

/*
 * Initialize the configuration and calibration registers.
 */
static int ina2xx_init(struct ina2xx_data *data)
{
	int ret = regmap_write(data->regmap, INA2XX_CONFIG,
			       data->config->config_default);
	if (ret < 0)
		return ret;

	return ina2xx_calibrate(data);
}

static int ina2xx_read_reg(struct device *dev, int reg, unsigned int *regval)
{
	struct ina2xx_data *data = dev_get_drvdata(dev);
	int ret, retry;

	dev_dbg(dev, "Starting register %d read\n", reg);

	for (retry = 5; retry; retry--) {

		ret = regmap_read(data->regmap, reg, regval);
		if (ret < 0)
			return ret;

		dev_dbg(dev, "read %d, val = 0x%04x\n", reg, *regval);

		/*
		 * If the current value in the calibration register is 0, the
		 * power and current registers will also remain at 0. In case
		 * the chip has been reset let's check the calibration
		 * register and reinitialize if needed.
		 * We do that extra read of the calibration register if there
		 * is some hint of a chip reset.
		 */
		if (*regval == 0) {
			unsigned int cal;

			ret = regmap_read(data->regmap, INA2XX_CALIBRATION,
					  &cal);
			if (ret < 0)
				return ret;

			if (cal == 0) {
				dev_warn(dev, "chip not calibrated, reinitializing\n");

				ret = ina2xx_init(data);
				if (ret < 0)
					return ret;
				/*
				 * Let's make sure the power and current
				 * registers have been updated before trying
				 * again.
				 */
				msleep(INA2XX_MAX_DELAY);
				continue;
			}
		}
		return 0;
	}

	/*
	 * If we're here then although all write operations succeeded, the
	 * chip still returns 0 in the calibration register. Nothing more we
	 * can do here.
	 */
	dev_err(dev, "unable to reinitialize the chip\n");
	return -ENODEV;
}

static int ina2xx_get_value(struct ina2xx_data *data, u8 reg,
			    unsigned int regval)
{
	int val;

	switch (reg) {
	case INA2XX_SHUNT_VOLTAGE:
		/* signed register */
		val = DIV_ROUND_CLOSEST((s16)regval, data->config->shunt_div);
		break;
	case INA2XX_BUS_VOLTAGE:
		val = (regval >> data->config->bus_voltage_shift)
		  * data->config->bus_voltage_lsb;
		val = DIV_ROUND_CLOSEST(val, 1000);
		break;
	case INA2XX_POWER:
		val = regval * data->power_lsb_uW;
		break;
	case INA2XX_CURRENT:
		/* signed register, result in mA */
		val = (s16)regval * data->current_lsb_uA;
		val = DIV_ROUND_CLOSEST(val, 1000);
		break;
	case INA2XX_CALIBRATION:
		val = regval;
		break;
	default:
		/* programmer goofed */
		WARN_ON_ONCE(1);
		val = 0;
		break;
	}

	return val;
}

static ssize_t ina2xx_value_store(struct device *dev,
				  struct device_attribute *da,
				  const char *buf, size_t count)
{
	return ina209_value_store(dev, da, buf, count);
}


static ssize_t ina2xx_value_show(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina2xx_data *data = dev_get_drvdata(dev);
	unsigned int regval;

	if (data->is_ina209)
		return ina209_value_show(dev, da, buf);

	int err = ina2xx_read_reg(dev, attr->index, &regval);

	if (err < 0)
		return err;

	return sysfs_emit(buf, "%d\n", ina2xx_get_value(data, attr->index, regval));
}

static int ina226_reg_to_alert(struct ina2xx_data *data, u8 bit, u16 regval)
{
	int reg;

	switch (bit) {
	case INA226_SHUNT_OVER_VOLTAGE_BIT:
	case INA226_SHUNT_UNDER_VOLTAGE_BIT:
		reg = INA2XX_SHUNT_VOLTAGE;
		break;
	case INA226_BUS_OVER_VOLTAGE_BIT:
	case INA226_BUS_UNDER_VOLTAGE_BIT:
		reg = INA2XX_BUS_VOLTAGE;
		break;
	case INA226_POWER_OVER_LIMIT_BIT:
		reg = INA2XX_POWER;
		break;
	default:
		/* programmer goofed */
		WARN_ON_ONCE(1);
		return 0;
	}

	return ina2xx_get_value(data, reg, regval);
}

/*
 * Turns alert limit values into register values.
 * Opposite of the formula in ina2xx_get_value().
 */
static s16 ina226_alert_to_reg(struct ina2xx_data *data, u8 bit, int val)
{
	switch (bit) {
	case INA226_SHUNT_OVER_VOLTAGE_BIT:
	case INA226_SHUNT_UNDER_VOLTAGE_BIT:
		val *= data->config->shunt_div;
		return clamp_val(val, SHRT_MIN, SHRT_MAX);
	case INA226_BUS_OVER_VOLTAGE_BIT:
	case INA226_BUS_UNDER_VOLTAGE_BIT:
		val = (val * 1000) << data->config->bus_voltage_shift;
		val = DIV_ROUND_CLOSEST(val, data->config->bus_voltage_lsb);
		return clamp_val(val, 0, SHRT_MAX);
	case INA226_POWER_OVER_LIMIT_BIT:
		val = DIV_ROUND_CLOSEST(val, data->power_lsb_uW);
		return clamp_val(val, 0, USHRT_MAX);
	default:
		/* programmer goofed */
		WARN_ON_ONCE(1);
		return 0;
	}
}

static ssize_t ina226_alert_show(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina2xx_data *data = dev_get_drvdata(dev);
	int regval;
	int val = 0;
	int ret;

	if (data->is_ina209)
		return ina209_value_show(dev, da, buf);

	mutex_lock(&data->config_lock);
	ret = regmap_read(data->regmap, INA226_MASK_ENABLE, &regval);
	if (ret)
		goto abort;

	if (regval & BIT(attr->index)) {
		ret = regmap_read(data->regmap, INA226_ALERT_LIMIT, &regval);
		if (ret)
			goto abort;
		val = ina226_reg_to_alert(data, attr->index, regval);
	}

	ret = sysfs_emit(buf, "%d\n", val);
abort:
	mutex_unlock(&data->config_lock);
	return ret;
}

static ssize_t ina226_alert_store(struct device *dev,
				  struct device_attribute *da,
				  const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina2xx_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	if (data->is_ina209)
		return ina209_value_show(dev, da, buf);

	ret = kstrtoul(buf, 10, &val);
	if (ret < 0)
		return ret;

	/*
	 * Clear all alerts first to avoid accidentally triggering ALERT pin
	 * due to register write sequence. Then, only enable the alert
	 * if the value is non-zero.
	 */
	mutex_lock(&data->config_lock);
	ret = regmap_update_bits(data->regmap, INA226_MASK_ENABLE,
				 INA226_ALERT_CONFIG_MASK, 0);
	if (ret < 0)
		goto abort;

	ret = regmap_write(data->regmap, INA226_ALERT_LIMIT,
			   ina226_alert_to_reg(data, attr->index, val));
	if (ret < 0)
		goto abort;

	if (val != 0) {
		ret = regmap_update_bits(data->regmap, INA226_MASK_ENABLE,
					 INA226_ALERT_CONFIG_MASK,
					 BIT(attr->index));
		if (ret < 0)
			goto abort;
	}

	ret = count;
abort:
	mutex_unlock(&data->config_lock);
	return ret;
}

static ssize_t ina226_alarm_show(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina2xx_data *data = dev_get_drvdata(dev);
	int regval;
	int alarm = 0;
	int ret;

	if (data->is_ina209)
		return ina209_alarm_show(dev, da, buf);

	ret = regmap_read(data->regmap, INA226_MASK_ENABLE, &regval);
	if (ret)
		return ret;

	alarm = (regval & BIT(attr->index)) &&
		(regval & INA226_ALERT_FUNCTION_FLAG);
	return sysfs_emit(buf, "%d\n", alarm);
}

/*
 * In order to keep calibration register value fixed, the product
 * of current_lsb and shunt_resistor should also be fixed and equal
 * to shunt_voltage_lsb = 1 / shunt_div multiplied by 10^9 in order
 * to keep the scale.
 */
static int ina2xx_set_shunt(struct ina2xx_data *data, long val)
{
	unsigned int dividend = DIV_ROUND_CLOSEST(1000000000,
						  data->config->shunt_div);
	if (val <= 0 || val > dividend)
		return -EINVAL;

	mutex_lock(&data->config_lock);
	data->rshunt = val;
	data->current_lsb_uA = DIV_ROUND_CLOSEST(dividend, val);
	data->power_lsb_uW = data->config->power_lsb_factor *
			     data->current_lsb_uA;
	mutex_unlock(&data->config_lock);

	return 0;
}

static ssize_t ina2xx_shunt_show(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct ina2xx_data *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%li\n", data->rshunt);
}

static ssize_t ina2xx_shunt_store(struct device *dev,
				  struct device_attribute *da,
				  const char *buf, size_t count)
{
	unsigned long val;
	int status;
	struct ina2xx_data *data = dev_get_drvdata(dev);

	status = kstrtoul(buf, 10, &val);
	if (status < 0)
		return status;

	status = ina2xx_set_shunt(data, val);
	if (status < 0)
		return status;
	return count;
}

static ssize_t ina226_interval_store(struct device *dev,
				     struct device_attribute *da,
				     const char *buf, size_t count)
{
	struct ina2xx_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int status;

	if (data->is_ina209)
		return ina209_interval_store;

	status = kstrtoul(buf, 10, &val);
	if (status < 0)
		return status;

	if (val > INT_MAX || val == 0)
		return -EINVAL;

	status = regmap_update_bits(data->regmap, INA2XX_CONFIG,
				    INA226_AVG_RD_MASK,
				    ina226_interval_to_reg(val));
	if (status < 0)
		return status;

	return count;
}

static ssize_t ina226_interval_show(struct device *dev,
				    struct device_attribute *da, char *buf)
{
	struct ina2xx_data *data = dev_get_drvdata(dev);
	int status;
	unsigned int regval;

	if (data->is_ina209)
		return ina209_interval_show;

	status = regmap_read(data->regmap, INA2XX_CONFIG, &regval);
	if (status)
		return status;

	return sysfs_emit(buf, "%d\n", ina226_reg_to_interval(regval));
}

/* shunt voltage */
static SENSOR_DEVICE_ATTR_RO(in0_input, ina2xx_value, INA2XX_SHUNT_VOLTAGE);
/* shunt voltage over/under voltage alert setting and alarm */
static SENSOR_DEVICE_ATTR_RW(in0_crit, ina226_alert,
			     INA226_SHUNT_OVER_VOLTAGE_BIT);
static SENSOR_DEVICE_ATTR_RW(in0_lcrit, ina226_alert,
			     INA226_SHUNT_UNDER_VOLTAGE_BIT);
static SENSOR_DEVICE_ATTR_RO(in0_crit_alarm, ina226_alarm,
			     INA226_SHUNT_OVER_VOLTAGE_BIT);
static SENSOR_DEVICE_ATTR_RO(in0_lcrit_alarm, ina226_alarm,
			     INA226_SHUNT_UNDER_VOLTAGE_BIT);

/* bus voltage */
static SENSOR_DEVICE_ATTR_RO(in1_input, ina2xx_value, INA2XX_BUS_VOLTAGE);
/* bus voltage over/under voltage alert setting and alarm */
static SENSOR_DEVICE_ATTR_RW(in1_crit, ina226_alert,
			     INA226_BUS_OVER_VOLTAGE_BIT);
static SENSOR_DEVICE_ATTR_RW(in1_lcrit, ina226_alert,
			     INA226_BUS_UNDER_VOLTAGE_BIT);
static SENSOR_DEVICE_ATTR_RO(in1_crit_alarm, ina226_alarm,
			     INA226_BUS_OVER_VOLTAGE_BIT);
static SENSOR_DEVICE_ATTR_RO(in1_lcrit_alarm, ina226_alarm,
			     INA226_BUS_UNDER_VOLTAGE_BIT);

/* calculated current */
static SENSOR_DEVICE_ATTR_RO(curr1_input, ina2xx_value, INA2XX_CURRENT);

/* calculated power */
static SENSOR_DEVICE_ATTR_RO(power1_input, ina2xx_value, INA2XX_POWER);
/* over-limit power alert setting and alarm */
static SENSOR_DEVICE_ATTR_RW(power1_crit, ina226_alert,
			     INA226_POWER_OVER_LIMIT_BIT);
static SENSOR_DEVICE_ATTR_RO(power1_crit_alarm, ina226_alarm,
			     INA226_POWER_OVER_LIMIT_BIT);

/* shunt resistance */
static SENSOR_DEVICE_ATTR_RW(shunt_resistor, ina2xx_shunt, INA2XX_CALIBRATION);

/* update interval (ina226 only) */
static SENSOR_DEVICE_ATTR_RW(update_interval, ina226_interval, 0);

/* INA209 device attributes */

/* Shunt voltage, history, limits, alarms */
static SENSOR_DEVICE_ATTR_RO(in0_input_highest, ina209_value,
			     INA209_SHUNT_VOLTAGE_POS_PEAK);
static SENSOR_DEVICE_ATTR_RO(in0_input_lowest, ina209_value,
			     INA209_SHUNT_VOLTAGE_NEG_PEAK);
static SENSOR_DEVICE_ATTR_WO(in0_reset_history, ina209_history,
			     (1 << 0) | (1 << 1));
static SENSOR_DEVICE_ATTR_RW(in0_max, ina209_value,
			     INA209_SHUNT_VOLTAGE_POS_WARN);
static SENSOR_DEVICE_ATTR_RW(in0_min, ina209_value,
			     INA209_SHUNT_VOLTAGE_NEG_WARN);
static SENSOR_DEVICE_ATTR_RW(in0_crit_max, ina209_value,
			     INA209_CRITICAL_DAC_POS);
static SENSOR_DEVICE_ATTR_RW(in0_crit_min, ina209_value,
			     INA209_CRITICAL_DAC_NEG);

static SENSOR_DEVICE_ATTR_RO(in0_min_alarm, ina209_alarm, 1 << 11);
static SENSOR_DEVICE_ATTR_RO(in0_max_alarm, ina209_alarm, 1 << 12);
static SENSOR_DEVICE_ATTR_RO(in0_crit_min_alarm, ina209_alarm, 1 << 6);
static SENSOR_DEVICE_ATTR_RO(in0_crit_max_alarm, ina209_alarm, 1 << 7);

/* Bus voltage, history, limits, alarms */
static SENSOR_DEVICE_ATTR_RO(in1_input_highest, ina209_value,
			     INA209_BUS_VOLTAGE_MAX_PEAK);
static SENSOR_DEVICE_ATTR_RO(in1_input_lowest, ina209_value,
			     INA209_BUS_VOLTAGE_MIN_PEAK);
static SENSOR_DEVICE_ATTR_WO(in1_reset_history, ina209_history,
			     (1 << 2) | (1 << 3));
static SENSOR_DEVICE_ATTR_RW(in1_max, ina209_value,
			     INA209_BUS_VOLTAGE_OVER_WARN);
static SENSOR_DEVICE_ATTR_RW(in1_min, ina209_value,
			     INA209_BUS_VOLTAGE_UNDER_WARN);
static SENSOR_DEVICE_ATTR_RW(in1_crit_max, ina209_value,
			     INA209_BUS_VOLTAGE_OVER_LIMIT);
static SENSOR_DEVICE_ATTR_RW(in1_crit_min, ina209_value,
			     INA209_BUS_VOLTAGE_UNDER_LIMIT);

static SENSOR_DEVICE_ATTR_RO(in1_min_alarm, ina209_alarm, 1 << 14);
static SENSOR_DEVICE_ATTR_RO(in1_max_alarm, ina209_alarm, 1 << 15);
static SENSOR_DEVICE_ATTR_RO(in1_crit_min_alarm, ina209_alarm, 1 << 9);
static SENSOR_DEVICE_ATTR_RO(in1_crit_max_alarm, ina209_alarm, 1 << 10);

/* Power */
static SENSOR_DEVICE_ATTR_RO(power1_input_highest, ina209_value,
			     INA209_POWER_PEAK);
static SENSOR_DEVICE_ATTR_WO(power1_reset_history, ina209_history, 1 << 4);
static SENSOR_DEVICE_ATTR_RW(power1_max, ina209_value, INA209_POWER_WARN);

static SENSOR_DEVICE_ATTR_RO(power1_max_alarm, ina209_alarm, 1 << 13);

/* pointers to created device attributes */
static struct attribute *ina2xx_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_power1_input.dev_attr.attr,
	&sensor_dev_attr_shunt_resistor.dev_attr.attr,
	NULL,
};

static const struct attribute_group ina2xx_group = {
	.attrs = ina2xx_attrs,
};

static struct attribute *ina226_attrs[] = {
	&sensor_dev_attr_in0_crit.dev_attr.attr,
	&sensor_dev_attr_in0_lcrit.dev_attr.attr,
	&sensor_dev_attr_in0_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_in0_lcrit_alarm.dev_attr.attr,
	&sensor_dev_attr_in1_crit.dev_attr.attr,
	&sensor_dev_attr_in1_lcrit.dev_attr.attr,
	&sensor_dev_attr_in1_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_in1_lcrit_alarm.dev_attr.attr,
	&sensor_dev_attr_power1_crit.dev_attr.attr,
	&sensor_dev_attr_power1_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_update_interval.dev_attr.attr,
	NULL,
};

static const struct attribute_group ina226_group = {
	.attrs = ina226_attrs,
};

static struct attribute *ina209_attrs[] = {
	&sensor_dev_attr_in0_input_highest.dev_attr.attr,
	&sensor_dev_attr_in0_input_lowest.dev_attr.attr,
	&sensor_dev_attr_in0_reset_history.dev_attr.attr,
	&sensor_dev_attr_in0_max.dev_attr.attr,
	&sensor_dev_attr_in0_min.dev_attr.attr,
	&sensor_dev_attr_in0_crit_max.dev_attr.attr,
	&sensor_dev_attr_in0_crit_min.dev_attr.attr,
	&sensor_dev_attr_in0_max_alarm.dev_attr.attr,
	&sensor_dev_attr_in0_min_alarm.dev_attr.attr,
	&sensor_dev_attr_in0_crit_max_alarm.dev_attr.attr,
	&sensor_dev_attr_in0_crit_min_alarm.dev_attr.attr,
	&sensor_dev_attr_in1_input_highest.dev_attr.attr,
	&sensor_dev_attr_in1_input_lowest.dev_attr.attr,
	&sensor_dev_attr_in1_reset_history.dev_attr.attr,
	&sensor_dev_attr_in1_max.dev_attr.attr,
	&sensor_dev_attr_in1_min.dev_attr.attr,
	&sensor_dev_attr_in1_crit_max.dev_attr.attr,
	&sensor_dev_attr_in1_crit_min.dev_attr.attr,
	&sensor_dev_attr_in1_max_alarm.dev_attr.attr,
	&sensor_dev_attr_in1_min_alarm.dev_attr.attr,
	&sensor_dev_attr_in1_crit_max_alarm.dev_attr.attr,
	&sensor_dev_attr_in1_crit_min_alarm.dev_attr.attr,
	&sensor_dev_attr_power1_input_highest.dev_attr.attr,
	&sensor_dev_attr_power1_reset_history.dev_attr.attr,
	&sensor_dev_attr_power1_max.dev_attr.attr,
	&sensor_dev_attr_power1_crit.dev_attr.attr,
	&sensor_dev_attr_power1_max_alarm.dev_attr.attr,
	&sensor_dev_attr_power1_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_update_interval.dev_attr.attr,
	NULL,
};

static const struct attribute_group ina209_group = {
	.attrs = ina209_attrs,
};

static void ina209_restore_conf(struct i2c_client *client,
				struct ina2xx_data *data)
{
	/* Restore initial configuration */
	i2c_smbus_write_word_swapped(client, INA209_CONFIGURATION,
				     data->config_orig);
	i2c_smbus_write_word_swapped(client, INA209_CALIBRATION,
				     data->calibration_orig);
}

static int ina209_init_client(struct i2c_client *client,
			      struct ina2xx_data *data)
{
	struct ina2xx_platform_data *pdata = dev_get_platdata(&client->dev);
	u32 shunt;
	int reg;

	reg = i2c_smbus_read_word_swapped(client, INA209_CALIBRATION);
	if (reg < 0)
		return reg;
	data->calibration_orig = reg;

	reg = i2c_smbus_read_word_swapped(client, INA209_CONFIGURATION);
	if (reg < 0)
		return reg;
	data->config_orig = reg;

	if (pdata) {
		if (pdata->shunt_uohms <= 0)
			return -EINVAL;
		shunt = pdata->shunt_uohms;
	} else if (!of_property_read_u32(client->dev.of_node, "shunt-resistor",
					 &shunt)) {
		if (shunt == 0)
			return -EINVAL;
	} else {
		shunt = data->calibration_orig ?
		  40960000 / data->calibration_orig : INA209_SHUNT_DEFAULT;
	}

	i2c_smbus_write_word_swapped(client, INA209_CONFIGURATION,
				     INA209_CONFIG_DEFAULT);
	data->update_interval = ina209_interval_from_reg(INA209_CONFIG_DEFAULT);

	/*
	 * Calibrate current LSB to 1mA. Shunt is in uOhms.
	 * See equation 13 in datasheet.
	 */
	i2c_smbus_write_word_swapped(client, INA209_CALIBRATION,
				     clamp_val(40960000 / shunt, 1, 65535));

	/* Clear status register */
	i2c_smbus_read_word_swapped(client, INA209_STATUS);

	return 0;
}

static const struct i2c_device_id ina2xx_id[];

static int ina2xx_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ina2xx_data *data;
	struct device *hwmon_dev;
	u32 val;
	int ret, group = 0;
	enum ina2xx_ids chip;

	if (client->dev.of_node)
		chip = (enum ina2xx_ids)of_device_get_match_data(&client->dev);
	else
		chip = i2c_match_id(ina2xx_id, client)->driver_data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* set the device type */
	data->config = &ina2xx_config[chip];
	mutex_init(&data->config_lock);

	/* Check if the device is INA209 */
	ret = i2c_smbus_read_word_swapped(client, INA209_CONFIGURATION);
	if (ret < 0)
		return ret;
	data->is_ina209 = ((ret & 0x4000) == 0);

	if(data->is_ina209) {
		i2c_set_clientdata(client, data);
		data->client = client;
		ret = ina209_init_client(client, data);
		if (ret)
			return ret;
	} else {
		if (of_property_read_u32(dev->of_node, "shunt-resistor", &val) < 0) {
			struct ina2xx_platform_data *pdata = dev_get_platdata(dev);

			if (pdata)
				val = pdata->shunt_uohms;
			else
				val = INA2XX_RSHUNT_DEFAULT;
		}
		ina2xx_set_shunt(data, val);
		ina2xx_regmap_config.max_register = data->config->registers;
		data->regmap = devm_regmap_init_i2c(client, &ina2xx_regmap_config);
		if (IS_ERR(data->regmap)) {
			dev_err(dev, "failed to allocate register map\n");
			return PTR_ERR(data->regmap);
		}
		ret = ina2xx_init(data);
		if (ret < 0) {
			dev_err(dev, "error configuring the device: %d\n", ret);
			return -ENODEV;
		}
	}
	data->groups[group++] = &ina2xx_group;
	if (data->is_ina209)
		data->groups[group++] = &ina209_group;
	else if (chip == ina226)
		data->groups[group++] = &ina226_group;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
							   data, data->groups);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	dev_info(dev, "power monitor %s (Rshunt = %li uOhm)\n",
		 client->name, data->rshunt);

	return 0;
}

static const struct i2c_device_id ina2xx_id[] = {
	{ "ina219", ina219 },
	{ "ina220", ina219 },
	{ "ina226", ina226 },
	{ "ina230", ina226 },
	{ "ina231", ina226 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ina2xx_id);

static const struct of_device_id __maybe_unused ina2xx_of_match[] = {
	{
		.compatible = "ti,ina219",
		.data = (void *)ina219
	},
	{
		.compatible = "ti,ina220",
		.data = (void *)ina219
	},
	{
		.compatible = "ti,ina226",
		.data = (void *)ina226
	},
	{
		.compatible = "ti,ina230",
		.data = (void *)ina226
	},
	{
		.compatible = "ti,ina231",
		.data = (void *)ina226
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ina2xx_of_match);

static struct i2c_driver ina2xx_driver = {
	.driver = {
		.name	= "ina2xx",
		.of_match_table = of_match_ptr(ina2xx_of_match),
	},
	.probe_new	= ina2xx_probe,
	.id_table	= ina2xx_id,
};

module_i2c_driver(ina2xx_driver);

MODULE_AUTHOR("Lothar Felten <l-felten@ti.com>");
MODULE_DESCRIPTION("ina2xx driver");
MODULE_LICENSE("GPL");
