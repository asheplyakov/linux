/*
 * Maxim MAX17201/MAX17205 fuel gauge driver
 *
 * Author: Kerem Sahin <kerem.sahin@maximintegrated.com>
 *         Mahir Ozturk <mahir.ozturk@maximintegrated.com>
 * Copyright (C) 2018 Maxim Integrated
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define DRV_NAME "max1720x"

/* CONFIG register bits */
#define MAX1720X_CONFIG_ALRT_EN		(1 << 2)

/* STATUS register bits */
#define MAX1720X_STATUS_BST		(1 << 3)
#define MAX1720X_STATUS_POR		(1 << 1)

/* FSTAT register bits */
#define MAX1720X_FSTAT_DNR		(1)
#define MAX1720X_FSTAT_FQ		(0x80)

/* STATUS interrupt status bits */
#define MAX1720X_STATUS_ALRT_CLR_MASK	(0x88BB)
#define MAX1720X_STATUS_SOC_MAX_ALRT	(1 << 14)
#define MAX1720X_STATUS_TEMP_MAX_ALRT	(1 << 13)
#define MAX1720X_STATUS_VOLT_MAX_ALRT	(1 << 12)
#define MAX1720X_STATUS_SOC_MIN_ALRT	(1 << 10)
#define MAX1720X_STATUS_TEMP_MIN_ALRT	(1 << 9)
#define MAX1720X_STATUS_VOLT_MIN_ALRT	(1 << 8)
#define MAX1720X_STATUS_CURR_MAX_ALRT	(1 << 6)
#define MAX1720X_STATUS_CURR_MIN_ALRT	(1 << 2)

#define MAX1720X_VMAX_TOLERANCE		50 /* 50 mV */

#define MODELGAUGE_DATA_I2C_ADDR	0x36
#define NONVOLATILE_DATA_I2C_ADDR	0x0B



struct max1720x_platform_data {
	/*
	 * rsense in miliOhms.
	 * default 10 (if rsense = 0) as it is the recommended value by
	 * the datasheet although it can be changed by board designers.
	 */
	unsigned int rsense;
	int volt_min;	/* in mV */
	int volt_max;	/* in mV */
	int temp_min;	/* in DegreC */
	int temp_max;	/* in DegreeC */
	int soc_max;	/* in percent */
	int soc_min;	/* in percent */
	int curr_max;	/* in mA */
	int curr_min;	/* in mA */
};

enum max1720x_register {
	MAX1720X_STATUS_REG		= 0x00,
	MAX1720X_VALRTTH_REG		= 0x01,
	MAX1720X_TALRTTH_REG		= 0x02,
	MAX1720X_SALRTTH_REG		= 0x03,
	MAX1720X_ATRATE_REG		= 0x04,
	MAX1720X_REPCAP_REG		= 0x05,
	MAX1720X_REPSOC_REG		= 0x06,
	MAX1720X_AGE_REG		= 0x07,
	MAX1720X_TEMP_REG		= 0x08,
	MAX1720X_VCELL_REG		= 0x09,
	MAX1720X_CURRENT_REG		= 0x0A,
	MAX1720X_AVGCURRENT_REG		= 0x0B,
	MAX1720X_REMCAP_REG		= 0x0F,

	MAX1720X_FULLCAPREP_REG		= 0x10,
	MAX1720X_TTE_REG		= 0X11,
	MAX1720X_QRTABLE00_REG		= 0x12,
	MAX1720X_FULLSOCTHR_REG		= 0x13,
	MAX1720X_CYCLES_REG		= 0x17,
	MAX1720X_DESIGNCAP_REG		= 0x18,
	MAX1720X_AVGVCELL_REG		= 0x19,
	MAX1720X_MAXMINVOLT_REG		= 0x1B,
	MAX1720X_CONFIG_REG		= 0x1D,
	MAX1720X_ICHGTERM_REG		= 0x1E,

	MAX1720X_VERSION_REG		= 0x21,
	MAX1720X_QRTABLE10_REG		= 0x22,
	MAX1720X_FULLCAPNOM_REG		= 0x23,
	MAX1720X_LEARNCFG_REG		= 0x28,
	MAX1720X_RELAXCFG_REG		= 0x2A,
	MAX1720X_TGAIN_REG		= 0x2C,
	MAX1720X_TOFF_REG		= 0x2D,

	MAX1720X_QRTABLE20_REG		= 0x32,
	MAX1720X_RCOMP0_REG		= 0x38,
	MAX1720X_TEMPCO_REG		= 0x39,
	MAX1720X_VEMPTY_REG		= 0x3A,
	MAX1720X_FSTAT_REG		= 0x3D,

	MAX1720X_QRTABLE30_REG		= 0x42,
	MAX1720X_DQACC_REG		= 0x45,
	MAX1720X_DPACC_REG		= 0x46,
	MAX1720X_VFREMCAP_REG		= 0x4A,
	MAX1720X_QH_REG			= 0x4D,

	MAX1720X_STATUS2_REG		= 0xB0,
	MAX1720X_IALRTTH_REG		= 0xB4,
	MAX1720X_VSHDNCFG_REG		= 0xB8,
	MAX1720X_AGEFORECAST_REG	= 0xB9,
	MAX1720X_HIBCFG_REG		= 0xBA,
	MAX1720X_CONFIG2_REG		= 0xBB,
	MAX1720X_VRIPPLE_REG		= 0xBC,
	MAX1720X_PACKCFG_REG		= 0xBD,
	MAX1720X_TIMERH_REG		= 0xBE,

	MAX1720X_AVGCELL4_REG		= 0xD1,
	MAX1720X_AVGCELL3_REG		= 0xD2,
	MAX1720X_AVGCELL2_REG		= 0xD3,
	MAX1720X_AVGCELL1_REG		= 0xD4,
	MAX1720X_CELL4_REG		= 0xD5,
	MAX1720X_CELL3_REG		= 0xD6,
	MAX1720X_CELL2_REG		= 0xD7,
	MAX1720X_CELL1_REG		= 0xD8,
	MAX1720X_CELLX_REG		= 0xD9,
	MAX1720X_BATT_REG		= 0xDA,
	MAX1720X_ATQRESIDUAL_REG	= 0xDC,
	MAX1720X_ATTTE_REG		= 0xDD,
	MAX1720X_ATAVSOC_REG		= 0xDE,
	MAX1720X_ATAVCAP_REG		= 0xDF,

	MAX1720X_VFOCV_REG		= 0xFB,
	MAX1720X_VFSOC_REG		= 0xFF,
    
    MAX1720X_REG_MFG_STR	= 0x1CC,
    MAX1720X_REG_DEV_STR	= 0x1DB,
    MAX1720X_REG_SER_HEX	= 0x1D8,
    
    MAX1720X_REG_NRSENSE	= 0x1CF
};

static enum power_supply_property max1720x_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX,
	POWER_SUPPLY_PROP_CHARGE_FULL,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
    /* strings */
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

enum chip_id {
	ID_MAX17201,
	ID_MAX17205,
};

struct max1720x_priv {
	struct i2c_client		*client;
	struct device			*dev;
	struct regmap			*regmap;
	struct power_supply		*battery;
	struct max1720x_platform_data	*pdata;
	struct work_struct		init_worker;
	struct attribute_group		*attr_grp;
    
    char device_name[32];
	char manufacturer_name[32];
    char serial_number[32];
};

static inline int max1720x_lsb_to_uvolts(struct max1720x_priv *priv, int lsb)
{
	return lsb * 625 / 8; /* 78.125uV per bit */
}

static int max1720x_raw_current_to_uamps(struct max1720x_priv *priv, u32 curr)
{
	int res = curr;

	/* Negative */
	if (res & 0x8000)
		res |= 0xFFFF0000;

	res *= 1562500 / (priv->pdata->rsense * 1000);
	return res;
}

int max1720x_get_charging_status(struct max1720x_priv *priv)
{
    union power_supply_propval current_now;
    u32 val;
    
    power_supply_get_property(priv->battery, POWER_SUPPLY_PROP_CURRENT_NOW, &current_now);
    regmap_read(priv->regmap, MAX1720X_ICHGTERM_REG, &val);
    if(current_now.intval > ((int32_t)(val / 8 * 1000))) // IChgTerm * 0.125
        return POWER_SUPPLY_STATUS_CHARGING;
    else {
        if(power_supply_am_i_supplied(priv->battery)) {
            regmap_read(priv->regmap, MAX1720X_FSTAT_REG, &val);
            if(val & MAX1720X_FSTAT_FQ)
                return POWER_SUPPLY_STATUS_FULL;
            else
                return POWER_SUPPLY_STATUS_NOT_CHARGING;
        } else
            return POWER_SUPPLY_STATUS_DISCHARGING;
    }
}

static ssize_t max1720x_log_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct max1720x_priv *priv = dev_get_drvdata(dev);
	int rc = 0, reg = 0;
	u32 val = 0;

	for (reg = 0; reg < 0xE0; reg++) {
		regmap_read(priv->regmap, reg, &val);
		rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "0x%04X,", val);

		if (reg == 0x4F)
			reg += 0x60;

		if (reg == 0xBF)
			reg += 0x10;
	}

	rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "\n");

	return rc;
}

static ssize_t max1720x_nvmem_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max1720x_priv *priv = dev_get_drvdata(dev);
	int rc = 0, reg = 0;
	u32 val = 0;
	int ret;
	int i;

	priv->client->addr = NONVOLATILE_DATA_I2C_ADDR;

	for (reg = 0x80; reg < 0xDF; reg += 16) {
		rc += snprintf(buf+rc, PAGE_SIZE-rc, "Page %02Xh: ",
			       (reg + 0x100) >> 4);
		for (i = 0; i < 16; i++) {
			ret = regmap_read(priv->regmap, reg + i, &val);
			if (ret) {
				dev_err(dev, "NV memory reading failed (%d)\n",
					ret);
				return 0;
			}
			rc += snprintf(buf+rc, PAGE_SIZE-rc, "0x%04X ", val);
		}
		rc += snprintf(buf+rc, PAGE_SIZE-rc, "\n");

	}

	priv->client->addr = MODELGAUGE_DATA_I2C_ADDR;

	return rc;
}

static ssize_t max1720x_atrate_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max1720x_priv *priv = dev_get_drvdata(dev);
	u32 val = 0;
	int ret;

	ret = regmap_read(priv->regmap, MAX1720X_ATRATE_REG, &val);
	if (ret) {
		return 0;
	}

	return sprintf(buf, "%d", (short)val);
}

static ssize_t max1720x_atrate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1720x_priv *priv = dev_get_drvdata(dev);
	s32 val = 0;
	int ret;

	if (kstrtos32(buf, 0, &val))
		return -EINVAL;

	ret = regmap_write(priv->regmap, MAX1720X_ATRATE_REG, val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t max1720x_attte_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max1720x_priv *priv = dev_get_drvdata(dev);
	u32 val = 0;
	int ret;

	ret = regmap_read(priv->regmap, MAX1720X_ATTTE_REG, &val);
	if (ret) {
		return 0;
	}

	return sprintf(buf, "%d", (short)val);
}

static DEVICE_ATTR(log, S_IRUGO, max1720x_log_show, NULL);
static DEVICE_ATTR(nvmem, S_IRUGO, max1720x_nvmem_show, NULL);
static DEVICE_ATTR(atrate, S_IRUGO | S_IWUSR, max1720x_atrate_show,
		   max1720x_atrate_store);
static DEVICE_ATTR(attte, S_IRUGO, max1720x_attte_show, NULL);

static struct attribute *max1720x_attr[] = {
	&dev_attr_log.attr,
	&dev_attr_nvmem.attr,
	&dev_attr_atrate.attr,
	&dev_attr_attte.attr,
	NULL
};

static struct attribute_group max1720x_attr_group = {
	.attrs = max1720x_attr,
};

static int max1720x_get_temperature(struct max1720x_priv *priv, int *temp)
{
	int ret;
	u32 data;
	struct regmap *map = priv->regmap;

	ret = regmap_read(map, MAX1720X_TEMP_REG, &data);
	if (ret < 0)
		return ret;

	*temp = data;
	/* The value is signed. */
	if (*temp & 0x8000)
		*temp |= 0xFFFF0000;

	/* The value is converted into centigrade scale */
	/* Units of LSB = 1 / 256 degree Celsius */
	*temp >>= 8;

	return 0;
}

static int max1720x_set_temp_lower_limit(struct max1720x_priv *priv,
						int temp)
{
	int ret;
	u32 data;
	struct regmap *map = priv->regmap;

	ret = regmap_read(map, MAX1720X_TALRTTH_REG, &data);
	if (ret < 0)
		return ret;

	data &= 0xFF00;
	data |= (temp & 0xFF);

	ret = regmap_write(map, MAX1720X_TALRTTH_REG, data);
	if (ret < 0)
		return ret;

	return 0;
}

static int max1720x_get_temperature_lower_limit(struct max1720x_priv *priv,
						int *temp)
{
	int ret;
	u32 data;
	struct regmap *map = priv->regmap;

	ret = regmap_read(map, MAX1720X_TALRTTH_REG, &data);
	if (ret < 0)
		return ret;

	*temp = (int)(s8)(data & 0xFF);

	return 0;
}

static int max1720x_set_temp_upper_limit(struct max1720x_priv *priv,
						int temp)
{
	int ret;
	u32 data;
	struct regmap *map = priv->regmap;

	ret = regmap_read(map, MAX1720X_TALRTTH_REG, &data);
	if (ret < 0)
		return ret;

	data &= 0xFF;
	data |= ((temp << 8) & 0xFF00);

	ret = regmap_write(map, MAX1720X_TALRTTH_REG, data);
	if (ret < 0)
		return ret;

	return 0;
}

static int max1720x_get_temperature_upper_limit(struct max1720x_priv *priv,
						int *temp)
{
	int ret;
	u32 data;
	struct regmap *map = priv->regmap;

	ret = regmap_read(map, MAX1720X_TALRTTH_REG, &data);
	if (ret < 0)
		return ret;

	*temp = (int)(s8)(data >> 8);

	return 0;
}

static int max1720x_get_battery_health(struct max1720x_priv *priv, int *health)
{
	int temp, vavg, vbatt, ret;
	u32 val;

	ret = regmap_read(priv->regmap, MAX1720X_AVGVCELL_REG, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vavg = max1720x_lsb_to_uvolts(priv, val);
	/* Convert to millivolts */
	vavg /= 1000;

	ret = regmap_read(priv->regmap, MAX1720X_VCELL_REG, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vbatt = max1720x_lsb_to_uvolts(priv, val);
	/* Convert to millivolts */
	vbatt /= 1000;

	if (vavg < priv->pdata->volt_min) {
		*health = POWER_SUPPLY_HEALTH_DEAD;
		goto out;
	}

	if (vbatt > priv->pdata->volt_max + MAX1720X_VMAX_TOLERANCE) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		goto out;
	}

	ret = max1720x_get_temperature(priv, &temp);
	if (ret < 0)
		goto health_error;

	if (temp <= priv->pdata->temp_min) {
		*health = POWER_SUPPLY_HEALTH_COLD;
		goto out;
	}

	if (temp >= priv->pdata->temp_max) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		goto out;
	}

	*health = POWER_SUPPLY_HEALTH_GOOD;

out:
	return 0;

health_error:
	return ret;
}

static int max1720x_get_min_capacity_alert_th(struct max1720x_priv *priv,
					      unsigned int *th)
{
	int ret;
	struct regmap *map = priv->regmap;

	ret = regmap_read(map, MAX1720X_SALRTTH_REG, th);
	if (ret < 0)
		return ret;

	*th &= 0xFF;

	return 0;
}

static int max1720x_set_min_capacity_alert_th(struct max1720x_priv *priv,
					      unsigned int th)
{
	int ret;
	unsigned int data;
	struct regmap *map = priv->regmap;

	ret = regmap_read(map, MAX1720X_SALRTTH_REG, &data);
	if (ret < 0)
		return ret;

	data &= 0xFF00;
	data |= (th & 0xFF);

	ret = regmap_write(map, MAX1720X_SALRTTH_REG, data);
	if (ret < 0)
		return ret;

	return 0;
}

static int max1720x_get_max_capacity_alert_th(struct max1720x_priv *priv,
					      unsigned int *th)
{
	int ret;
	struct regmap *map = priv->regmap;

	ret = regmap_read(map, MAX1720X_SALRTTH_REG, th);
	if (ret < 0)
		return ret;

	*th >>= 8;

	return 0;
}

static int max1720x_set_max_capacity_alert_th(struct max1720x_priv *priv,
					      unsigned int th)
{
	int ret;
	unsigned int data;
	struct regmap *map = priv->regmap;

	ret = regmap_read(map, MAX1720X_SALRTTH_REG, &data);
	if (ret < 0)
		return ret;

	data &= 0xFF;
	data |= ((th & 0xFF) << 8);

	ret = regmap_write(map, MAX1720X_SALRTTH_REG, data);
	if (ret < 0)
		return ret;

	return 0;
}

static int max1720x_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct max1720x_priv *priv = power_supply_get_drvdata(psy);
	struct regmap *regmap = priv->regmap;
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(regmap, MAX1720X_STATUS_REG, &reg);
		if (ret < 0)
			return ret;
		if (reg & MAX1720X_STATUS_BST)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = regmap_read(regmap, MAX1720X_CYCLES_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = reg;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = regmap_read(regmap, MAX1720X_MAXMINVOLT_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = reg >> 8;
		val->intval *= 20000; /* Units of LSB = 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = regmap_read(regmap, MAX1720X_VEMPTY_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = reg >> 7;
		val->intval *= 10000; /* Units of LSB = 10mV */
		break;
	case POWER_SUPPLY_PROP_STATUS:
			val->intval = max1720x_get_charging_status(priv);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(regmap, MAX1720X_VCELL_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = max1720x_lsb_to_uvolts(priv, reg);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = regmap_read(regmap, MAX1720X_AVGVCELL_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = max1720x_lsb_to_uvolts(priv, reg);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = regmap_read(regmap, MAX1720X_VFOCV_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = max1720x_lsb_to_uvolts(priv, reg);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = regmap_read(regmap, MAX1720X_REPSOC_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = reg >> 8; /* RepSOC LSB: 1/256 % */
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		ret = max1720x_get_min_capacity_alert_th(priv, &val->intval);
		if (ret < 0)
			return ret;

		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		ret = max1720x_get_max_capacity_alert_th(priv, &val->intval);
		if (ret < 0)
			return ret;

		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = regmap_read(regmap, MAX1720X_FULLCAPREP_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = (reg * 1000) >> 1; /* FullCAPRep LSB: 0.5 mAh */
		break;
        
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = regmap_read(regmap, MAX1720X_FULLCAPREP_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = (reg * 1000) >> 1; /* FullCAPRep LSB: 0.5 mAh */
		break;
        
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = regmap_read(regmap, MAX1720X_QH_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = ((s16)reg * 1000) >> 1; /* QH LSB: 0.5 mAh */
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = regmap_read(regmap, MAX1720X_REPCAP_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = (reg * 1000) >> 1; /* RepCAP LSB: 0.5 mAh */
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = max1720x_get_temperature(priv, &val->intval);
		if (ret < 0)
			return ret;

		val->intval *= 10; /* Convert 1DegreeC LSB to 0.1DegreeC LSB */
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = max1720x_get_temperature_lower_limit(priv, &val->intval);
		if (ret < 0)
			return ret;

		val->intval *= 10; /* Convert 1DegreeC LSB to 0.1DegreeC LSB */
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = max1720x_get_temperature_upper_limit(priv, &val->intval);
		if (ret < 0)
			return ret;

		val->intval *= 10; /* Convert 1DegreeC LSB to 0.1DegreeC LSB */
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max1720x_get_battery_health(priv, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = regmap_read(regmap, MAX1720X_CURRENT_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = max1720x_raw_current_to_uamps(priv, reg);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = regmap_read(regmap, MAX1720X_AVGCURRENT_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = max1720x_raw_current_to_uamps(priv, reg);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = regmap_read(regmap, MAX1720X_TTE_REG, &reg);
		if (ret < 0)
			return ret;

		val->intval = (reg * 45) >> 3; /* TTE LSB: 5.625 sec */
		break;
    case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = priv->device_name;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = priv->manufacturer_name;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = priv->serial_number;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max1720x_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct max1720x_priv *priv = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = max1720x_set_temp_lower_limit(priv, val->intval / 10);
		if (ret < 0)
			dev_err(priv->dev, "temp alert min set fail:%d\n",
				ret);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = max1720x_set_temp_upper_limit(priv, val->intval / 10);
		if (ret < 0)
			dev_err(priv->dev, "temp alert max set fail:%d\n",
				ret);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		ret = max1720x_set_min_capacity_alert_th(priv, val->intval);
		if (ret < 0)
			dev_err(priv->dev, "capacity alert min set fail:%d\n",
				ret);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		ret = max1720x_set_max_capacity_alert_th(priv, val->intval);
		if (ret < 0)
			dev_err(priv->dev, "capacity alert max set fail:%d\n",
				ret);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int max1720x_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static irqreturn_t max1720x_irq_handler(int id, void *dev)
{
	struct max1720x_priv *priv = dev;
	u32 val;

	/* Check alert type */
	regmap_read(priv->regmap, MAX1720X_STATUS_REG, &val);

	if (val & MAX1720X_STATUS_SOC_MAX_ALRT)
		dev_info(priv->dev, "Alert: SOC MAX!\n");
	if (val & MAX1720X_STATUS_SOC_MIN_ALRT)
		dev_info(priv->dev, "Alert: SOC MIN!\n");
	if (val & MAX1720X_STATUS_TEMP_MAX_ALRT)
		dev_info(priv->dev, "Alert: TEMP MAX!\n");
	if (val & MAX1720X_STATUS_TEMP_MIN_ALRT)
		dev_info(priv->dev, "Alert: TEMP MIN!\n");
	if (val & MAX1720X_STATUS_VOLT_MAX_ALRT)
		dev_info(priv->dev, "Alert: VOLT MAX!\n");
	if (val & MAX1720X_STATUS_VOLT_MIN_ALRT)
		dev_info(priv->dev, "Alert: VOLT MIN!\n");
	if (val & MAX1720X_STATUS_CURR_MAX_ALRT)
		dev_info(priv->dev, "Alert: CURR MAX!\n");
	if (val & MAX1720X_STATUS_CURR_MIN_ALRT)
		dev_info(priv->dev, "Alert: CURR MIN!\n");

	/* Clear alerts */
	regmap_write(priv->regmap, MAX1720X_STATUS_REG,
				  val & MAX1720X_STATUS_ALRT_CLR_MASK);

	power_supply_changed(priv->battery);

	return IRQ_HANDLED;
}

static void max1720x_set_alert_thresholds(struct max1720x_priv *priv)
{
	struct max1720x_platform_data *pdata = priv->pdata;
	struct regmap *regmap = priv->regmap;
	u32 val;

	/* Set VAlrtTh */
	val = (pdata->volt_min / 20);
	val |= ((pdata->volt_max / 20) << 8);
	regmap_write(regmap, MAX1720X_VALRTTH_REG, val);

	/* Set TAlrtTh */
	val = pdata->temp_min & 0xFF;
	val |= ((pdata->temp_max & 0xFF) << 8);
	regmap_write(regmap, MAX1720X_TALRTTH_REG, val);

	/* Set SAlrtTh */
	val = pdata->soc_min;
	val |= (pdata->soc_max << 8);
	regmap_write(regmap, MAX1720X_SALRTTH_REG, val);

	/* Set IAlrtTh */
	val = (pdata->curr_min * pdata->rsense / 400) & 0xFF;
	val |= (((pdata->curr_max * pdata->rsense / 400) & 0xFF) << 8);
	regmap_write(regmap, MAX1720X_IALRTTH_REG, val);
}

static int max1720x_init(struct max1720x_priv *priv)
{
	struct regmap *regmap = priv->regmap;
	int ret;
	unsigned int reg;
	u32 fgrev;

	ret = regmap_read(regmap, MAX1720X_VERSION_REG, &fgrev);
	if (ret < 0)
		return ret;

	dev_info(priv->dev, "IC Version: 0x%04x\n", fgrev);

	/* Optional step - alert threshold initialization */
	max1720x_set_alert_thresholds(priv);

	/* Clear Status.POR */
	ret = regmap_read(regmap, MAX1720X_STATUS_REG, &reg);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, MAX1720X_STATUS_REG,
			   reg & ~MAX1720X_STATUS_POR);
	if (ret < 0)
		return ret;

	return 0;
}

static void max1720x_init_worker(struct work_struct *work)
{
	struct max1720x_priv *priv = container_of(work,
			struct max1720x_priv,
			init_worker);

	max1720x_init(priv);
}

static struct max1720x_platform_data *max1720x_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct max1720x_platform_data *pdata;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	ret = of_property_read_u32(np, "talrt-min", &pdata->temp_min);
	if (ret)
		pdata->temp_min = -128; /* DegreeC */ /* Disable alert */

	ret = of_property_read_u32(np, "talrt-max", &pdata->temp_max);
	if (ret)
		pdata->temp_max = 127; /* DegreeC */ /* Disable alert */

	ret = of_property_read_u32(np, "valrt-min", &pdata->volt_min);
	if (ret)
		pdata->volt_min = 0; /* mV */ /* Disable alert */

	ret = of_property_read_u32(np, "valrt-max", &pdata->volt_max);
	if (ret)
		pdata->volt_max = 5100; /* mV */ /* Disable alert */

	ret = of_property_read_u32(np, "ialrt-min", &pdata->curr_min);
	if (ret)
		pdata->curr_min = -5120; /* mA */ /* Disable alert */

	ret = of_property_read_u32(np, "ialrt-max", &pdata->curr_max);
	if (ret)
		pdata->curr_max = 5080; /* mA */ /* Disable alert */

	ret = of_property_read_u32(np, "salrt-min", &pdata->soc_min);
	if (ret)
		pdata->soc_min = 0; /* Percent */ /* Disable alert */

	ret = of_property_read_u32(np, "salrt-max", &pdata->soc_max);
	if (ret)
		pdata->soc_max = 255; /* Percent */ /* Disable alert */

	return pdata;
}

static const struct regmap_config max1720x_regmap = {
	.reg_bits		= 8,
	.val_bits		= 16,
	.val_format_endian	= REGMAP_ENDIAN_NATIVE,
};


static char power_supply_name[64] = "max1720x_battery";

static struct power_supply_desc max1720x_fg_desc = {
	.name			= power_supply_name,
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= max1720x_battery_props,
	.num_properties		= ARRAY_SIZE(max1720x_battery_props),
	.get_property		= max1720x_get_property,
	.set_property		= max1720x_set_property,
	.property_is_writeable	= max1720x_property_is_writeable,
};

static void max1720x_read_strings(struct max1720x_priv *priv)
{
    int i;
    int ret = 0;
    union {
        uint64_t all;
        uint16_t part[4];
    } ser;
    memset(priv->device_name, 0, sizeof(priv->device_name));
    memset(priv->manufacturer_name, 0, sizeof(priv->manufacturer_name));
    memset(&priv->serial_number, 0, sizeof(priv->serial_number));
    
    priv->client->addr = NONVOLATILE_DATA_I2C_ADDR;
    
    for(i = 0; i < 3; i++)
    {
        ret = regmap_read(priv->regmap, MAX1720X_REG_MFG_STR + i - 0x100, (u32*)(&priv->manufacturer_name[i * 2]));
        if(ret < 0)
            break;
    }
    
    if(ret || !strlen(priv->manufacturer_name)) {
        sprintf(priv->manufacturer_name, "UNKNWN");
    }

    ret = 0;
    for(i = 0; i < 5; i++)
    {
        ret = regmap_read(priv->regmap, MAX1720X_REG_DEV_STR + i - 0x100, (u32*)(&priv->device_name[i * 2]));
        if(ret < 0)
            break;
    }

    if(ret || !strlen(priv->device_name)) {
        sprintf(priv->device_name, "MAX1720x");
    }
    
    ret = 0;
    for(i = 0; i < 3; i++)
    {
        ret = regmap_read(priv->regmap, MAX1720X_REG_SER_HEX + i - 0x100, (u32*)&ser.part[i]);
        if(ret < 0)
            break;
    }
        
    snprintf(priv->serial_number, sizeof(priv->serial_number), "%llu", ser.all);
    
	priv->client->addr = MODELGAUGE_DATA_I2C_ADDR;
}

static int max1720x_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	/*struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);*/
	struct max1720x_priv *priv;
	struct power_supply_config psy_cfg = {};
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (client->dev.of_node)
		priv->pdata = max1720x_parse_dt(&client->dev);
	else
		priv->pdata = client->dev.platform_data;

	priv->dev = &client->dev;

	i2c_set_clientdata(client, priv);

	priv->client = client;
	priv->regmap = devm_regmap_init_i2c(client, &max1720x_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	INIT_WORK(&priv->init_worker, max1720x_init_worker);
	schedule_work(&priv->init_worker);
    
    max1720x_read_strings(priv);
    
    sprintf(power_supply_name, "%s-%s", priv->manufacturer_name, priv->device_name);
    
	priv->client->addr = NONVOLATILE_DATA_I2C_ADDR;
	if (regmap_read(priv->regmap, MAX1720X_REG_NRSENSE, &priv->pdata->rsense)) {
		dev_err(&client->dev, "Can't read RSense. Hardware error.\n");
		ret = -ENODEV;
        goto err_supply;
	}
	priv->client->addr = MODELGAUGE_DATA_I2C_ADDR;
	
	priv->pdata->rsense /= 100; // Convert to milliOhms from tens of microOhms

	psy_cfg.drv_data = priv;
    psy_cfg.of_node = client->dev.of_node;
	priv->battery = power_supply_register(&client->dev,
					      &max1720x_fg_desc, &psy_cfg);
	if (IS_ERR(priv->battery)) {
		ret = PTR_ERR(priv->battery);
		dev_err(&client->dev, "failed to register battery: %d\n", ret);
		goto err_supply;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(priv->dev, client->irq,
						NULL,
						max1720x_irq_handler,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						priv->battery->desc->name,
						priv);
		if (ret) {
			dev_err(priv->dev, "Failed to request irq %d\n",
				client->irq);
			goto err_irq;
		} else {
			regmap_update_bits(priv->regmap, MAX1720X_CONFIG_REG,
					   MAX1720X_CONFIG_ALRT_EN,
					   MAX1720X_CONFIG_ALRT_EN);
		}
	}

	/* Create max1720x sysfs attributes */
	priv->attr_grp = &max1720x_attr_group;
	ret = sysfs_create_group(&priv->dev->kobj, priv->attr_grp);
	if (ret) {
		dev_err(priv->dev, "Failed to create attribute group [%d]\n",
			ret);
		priv->attr_grp = NULL;
		goto err_attr;
	}

	return 0;
err_attr:
	sysfs_remove_group(&priv->dev->kobj, priv->attr_grp);
err_irq:
	power_supply_unregister(priv->battery);
err_supply:
	cancel_work_sync(&priv->init_worker);
	return ret;
}

static int max1720x_remove(struct i2c_client *client)
{
	struct max1720x_priv *priv = i2c_get_clientdata(client);

	cancel_work_sync(&priv->init_worker);
	sysfs_remove_group(&priv->dev->kobj, priv->attr_grp);
	power_supply_unregister(priv->battery);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max1720x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq) {
		disable_irq(client->irq);
		enable_irq_wake(client->irq);
	}

	return 0;
}

static int max1720x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq) {
		disable_irq_wake(client->irq);
		enable_irq(client->irq);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(max1720x_pm_ops, max1720x_suspend, max1720x_resume);
#define MAX1720X_PM_OPS (&max1720x_pm_ops)
#else
#define MAX1720X_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id max1720x_match[] = {
	{ .compatible = "maxim,max17201", },
	{ .compatible = "maxim,max17205", },
	{ },
};
MODULE_DEVICE_TABLE(of, max1720x_match);
#endif

static const struct i2c_device_id max1720x_id[] = {
	{ "max17201", ID_MAX17201 },
	{ "max17205", ID_MAX17205 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max1720x_id);

static struct i2c_driver max1720x_i2c_driver = {
	.driver = {
		.name		= DRV_NAME,
		.of_match_table	= of_match_ptr(max1720x_match),
		.pm		= MAX1720X_PM_OPS,
	},
	.probe		= max1720x_probe,
	.remove		= max1720x_remove,
	.id_table	= max1720x_id,
};
module_i2c_driver(max1720x_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kerem Sahin <kerem.sahin@maximintegrated.com>");
MODULE_AUTHOR("Mahir Ozturk <mahir.ozturk@maximintegrated.com>");
MODULE_DESCRIPTION("Maxim Max17201/Max17205 Fuel Gauge driver");
