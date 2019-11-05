// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ADS1000/ADS1100 12-16-bit ADC
 *
 * Copyright (C) 2018 T-platforms JSC (fancer.lancer@gmail.com)
 *
 * Based on the ads1015 driver by Dirk Eibach.
 *
 * Datasheet available at: http://focus.ti.com/lit/ds/symlink/ads1000.pdf
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/platform_data/ads1000.h>

/* Data rates scale table */
static const unsigned int scale_table[4] = {
	2048, 8192, 16384, 32768
};

/* Minimal data rates in samples per second */
static const unsigned int data_rate_table[4] = {
	100, 25, 12, 5
};

/*  */

#define ADS1000_DEFAULT_PGA 0
#define ADS1000_DEFAULT_DATA_RATE 0
#define ADS1000_DEFAULT_R1_DIVIDER 0
#define ADS1000_DEFAULT_R2_DIVIDER 0

enum ads1000_chips {
	ads1000,
	ads1100,
};

struct ads1000 {
	struct device *hwmon_dev;
	struct mutex update_lock;
	struct i2c_client *client;
	struct ads1000_platform_data data;
	enum ads1000_chips id;
};

static inline int ads1000_enable_vdd(struct ads1000 *priv)
{
	return regulator_enable(priv->data.vdd);
}

static inline int ads1000_get_vdd(struct ads1000 *priv)
{
	return regulator_get_voltage(priv->data.vdd);
}

static int ads1000_read_adc(struct ads1000 *priv)
{
	struct i2c_client *client = priv->client;
	unsigned int delay_ms;
	u8 data[3] = {0};
	int res;

	mutex_lock(&priv->update_lock);

	delay_ms = DIV_ROUND_UP(1000, data_rate_table[priv->data.data_rate]);

	/* setup and start single conversion */
	data[2] |= (1 << 7) | (1 << 4);
	data[2] |= priv->data.pga;
	data[2] |= priv->data.data_rate << 2;

	res = i2c_master_send(client, &data[2], 1);
	if (res < 0)
		goto err_unlock;

	/* wait until conversion finished */
	msleep(delay_ms);
	res = i2c_master_recv(client, data, 3);
	if (res < 0)
		goto err_unlock;

	if (data[2] & (1 << 7)) {
		res = -EIO;
		goto err_unlock;
	}

	res = ((u16)data[0] << 8) | data[1];

err_unlock:
	mutex_unlock(&priv->update_lock);

	return res;
}

static int ads1000_reg_to_mv(struct ads1000 *priv, s16 reg)
{
	unsigned int *divider = priv->data.divider;
	int voltage = ads1000_get_vdd(priv);
	int gain = 1 << priv->data.pga;
	int c = 0;

	voltage = reg*DIV_ROUND_CLOSEST(voltage, 1000);
	gain = gain*scale_table[priv->data.data_rate];
	voltage = DIV_ROUND_CLOSEST(voltage, gain);

	if (divider[0] && divider[1]) {
		c = divider[0]*voltage;
		c = DIV_ROUND_CLOSEST(c, (int)divider[1]);
	}

	return voltage + c;
}

static ssize_t show_in(struct device *dev, struct device_attribute *da,
		       char *buf)
{
	struct ads1000 *priv = dev_get_drvdata(dev);
	int res;

	res = ads1000_read_adc(priv);
	if (res < 0)
		return res;

	return sprintf(buf, "%d\n", ads1000_reg_to_mv(priv, res));
}

static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_in, NULL, 0);

static struct attribute *ads1000_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(ads1000);

static struct ads1000 *ads1000_create_priv(struct i2c_client *client,
					   const struct i2c_device_id *id)
{
	struct ads1000 *priv;

	priv = devm_kzalloc(&client->dev, sizeof(struct ads1000),
			    GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	if (client->dev.of_node)
		priv->id = (enum ads1000_chips)
			of_device_get_match_data(&client->dev);
	else
		priv->id = id->driver_data;

	i2c_set_clientdata(client, priv);
	priv->client = client;
	mutex_init(&priv->update_lock);

	return priv;
}

#ifdef CONFIG_OF
static int ads1000_get_config_of(struct ads1000 *priv)
{
	struct i2c_client *client = priv->client;
	struct device_node *node = client->dev.of_node;
	u32 divider[2];
	u32 val;

	if (!node)
		return -EINVAL;

	if (!of_property_read_u32(node, "ti,gain", &val))
		priv->data.pga = val;

	if (!of_property_read_u32(node, "ti,datarate", &val))
		priv->data.data_rate = val;

	if (!of_property_read_u32_array(node, "ti,voltage-divider",
					divider, 2)) {
		priv->data.divider[0] = divider[0];
		priv->data.divider[1] = divider[1];
	}

	priv->data.vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(priv->data.vdd))
		return PTR_ERR(priv->data.vdd);

	return 0;
}
#endif

static int ads1000_get_config(struct ads1000 *priv)
{
	struct i2c_client *client = priv->client;
	struct ads1000_platform_data *pdata = dev_get_platdata(&client->dev);

	priv->data.pga = ADS1000_DEFAULT_PGA;
	priv->data.data_rate = ADS1000_DEFAULT_DATA_RATE;
	priv->data.divider[0] = ADS1000_DEFAULT_R1_DIVIDER;
	priv->data.divider[1] = ADS1000_DEFAULT_R2_DIVIDER;

	/* prefer platform data */
	if (pdata) {
		memcpy(&priv->data, pdata, sizeof(priv->data));
	} else {
#ifdef CONFIG_OF
		int ret;

		ret = ads1000_get_config_of(priv);
		if (ret)
			return ret;
#endif
	}

	if (!priv->data.vdd) {
		dev_err(&client->dev, "No VDD regulator\n");
		return -EINVAL;
	}

	if (priv->data.pga > 4) {
		dev_err(&client->dev, "Invalid gain, using default\n");
		priv->data.pga = ADS1000_DEFAULT_PGA;
	}

	if (priv->data.data_rate > 4) {
		dev_err(&client->dev, "Invalid datarate, using default\n");
		priv->data.data_rate = ADS1000_DEFAULT_DATA_RATE;
	}

	if (priv->id == ads1000 && priv->data.data_rate != 0) {
		dev_warn(&client->dev, "ADC data rate can be 128SPS only\n");
		priv->data.data_rate = 0;
	}

	return 0;
}

static int ads1000_set_config(struct ads1000 *priv)
{
	u8 data = 0;
	int ret;

	/* disable continuous conversion */
	data |= (1 << 4);
	data |= priv->data.pga;
	data |= priv->data.data_rate << 2;

	ret = i2c_master_send(priv->client, &data, 1);

	return ret < 0 ? ret : 0;
}

static int ads1000_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ads1000 *priv;
	int ret;

	priv = ads1000_create_priv(client, id);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	ret = ads1000_get_config(priv);
	if (ret)
		return ret;

	ret = ads1000_enable_vdd(priv);
	if (ret)
		return ret;

	ret = ads1000_set_config(priv);
	if (ret)
		return ret;

	priv->hwmon_dev = devm_hwmon_device_register_with_groups(&client->dev,
				client->name, priv, ads1000_groups);
	if (IS_ERR(priv->hwmon_dev))
		return PTR_ERR(priv->hwmon_dev);

	return 0;
}

static const struct i2c_device_id ads1000_id[] = {
	{ "ads1000",  ads1000},
	{ "ads1100",  ads1100},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads1000_id);

static const struct of_device_id ads1000_of_match[] = {
	{
		.compatible = "ti,ads1000",
		.data = (void *)ads1000
	},
	{
		.compatible = "ti,ads1100",
		.data = (void *)ads1100
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ads1000_of_match);

static struct i2c_driver ads1000_driver = {
	.driver = {
		.name = "ads1000",
		.of_match_table = of_match_ptr(ads1000_of_match),
	},
	.probe = ads1000_probe,
	.id_table = ads1000_id,
};
module_i2c_driver(ads1000_driver);

MODULE_AUTHOR("Serge Semin <fancer.lancer@gmail.com>");
MODULE_DESCRIPTION("ADS1000 driver");
MODULE_LICENSE("GPL v2");
