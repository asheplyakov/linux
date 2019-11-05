#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/sysfs.h>

enum I2C_REGS {
	R_ID1 = 0,
	R_ID2,
	R_ID3,
	R_ID4,
	R_SOFTOFF_RQ,
	R_PWROFF_RQ,
	R_PWRBTN_STATE,
	R_VERSION1,
	R_VERSION2,
	R_BOOTREASON,
	R_BOOTREASON_ARG,
	R_SCRATCH1,
	R_SCRATCH2,
	R_SCRATCH3,
	R_SCRATCH4,
	R_COUNT
};

#define BMC_ID1_VAL 0x49
#define BMC_ID2_VAL 0x54
#define BMC_ID3_VAL 0x58
#define BMC_ID4_VAL0 0x32
#define BMC_ID4_VAL1 0x2

#define POLL_DELAY 100

struct bmc_data {
	unsigned long singleton;

	struct i2c_client *client;

	struct input_dev *button;
	uint8_t button_state;

	struct task_struct *poller;

	uint8_t bmc_proto_version[3];
	uint8_t bmc_bootreason[2];
	uint8_t bmc_scratch[4];
};

static struct bmc_data priv;

static void __maybe_unused bmc_pwroff_rq(void)
{
	dev_info(&priv.client->dev, "Set Baikal-T BMC reg R_PWROFF_RQ\n");

	(void)i2c_smbus_write_byte_data(priv.client, R_PWROFF_RQ, 0x01);
}

static int bmc_pwroff_rq_poll_fn(void *data)
{
	int ret;

	while (1) {
		if (kthread_should_stop())
			break;

		ret = i2c_smbus_read_byte_data(priv.client, R_SOFTOFF_RQ);
		if (ret < 0) {
			dev_err(&priv.client->dev, "Could not read register R_SOFTOFF_RQ\n");
			return ret;
		}

		if (priv.button_state != ret) {
			dev_info(&priv.client->dev, "R_SOFTOFF_RQ key [%i]->[%i]\n",
				priv.button_state, ret);

			if (ret != 0) {
				input_event(priv.button, EV_KEY, KEY_POWER, 1);
			} else {
				input_event(priv.button, EV_KEY, KEY_POWER, 0);
			}
		        input_sync(priv.button);
		}

		priv.button_state = ret;
		msleep_interruptible(POLL_DELAY);
	}

	do_exit(1);
	return 0;
}

static int bmc_check_host(struct i2c_client *client)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EINVAL;

	if (test_and_set_bit(0, &priv.singleton)) {
		dev_err(&client->dev, "Can't have more than one BMC device\n");
		return -EBUSY;
	}

	priv.client = client;
	return 0;
}

static void bmc_clear_priv(void)
{
	clear_bit(0, &priv.singleton);
	priv.client = NULL;
}

static int bmc_validate_dev(struct i2c_client *client)
{
	const uint8_t vals[] = {BMC_ID1_VAL, BMC_ID2_VAL, BMC_ID3_VAL};
	const uint8_t regs[] = {R_ID1, R_ID2, R_ID3};
	int ret, idx;

	for (idx = 0; idx < ARRAY_SIZE(regs); idx++) {
		ret = i2c_smbus_read_byte_data(client, regs[idx]);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %i\n", regs[idx]);
			return ret;
		}
		if (ret != vals[idx]) {
			dev_err(&client->dev, "Unexpected register [0x%02hhx] value 0x%02hhx != 0x%02hhx\n",
				regs[idx], vals[idx], ret);
			return -EINVAL;
		}

	}

	ret = i2c_smbus_read_byte_data(client, R_ID4);
	if (ret < 0) {
		dev_err(&client->dev, "Could not read register R_ID4\n");
		return ret;
	}

	if (ret == BMC_ID4_VAL0) {
		priv.bmc_proto_version[0] = 0;
	} else if (ret == BMC_ID4_VAL1) {
		priv.bmc_proto_version[0] = 2;

		ret = i2c_smbus_read_byte_data(client, R_VERSION1);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register R_VERSION1\n");
			return ret;
		}
		priv.bmc_proto_version[1] = ret;

		ret = i2c_smbus_read_byte_data(client, R_VERSION2);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register R_VERSION2\n");
			return ret;
		}
		priv.bmc_proto_version[2] = ret;

		ret = i2c_smbus_read_byte_data(client, R_BOOTREASON);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register R_BOOTREASON\n");
			return ret;
		}
		priv.bmc_bootreason[0] = ret;

		ret = i2c_smbus_read_byte_data(client, R_BOOTREASON_ARG);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register R_BOOTREASON_ARG\n");
			return ret;
		}
		priv.bmc_bootreason[1] = ret;

		for (idx = R_SCRATCH1; idx <= R_SCRATCH4; idx++) {
			ret = i2c_smbus_read_byte_data(client, idx);
			if (ret < 0) {
				dev_err(&client->dev, "Could not read register R_SCRATCH%x\n",
					idx - R_SCRATCH1);
				return ret;
			}
			priv.bmc_scratch[idx - R_SCRATCH1] = ret;
		}
	} else {
		dev_err(&client->dev, "Bad value [0x%02x] in register 0x%02x\n", ret, R_ID4);
		return -EINVAL;
	}

	dev_info(&client->dev, "Found valid Baikal-T BMC\n");
	dev_info(&client->dev, "Baikal-T BMC bootreason[0]->%i\n", ret);
	dev_info(&client->dev, "Baikal-T BMC bootreason[1]->%i\n", ret);

	return 0;
}

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i.%i.%i\n", priv.bmc_proto_version[0],
		priv.bmc_proto_version[1], priv.bmc_proto_version[2]);
}
static struct kobj_attribute version_attribute =
	__ATTR(version, 0664, version_show, NULL);

static ssize_t bootreason_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", (priv.bmc_bootreason[0] | (priv.bmc_bootreason[1] << 8)));
}
static struct kobj_attribute bootreason_attribute =
	__ATTR(bootreason, 0664, bootreason_show, NULL);

static ssize_t scratch_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", (priv.bmc_scratch[0] | (priv.bmc_scratch[1] << 8) |
				     (priv.bmc_scratch[2] << 16) | (priv.bmc_scratch[3]<<24)));
}
static struct kobj_attribute scratch_attribute =
	__ATTR(scratch, 0664, scratch_show, NULL);

static struct attribute *bmc_attrs[] = {
	&version_attribute.attr,
	&bootreason_attribute.attr,
	&scratch_attribute.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bmc);

static int bmc_init_button(void)
{
	int ret;

	priv.button = input_allocate_device();
	if (!priv.button) {
		dev_err(&priv.client->dev, "No memory for input dev\n");
		return -ENOMEM;
	}

	priv.button->id.bustype = BUS_I2C;
	priv.button->dev.parent = &priv.client->dev;
	priv.button->name = "BMC input dev";
	priv.button->phys = "bmc-input0";
	priv.button->evbit[0] = BIT_MASK(EV_KEY);
	priv.button->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	ret = input_register_device(priv.button);
	if (ret) {
		dev_err(&priv.client->dev, "Failed to register input device\n");
		input_free_device(priv.button);
	}

	return ret;
}

static void bmc_clear_button(void)
{
	input_unregister_device(priv.button);
	input_free_device(priv.button);
}

static int bmc_init_pwroff(void)
{
	priv.poller = kthread_run(bmc_pwroff_rq_poll_fn, NULL, "tp_bmc_poller");
	if (!priv.poller) {
		dev_err(&priv.client->dev, "Failed to run poll thread\n");
		return -ENOMEM;
	}

#if defined(CONFIG_TP_BAIKALT_BMC_PM)
	pm_power_off = bmc_pwroff_rq;

	dev_info(&priv.client->dev, "Use Baikal-T BMC R_PWROFF_RQ for system shutdown\n");
#endif

	return 0;
}

static void bmc_clear_pwroff(void)
{
#if defined(CONFIG_TP_BAIKALT_BMC_PM)
	pm_power_off = NULL;
#endif
	kthread_stop(priv.poller);
}

static int bmc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	ret = bmc_check_host(client);
	if (ret)
		return ret;

	ret = bmc_validate_dev(client);
	if (ret)
		goto err_clear_priv;

	ret = bmc_init_button();
	if (ret)
		goto err_clear_priv;

	ret = bmc_init_pwroff();
	if (ret)
		goto err_clear_button;

	return 0;

err_clear_button:
	bmc_clear_button();

err_clear_priv:
	bmc_clear_priv();

	return ret;
}

static int bmc_remove(struct i2c_client *client)
{
	bmc_clear_pwroff();
	bmc_clear_button();
	bmc_clear_priv();

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bmc_of_match[] = {
	{ .compatible = "tp,bt-bmc" },
	{}
};
MODULE_DEVICE_TABLE(of, bmc_of_match);
#endif

static const struct i2c_device_id bmc_i2c_id[] = {
	{ "bt-bmc", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, bmc_i2c_id);

static struct i2c_driver bmc_driver = {
	.driver = {
		.name = "bt-bmc",
		.of_match_table = of_match_ptr(bmc_of_match),
		.groups = bmc_groups,
	},
	.probe = bmc_probe,
	.remove = bmc_remove,
	.id_table = bmc_i2c_id,
};
module_i2c_driver(bmc_driver);

MODULE_AUTHOR("Konstantin Kirik");
MODULE_DESCRIPTION("T-platforms Baikal-T(1) BMC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("serial:bmc");
