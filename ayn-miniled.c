// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/version.h>

#define	MINILED_GAIN_CURRENT				0x0000
#define	MINILED_VERSION						0x0022
#define	MINILED_BLACK_LEVEL					0x0300
#define		MINILED_BLACK_LEVEL_VALUE(x)	(x << 8)
#define	MINILED_RSV_OP						0x0500
#define		MINILED_RSV_OP_OFF				(0 << 15)
#define		MINILED_RSV_OP_ON				(1 << 15)

#define MINILED_MAX_BRIGHTNESS 0xFF

struct miniled_bl {
	struct i2c_client *client;
	struct regulator *vdd;
	struct regulator *vcc_1v2;
	struct regulator *vcc_3v3;
	uint brightness;
};

static int miniled_write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	int ret, retry = 0;
	u8 buf[] = { reg >> 8, reg & 0xFF, value >> 8, value & 0xFF };

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = buf,
			.len = sizeof(buf),
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret != 1) {
			msleep(50);
			if (retry >= 10) {
				dev_err(&client->dev, "I2C Max retry reached, giving up\n");
				return ret;
			}
			dev_err(&client->dev, "I2C write error %04x:%04x, retry %d\n", reg, value, retry++);
		}
	} while (ret != 1);

	return 0;
}

static int miniled_adjust_brightness(struct miniled_bl *miniled)
{
	long brightness = miniled->brightness;
	int rc;
	uint8_t gain, cur;

	if (brightness < 19)
	{
		gain = 1 + (brightness * 7);
		cur = 0x1f;
	}
	else if (brightness < 44)
	{
		gain = 131 + ((brightness - 19) * 5);
		cur = 0x1f;
	}
	else if (brightness < 96)
	{
		gain = 12 + (brightness - 44);
		cur = 0x03;
	}
	else if (brightness < 224)
	{
		gain = 128 + (brightness - 96);
		cur = 0x05;
	}
	else
	{
		gain = 192 + (brightness - 224);
		cur = 0x03;
	}

	rc = miniled_write_reg(miniled->client, MINILED_GAIN_CURRENT,
					(gain << 8) | cur);
	if (rc < 0)
		return rc;

	return 0;
}

static int miniled_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	struct miniled_bl *miniled = bl_get_data(bl);

	miniled->brightness = brightness;

	return miniled_adjust_brightness(miniled);
}

static int miniled_backlight_get_brightness(struct backlight_device *bl)
{
	struct miniled_bl *miniled = bl_get_data(bl);

	return miniled->brightness;
}

static const struct backlight_ops miniled_backlight_ops = {
	.update_status = miniled_backlight_update_status,
	.get_brightness = miniled_backlight_get_brightness,
};

#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
static int miniled_backlight_probe(struct i2c_client *client)
#else
static int miniled_backlight_probe(struct i2c_client *client, const struct i2c_device_id *idp)
#endif
{
	struct miniled_bl *miniled;
	struct backlight_properties props;
	struct backlight_device *bl;
	int rc;

	miniled = devm_kzalloc(&client->dev, sizeof(struct miniled_bl),
			       GFP_KERNEL);
	if (!miniled)
		return -ENOMEM;

	miniled->client = client;
	miniled->brightness = MINILED_MAX_BRIGHTNESS;

	miniled->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(miniled->vdd)) {
		rc = PTR_ERR(miniled->vdd);
		dev_err(&client->dev, "Regulator get failed vdd: %d\n", rc);
		return rc;
	}

	miniled->vcc_1v2 = devm_regulator_get(&client->dev, "vcc_1v2");
	if (IS_ERR(miniled->vcc_1v2)) {
		rc = PTR_ERR(miniled->vcc_1v2);
		dev_err(&client->dev, "Regulator get failed vcc_1v2: %d\n", rc);
		return rc;
	}

	miniled->vcc_3v3 = devm_regulator_get(&client->dev, "vcc_3v3");
	if (IS_ERR(miniled->vcc_3v3)) {
		rc = PTR_ERR(miniled->vcc_3v3);
		dev_err(&client->dev, "Regulator get failed vcc_3v3: %d\n", rc);
		return rc;
	}

	rc = regulator_enable(miniled->vdd);
	if (rc < 0) {
		dev_err(&client->dev,
			"Failed to enable vdd\n");
		return rc;
	}

	rc = regulator_enable(miniled->vcc_1v2);
	if (rc < 0) {
		dev_err(&client->dev,
			"Failed to enable vcc_1v2\n");
		return rc;
	}

	rc = regulator_enable(miniled->vcc_3v3);
	if (rc < 0) {
		dev_err(&client->dev,
			"Failed to enable vcc_3v3\n");
		return rc;
	}

	props.type = BACKLIGHT_RAW;
	props.max_brightness = MINILED_MAX_BRIGHTNESS;
	props.brightness = MINILED_MAX_BRIGHTNESS;

	bl = devm_backlight_device_register(&client->dev, client->name,
					    &client->dev, miniled,
					    &miniled_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&client->dev, "Failed to register backlight\n");
		return PTR_ERR(bl);
	}

	dev_set_drvdata(&client->dev, bl);
	return 0;
}

#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
static void miniled_backlight_remove(struct i2c_client *client)
#else
static int miniled_backlight_remove(struct i2c_client *client)
#endif
{
	struct backlight_device *bl = dev_get_drvdata(&client->dev);
	struct miniled_bl *miniled = bl_get_data(bl);

	miniled->brightness = 0;
	miniled_adjust_brightness(miniled);

	regulator_disable(miniled->vcc_3v3);
	regulator_disable(miniled->vcc_1v2);
	regulator_disable(miniled->vdd);

#if (KERNEL_VERSION(6, 3, 0) > LINUX_VERSION_CODE)
	return 0;
#endif
}

static int __maybe_unused miniled_suspend(struct device *dev)
{
	struct miniled_bl *miniled = dev_get_drvdata(dev);

	regulator_disable(miniled->vcc_3v3);
	regulator_disable(miniled->vcc_1v2);
	regulator_disable(miniled->vdd);

	return 0;
}

static int __maybe_unused miniled_resume(struct device *dev)
{
	struct miniled_bl *miniled = dev_get_drvdata(dev);
	int rc;

	rc = regulator_enable(miniled->vdd);
	if (rc < 0) {
		dev_err(&miniled->client->dev,
			"Failed to enable vdd\n");
		return rc;
	}

	rc = regulator_enable(miniled->vcc_1v2);
	if (rc < 0) {
		dev_err(&miniled->client->dev,
			"Failed to enable vcc_1v2\n");
		return rc;
	}

	rc = regulator_enable(miniled->vcc_3v3);
	if (rc < 0) {
		dev_err(&miniled->client->dev,
			"Failed to enable vcc_3v3\n");
		return rc;
	}

	usleep_range(100000, 200000);

	miniled_adjust_brightness(miniled);

	return 0;
}

static const struct of_device_id miniled_of_match_table[] = {
	{
		.compatible = "ayn,miniled",
	},
	{},
};

static const struct i2c_device_id miniled_device_id[] = {
	{"ayn-miniled", 0},
	{}
};

static const struct dev_pm_ops miniled_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(miniled_suspend, miniled_resume)
};

static struct i2c_driver miniled_i2c_driver = {
	.driver	= {
		.name	= "ayn-miniled",
		.of_match_table = miniled_of_match_table,
		.pm = &miniled_pm_ops,
	},
	.probe		= miniled_backlight_probe,
	.remove		= miniled_backlight_remove,
	.id_table	= miniled_device_id,
};

static int __init miniled_driver_init(void)
{
	return i2c_add_driver(&miniled_i2c_driver);
}

static void __exit miniled_driver_exit(void)
{
	i2c_del_driver(&miniled_i2c_driver);
}

module_init(miniled_driver_init);
module_exit(miniled_driver_exit);

MODULE_AUTHOR("Balázs Triszka <info@balika011.hu>");
MODULE_DESCRIPTION("Backlight Driver for Ayn Odin2 Mini Mini-LED");
MODULE_LICENSE("GPL");
