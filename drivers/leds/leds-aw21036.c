/*
 * leds-aw21036.c   aw21036 led module
 *
 * Version: v1.0.0
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: hushanping <hushanping@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/leds.h>
#include "leds-aw21036.h"
#include "leds-aw21036-reg.h"
/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW21036_I2C_NAME "aw21036_led"
#define AW21036_VERSION "v1.0.0"
#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1


/******************************************************
 *
 * aw21036 led parameter
 *
 ******************************************************/
#define AW21036_CFG_NAME_MAX		64

AW21036_CFG aw21036_cfg_array[] = {
	{aw21036_cfg_led_off, sizeof(aw21036_cfg_led_off)},
	{aw21036_led_all_on, sizeof(aw21036_led_all_on)},
	{aw21036_led_red_on, sizeof(aw21036_led_red_on)},
	{aw21036_led_green_on, sizeof(aw21036_led_green_on)},
	{aw21036_led_blue_on, sizeof(aw21036_led_blue_on)},
	{aw21036_led_breath_forever, sizeof(aw21036_led_breath_forever)}
};

/******************************************************
 *
 * aw21036 i2c write/read
 *
 ******************************************************/
static int aw21036_i2c_write(struct aw21036 *aw21036,
			     unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret =
		    i2c_smbus_write_byte_data(aw21036->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw21036_i2c_read(struct aw21036 *aw21036,
			    unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw21036->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw21036_i2c_write_bits(struct aw21036 *aw21036,
				  unsigned char reg_addr, unsigned int mask,
				  unsigned char reg_data)
{
	unsigned char reg_val;

	aw21036_i2c_read(aw21036, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw21036_i2c_write(aw21036, reg_addr, reg_val);

	return 0;
}

/*
static void aw21036_led_clear(struct aw21036 *aw21036)
{
	unsigned int i = 0;
	pr_info("%s: enter\n", __func__);
	for (i =0; i < 36; i++) {
		aw21036_i2c_write(aw21036, AW21036_REG_BR0 + i, 0x00);
		aw21036_i2c_write(aw21036, AW21036_REG_COL0 + i, 0x00);
	}
}
*/

static int aw21036_chip_enable(struct aw21036 *aw21036, bool flag)
{
	if (flag)
		aw21036_i2c_write_bits(aw21036, AW21036_REG_GCR,
					AW21036_BIT_GCR_CHIPEN_MASK,
					AW21036_BIT_GCR_CHIPEN_ENABLE);
	else
		aw21036_i2c_write_bits(aw21036, AW21036_REG_GCR,
					AW21036_BIT_GCR_CHIPEN_MASK,
					AW21036_BIT_GCR_CHIPEN_DISABLE);

	return 0;
}

static int aw21036_pwm_freq_cfg(struct aw21036 *aw21036)
{
	pr_info("%s: enter\n", __func__);

	aw21036_i2c_write_bits(aw21036, AW21036_REG_GCR,
				AW21036_BIT_GCR_CLKFRQ_MASK,
				aw21036->pwm_freq);
	pr_info("%s: osc clk freq: 0x%x\n", __func__, aw21036->pwm_freq);

	return 0;
}

static int aw21036_led_update(struct aw21036 *aw21036)
{
	pr_info("%s enter\n", __func__);
	aw21036_i2c_write(aw21036, AW21036_REG_UPDATE, 0x00);
	return 0;
}

/*****************************************************
 *
 * firmware/cfg update
 *
 *****************************************************/
static void aw21036_update_cfg_array(struct aw21036 *aw21036,
				     unsigned char *p_cfg_data,
				     unsigned int cfg_size)
{
	unsigned int i = 0;

	for (i = 0; i < cfg_size; i += 2)
		aw21036_i2c_write(aw21036, p_cfg_data[i], p_cfg_data[i + 1]);
}

static int aw21036_cfg_update(struct aw21036 *aw21036)
{
	pr_info("%s: enter\n", __func__);
	aw21036_update_cfg_array(aw21036,
				 (aw21036_cfg_array[aw21036->effect].p),
				 aw21036_cfg_array[aw21036->effect].count);
	return 0;
}

static int aw21036_hw_reset(struct aw21036 *aw21036)
{
	pr_info("%s: enter\n", __func__);

	if (aw21036 && gpio_is_valid(aw21036->reset_gpio)) {
		gpio_set_value_cansleep(aw21036->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw21036->reset_gpio, 1);
		usleep_range(2000, 2500);
	} else {
		dev_err(aw21036->dev, "%s:  failed\n", __func__);
	}
	pr_info("%s: enter out\n", __func__);
	return 0;
}

static int aw21036_hw_off(struct aw21036 *aw21036)
{
	pr_info("%s: enter\n", __func__);
	if (aw21036 && gpio_is_valid(aw21036->reset_gpio)) {
		gpio_set_value_cansleep(aw21036->reset_gpio, 0);
		msleep(1);
	} else {
		dev_err(aw21036->dev, "%s:  failed\n", __func__);
	}

	return 0;
}


/*****************************************************
 *
 * check chip id and version
 *
 *****************************************************/
static int aw21036_read_chipid(struct aw21036 *aw21036)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw21036_i2c_read(aw21036, AW21036_REG_RESET, &reg_val);
		if (reg_val == AW21036_CHIPID) {
			pr_info("%s: Match AW21036, chip id is 0x%x\n",
				__func__, reg_val);
			return 0;
		} else if (ret < 0) {
			dev_err(aw21036->dev,
				"%s: failed to read AW21036 CHIP ID: %d\n",
				__func__, ret);
			return -EIO;
		} else {
			pr_info("This Chip is Unsupported! CHIP_ID: 0x%x\n",
				reg_val);
		}
		cnt++;
		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}

static int aw21036_read_version(struct aw21036 *aw21036)
{
	int ret = -1;
	unsigned char reg_val = 0;

	ret = aw21036_i2c_read(aw21036, AW21036_REG_VER, &reg_val);
	if (ret < 0) {
		dev_err(aw21036->dev, "%s:failed to read version: %d\n",
			__func__, ret);
		return -EIO;
	} else {
		pr_info("%s: This Chip Version is 0x%x\n", __func__,
			reg_val);
		return 0;
	}
	return -EINVAL;
}

/*****************************************************
 *
 * aw21036 led cfg
 *
 *****************************************************/
static void aw21036_brightness_work(struct work_struct *work)
{
	struct aw21036 *aw21036 = container_of(work, struct aw21036,
					       brightness_work);

	pr_info("%s: enter\n", __func__);

	if (aw21036->cdev.brightness > aw21036->cdev.max_brightness)
		aw21036->cdev.brightness = aw21036->cdev.max_brightness;

	aw21036_i2c_write(aw21036, AW21036_REG_GCCR, aw21036->cdev.brightness);
}

static void aw21036_set_brightness(struct led_classdev *cdev,
				   enum led_brightness brightness)
{
	struct aw21036 *aw21036 = container_of(cdev, struct aw21036, cdev);

	aw21036->cdev.brightness = brightness;

	schedule_work(&aw21036->brightness_work);
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw21036_parse_dt(struct device *dev, struct aw21036 *aw21036,
			    struct device_node *np)
{
	unsigned int ret = 0;

	aw21036->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw21036->reset_gpio < 0) {
		dev_err(dev,
			"%s: no reset gpio provided, HW reset unsupported\n",
			__func__);
		return -1;
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}

	ret = of_property_read_u32(np, "led_current", &aw21036->led_current);
	if (ret) {
		aw21036->led_current = 255;
		dev_err(dev, "%s: current not found\n", __func__);
	} else {
		dev_info(dev, "%s: current read from dts: %dma\n",
			 __func__, aw21036->led_current);
	}

	ret = of_property_read_u32(np, "pwm_freq", &aw21036->pwm_freq);
	if (ret) {
		aw21036->pwm_freq = 0;
		dev_err(dev, "%s: pwm_freq not found\n", __func__);
	} else {
		dev_info(dev, "%s: pwm_freq read from dts: %d\n",
			 __func__, aw21036->pwm_freq);
	}

	return 0;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw21036_reg_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21036 *aw21036 = container_of(led_cdev, struct aw21036, cdev);
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw21036_i2c_write(aw21036, databuf[0], databuf[1]);
	return count;
}

static ssize_t aw21036_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21036 *aw21036 = container_of(led_cdev, struct aw21036, cdev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW21036_REG_MAX; i++) {
		aw21036_i2c_read(aw21036, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	return len;
}

static ssize_t aw21036_hwen_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21036 *aw21036 = container_of(led_cdev, struct aw21036, cdev);

	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		if (databuf[0] == 1)
			aw21036_hw_reset(aw21036);
		else
			aw21036_hw_off(aw21036);
	}

	return count;
}

static ssize_t aw21036_hwen_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21036 *aw21036 = container_of(led_cdev, struct aw21036, cdev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "hwen=%d\n",
			gpio_get_value(aw21036->reset_gpio));

	return len;
}

static ssize_t aw21036_effect_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21036 *aw21036 = container_of(led_cdev, struct aw21036, cdev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"Echo Corresponding Number to Effect:\n");
	for (i = 0; i < sizeof(aw21036_cfg_array) / sizeof(struct aw21036_cfg);
	     i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "Effect %x: %pf\n",
				i, aw21036_cfg_array[i].p);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "Current Effect: %pf\n",
			aw21036_cfg_array[aw21036->effect].p);

	return len;
}

static ssize_t aw21036_effect_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21036 *aw21036 = container_of(led_cdev, struct aw21036, cdev);

	sscanf(buf, "%x", &databuf[0]);
	aw21036->effect = databuf[0];

	aw21036_cfg_update(aw21036);

	return len;
}

static ssize_t aw21036_rgbcolor_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	unsigned int databuf[2] = { 0, 0 };
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21036 *aw21036 = container_of(led_cdev, struct aw21036, cdev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw21036_i2c_write(aw21036, AW21036_REG_GCFG0, 0x00);/*GEn = 0 */
		aw21036_i2c_write(aw21036, AW21036_REG_GCFG1, 0x10);/*GCOLDIS = 1 GEn = 0*/
		aw21036_i2c_write(aw21036, AW21036_REG_GCFG1, 0x10);
		aw21036_i2c_write(aw21036, AW21036_REG_GCR2, 0x01);/*RGBMD = 1*/
		aw21036_i2c_write(aw21036, AW21036_REG_BR0 + databuf[0], 0xFF);
		aw21036_i2c_write(aw21036, AW21036_REG_GCCR, 0x0F);

		aw21036->rgbcolor = (databuf[1] & 0x00ff0000) >> 16;
		aw21036_i2c_write(aw21036, AW21036_REG_COL0 + databuf[0] * 3,
				  aw21036->rgbcolor);

		aw21036->rgbcolor = (databuf[1] & 0x0000ff00) >> 8;
		aw21036_i2c_write(aw21036, AW21036_REG_COL0 + databuf[0] * 3 + 1,
				  aw21036->rgbcolor);

		aw21036->rgbcolor = (databuf[1] & 0x000000ff);
		aw21036_i2c_write(aw21036, AW21036_REG_COL0 + databuf[0] * 3 + 2,
				  aw21036->rgbcolor);

		aw21036_led_update(aw21036);
	}
	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw21036_reg_show, aw21036_reg_store);
static DEVICE_ATTR(hwen, S_IWUSR | S_IRUGO, aw21036_hwen_show,
		   aw21036_hwen_store);
static DEVICE_ATTR(effect, S_IWUSR | S_IRUGO, aw21036_effect_show,
		   aw21036_effect_store);
static DEVICE_ATTR(rgbcolor, S_IWUSR | S_IRUGO, NULL, aw21036_rgbcolor_store);


static struct attribute *aw21036_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_effect.attr,
	&dev_attr_rgbcolor.attr,
	NULL
};

static struct attribute_group aw21036_attribute_group = {
	.attrs = aw21036_attributes
};


/******************************************************
 *
 * led class dev
 ******************************************************/
static int aw21036_led_init(struct aw21036 *aw21036)
{
	pr_info("%s: enter\n", __func__);

	aw21036_i2c_write(aw21036, AW21036_REG_RESET, 0x00);
	usleep_range(2000, 2500);
	aw21036_chip_enable(aw21036, true);
	usleep_range(200, 300);

	aw21036_pwm_freq_cfg(aw21036);

	aw21036_i2c_write_bits(aw21036, AW21036_REG_GCR,
				AW21036_BIT_GCR_APSE_MASK,
				AW21036_BIT_GCR_APSE_ENABLE);
	aw21036_i2c_write(aw21036, AW21036_REG_GCCR, aw21036->led_current);
	pr_info("%s: DONE!\n", __func__);
	return 0;
}

static int aw21036_parse_led_cdev(struct aw21036 *aw21036,
				  struct device_node *np)
{
	struct device_node *temp;
	int ret = -1;

	pr_info("%s: enter\n", __func__);

	for_each_child_of_node(np, temp) {
		ret = of_property_read_string(temp, "aw21036,name",
					      &aw21036->cdev.name);
		if (ret < 0) {
			dev_err(aw21036->dev,
				"Failure reading led name, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw21036,imax",
					   &aw21036->imax);
		if (ret < 0) {
			dev_err(aw21036->dev,
				"Failure reading imax, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw21036,brightness",
					   &aw21036->cdev.brightness);
		if (ret < 0) {
			dev_err(aw21036->dev,
				"Failure reading brightness, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw21036,max_brightness",
					   &aw21036->cdev.max_brightness);
		if (ret < 0) {
			dev_err(aw21036->dev,
				"Failure reading max brightness, ret = %d\n",
				ret);
			goto free_pdata;
		}
	}
	aw21036_led_init(aw21036);
	INIT_WORK(&aw21036->brightness_work, aw21036_brightness_work);
	aw21036->cdev.brightness_set = aw21036_set_brightness;
	ret = led_classdev_register(aw21036->dev, &aw21036->cdev);
	if (ret) {
		dev_err(aw21036->dev, "unable to register led ret=%d\n", ret);
		goto free_pdata;
	}
	ret = sysfs_create_group(&aw21036->cdev.dev->kobj,
				 &aw21036_attribute_group);
	if (ret) {
		dev_err(aw21036->dev, "led sysfs ret: %d\n", ret);
		goto free_class;
	}
	return 0;

free_class:
	led_classdev_unregister(&aw21036->cdev);
free_pdata:
	return ret;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw21036_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct aw21036 *aw21036;
	struct device_node *np = i2c->dev.of_node;
	int ret;

	pr_info("%s: enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw21036 = devm_kzalloc(&i2c->dev, sizeof(struct aw21036), GFP_KERNEL);
	if (aw21036 == NULL)
		return -ENOMEM;

	aw21036->dev = &i2c->dev;
	aw21036->i2c = i2c;

	i2c_set_clientdata(i2c, aw21036);

	mutex_init(&aw21036->cfg_lock);

	/* aw21036 rst */
	if (np) {
		ret = aw21036_parse_dt(&i2c->dev, aw21036, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err_parse_dt;
		}
	} else {
		aw21036->reset_gpio = -1;
	}

	if (gpio_is_valid(aw21036->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw21036->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw21036_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			goto err_gpio_request;
		}
	}

	/* hardware reset */
	aw21036_hw_reset(aw21036);

	/* aw21036 chip id */
	ret = aw21036_read_chipid(aw21036);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw21036_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}
	ret = aw21036_read_version(aw21036);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw21036_read_version failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	dev_set_drvdata(&i2c->dev, aw21036);

	aw21036_parse_led_cdev(aw21036, np);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s error creating led class dev\n",
			__func__);
		goto err_sysfs;
	}

	pr_info("%s probe completed successfully!\n", __func__);
	return 0;

err_sysfs:
err_id:
	devm_gpio_free(&i2c->dev, aw21036->reset_gpio);
err_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, aw21036);
	aw21036 = NULL;
	return ret;
}

static int aw21036_i2c_remove(struct i2c_client *i2c)
{
	struct aw21036 *aw21036 = i2c_get_clientdata(i2c);

	pr_info("%s: enter\n", __func__);
	sysfs_remove_group(&aw21036->cdev.dev->kobj, &aw21036_attribute_group);
	led_classdev_unregister(&aw21036->cdev);

	if (gpio_is_valid(aw21036->reset_gpio))
		devm_gpio_free(&i2c->dev, aw21036->reset_gpio);

	devm_kfree(&i2c->dev, aw21036);
	aw21036 = NULL;

	return 0;
}

static const struct i2c_device_id aw21036_i2c_id[] = {
	{AW21036_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw21036_i2c_id);

static const struct of_device_id aw21036_dt_match[] = {
	{.compatible = "awinic,aw21036_led"},
	{},
};

static struct i2c_driver aw21036_i2c_driver = {
	.driver = {
		.name = AW21036_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw21036_dt_match),
		},
	.probe = aw21036_i2c_probe,
	.remove = aw21036_i2c_remove,
	.id_table = aw21036_i2c_id,
};

static int __init aw21036_i2c_init(void)
{
	int ret = 0;

	pr_info("aw21036 driver version %s\n", AW21036_VERSION);

	ret = i2c_add_driver(&aw21036_i2c_driver);
	if (ret) {
		pr_err("fail to add aw21036 device into i2c\n");
		return ret;
	}
	return 0;
}
module_init(aw21036_i2c_init);

static void __exit aw21036_i2c_exit(void)
{
	i2c_del_driver(&aw21036_i2c_driver);
}
module_exit(aw21036_i2c_exit);

MODULE_DESCRIPTION("AW21036 LED Driver");
MODULE_LICENSE("GPL v2");
