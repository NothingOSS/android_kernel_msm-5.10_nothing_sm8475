/*
 * leds-aw20036.c   aw20036 led module
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Joseph <zhangzetao@awinic.com.cn>
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
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/mman.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/leds.h>
#include "leds-aw20036.h"
#include "leds-aw20036-reg.h"
/******************************************************
 *
 * Marco
 *
 ******************************************************/

#define AW20036_I2C_NAME "aw20036_led"

#define AW20036_DRIVER_VERSION "v1.5.0"

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1

struct pinctrl *pinctrl_aw20036;
struct pinctrl_state *aw20036_pins_cfg, *aw20036_rst_output0, *aw20036_rst_output1;
unsigned int aw20036debounce;

const struct of_device_id aw20036_of_match[] = {
	{.compatible = "awinic,aw20036_led",},
	{},
};

/******************************************************
 *
 * aw20036 led parameter
 *
 ******************************************************/
#define AW20036_CFG_NAME_MAX        64
#if defined(AW20036_BIN_CONFIG)
static char aw20036_cfg_name[][AW20036_CFG_NAME_MAX] = {
	{"aw20036_led_all_on.bin"},
	{"aw20036_led_red_on.bin"},
	{"aw20036_led_green_on.bin"},
	{"aw20036_led_blue_on.bin"},
	{"aw20036_led_breath_forever.bin"},
	{"aw20036_cfg_led_off.bin"},
};
#elif defined(AW20036_ARRAY_CONFIG)
AW20036_CFG aw20036_cfg_array[] = {
	{aw20036_cfg_led_off, sizeof(aw20036_cfg_led_off)},
	{aw20036_led_all_on, sizeof(aw20036_led_all_on)},
};
#else
    /*Nothing */
#endif

#define POWER_SAVE_MODE

static char hw_ver[4] = "DVT";
static char dev_color[6] = "white";

#define AW20036_IMAX_NAME_MAX       32
static char aw20036_imax_name[][AW20036_IMAX_NAME_MAX] = {
	{"AW20036_IMAX_10mA"},
	{"AW20036_IMAX_20mA"},
	{"AW20036_IMAX_30mA"},
	{"AW20036_IMAX_40mA"},
	{"AW20036_IMAX_60mA"},
	{"AW20036_IMAX_80mA"},
	{"AW20036_IMAX_120mA"},
	{"AW20036_IMAX_160mA"},
	{"AW20036_IMAX_3P3mA"},
	{"AW20036_IMAX_6P7mA"},
	{"AW20036_IMAX_10P0mA"},
	{"AW20036_IMAX_13P3mA"},
	{"AW20036_IMAX_20P0mA"},
	{"AW20036_IMAX_26P7mA"},
	{"AW20036_IMAX_40P0mA"},
	{"AW20036_IMAX_53P3mA"},
};

static DECLARE_WAIT_QUEUE_HEAD(aw20036_waitq);
int ev_happen = 0;
char ev_code = '0';
struct aw20036 *g_aw20036;
/******************************************************
 *
 * aw20036 i2c write/read
 *
 ******************************************************/
static int aw20036_i2c_write_block(struct aw20036 *aw20036,
			     unsigned char reg_addr, u8 length, unsigned char * reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;
	u8 buf[64] = {0};

	struct i2c_msg msg = {
		.addr = aw20036->i2c->addr,
		.flags = 0,
		.buf = buf,
		.len = length +1
	};

	if (aw20036->dev_suspend) {
		pr_err("%s: ignore 0x%x(%d) when suspend\n", __func__, reg_addr, length);
		return ret;
	}

	/* Copy Register Address. */
	buf[0] = reg_addr;

	/* Copy Register Data. */
	memcpy(&buf[1], reg_data, length);

	while (cnt < AW_I2C_RETRIES) {
		ret =
		    i2c_transfer(aw20036->i2c->adapter, &msg, 1);
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

static int aw20036_i2c_write(struct aw20036 *aw20036,
			     unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	if (aw20036->dev_suspend) {
		pr_err("%s: ignore 0x%x when suspend\n", __func__, reg_addr);
		return ret;
	}

	while (cnt < AW_I2C_RETRIES) {
		ret =
		    i2c_smbus_write_byte_data(aw20036->i2c, reg_addr, reg_data);
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

static int aw20036_i2c_read(struct aw20036 *aw20036,
			    unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	if (aw20036->dev_suspend) {
		pr_err("%s: ignore 0x%x when suspend\n", __func__, reg_addr);
		return ret;
	}

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw20036->i2c, reg_addr);
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

static int aw20036_i2c_write_bits(struct aw20036 *aw20036,
				  unsigned char reg_addr, unsigned int mask,
				  unsigned char reg_data)
{
	unsigned char reg_val;
       int ret = 0;
	ret = aw20036_i2c_read(aw20036, reg_addr, &reg_val);
	if(ret < 0){
		return ret;
	}
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	ret = aw20036_i2c_write(aw20036, reg_addr, reg_val);
	if(ret < 0){
		return ret;
	}
	return ret;
}

/*****************************************************
 *
 * aw20036 led cfg
 *
 *****************************************************/
static int aw20036_reg_page_cfg(struct aw20036 *aw20036, unsigned char page)
{
	int ret = 0;
	ret = aw20036_i2c_write(aw20036, REG_PAGE, page);
	return ret;
}

static int aw20036_imax_cfg(struct aw20036 *aw20036, unsigned char imax)
{
	int ret = 0;

	if (imax > 0xF)
		imax = 0xF;

	aw20036->imax = imax;
	ret = aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
	if (ret < 0){
		return ret;
	}
	ret = aw20036_i2c_write_bits(aw20036, REG_GCCR, BIT_IMAX_MASK, imax << 4);
	if (ret < 0){
		return ret;
	}
	return ret;
}

static int aw20036_dbgdim_cfg(struct aw20036 *aw20036, unsigned int data)
{
	int i;

	aw20036_i2c_write(aw20036, 0xF0, 0xC4);
	for (i = 0; i < AW20036_REG_NUM_PAG4; i = i + 2) {
		aw20036->rgbcolor = data;
		aw20036_i2c_write(aw20036, i, aw20036->rgbcolor);
	}
	return 0;
}

static int aw20036_dbgfdad_cfg(struct aw20036 *aw20036, unsigned int data)
{
	int i;

	aw20036_i2c_write(aw20036, 0xF0, 0xC4);
	for (i = 1; i < AW20036_REG_NUM_PAG4; i = i + 2) {
		aw20036->rgbcolor = data;
		aw20036_i2c_write(aw20036, i, aw20036->rgbcolor);
	}

	return 0;
}

static void aw20036_brightness_work(struct work_struct *work)
{
	struct aw20036 *aw20036 = container_of(work, struct aw20036,
					       brightness_work);

	pr_info("%s: enter\n", __func__);

	if (aw20036->cdev.brightness > aw20036->cdev.max_brightness)
		aw20036->cdev.brightness = aw20036->cdev.max_brightness;

	if (aw20036->cdev.brightness) {
		aw20036_i2c_write(aw20036, 0xF0, 0xC0);
		aw20036_i2c_write(aw20036, 0x01, 0x00);
		aw20036_dbgdim_cfg(aw20036, AW20036_DBGCTR_DIM);
		aw20036_dbgfdad_cfg(aw20036, aw20036->cdev.brightness);
	} else {
		aw20036_dbgdim_cfg(aw20036, 0x00);
		aw20036_dbgfdad_cfg(aw20036, 0);
		aw20036_i2c_write(aw20036, 0xF0, 0xC0);
		aw20036_i2c_write(aw20036, 0x01, 0x80);
	}
}

static void aw20036_set_brightness(struct led_classdev *cdev,
				   enum led_brightness brightness)
{
	struct aw20036 *aw20036 = container_of(cdev, struct aw20036, cdev);

	aw20036->cdev.brightness = brightness;

	schedule_work(&aw20036->brightness_work);
}

/*****************************************************
 *
 * firmware/cfg update
 *
 *****************************************************/
#if defined(AW20036_ARRAY_CONFIG)
static void aw20036_update_cfg_array(struct aw20036 *aw20036,
				     unsigned char *p_cfg_data,
				     unsigned int cfg_size)
{
	unsigned int i = 0;
	unsigned char page = 0;

	for (i = 0; i < cfg_size; i += 2) {
		aw20036_i2c_write(aw20036, p_cfg_data[i], p_cfg_data[i + 1]);
		if (p_cfg_data[i] == 0xf0)
			page = p_cfg_data[i + 1];
		if ((page == AW20036_REG_PAGE0)
		    && (p_cfg_data[i] == REG_SWRST)
		    && (p_cfg_data[i + 1] == 0x01)) {
			usleep_range(2000, 2500);
		}
	}
}

static int aw20036_cfg_update_array(struct aw20036 *aw20036)
{
	pr_info("%s: enter\n", __func__);

	aw20036_update_cfg_array(aw20036,
				 (aw20036_cfg_array[aw20036->effect].p),
				 aw20036_cfg_array[aw20036->effect].count);
	return 0;
}
#endif
#if defined(AW20036_BIN_CONFIG)
static void aw20036_cfg_loaded(const struct firmware *cont, void *context)
{
	struct aw20036 *aw20036 = context;
	int i = 0;
	unsigned char page = 0;
	unsigned char reg_addr = 0;
	unsigned char reg_val = 0;

	pr_info("%s: enter\n", __func__);

	if (!cont) {
		pr_info("%s: failed to read %s\n", __func__,
			aw20036_cfg_name[aw20036->effect]);
		release_firmware(cont);
		return;
	}
	mutex_lock(&aw20036->cfg_lock);
	pr_info("%s: loaded %s - size: %zu\n", __func__,
		aw20036_cfg_name[aw20036->effect], cont ? cont->size : 0);

	for (i = 0; i < cont->size; i += 2) {
		if (*(cont->data + i) == 0xf0)
			page = *(cont->data + i + 1);
		aw20036_i2c_write(aw20036, *(cont->data + i),
				  *(cont->data + i + 1));
		pr_debug("%s: addr:0x%02x, data:0x%02x\n", __func__,
			 *(cont->data + i), *(cont->data + i + 1));

		if (page == AW20036_REG_PAGE0) {
			reg_addr = *(cont->data + i);
			reg_val = *(cont->data + i + 1);
			/* gcr chip enable delay */
			if ((reg_addr == REG_SWRST) && (reg_val == 0x01))
				usleep_range(2000, 2500);
		}
	}

	release_firmware(cont);
	mutex_unlock(&aw20036->cfg_lock);
	pr_info("%s: cfg update complete\n", __func__);

}

static int aw20036_cfg_update(struct aw20036 *aw20036)
{
	int ret;

	pr_info("%s: enter\n", __func__);
	ret = 0;

	if (aw20036->effect < (sizeof(aw20036_cfg_name) / AW20036_CFG_NAME_MAX)) {
		pr_info("%s: cfg name=%s\n", __func__,
			aw20036_cfg_name[aw20036->effect]);
	} else {
		pr_err("%s: effect 0x%02x over s value\n", __func__,
		       aw20036->effect);
		return (-1);
	}

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw20036_cfg_name[aw20036->effect],
				       aw20036->dev, GFP_KERNEL, aw20036,
				       aw20036_cfg_loaded);
}
#endif

static int aw20036_rgbcolor_config(struct aw20036 *aw20036)
{
	int ret = 0;
	/*
	led0-36 dim cfg
	R1C1,  R2C1, ...R12C1,
	R1C2,  R2C2, ...R12C2,
	R1C3,  R2C3, ...R12C3,
	38, 0,   4,  4,  4,  4,   4,   4,  4,  4, 3, 4,
	53, 63, 4,  4,  4,  4,   4,   4,  4,  2, 4, 4,
	33, 0,   4,  5, 30, 27, 63, 30, 21, 2, 4, 4
	*/
	unsigned char aw20036_rgb_color_cfg[36] ={
		0x26, 0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03, 0x04,
		0x35, 0x3F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x02, 0x04, 0x04,
		0x21, 0x00, 0x04, 0x05, 0x1E, 0x1B, 0x3F, 0x1E, 0x15, 0x02, 0x04, 0x04
	};

	if(strncmp(hw_ver, "T0", 2) == 0){
		aw20036_rgb_color_cfg[1] = 0x21;
		aw20036_rgb_color_cfg[24] = 0x00;
		aw20036_rgb_color_cfg[29] = 0x1D;
		pr_info("%s: hwid is T0\n", __func__);
	}else if(strncmp(hw_ver, "EVT", 3) == 0){
		aw20036_rgb_color_cfg[29] = 0x1D;
		pr_info("%s: hwid is EVT\n", __func__);
	}

	ret = aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE1);
	if(ret < 0){
		return ret;
	}
	ret = aw20036_i2c_write_block(aw20036, 0x00, 36, aw20036_rgb_color_cfg);
	if(ret < 0){
		return ret;
	}
	return ret;
}

static int aw20036_hw_reset(struct aw20036 *aw20036)
{
	pr_info("%s: enter\n", __func__);

	if (aw20036 && gpio_is_valid(aw20036->reset_gpio)) {
		gpio_set_value_cansleep(aw20036->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw20036->reset_gpio, 1);
		usleep_range(2000, 2500);
	} else {
		dev_err(aw20036->dev, "%s:  failed\n", __func__);
	}
	pr_info("%s: enter out\n", __func__);
	return 0;
}

static int aw20036_hw_off(struct aw20036 *aw20036)
{
	pr_info("%s: enter\n", __func__);
	if (aw20036 && gpio_is_valid(aw20036->reset_gpio)) {
		gpio_set_value_cansleep(aw20036->reset_gpio, 0);
		msleep(1);
	} else {
		dev_err(aw20036->dev, "%s:  failed\n", __func__);
	}

	return 0;
}

 /******************************************************
 *
 * led class dev
 *
 ******************************************************/
static int aw20036_led_init(struct aw20036 *aw20036)
{
	pr_info("%s: enter\n", __func__);

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
	aw20036_i2c_write(aw20036, 0x02, 0x01);
	usleep_range(2000, 2500);
	//set imax
	if(strncmp(dev_color, "gray", 4) == 0){
		pr_info("%s: device color is gray\n", __func__);
		aw20036_imax_cfg(aw20036, 0x05);
	}else{
		aw20036_imax_cfg(aw20036, 0x05);
	}
	aw20036_i2c_write_bits(aw20036, REG_GCCR, BIT_ALLON_MASK,
			       BIT_GCR_ALLON_ENABLE);
	aw20036_i2c_write(aw20036, 0x80, 0x02);
	/*Solve the LED residual image problem(For example,
	  when LED 13 is on, LED 1 will be slightly bright)*/
	aw20036_i2c_write(aw20036, 0x81, 0x11);
	aw20036_i2c_write(aw20036, 0x82, 0x36);
	aw20036_i2c_write(aw20036, 0x83, 0xFF);
	aw20036_i2c_write(aw20036, 0x84, 0x04);
	aw20036_i2c_write(aw20036, 0x85, 0x58);
	aw20036_i2c_write(aw20036, 0x86, 0x03);
#ifdef POWER_SAVE_MODE
	aw20036_i2c_write(aw20036, 0x01, 0x80);
#else
	aw20036_i2c_write(aw20036, 0x01, 0x00);
	aw20036_rgbcolor_config(aw20036);
#endif
	return 0;
}

/******************************************************
 *
 * irq
 *
 ******************************************************/

static irqreturn_t aw20036_irq(int irq, void *data)
{
	struct aw20036 *aw20036 = data;
	unsigned char reg_val;

	pr_info("%s: enter\n", __func__);

	aw20036_i2c_read(aw20036, REG_ISRFLT, &reg_val);
	pr_info("%s: reg INTST=0x%x\n", __func__, reg_val);
	pr_info("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw20036_parse_dt(struct device *dev, struct aw20036 *aw20036,
			    struct device_node *np)
{

	aw20036->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw20036->reset_gpio < 0) {
		dev_err(dev,
			"%s: no reset gpio provided, will not HW reset device\n",
			__func__);
		return (-1);
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw20036->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw20036->irq_gpio < 0) {
		dev_err(dev,
			"%s: no irq gpio provided, will not suppport intterupt\n",
			__func__);
		return (-1);
	} else {
		dev_info(dev, "%s: irq gpio provided ok\n", __func__);
	}

	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw20036_read_chipid(struct aw20036 *aw20036)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg_val = 0;

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);

	while (cnt++ < AW_READ_CHIPID_RETRIES) {
		ret = aw20036_i2c_read(aw20036, REG_CHIPID, &reg_val);
		if (reg_val == AW20036_CHIPID) {
			pr_info("This Chip is  AW20036    REG_ID: 0x%x\n",
				reg_val);
			return 0;
		} else if (ret < 0) {
			dev_err(aw20036->dev,
				"%s: failed to AW20036_REG_ID: %d\n", __func__,
				ret);
		} else {
			pr_info("This Chip    read register   REG_ID: 0x%x\n",
				reg_val);
		}
		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}
	return -EINVAL;
}

static void aw20036_breath_pattern_0(struct aw20036 *aw20036,  unsigned char *data)
{
	unsigned char T1, T2, T3, T4, T1T2, T3T4;
	unsigned char maxbrihtness = 255;
	unsigned char minbrightness = 0;
	unsigned char led0, led1, led2, led3, led4, led5, ledOn;
	unsigned char allON = 0x3F;
	unsigned int i;

	pr_info("%s\n", __func__);

	T1 = data[0];
	T2 = data[1];
	T3 = data[2];
	T4 = data[3];
	maxbrihtness = data[4];
	minbrightness = data[5];
	T1T2 = T2 | (T1 << 4);
	T3T4 = T4 | (T3 << 4);

	led0 = data[6] & allON;
	led1 = data[7] & allON;
	led2 = data[8] & allON;
	led3 = data[9] & allON;
	led4 = data[10] & allON;
	led5 = data[11] & allON;

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE3);//set PATn to pattern, step c
	for (i = 0; i < 36; i ++ ) {
		ledOn = 0;
		if (i < 6) {
			if ((led0 & (1 << i)) != 0) {
				ledOn = 1;
			}
		} else if (i < 12) {
			if ((led1 & (1 << (i - 6))) != 0) {
				ledOn = 1;
			}
		} else if (i < 18) {
			if ((led2 & (1 << (i - 12))) != 0) {
				ledOn = 1;
			}
		} else if (i < 24) {
			if ((led3 & (1 << (i - 18))) != 0) {
				ledOn = 1;
			}
		} else if (i < 30) {
			if ((led4 & (1 << (i - 24))) != 0) {
				ledOn = 1;
			}
		} else if (i < 36) {
			if ((led5 & (1 << (i - 30))) != 0) {
				ledOn = 1;
			}
		}
		if (ledOn == 1) {
			aw20036_i2c_write(aw20036, (0x00 + i), 1);//pattern 0
		}
	}

	if(aw20036->vip_notification== 0){
		return ;
	}

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
	aw20036_i2c_write(aw20036, REG_PAT0T0, T1T2); //set Time and fade, step d
	aw20036_i2c_write(aw20036, REG_PAT0T1, T3T4);
	aw20036_i2c_write(aw20036, REG_FADEH0, maxbrihtness);
	aw20036_i2c_write(aw20036, REG_FADEL0, minbrightness);
	aw20036_i2c_write_bits(aw20036, REG_PATE, BIT_PAT0_MASK, BIT_PAT0_ENABLE);//Pattern Enabl control,enable pattern0 ,step e
	aw20036_i2c_write(aw20036, REG_PAT0CFG, 1); //pattern1 PTCMD auto mode, step f
	aw20036_i2c_write_bits(aw20036, REG_PATGO, BIT_PAT0_MASK, BIT_PAT0_ENABLE);//runs pattern0,step g
}

static void aw20036_breath_pattern_1(struct aw20036 *aw20036, unsigned char *data)
{
	unsigned char T1, T2, T3, T4, T1T2, T3T4;
	unsigned char maxbrihtness = 255;
	unsigned char minbrightness = 0;
	unsigned char led0, led1, led2, led3, led4, led5, ledOn;
	unsigned char allON = 0x3F;
	unsigned int i;

	pr_info("%s\n", __func__);

	T1 = data[0];
	T2 = data[1];
	T3 = data[2];
	T4 = data[3];
	maxbrihtness = data[4];
	minbrightness = data[5];
	T1T2 = T2 | (T1 << 4);
	T3T4 = T4 | (T3 << 4);

	led0 = data[6] & allON;
	led1 = data[7] & allON;
	led2 = data[8] & allON;
	led3 = data[9] & allON;
	led4 = data[10] & allON;
	led5 = data[11] & allON;

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE3);//set PATn to pattern, step c
	for (i = 0; i < 36; i ++ ) {
		ledOn = 0;
		if (i < 6) {
			if ((led0 & (1 << i)) != 0) {
				ledOn = 1;
			}
		} else if (i < 12) {
			if ((led1 & (1 << (i - 6))) != 0) {
				ledOn = 1;
			}
		} else if (i < 18) {
			if ((led2 & (1 << (i - 12))) != 0) {
				ledOn = 1;
			}
		} else if (i < 24) {
			if ((led3 & (1 << (i - 18))) != 0) {
				ledOn = 1;
			}
		} else if (i < 30) {
			if ((led4 & (1 << (i - 24))) != 0) {
				ledOn = 1;
			}
		} else if (i < 36) {
			if ((led5 & (1 << (i - 30))) != 0) {
				ledOn = 1;
			}
		}

		if (ledOn == 1) {
			aw20036_i2c_write(aw20036, (0x00 + i), 2); //from pattern 1
		}
	}

	if(aw20036->vip_notification== 0){
		return ;
	}

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
	aw20036_i2c_write(aw20036, REG_PAT1T0, T1T2); //set Time and fade, step d
	aw20036_i2c_write(aw20036, REG_PAT1T1, T3T4);
	aw20036_i2c_write(aw20036, REG_FADEH1, maxbrihtness);
	aw20036_i2c_write(aw20036, REG_FADEL1, minbrightness);
	aw20036_i2c_write_bits(aw20036, REG_PATE, BIT_PAT1_MASK, BIT_PAT1_ENABLE);//Pattern Enabl control,enable pattern1 ,step e
	aw20036_i2c_write(aw20036, REG_PAT1CFG, 1); //pattern1 PTCMD auto mode, step f
	aw20036_i2c_write_bits(aw20036, REG_PATGO, BIT_PAT1_MASK, BIT_PAT1_ENABLE);//runs pattern1,step g
}

static void aw20036_breath_pattern_2(struct aw20036 *aw20036,  unsigned char *data)
{
	unsigned char T1, T2, T3, T4, T1T2, T3T4;
	unsigned char maxbrihtness = 255;
	unsigned char minbrightness = 0;
	unsigned char led0, led1, led2, led3, led4, led5, ledOn;
	unsigned char allON = 0x3F;
	unsigned int i;

	pr_info("%s\n", __func__);

	T1 = data[0];
	T2 = data[1];
	T3 = data[2];
	T4 = data[3];
	maxbrihtness = data[4];
	minbrightness = data[5];
	T1T2 = T2 | (T1 << 4);
	T3T4 = T4 | (T3 << 4);

	led0 = data[6] & allON;
	led1 = data[7] & allON;
	led2 = data[8] & allON;
	led3 = data[9] & allON;
	led4 = data[10] & allON;
	led5 = data[11] & allON;

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE3);//set PATn to pattern, step c
	for (i = 0; i < 36; i ++ ) {
		ledOn = 0;
		if (i < 6) {
			if ((led0 & (1 << i)) != 0) {
				ledOn = 1;
			}
		} else if (i < 12) {
			if ((led1 & (1 << (i - 6))) != 0) {
				ledOn = 1;
			}
		} else if (i < 18) {
			if ((led2 & (1 << (i - 12))) != 0) {
				ledOn = 1;
			}
		} else if (i < 24) {
			if ((led3 & (1 << (i - 18))) != 0) {
				ledOn = 1;
			}
		} else if (i < 30) {
			if ((led4 & (1 << (i - 24))) != 0) {
				ledOn = 1;
			}
		} else if (i < 36) {
			if ((led5 & (1 << (i - 30))) != 0) {
				ledOn = 1;
			}
		}
		if (ledOn == 1) {
			aw20036_i2c_write(aw20036, (0x00 + i), 3); //from pattern2
		}
	}

	if(aw20036->vip_notification== 0){
		return ;
	}

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
	aw20036_i2c_write(aw20036, REG_PAT2T0, T1T2); //set Time and fade, step d
	aw20036_i2c_write(aw20036, REG_PAT2T1, T3T4);
	aw20036_i2c_write(aw20036, REG_FADEH2, maxbrihtness);
	aw20036_i2c_write(aw20036, REG_FADEL2, minbrightness);
	aw20036_i2c_write_bits(aw20036, REG_PATE, BIT_PAT2_MASK, BIT_PAT2_ENABLE);//Pattern Enabl control,enable pattern1 ,step e
	aw20036_i2c_write(aw20036, REG_PAT2CFG, 1); //PTCMD auto mode, step f
	aw20036_i2c_write_bits(aw20036, REG_PATGO, BIT_PAT2_MASK, BIT_PAT2_ENABLE);//runs pattern1,step g
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw20036_reg_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw20036_i2c_write(aw20036, databuf[0], databuf[1]);
	return count;
}

static ssize_t aw20036_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	unsigned char reg_page = 0;

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
	for (i = 0; i < AW20036_REG_MAX; i++) {
		if (!reg_page) {
			if (!(aw20036_reg_access[i] & REG_RD_ACCESS))
				continue;
		}
		aw20036_i2c_read(aw20036, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	return len;
}

static ssize_t aw20036_hwen_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		if (databuf[0] == 1){
			aw20036_hw_reset(aw20036);
			aw20036->operating_mode =2;

		}else{
			aw20036_hw_off(aw20036);
			aw20036->operating_mode =0;
		}
	}

	return count;
}

static ssize_t aw20036_hwen_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "hwen=%d\n",
			gpio_get_value(aw20036->reset_gpio));

	return len;
}

static ssize_t aw20036_imax_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	sscanf(buf, "%d", &databuf[0]);
	aw20036_imax_cfg(aw20036, databuf[0]);
	return len;
}

static ssize_t aw20036_imax_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	for (i = 0; i < sizeof(aw20036_imax_name) / AW20036_IMAX_NAME_MAX; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"imax[%x] = %s\n", i, aw20036_imax_name[i]);
	}
	len +=
	    snprintf(buf + len, PAGE_SIZE - len,
		     "current id = 0x%02x, imax = %s\n", aw20036->imax,
		     aw20036_imax_name[aw20036->imax]);

	return len;
}

static ssize_t aw20036_effect_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);
#if defined(AW20036_BIN_CONFIG)
	for (i = 0; i < sizeof(aw20036_cfg_name) / AW20036_CFG_NAME_MAX; i++) {
		len +=
		    snprintf(buf + len, PAGE_SIZE - len, "cfg[%x] = %s\n", i,
			     aw20036_cfg_name[i]);
	}
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "current cfg = %s\n",
		     aw20036_cfg_name[aw20036->effect]);
#elif defined(AW20036_ARRAY_CONFIG)
	for (i = 0; i < sizeof(aw20036_cfg_array) / sizeof(struct aw20036_cfg);
	     i++) {
		len +=
		    snprintf(buf + len, PAGE_SIZE - len, "cfg[%x] = %pf\n", i,
			     aw20036_cfg_array[i].p);
	}
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "current cfg = %pf\n",
		     aw20036_cfg_array[aw20036->effect].p);
#else
	/*Nothing */
#endif
	return len;
}

static ssize_t aw20036_effect_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	sscanf(buf, "%d", &databuf[0]);
	aw20036->effect = databuf[0];
#if defined(AW20036_BIN_CONFIG)
	aw20036_cfg_update(aw20036);
#elif defined(AW20036_ARRAY_CONFIG)
	aw20036_cfg_update_array(aw20036);
#else
	/*Nothing */
#endif

	return len;
}

static ssize_t aw20036_single_brightness_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t len)
{
	unsigned int databuf[2] = { 0, 0 };
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE2);
		aw20036_i2c_write(aw20036, databuf[0], databuf[1]);
	}
	return len;
}

static ssize_t aw20036_all_white_brightness_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	unsigned int val;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);
	unsigned char data_1[13];
	unsigned char data_2[22];

	sscanf(buf, "%d", &val);
	pr_info("%s: %d\n", __func__, val);

	for (i = 0; i < 13; i ++) {data_1[i] = val;}
	for (i = 0; i < 22; i ++) {data_2[i] = val;}

	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE2);
	aw20036_i2c_write_block(aw20036, 0x00, 13, data_1); /*led(0-12)*/
	aw20036_i2c_write_block(aw20036, 0x0E, 22, data_2); /*led(14-35)*/

	return len;
}

//for runin
static ssize_t aw20036_all_brightness_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	unsigned int val;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);
	unsigned char data[36];

	sscanf(buf, "%d", &val);
	pr_info("%s: %d\n", __func__, val);

#ifdef POWER_SAVE_MODE
	if(val > 0){
		aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
		aw20036_i2c_write(aw20036, 0x01, 0x00);
		aw20036->operating_mode =1;
		aw20036_rgbcolor_config(aw20036);
	}else if(val ==0){
		aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
		aw20036_i2c_write(aw20036, 0x01, 0x80);
		aw20036->operating_mode =2;
	}
#endif

	for (i = 0; i < 36; i ++) {data[i] = val;}

	/*Set pag 2 PAD0-PAD35 */
	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE2);
	aw20036_i2c_write_block(aw20036, 0x00, 36, data);
	return len;
}

static ssize_t aw20036_frame_brightness_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);
	char ch = ' ';
	const char *p;
	int val=0;
	unsigned int frame_brightness[33];
	int frame_num =0;
	unsigned int i, num;
	unsigned char brightness_1[13] = {0};
	unsigned char brightness_2[22] = {0};

	pr_info("%s\n", __func__);

	pm_stay_awake(aw20036->dev);
	frame_num = 0;
	if(sscanf(buf, "%d", &val) ==1){
		frame_brightness[frame_num] = val;
		p = strchr(buf, ch);
		while(p){
			p = p+1;
			if(sscanf(p, "%d", &val) ==1){
				frame_num++;
				frame_brightness[frame_num]= val;
				p = strchr(p, ch);
			}else{
				break;
			}
		}
		frame_num ++;
	}

	if(frame_num ==5){
		//int led0[2] = {12,0};
		//int led1= 24;
		int led2[21] = {2, 3, 4, 5, 6, 7, 8, 14, 15, 16, 17, 18, 19, 20, 26, 27, 28, 29, 30, 31, 32};
		int led3[8] = {21, 33, 10, 22, 34, 11, 23, 35};
		//int led4 = 9;

		/*led0*/
		brightness_1[0] = frame_brightness[0];
		brightness_1[12] = frame_brightness[0];

		/*led1*/
		if(strncmp(hw_ver, "T0", 2) == 0){
			brightness_1[1] = frame_brightness[1];
		}else{
			brightness_2[10] = frame_brightness[1];
		}

		/*led2*/
		for(i=0; i<21; i++){
			num = led2[i];
			if(num < 13){
				brightness_1[num] = frame_brightness[2];
			}else{
				brightness_2[num-14] = frame_brightness[2];
			}
		}

		/*led3*/
		for(i=0; i<8; i++){
			num = led3[i];
			if(num < 13){
				brightness_1[num] = frame_brightness[3];
			}else{
				brightness_2[num-14] = frame_brightness[3];
			};
		}

		/*led4*/
		brightness_1[9] = frame_brightness[4];
	}else if(frame_num == 9){
		int led_chager[21] = {9, 21, 33, 10, 22, 34, 11, 23, 35};
		for(i=0; i < 9; i++){
			num = led_chager[i];
			if(num < 13){
				brightness_1[num] = frame_brightness[i];
			}else{
				brightness_2[num-14] = frame_brightness[i];
			};
		}
	}else if(frame_num == 16){
		int led_C1[16] = { 2, 3, 4, 5, 6, 7, 8, 14, 15, 16, 17, 18, 19, 20, 26, 27};
		for(i=0; i < 16; i++){
			num = led_C1[i];
			if(num < 13){
				brightness_1[num] = frame_brightness[i];
			}else{
				brightness_2[num-14] = frame_brightness[i];
			};
		}
	}else if(frame_num == 33){
		int led_33[33] = {12, 0, 24, 2, 3, 4, 5, 6, 7, 8, 14, 15, 16, 17, 18, 19, 20, 26, 27, 28, 29, 30, 31, 32, 9, 21, 33, 10, 22, 34, 11, 23, 35};
		if(strncmp(hw_ver, "T0", 2) == 0){
			led_33[2] = 1;
		}
		for(i=0; i < 33; i++){
			num = led_33[i];
			if(num < 13){
				brightness_1[num] = frame_brightness[i];
			}else{
				brightness_2[num-14] = frame_brightness[i];
			};
		}
	}
	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE2);
	aw20036_i2c_write_block(aw20036, 0x00, 13, brightness_1); /*led(0-12)*/
	aw20036_i2c_write_block(aw20036, 0x0E, 22, brightness_2); /*led(14-35)*/
	pm_relax(aw20036->dev);

	return len;
}

static ssize_t aw20036_operating_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	return sprintf(buf, "%d\n", aw20036->operating_mode);
}

static ssize_t aw20036_operating_mode_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	unsigned int val;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	sscanf(buf, "%d", &val);
	pr_info("%s: %d\n", __func__, val);
	if(val ==1){/*active*/
		if(aw20036->operating_mode ==0){
			aw20036_hw_reset(aw20036);
			aw20036_led_init(aw20036);
#ifdef POWER_SAVE_MODE
			aw20036_i2c_write(aw20036, 0x01, 0x00);
			aw20036_rgbcolor_config(aw20036);
#endif
			aw20036->operating_mode =1;
		}else if(aw20036->operating_mode ==2){
			aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
			aw20036_i2c_write(aw20036, 0x01, 0x00);
			aw20036_rgbcolor_config(aw20036);
			aw20036->operating_mode =1;
		}
	}else if(val ==2){/*stand-by*/
		if(aw20036->operating_mode ==0){
			aw20036_hw_reset(aw20036);
			aw20036->operating_mode =2;
		}else if(aw20036->operating_mode ==1){
			aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
			aw20036_i2c_write(aw20036, 0x01, 0x80);
			aw20036->operating_mode =2;
		}
	}else if(val ==0){/*shut down*/
		aw20036_hw_off(aw20036);
		aw20036->operating_mode =0;
	}
	return len;
}

static ssize_t aw20036_hwid_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	pr_info("%s\n", __func__);

	return sprintf(buf, "%s\n", hw_ver);
}

static ssize_t aw20036_hwid_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	pr_info("%s %s %d\n", __func__, buf, len);

	if(len > sizeof(hw_ver)){
		pr_info("%s: invalid hwid \n", __func__);
		return len;
	}
	memset(hw_ver, 0, sizeof(hw_ver));
	strcpy(hw_ver, buf);
	if(strncmp(hw_ver, "T0", 2) == 0){
		pr_info("%s: hwid is T0\n", __func__);
		aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE1);
		aw20036_i2c_write(aw20036, 0x01, 0x21);
		aw20036_i2c_write(aw20036, 0x18, 0x00);
		aw20036_i2c_write(aw20036, 0x1D, 0x1D);
	}else if(strncmp(hw_ver, "EVT", 3) == 0){
		pr_info("%s: hwid is EVT\n", __func__);
		aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE1);
		aw20036_i2c_write(aw20036, 0x01, 0x00);
		aw20036_i2c_write(aw20036, 0x18, 0x21);
		aw20036_i2c_write(aw20036, 0x1D, 0x1D);
	}else if(strncmp(hw_ver, "DVT", 3) == 0){
		pr_info("%s: hwid is DVT\n", __func__);
		aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE1);
		aw20036_i2c_write(aw20036, 0x01, 0x00);
		aw20036_i2c_write(aw20036, 0x18, 0x21);
		aw20036_i2c_write(aw20036, 0x1D, 0x1B);
	}

	return len;
}

static ssize_t aw20036_dev_color_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	pr_info("%s\n", __func__);

	return sprintf(buf, "%s\n", dev_color);
}

static ssize_t aw20036_dev_color_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	pr_info("%s %s %d\n", __func__, buf, len);

	if(len > sizeof(dev_color)){
		pr_info("%s: invalid hwid \n", __func__);
		return len;
	}
	memset(dev_color, 0, sizeof(dev_color));
	strcpy(dev_color, buf);
	if(strncmp(dev_color, "gray", 4) == 0){
		pr_info("%s: device color is gray\n", __func__);
		aw20036_imax_cfg(aw20036, 0x05);
	}else if(strncmp(dev_color, "white", 5) == 0){
		pr_info("%s: device color is white\n", __func__);
		aw20036_imax_cfg(aw20036, 0x05);
	}

	return len;
}

//for cit
int factory_led1=0, factory_led2=0, factory_led3=0, factory_led4=0, factory_led5 =0;
static ssize_t aw20036_factory_test_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	pr_info("%s\n", __func__);

	return sprintf(buf, "%d %d %d %d %d\n", factory_led1, factory_led2, factory_led3, factory_led4, factory_led5);
}

static ssize_t aw20036_factory_test_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);
	int f_cam_led_br=0, r_cam_leds_br=0, round_leds_br=0, vline_leds_br=0, red_led_br =0;
	unsigned int num;
	int led0[2] = {12, 0};
	int led1 = 24;
	int led2[21] = {2, 3, 4, 5, 6, 7, 8, 14, 15, 16, 17, 18, 19, 20, 26, 27, 28, 29, 30, 31, 32};
	int led3[9] = {9, 21, 33, 10, 22, 34, 11, 23, 35};
	int led4 =13;
	int ret = 0;

	pr_info("%s enter\n", __func__);

	if (sscanf(buf, "%d %d %d %d %d",
		&r_cam_leds_br, &f_cam_led_br, &round_leds_br, &vline_leds_br, &red_led_br) == 5) {

		pr_info("%s set brightness %d %d %d %d %d\n", __func__, r_cam_leds_br, f_cam_led_br, round_leds_br, vline_leds_br, red_led_br);

		if(r_cam_leds_br || f_cam_led_br || round_leds_br || vline_leds_br || red_led_br){
			aw20036->factory_test = 1;
			aw20036->factory_test = 0;
			if(strncmp(dev_color, "gray", 4) == 0){
				pr_info("%s: device color is gray\n", __func__);
				aw20036_imax_cfg(aw20036, 0x05);
			}else{
				aw20036_imax_cfg(aw20036, 0x03);
			}
#ifdef POWER_SAVE_MODE
			ret =aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
			if(ret < 0){
				return ret;
			}
			ret = aw20036_i2c_write(aw20036, 0x01, 0x00);
			if(ret < 0){
				return ret;
			}
			aw20036->operating_mode =1;
			aw20036_rgbcolor_config(aw20036);
			if(ret < 0){
				return ret;
			}
#endif
		}else{
			aw20036->factory_test = 0;
			if(strncmp(dev_color, "gray", 4) == 0){
				pr_info("%s: device color is gray\n", __func__);
				aw20036_imax_cfg(aw20036, 0x05);
			}else{
				aw20036_imax_cfg(aw20036, 0x05);
			}
#ifdef POWER_SAVE_MODE
			ret = aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
			if(ret < 0){
				return ret;
			}
			ret = aw20036_i2c_write(aw20036, 0x01, 0x80);
			if(ret < 0){
				return ret;
			}
			aw20036->operating_mode =2;
#endif
		}

		ret = aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE2);
		if(ret < 0){
			return ret;
		}
		for(num=0; num<2; num++){
			ret = aw20036_i2c_write(aw20036, led0[num], r_cam_leds_br);
			if(ret < 0){
				return ret;
			}
		}
		if(strncmp(hw_ver, "T0", 2) == 0){
			led1 = 1;
			pr_info("%s: hwid is T0\n", __func__);
		}
		ret = aw20036_i2c_write(aw20036, led1, f_cam_led_br);
		if(ret < 0){
			return ret;
		}
		for(num=0; num<21; num++){
			ret = aw20036_i2c_write(aw20036, led2[num], round_leds_br);
			if(ret < 0){
				return ret;
			}
		}
		for(num=0; num<9; num++){
			ret =  aw20036_i2c_write(aw20036, led3[num], vline_leds_br);
			if(ret < 0){
				return ret;
			}
		}
		ret = aw20036_i2c_write(aw20036, led4, red_led_br);
		if(ret < 0){
			return ret;
		}
		factory_led1 = r_cam_leds_br;
		factory_led2 = f_cam_led_br;
		factory_led3 = round_leds_br;
		factory_led4 = vline_leds_br;
		factory_led5 = red_led_br;
		return len;
	}else{
		return -1;
	}
}

 static ssize_t aw20036_vip_notification_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);
	int state, id;

	pr_info("%s\n", __func__);

	if (sscanf(buf, "%d %d", &state, &id) == 2){
		pr_info("%s state:%d, id:%d\n", __func__, state, id);
		if(aw20036->vip_notification != state){
			if(state == 1){
				atomic_set(&aw20036->breath_config, 0);
				aw20036->vip_notification_id = id;
				queue_work(aw20036->leds_workqueue, &aw20036->vip_notification_work);
				aw20036->vip_notification = 1;
			}else if(state == 0){
				aw20036->vip_notification = 0;
				if(atomic_read(&aw20036->breath_config) ==0){
					pr_info("%s breath_config 0\n", __func__);
					wait_for_completion(&aw20036->completion);
				}
				aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
				aw20036_i2c_write(aw20036, REG_PATE, 0x00); //set pattern disable
				aw20036_i2c_write(aw20036, REG_PATGO, 0x00); //set run disable
				aw20036_i2c_write_bits(aw20036, REG_GCCR, BIT_ALLON_MASK, BIT_GCR_ALLON_ENABLE);
				pr_info("%s state:%d end\n", __func__, state);
			}
		}
	}
	return len;
}

static ssize_t aw20036_always_on_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);

	pr_info("%s aw20036->always_on:%d\n", __func__, aw20036->always_on);

	return sprintf(buf, "%d\n", aw20036->always_on);
}

static ssize_t aw20036_always_on_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw20036 *aw20036 = container_of(led_cdev, struct aw20036, cdev);
	unsigned int val;

	sscanf(buf, "%d", &val);

	pr_info("%s val:%d\n", __func__, val);

	aw20036->always_on = val;
	return len;
}

static DEVICE_ATTR(reg, 0664, aw20036_reg_show, aw20036_reg_store);
static DEVICE_ATTR(hwen, 0664, aw20036_hwen_show, aw20036_hwen_store);
static DEVICE_ATTR(imax, 0664, aw20036_imax_show, aw20036_imax_store);
static DEVICE_ATTR(effect, 0664, aw20036_effect_show,  aw20036_effect_store);
static DEVICE_ATTR(single_brightness, 0220, NULL, aw20036_single_brightness_store);
static DEVICE_ATTR(all_white_brightness, 0220, NULL, aw20036_all_white_brightness_store);
static DEVICE_ATTR(all_brightness, 0220, NULL, aw20036_all_brightness_store);
static DEVICE_ATTR(frame_brightness, 0220, NULL, aw20036_frame_brightness_store);
static DEVICE_ATTR(operating_mode, 0664, aw20036_operating_mode_show, aw20036_operating_mode_store);
static DEVICE_ATTR(hwid, 0664, aw20036_hwid_show, aw20036_hwid_store);
static DEVICE_ATTR(dev_color, 0664, aw20036_dev_color_show, aw20036_dev_color_store);
static DEVICE_ATTR(factory_test, 0664, aw20036_factory_test_show, aw20036_factory_test_store);
static DEVICE_ATTR(vip_notification, 0220, NULL, aw20036_vip_notification_store);
static DEVICE_ATTR(always_on, 0664, aw20036_always_on_show, aw20036_always_on_store);

static struct attribute *aw20036_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_imax.attr,
	&dev_attr_effect.attr,
	&dev_attr_single_brightness.attr,
	&dev_attr_all_white_brightness.attr,
	&dev_attr_all_brightness.attr,
	&dev_attr_frame_brightness.attr,
	&dev_attr_operating_mode.attr,
	&dev_attr_hwid.attr,
	&dev_attr_dev_color.attr,
	&dev_attr_factory_test.attr,
	&dev_attr_vip_notification.attr,
	&dev_attr_always_on.attr,
	NULL,
};

static struct attribute_group aw20036_attribute_group = {
	.attrs = aw20036_attributes
};

static int aw20036_init_led_cdev(struct aw20036 *aw20036,
				  struct device_node *np)
{
	struct device_node *temp;
	int ret = -1;

	pr_info("%s: enter\n", __func__);

	for_each_child_of_node(np, temp) {
		ret = of_property_read_string(temp, "aw20036,name",
					      &aw20036->cdev.name);
		if (ret < 0) {
			dev_err(aw20036->dev,
				"Failure reading led name, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw20036,imax",
					   &aw20036->imax);
		if (ret < 0) {
			dev_err(aw20036->dev,
				"Failure reading imax, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw20036,brightness",
					   &aw20036->cdev.brightness);
		if (ret < 0) {
			dev_err(aw20036->dev,
				"Failure reading brightness, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw20036,max_brightness",
					   &aw20036->cdev.max_brightness);
		if (ret < 0) {
			dev_err(aw20036->dev,
				"Failure reading max brightness, ret = %d\n",
				ret);
			goto free_pdata;
		}
	}
	aw20036_led_init(aw20036);
#ifdef POWER_SAVE_MODE
	aw20036->operating_mode =2;
#else
	aw20036->operating_mode =1;
#endif
	INIT_WORK(&aw20036->brightness_work, aw20036_brightness_work);
	aw20036->cdev.brightness_set = aw20036_set_brightness;
	ret = led_classdev_register(aw20036->dev, &aw20036->cdev);
	if (ret) {
		dev_err(aw20036->dev, "unable to register led ret=%d\n", ret);
		goto free_pdata;
	}
	pr_info("%s: sysfs_create_group\n", __func__);

	ret = sysfs_create_group(&aw20036->cdev.dev->kobj,
			       &aw20036_attribute_group);
	if (ret) {
		dev_err(aw20036->dev, "led sysfs ret: %d\n", ret);
		goto free_class;
	}
	return 0;

 free_class:
	led_classdev_unregister(&aw20036->cdev);
 free_pdata:
	return ret;
}

static void aw20036_vip_notification_work(struct work_struct *work)
{
	struct aw20036 *aw20036 = container_of(work, struct aw20036,
			vip_notification_work);
	int i =0;
	unsigned char allON[6] = {0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F};
	/*T1, T2, T3, T4, maxbrihtness, minbrightness, ledon0, ledon1, ledon2, ledon3, ledon4, ledon5*/
	unsigned char pat0_0[12] = {3, 5, 3, 12, 255, 0, 0, 0, 1, 0, 0, 0};
	unsigned char pat1_0[12] = {3, 5, 3, 12, 255, 0, 1, 0, 0, 0, 0, 0};
	unsigned char pat2_0[12] = {3, 5, 3, 12, 255, 0, 0, 0, 0, 0, 1, 0};
	struct aw20036_breath_group breath_effect[]= {
		{pat0_0, pat1_0, pat2_0}
	};
	pr_info("%s\n", __func__);

	pm_stay_awake(aw20036->dev);
	if(aw20036->vip_notification_id > (sizeof(breath_effect)/sizeof(breath_effect[0])-1)){
		pr_info("%s not support this vip notification", __func__);
		if(aw20036->vip_notification == 0){
			complete(&aw20036->completion);
		}else{
			atomic_set(&aw20036->breath_config, 1);
			pr_info("%s end", __func__);
		}
		goto end;
	}
	if(strncmp(hw_ver, "T0", 2) == 0){
		pat2_0[6] = 2;
		pat2_0[10] = 0;
	}
	//step b
	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
	aw20036_i2c_write_bits(aw20036, REG_GCCR, BIT_ALLON_MASK, BIT_GCR_ALLON_DISABLE);//close all
	aw20036_i2c_write_block(aw20036, REG_LEDON0, 6, allON);

	if(aw20036->vip_notification == 0){
		complete(&aw20036->completion);
		goto end;
	}
	aw20036_breath_pattern_0(aw20036, breath_effect[aw20036->vip_notification_id].pat0);
	for(i = 0; i< 500 ; i++){
		if(aw20036->vip_notification == 0){
			complete(&aw20036->completion);
			goto end;
		}
		usleep_range(2000, 2000);
	}
	aw20036_breath_pattern_1(aw20036,  breath_effect[aw20036->vip_notification_id].pat1);
	for(i = 0; i < 500 ; i++){
		if(aw20036->vip_notification == 0){
			complete(&aw20036->completion);
			goto end;
		}
		usleep_range(2000, 2000);
	}
	aw20036_breath_pattern_2(aw20036,  breath_effect[aw20036->vip_notification_id].pat2);
	if(aw20036->vip_notification == 0){
		complete(&aw20036->completion);
	}else{
		atomic_set(&aw20036->breath_config, 1);
		pr_info("%s complete ", __func__);
	}
end:
	pm_relax(aw20036->dev);
}

static void aw20036_clean_buf(struct aw20036 * aw20036, int status)
{
	struct mmap_buf_format *opbuf = aw20036->start_buf;
	int i = 0;

	for (i = 0; i < LED_MMAP_BUF_SUM; i++) {
		memset(opbuf->data, 0, LED_MMAP_BUF_SIZE*2);
		opbuf->status = status;
		opbuf = opbuf->kernel_next;
	}
}

#define MAX_BRIGHTNESS 4095
static void aw20036_leds_effect_work(struct work_struct *work)
{
	struct aw20036 *aw20036 = container_of(work, struct aw20036,
			leds_effect_work);
	uint32_t retry = 0;
	unsigned char brightness_1[13] = {0};
	unsigned char brightness_2[22] = {0};
	ktime_t start, runtime, delay;
       int i=0, j=0, num=0;
	int led2[21] = {2, 3, 4, 5, 6, 7, 8, 14, 15, 16, 17, 18, 19, 20, 26, 27, 28, 29, 30, 31, 32};
	int led3[8] = {21, 33, 10, 22, 34, 11, 23, 35};
	int led_33[33] = {12, 0, 24, 2, 3, 4, 5, 6, 7, 8, 14, 15, 16, 17, 18, 19, 20, 26, 27, 28, 29, 30, 31, 32, 9, 21, 33, 10, 22, 34, 11, 23, 35};

	pr_info("%s\n", __func__);

	pm_stay_awake(aw20036->dev);
	aw20036->curr_buf = aw20036->start_buf;
	do {
		if(aw20036->curr_buf->status == MMAP_BUF_DATA_VALID) {
			pr_info("%s aw20036->curr_buf->length:%d\n", __func__,  aw20036->curr_buf->length);
			if(aw20036->sec_num == 5){
				for (i = 0; i < aw20036->curr_buf->length; i += 5){
					start = ktime_get();

					/*led0*/
					brightness_1[0] = aw20036->curr_buf->data[i]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
					brightness_1[12] = aw20036->curr_buf->data[i]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;

					/*led1*/
					if(strncmp(hw_ver, "T0", 2) == 0){
						brightness_1[1] = aw20036->curr_buf->data[i+1]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
					}else{
						brightness_2[10] = aw20036->curr_buf->data[i+1]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
					}

					/*led2*/
					for(j=0; j<21; j++){
						num = led2[j];
						if(num < 13){
							brightness_1[num] = aw20036->curr_buf->data[i+2]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
						}else{
							brightness_2[num-14] = aw20036->curr_buf->data[i+2]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
						}
					}

					/*led3*/
					for(j=0; j<8; j++){
						num = led3[j];
						if(num < 13){
							brightness_1[num] = aw20036->curr_buf->data[i+3]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
						}else{
							brightness_2[num-14] = aw20036->curr_buf->data[i+3]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
						};
					}

					/*led4*/
					brightness_1[9] = aw20036->curr_buf->data[i+4]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
					if(aw20036->stream_mode ==0){
						pr_info("%s break 1-0\n", __func__);
						break;
					}
					aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE2);
					aw20036_i2c_write_block(aw20036, 0x00, 13, brightness_1); /*led(0-12)*/
					aw20036_i2c_write_block(aw20036, 0x0E, 22, brightness_2); /*led(14-35)*/
					if(aw20036->stream_mode ==0){
						pr_info("%s break 1-1\n", __func__);
						break;
					}
					runtime  = ktime_sub(ktime_get(), start);
					delay = 16666*1000 - runtime;//ns
					if(delay > 0){
						//pr_info("%s delay:%d\n", __func__, delay/1000);
						usleep_range(delay/1000, delay/1000);
					}
					if(aw20036->stream_mode ==0){
						pr_info("%s break 1-2\n", __func__);
						break;
					}
				}
				if((aw20036->curr_buf->length < 500) ||(aw20036->stream_mode ==0)){
					pr_info("%s break 3\n", __func__);
					break;
				}
			}else if(aw20036->sec_num == 33){
				for (i = 0; i < aw20036->curr_buf->length; i += 33){
					start = ktime_get();
					if(strncmp(hw_ver, "T0", 2) == 0){
					       led_33[2] = 1;
					}
					for(j=0; j < 33; j++){
					       num = led_33[j];
					       if(num < 13){
					               brightness_1[num] = aw20036->curr_buf->data[i+j]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
					       }else{
					               brightness_2[num-14] = aw20036->curr_buf->data[i+j]*aw20036->curr_buf->brightness/MAX_BRIGHTNESS;
					       };
					}
					if(aw20036->stream_mode ==0){
						pr_info("%s break 2-0\n", __func__);
						break;
					}
					aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE2);
					aw20036_i2c_write_block(aw20036, 0x00, 13, brightness_1); /*led(0-12)*/
					aw20036_i2c_write_block(aw20036, 0x0E, 22, brightness_2); /*led(14-35)*/
					if(aw20036->stream_mode ==0){
						pr_info("%s break 2-1\n", __func__);
						break;
					}
					runtime  = ktime_sub(ktime_get(), start);
					delay = 16666*1000 - runtime;//ns
					if(delay > 0){
						//pr_info("%s delay:%d\n", __func__, delay/1000);
						usleep_range(delay/1000, delay/1000);
					}
					if(aw20036->stream_mode ==0){
						pr_info("%s break 2-2\n", __func__);
						break;
					}
				}
				if((aw20036->curr_buf->length < 495) ||(aw20036->stream_mode ==0)){
					pr_info("%s break 2-3\n", __func__);
					break;
				}
			}
			aw20036->curr_buf->status = MMAP_BUF_DATA_INVALID;
			aw20036->curr_buf->length = 0;
			aw20036->curr_buf = aw20036->curr_buf->kernel_next;
			retry =0;
		} else if (aw20036->curr_buf->status == MMAP_BUF_DATA_FINISHED) {
			pr_info("%s break 4\n", __func__);
			break;
		} else {
			if(aw20036->stream_mode ==0){
				pr_info("%s break 5\n", __func__);
				break;
			}
			pr_info("%s msleep 1\n", __func__);
			msleep(1);
		}
		if(aw20036->stream_mode ==0){
			pr_info("%s break 6\n", __func__);
			break;
		}
	} while( retry++ < 30);

	memset(brightness_1, 0, 13);
	memset(brightness_2, 0, 22);
	aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE2);
	aw20036_i2c_write_block(aw20036, 0x00, 13, brightness_1); /*led(0-12)*/
	aw20036_i2c_write_block(aw20036, 0x0E, 22, brightness_2); /*led(14-35)*/

	ev_happen = 1;
	ev_code = '1';
	pr_info("%s ev_happen:%d ev_code:%c\n",__func__,ev_happen, ev_code);
	wake_up_interruptible(&aw20036_waitq);
	aw20036_clean_buf(aw20036, MMAP_BUF_DATA_FINISHED);
	pm_relax(aw20036->dev);
}

static int justOpenOnce = 0;
static int aw20036_open(struct inode *inode, struct file *filp)
{
	pr_info("enter\n");
	if (justOpenOnce == 0){
		justOpenOnce++;
	}
	else{
		pr_info("%s err\n", __func__);
	}
	return 0;

}
static int aw20036_release(struct inode *inode, struct file *filp)
{
	if( justOpenOnce > 0){
		pr_info("Now the led_strips has been closed!\n");
		justOpenOnce = 0;
	} else{
		pr_info("The the led_strips has already been closed!\n");
	}

	return 0;
}

static ssize_t aw20036_read(struct file *file, char __user *user, size_t size,loff_t *ppos)
{
	int ret =0;

	pr_info("%s\n", __func__);
	if (size != 1)
		return -EINVAL;
	wait_event_interruptible(aw20036_waitq, ev_happen);
	ret = copy_to_user(user, &ev_code, 1);
	if ( ret == 0 )
	{
		ev_code = '0';
		ev_happen = 0;
		return 1;
	}
	ev_code = '0';
	ev_happen = 0;
	return ret;
}
static int aw20036_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct aw20036 *aw20036 = g_aw20036;
	unsigned long phys;
	int ret = 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 7, 0)
	vm_flags_t vm_flags = calc_vm_prot_bits(PROT_READ|PROT_WRITE, 0) |
			      calc_vm_flag_bits(MAP_SHARED);

	vm_flags |= current->mm->def_flags | VM_MAYREAD | VM_MAYWRITE |
		    VM_MAYEXEC | VM_SHARED | VM_MAYSHARE;

	if (vma && (pgprot_val(vma->vm_page_prot) !=
		   pgprot_val(vm_get_page_prot(vm_flags)))) {
		pr_err("vm_page_prot error!");
		return -EPERM;
	}

	if (vma && ((vma->vm_end - vma->vm_start) !=
		   (PAGE_SIZE << LED_MMAP_PAGE_ORDER))) {
		pr_err("mmap size check err!");
		return -EPERM;
	}
#endif

	pr_info("%s\n", __func__);

	phys = virt_to_phys(aw20036->start_buf);

	ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT), (vma->vm_end - vma->vm_start), vma->vm_page_prot);
	if(ret)
	{
		dev_err(aw20036->dev, "Error mmap failed\n");
		return ret;
	}

	return ret;
}

static unsigned int aw20036_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	pr_info("%s\n", __func__);
	poll_wait(file, &aw20036_waitq, wait);
	if(ev_happen == 1)
	{
		mask |= POLLIN | POLLRDNORM;
	}
	return mask;
}

long aw20036_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct aw20036 *aw20036 = g_aw20036;

	void __user *argp = (void __user *)arg;
	unsigned char buf;
	unsigned char data0[4] = {0xFF, 0x04, 0x58, 0x03};/*init for residual image problem*/
	unsigned char data1[4] = {0xA6, 0x00, 0xA6, 0x00};/*camera fill-in light*/

	switch(cmd){
		case LED_STRIPS_STREAM_MODE:
			pr_info("%s LED_STRIPS_STREAM_MODE\n", __func__);
			if(copy_from_user(&aw20036->sec_num, argp, 1)){
				return -EFAULT;
			}
			pr_info("%s aw20036->sec_num:%d \n", __func__, aw20036->sec_num);
			aw20036->stream_mode =1;
			queue_work(aw20036->leds_workqueue, &aw20036->leds_effect_work);
			break;
		case LED_STRIPS_STOP_MODE:
			pr_info("%s LED_STRIPS_STOP_MODE\n", __func__);
			aw20036->stream_mode =0;
			aw20036_clean_buf(aw20036, MMAP_BUF_DATA_FINISHED);
			break;
		case LED_STRIPS_ALWAYS_ON:
			pr_info("%s LED_STRIPS_ALWAYS_ON \n", __func__);
			if(copy_from_user(&aw20036->always_on, argp, 1)){
				return -EFAULT;
			}
			pr_info("%s aw20036->always_on:%d \n", __func__, aw20036->always_on);
			break;
	      case LED_STRIPS_FREQ_SET:
			pr_info("%s LED_STRIPS_FREQ_SET \n", __func__);
			if(copy_from_user(&buf, argp, 1)){
				return -EFAULT;
			}
			pr_info("%s %d \n", __func__, buf);
			aw20036_reg_page_cfg(aw20036, AW20036_REG_PAGE0);
			if(buf == 0){
				aw20036_i2c_write_block(aw20036, 0x83, 4, data0);
			}else if(buf == 1){
				aw20036_i2c_write_block(aw20036, 0x83, 4, data1);
			}
			break;
		default:
			pr_info("%s:No match Mode.\n", __func__);
			break;
	}
	return 0;
}

static const struct file_operations aw20036_ioctl_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = aw20036_ioctl,
    .open = aw20036_open,
    .read = aw20036_read,
    .mmap = aw20036_mmap,
    .poll = aw20036_poll,
    .release = aw20036_release,
#ifdef CONFIG_COMPAT
    .compat_ioctl = aw20036_ioctl,
#endif
};

static struct miscdevice led_strips_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "led_strips",
	.fops = &aw20036_ioctl_fops,
};

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw20036_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct aw20036 *aw20036;
	struct device_node *np = i2c->dev.of_node;
	int ret;
	int irq_flags;

	pr_info("%s: enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw20036 = devm_kzalloc(&i2c->dev, sizeof(struct aw20036), GFP_KERNEL);
	if (aw20036 == NULL)
		return -ENOMEM;

	aw20036->dev = &i2c->dev;
	aw20036->i2c = i2c;

	aw20036->dev_suspend = 0;

	i2c_set_clientdata(i2c, aw20036);

	mutex_init(&aw20036->cfg_lock);

	/* aw20036 rst & int */
	if (np) {
		ret = aw20036_parse_dt(&i2c->dev, aw20036, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err_parse_dt;
		}
	} else {
		aw20036->reset_gpio = -1;
		aw20036->irq_gpio = -1;
	}

	if (gpio_is_valid(aw20036->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw20036->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw20036_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			goto err_gpio_request;
		}
	}

	if (gpio_is_valid(aw20036->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw20036->irq_gpio,
					    GPIOF_DIR_IN, "aw20036_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			goto err_gpio_request;
		}
	}

	/* hardware reset */
	aw20036_hw_reset(aw20036);
	aw20036->operating_mode = 2;

	/* aw20036 chip id */
	ret = aw20036_read_chipid(aw20036);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw20036_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	/* aw22xxx irq */
	if (gpio_is_valid(aw20036->irq_gpio) &&
	    !(aw20036->flags & AW20036_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw20036->irq_gpio),
						NULL, aw20036_irq, irq_flags,
						"aw20036", aw20036);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw20036->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw20036->flags |= AW20036_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw20036);

	aw20036_init_led_cdev(aw20036, np);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s error creating led class dev\n",
			__func__);
		goto err_sysfs;
	}

	aw20036->start_buf = (struct mmap_buf_format *)__get_free_pages(GFP_KERNEL, LED_MMAP_PAGE_ORDER);
	if(aw20036->start_buf == NULL) {
		dev_err(&i2c->dev, "Error __get_free_pages failed\n");
	}
	SetPageReserved(virt_to_page(aw20036->start_buf));
	{
		struct mmap_buf_format *temp;
		uint32_t i = 0;
		temp = aw20036->start_buf;
		for ( i = 1; i <LED_MMAP_BUF_SUM; i++)
		{
			temp->kernel_next = (aw20036->start_buf + i);
			temp = temp->kernel_next;
		}
		temp->kernel_next = aw20036->start_buf;

		temp = aw20036->start_buf ;
		for (i = 0; i < LED_MMAP_BUF_SUM; i++) {
			temp->bit = i;
			temp = temp->kernel_next;
		}
	}

	pr_info("%s leds_workqueue\n", __func__);
	aw20036->leds_workqueue = create_singlethread_workqueue("leds_wq");
	if (!aw20036->leds_workqueue) {
			dev_err(&i2c->dev, "%s create leds workqueue fail\n", __func__);
	}else{
		pr_info("%s creat work\n", __func__);
		INIT_WORK(&aw20036->vip_notification_work, aw20036_vip_notification_work);
		INIT_WORK(&aw20036->leds_effect_work, aw20036_leds_effect_work);
	}

	init_completion(&aw20036->completion);

	ret = misc_register(&led_strips_dev);
	if (ret) {
		dev_err(&i2c->dev, "%s: misc_register failed\n", __func__);
	}
	g_aw20036 = aw20036;

	pr_info("%s device_init_wakeup\n", __func__);
	device_init_wakeup(aw20036->dev, true);
	pr_info("%s probe completed successfully!\n", __func__);
	return 0;

 err_sysfs:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw20036->irq_gpio), aw20036);
 err_irq:
 err_id:
	devm_gpio_free(&i2c->dev, aw20036->reset_gpio);
	devm_gpio_free(&i2c->dev, aw20036->irq_gpio);
 err_gpio_request:
 err_parse_dt:
	devm_kfree(&i2c->dev, aw20036);
	aw20036 = NULL;
	return ret;
}

static int aw20036_i2c_remove(struct i2c_client *i2c)
{
	struct aw20036 *aw20036 = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);
	sysfs_remove_group(&aw20036->cdev.dev->kobj, &aw20036_attribute_group);
	led_classdev_unregister(&aw20036->cdev);

	devm_free_irq(&i2c->dev, gpio_to_irq(aw20036->irq_gpio), aw20036);

	if (gpio_is_valid(aw20036->reset_gpio))
		devm_gpio_free(&i2c->dev, aw20036->reset_gpio);
	if (gpio_is_valid(aw20036->irq_gpio))
		devm_gpio_free(&i2c->dev, aw20036->irq_gpio);

	devm_kfree(&i2c->dev, aw20036);
	aw20036 = NULL;

	return 0;
}

#define POWER_SUSPEND

#ifdef POWER_SUSPEND
static int aw20036_suspend(struct device *dev)
{
	struct aw20036 *aw20036 = dev_get_drvdata(dev);
	pr_info("%s vip %d fact %d always %d (%d)\n", __func__, aw20036->vip_notification,
		aw20036->factory_test, aw20036->always_on, aw20036->suspend);
	aw20036->dev_suspend = 1;
	if((aw20036->vip_notification !=1) && (aw20036->factory_test !=1) && (aw20036->always_on !=1)){
		pr_info("%s goto suspend\n", __func__);
		aw20036_hw_off(aw20036);
		aw20036->operating_mode =0;
		aw20036->suspend =1;
	}
	return 0;
}
static int aw20036_resume(struct device *dev)
{
	struct aw20036 *aw20036 = dev_get_drvdata(dev);
	pr_info("%s (%d)\n", __func__, aw20036->suspend);
	aw20036->dev_suspend = 0;
	if(aw20036->suspend == 1){
		pr_info("%s is suspend\n", __func__);
		aw20036_hw_reset(aw20036);
		aw20036_led_init(aw20036);
#ifdef POWER_SAVE_MODE
		aw20036->operating_mode =2;
#else
		aw20036->operating_mode =1;
#endif
		aw20036->suspend =0;
	}
	return 0;
}

static const struct dev_pm_ops aw20036_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(aw20036_suspend, aw20036_resume)
};
#endif

static const struct i2c_device_id aw20036_i2c_id[] = {
	{AW20036_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw20036_i2c_id);

static const struct of_device_id aw20036_dt_match[] = {
	{.compatible = "awinic,aw20036_led"},
	{},
};

static struct i2c_driver aw20036_i2c_driver = {
	.driver = {
		   .name = AW20036_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw20036_dt_match),
#ifdef POWER_SUSPEND
		   .pm = &aw20036_pm_ops,
#endif
		   },
	.probe = aw20036_i2c_probe,
	.remove = aw20036_i2c_remove,
	.id_table = aw20036_i2c_id,
};

static int __init aw20036_i2c_init(void)
{
	int ret = 0;

	pr_info("aw20036 driver version %s\n", AW20036_DRIVER_VERSION);

	ret = i2c_add_driver(&aw20036_i2c_driver);
	if (ret) {
		pr_err("fail to add aw20036 device into i2c\n");
		return ret;
	}
	return 0;
}

module_init(aw20036_i2c_init);

static void __exit aw20036_i2c_exit(void)
{
	i2c_del_driver(&aw20036_i2c_driver);
}

module_exit(aw20036_i2c_exit);

MODULE_DESCRIPTION("AW20036 LED Driver");
MODULE_LICENSE("GPL v2");
