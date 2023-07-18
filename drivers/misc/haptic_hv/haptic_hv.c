/*
 * File: haptic_hv.c
 *
 * Author: Ethan <renzhiqiang@awinic.com>
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/mman.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>

#include "haptic_hv.h"
#include "haptic_hv_reg.h"

#ifdef AAC_RICHTAP_SUPPORT
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#endif
#define HAPTIC_HV_DRIVER_VERSION	"v1.2.0"

char *aw_ram_name = "haptic_ram.bin";
char aw_rtp_name[][AW_RTP_NAME_MAX] = {
	{"haptic_rtp_osc_24K_5s.bin"},
	{"haptic_rtp.bin"},
	{"haptic_rtp_lighthouse.bin"},
	{"haptic_rtp_silk.bin"},
};

static char uefi_f0_calidata[MAX_CMDLINE_PARAM_LEN];
static char uefi_f0[MAX_CMDLINE_PARAM_LEN];

#ifdef AW_TIKTAP
static struct aw_haptic *g_aw_haptic;
#endif
#ifdef AW_DOUBLE
struct aw_haptic *left;
struct aw_haptic *right;
#elif defined(AAC_RICHTAP_SUPPORT)
struct aw_haptic *g_aw_haptic = NULL;
#endif

/*********************************************************
 *
 * I2C Read/Write
 *
 *********************************************************/
int haptic_hv_i2c_reads(struct aw_haptic *aw_haptic, uint8_t reg_addr,
			uint8_t *buf, uint32_t len)
{
	int ret;
	struct i2c_msg msg[] = {
		[0] = {
			.addr = aw_haptic->i2c->addr,
			.flags = 0,
			.len = sizeof(uint8_t),
			.buf = &reg_addr,
			},
		[1] = {
			.addr = aw_haptic->i2c->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
			},
	};

	ret = i2c_transfer(aw_haptic->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		aw_err("transfer failed.%d",ret);
		return ret;
	} else if (ret != AW_I2C_READ_MSG_NUM) {
		aw_err("transfer failed(size error).");
		return -ENXIO;
	}

	return ret;
}

int haptic_hv_i2c_writes(struct aw_haptic *aw_haptic, uint8_t reg_addr,
			 uint8_t *buf, uint32_t len)
{
	uint8_t *data = NULL;
	int ret = -1;

	data = kmalloc(len + 1, GFP_KERNEL);
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);
	ret = i2c_master_send(aw_haptic->i2c, data, len + 1);
	if (ret < 0)
		aw_err("i2c master send 0x%02x err", reg_addr);
	kfree(data);
	return ret;
}

int haptic_hv_i2c_write_bits(struct aw_haptic *aw_haptic, uint8_t reg_addr,
			     uint32_t mask, uint8_t reg_data)
{
	uint8_t reg_val = 0;
	int ret = -1;

	ret = haptic_hv_i2c_reads(aw_haptic, reg_addr, &reg_val,
				  AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_err("i2c read error, ret=%d", ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	ret = haptic_hv_i2c_writes(aw_haptic, reg_addr, &reg_val,
				   AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_err("i2c write error, ret=%d", ret);
		return ret;
	}
	return 0;
}

static int parse_dt_gpio(struct device *dev, struct aw_haptic *aw_haptic,
			 struct device_node *np)
{
	int val = 0;

	aw_haptic->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw_haptic->reset_gpio < 0) {
		aw_err("no reset gpio provide");
		return -EPERM;
	}
	aw_info("reset gpio provide ok %d", aw_haptic->reset_gpio);
	aw_haptic->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw_haptic->irq_gpio < 0)
		aw_err("no irq gpio provided.");
	else
		aw_info("irq gpio provide ok irq = %d.", aw_haptic->irq_gpio);
	val = of_property_read_u8(np, "mode", &aw_haptic->info.mode);
	if (val != 0)
		aw_info("mode not found");
#ifdef AW_DOUBLE
	if (of_device_is_compatible(np, "awinic,haptic_hv_l")) {
		aw_info("compatible left vibrator.");
		memcpy(aw_haptic->name, "left", sizeof("left"));
		left = NULL;
		left = aw_haptic;
	} else if (of_device_is_compatible(np, "awinic,haptic_hv_r")) {
		aw_info("compatible right vibrator.");
		memcpy(aw_haptic->name, "right", sizeof("right"));
		right = NULL;
		right = aw_haptic;
	} else {
		aw_err("compatible failed.");
		return -ERANGE;
	}
#endif

	return 0;
}

static void hw_reset(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	if (aw_haptic && gpio_is_valid(aw_haptic->reset_gpio)) {
		gpio_set_value_cansleep(aw_haptic->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(aw_haptic->reset_gpio, 1);
		usleep_range(8000, 8500);
	} else {
		aw_err("failed");
	}
}

static void sw_reset(struct aw_haptic *aw_haptic)
{
	uint8_t reset = AW_BIT_RESET;

	aw_dbg("enter");
	haptic_hv_i2c_writes(aw_haptic, AW_REG_CHIPID, &reset, AW_I2C_BYTE_ONE);
	usleep_range(3000, 3500);
}

#if 0
static int judge_value(uint8_t reg)
{
	int ret = 0;

	if (!reg)
		return -ERANGE;
	switch (reg) {
	case AW86925_BIT_RSTCFG_PRE_VAL:
	case AW86926_BIT_RSTCFG_PRE_VAL:
	case AW86927_BIT_RSTCFG_PRE_VAL:
	case AW86928_BIT_RSTCFG_PRE_VAL:
	case AW86925_BIT_RSTCFG_VAL:
	case AW86926_BIT_RSTCFG_VAL:
	case AW86927_BIT_RSTCFG_VAL:
	case AW86928_BIT_RSTCFG_VAL:
		ret = -ERANGE;
		break;
	default:
		break;
	}
	return ret;
}
#endif

static int read_chipid(struct aw_haptic *aw_haptic, uint32_t *reg_val)
{
	uint8_t value[2] = {0};
	int ret = 0;

	/* try the old way of read chip id */
	#if 0
	ret = haptic_hv_i2c_reads(aw_haptic, AW_REG_CHIPID, &value[0],
				  AW_I2C_BYTE_ONE);
	if (ret < 0)
		return ret;

	ret = judge_value(value[0]);
	if (!ret) {
		*reg_val = value[0];
		return ret;
	}
	#endif
	/* try the new way of read chip id */
	ret = haptic_hv_i2c_reads(aw_haptic, AW_REG_CHIPIDH, value,
				  AW_I2C_BYTE_TWO);
	if (ret < 0)
		return ret;
	*reg_val = value[0] << 8 | value[1];
	return ret;
}

static int ctrl_init(struct aw_haptic *aw_haptic)
{
	uint32_t reg = 0;
	uint8_t cnt = 0;

	aw_info("enter");
	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		/* hardware reset */
		hw_reset(aw_haptic);
		if (read_chipid(aw_haptic, &reg) < 0)
			aw_err("read chip id fail");
		switch (reg) {
#ifdef AW869X_DRIVER_ENABLE
		case AW8695_CHIPID:
		case AW8697_CHIPID:
			aw_haptic->func = &aw869x_func_list;
			return 0;
#endif
#ifdef AW869XX_DRIVER_ENABLE
		case AW86905_CHIPID:
		case AW86907_CHIPID:
		case AW86915_CHIPID:
		case AW86917_CHIPID:
			aw_haptic->func = &aw869xx_func_list;
			return 0;
#endif
#ifdef AW8671X_DRIVER_ENABLE
		case AW86715_CHIPID:
		case AW86716_CHIPID:
		case AW86717_CHIPID:
		case AW86718_CHIPID:
			aw_haptic->func = &aw8671x_func_list;
			return 0;
#endif
#ifdef AW8692X_DRIVER_ENABLE
		case AW86925_CHIPID:
		case AW86926_CHIPID:
		case AW86927_CHIPID:
		case AW86928_CHIPID:
			aw_haptic->func = &aw8692x_func_list;
			return 0;
#endif
		default:
			aw_info("unexpected chipid!");
			break;
		}
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}
	return -EINVAL;
}

static int parse_chipid(struct aw_haptic *aw_haptic)
{
	int ret = -1;
	uint32_t reg = 0;
	uint8_t cnt = 0;

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		ret = read_chipid(aw_haptic, &reg);
		if (ret < 0)
			aw_err("read chip id fail: %d", ret);
		switch (reg) {
#ifdef AW869X_DRIVER_ENABLE
		case AW8695_CHIPID:
			aw_haptic->chipid = AW8695_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->i2s_config = false;
			aw_info("detected aw8695.");
			return ret;
		case AW8697_CHIPID:
			aw_haptic->chipid = AW8697_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L2;
			aw_haptic->i2s_config = false;
			aw_info("detected aw8697.");
			return ret;
#endif
#ifdef AW869XX_DRIVER_ENABLE
		case AW86905_CHIPID:
			aw_haptic->chipid = AW86905_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->i2s_config = false;
			aw_info("detected aw86905.");
			return 0;
		case AW86907_CHIPID:
			aw_haptic->chipid = AW86907_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L2;
			aw_haptic->i2s_config = false;
			aw_info("detected aw86907.");
			return 0;
		case AW86915_CHIPID:
			aw_haptic->chipid = AW86915_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->i2s_config = true;
			aw_info("detected aw86915.");
			return 0;
		case AW86917_CHIPID:
			aw_haptic->chipid = AW86917_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L2;
			aw_haptic->i2s_config = true;
			aw_info("detected aw86917.");
			return 0;
#endif
#ifdef AW8671X_DRIVER_ENABLE
		case AW86715_CHIPID:
			aw_haptic->chipid = AW86715_CHIPID;
			aw_haptic->i2s_config = false;
			aw_info("detected aw86715.");
			return 0;
		case AW86716_CHIPID:
			aw_haptic->chipid = AW86716_CHIPID;
			aw_haptic->i2s_config = true;
			aw_info("detected aw86716.");
			return 0;
		case AW86717_CHIPID:
			aw_haptic->chipid = AW86717_CHIPID;
			aw_haptic->i2s_config = true;
			aw_info("detected aw86717.");
			return 0;
		case AW86718_CHIPID:
			aw_haptic->chipid = AW86718_CHIPID;
			aw_haptic->i2s_config = true;
			aw_info("detected aw86718.");
			return 0;
#endif
#ifdef AW8692X_DRIVER_ENABLE
		case AW86925_CHIPID:
			aw_haptic->chipid = AW86925_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_info("detected aw86925.");
			return 0;
		case AW86926_CHIPID:
			aw_haptic->chipid = AW86926_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_info("detected aw86926.");
			return 0;
		case AW86927_CHIPID:
			aw_haptic->chipid = AW86927_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_info("detected aw86927.");
			return 0;
		case AW86928_CHIPID:
			aw_haptic->chipid = AW86928_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_info("detected aw86928.");
			return 0;
#endif
		default:
			aw_info("unsupport device revision (0x%02X)", reg);
			break;
		}
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}
	return -EINVAL;
}

static void ram_play(struct aw_haptic *aw_haptic, uint8_t mode)
{
#ifdef AW_DOUBLE
	if (mode == AW_RAM_MODE)
		aw_haptic->func->set_wav_loop(aw_haptic, 0x00, 0x00);
	aw_haptic->func->play_mode(aw_haptic, mode);
	if (aw_haptic->dual_flag) {
		aw_haptic->dual_flag = false;
		if (down_trylock(&left->sema) == 0) {
			down_interruptible(&left->sema);
		} else {
			up(&left->sema);
			up(&left->sema);
		}
	}
#else
	aw_haptic->func->play_mode(aw_haptic, mode);
#endif
	aw_haptic->func->play_go(aw_haptic, true);
}

static int get_ram_num(struct aw_haptic *aw_haptic)
{
	uint8_t wave_addr[2] = {0};
	uint32_t first_wave_addr = 0;

	if (!aw_haptic->ram_init) {
		aw_err("ram init faild, ram_num = 0!");
		return -EPERM;
	}
	mutex_lock(&aw_haptic->lock);
	/* RAMINIT Enable */
	aw_haptic->func->ram_init(aw_haptic, true);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_ram_addr(aw_haptic);
	aw_haptic->func->get_first_wave_addr(aw_haptic, wave_addr);
	first_wave_addr = (wave_addr[0] << 8 | wave_addr[1]);
	aw_haptic->ram.ram_num =
			(first_wave_addr - aw_haptic->ram.base_addr - 1) / 4;
	aw_info("first wave addr = 0x%04x", first_wave_addr);
	aw_info("ram_num = %d", aw_haptic->ram.ram_num);
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);
	return 0;
}

static void ram_vbat_comp(struct aw_haptic *aw_haptic, bool flag)
{
	int temp_gain = 0;

	if (flag) {
		if (aw_haptic->ram_vbat_comp == AW_RAM_VBAT_COMP_ENABLE) {
			aw_haptic->func->get_vbat(aw_haptic);
			temp_gain = aw_haptic->gain * AW_VBAT_REFER /
				    aw_haptic->vbat;
			if (temp_gain >
			    (128 * AW_VBAT_REFER / AW_VBAT_MIN)) {
				temp_gain = 128 * AW_VBAT_REFER / AW_VBAT_MIN;
				aw_dbg("gain limit=%d", temp_gain);
			}
			aw_haptic->func->set_gain(aw_haptic, temp_gain);
			aw_info("ram vbat comp open");
		} else {
			aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
			aw_info("ram vbat comp close");
		}
	} else {
		aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
		aw_info("ram vbat comp close");
	}
}

static int judge_cali_range(struct aw_haptic *aw_haptic)
{
	uint32_t f0_cali_min = 0;
	uint32_t f0_cali_max = 0;

	f0_cali_min = aw_haptic->info.f0_pre *
			(100 - aw_haptic->info.f0_cali_percent) / 100;
	f0_cali_max = aw_haptic->info.f0_pre *
			(100 + aw_haptic->info.f0_cali_percent) / 100;

	aw_info("f0_pre = %d, f0_cali_min = %d, f0_cali_max = %d, f0 = %d",
		aw_haptic->info.f0_pre, f0_cali_min, f0_cali_max,
		aw_haptic->f0);

	if ((aw_haptic->f0 < f0_cali_min) || (aw_haptic->f0 > f0_cali_max)) {
		aw_err("f0 calibration out of range = %d!", aw_haptic->f0);
		return -ERANGE;
	}
	return 0;
}

static void calculate_cali_data(struct aw_haptic *aw_haptic)
{
	char f0_cali_lra = 0;
	int f0_cali_step = 0;

	/* calculate cali step */
	f0_cali_step = 100000 * ((int)aw_haptic->f0 -
				 (int)aw_haptic->info.f0_pre) /
				((int)aw_haptic->f0 * AW_OSC_CALI_ACCURACY);
	aw_info("f0_cali_step = %d", f0_cali_step);
	if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
		if (f0_cali_step % 10 >= 5)
			f0_cali_step = 32 + (f0_cali_step / 10 + 1);
		else
			f0_cali_step = 32 + f0_cali_step / 10;
	} else {	/* f0_cali_step < 0 */
		if (f0_cali_step % 10 <= -5)
			f0_cali_step = 32 + (f0_cali_step / 10 - 1);
		else
			f0_cali_step = 32 + f0_cali_step / 10;
	}
	if (f0_cali_step > 31)
		f0_cali_lra = (char)f0_cali_step - 32;
	else
		f0_cali_lra = (char)f0_cali_step + 32;
	/* update cali step */
	aw_haptic->f0_cali_data = (int)f0_cali_lra;
	aw_info("f0_cali_data = 0x%02X", aw_haptic->f0_cali_data);
}

static int f0_cali(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	if (aw_haptic->func->get_f0(aw_haptic)) {
		aw_err("get f0 error");
	} else {
		ret = judge_cali_range(aw_haptic);
		if (ret < 0)
			return -ERANGE;
		calculate_cali_data(aw_haptic);
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	}
	/* restore standby work mode */
	aw_haptic->func->play_stop(aw_haptic);
	return ret;
}

static int ram_f0_cali(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	if (aw_haptic->func->ram_get_f0(aw_haptic)) {
		aw_err("get f0 error");
	} else {
		ret = judge_cali_range(aw_haptic);
		if (ret < 0)
			return -ERANGE;
		calculate_cali_data(aw_haptic);
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	}
	/* restore standby work mode */
	aw_haptic->func->play_stop(aw_haptic);
	return ret;
}

static void pm_qos_enable(struct aw_haptic *aw_haptic, bool enable)
{
#ifdef KERNEL_OVER_5_10
	if (enable) {
		if (!cpu_latency_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			cpu_latency_qos_add_request(&aw_haptic->aw_pm_qos_req_vb, CPU_LATENCY_QOC_VALUE);
	} else {
		if (cpu_latency_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			cpu_latency_qos_remove_request(&aw_haptic->aw_pm_qos_req_vb);
	}
#else
	if (enable) {
		if (!pm_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			pm_qos_add_request(&aw_haptic->aw_pm_qos_req_vb,
					PM_QOS_CPU_DMA_LATENCY,
					AW_PM_QOS_VALUE_VB);
	} else {
		if (pm_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			pm_qos_remove_request(&aw_haptic->aw_pm_qos_req_vb);
	}
#endif
}

static int rtp_osc_cali(struct aw_haptic *aw_haptic)
{
	uint32_t buf_len = 0;
	int ret = -1;
	const struct firmware *rtp_file;

	aw_haptic->rtp_cnt = 0;
	aw_haptic->timeval_flags = 1;

	/* fw loaded */
	ret = request_firmware(&rtp_file, aw_rtp_name[0], aw_haptic->dev);
	if (ret < 0) {
		aw_err("failed to read %s", aw_rtp_name[0]);
		return ret;
	}
	/*aw_haptic add stop,for irq interrupt during calibrate */
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->rtp_init = false;
	mutex_lock(&aw_haptic->rtp_lock);
	vfree(aw_haptic->aw_rtp);
	aw_haptic->aw_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw_haptic->aw_rtp) {
		release_firmware(rtp_file);
		mutex_unlock(&aw_haptic->rtp_lock);
		aw_err("error allocating memory");
		return -ENOMEM;
	}
	aw_haptic->aw_rtp->len = rtp_file->size;
	aw_haptic->rtp_len = rtp_file->size;
	aw_info("rtp file:[%s] size = %dbytes", aw_rtp_name[0],
		aw_haptic->aw_rtp->len);
	memcpy(aw_haptic->aw_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);
	mutex_unlock(&aw_haptic->rtp_lock);
	/* gain */
	ram_vbat_comp(aw_haptic, false);
	/* rtp mode config */
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	/* bst mode */
	aw_haptic->func->bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
	disable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	/* haptic go */
	aw_haptic->func->play_go(aw_haptic, true);
	mutex_lock(&aw_haptic->rtp_lock);
	pm_qos_enable(aw_haptic, true);
	while (1) {
		if (!aw_haptic->func->rtp_get_fifo_afs(aw_haptic)) {
#ifdef AW_ENABLE_RTP_PRINT_LOG
			aw_info("not almost_full, aw_haptic->rtp_cnt=%d",
				aw_haptic->rtp_cnt);
#endif
			if ((aw_haptic->aw_rtp->len - aw_haptic->rtp_cnt) <
			    (aw_haptic->ram.base_addr >> 2))
				buf_len = aw_haptic->aw_rtp->len - aw_haptic->rtp_cnt;
			else
				buf_len = (aw_haptic->ram.base_addr >> 2);
			if (aw_haptic->rtp_cnt != aw_haptic->aw_rtp->len) {
				if (aw_haptic->timeval_flags == 1) {
					aw_haptic->kstart = ktime_get();
					aw_haptic->timeval_flags = 0;
				}
				aw_haptic->func->set_rtp_data(aw_haptic,
						&aw_haptic->aw_rtp->data
						[aw_haptic->rtp_cnt], buf_len);
				aw_haptic->rtp_cnt += buf_len;
			}
		}
		if (aw_haptic->func->get_osc_status(aw_haptic)) {
			aw_haptic->kend = ktime_get();
			aw_info("osc trim playback done aw_haptic->rtp_cnt= %d",
				aw_haptic->rtp_cnt);
			break;
		}
		aw_haptic->kend = ktime_get();
		aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend,
							    aw_haptic->kstart));
		if (aw_haptic->microsecond > AW_OSC_CALI_MAX_LENGTH) {
			aw_info("osc trim time out! aw_haptic->rtp_cnt %d",
				aw_haptic->rtp_cnt);
			break;
		}
	}
	pm_qos_enable(aw_haptic, false);
	mutex_unlock(&aw_haptic->rtp_lock);
	enable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend,
						       aw_haptic->kstart));
	/*calibration osc */
	aw_info("microsecond: %llu", aw_haptic->microsecond);
	return 0;
}

static void rtp_trim_lra_cali(struct aw_haptic *aw_haptic)
{
#ifdef AW_OSC_MULTI_CALI
	uint8_t last_trim_code = 0;
	int temp = 0;
	int count = 5;
#endif
	uint32_t lra_trim_code = 0;
	/*0.1 percent below no need to calibrate */
	uint32_t osc_cali_threshold = 10;
	uint32_t real_code = 0;
	uint32_t theory_time = 0;
	uint32_t real_time = 0;

	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
#ifdef AW_OSC_MULTI_CALI
	while (count) {
#endif
	rtp_osc_cali(aw_haptic);
	real_time = aw_haptic->microsecond;
	theory_time = aw_haptic->func->get_theory_time(aw_haptic);
	if (theory_time == real_time) {
		aw_info("theory_time == real_time: %d, no need to calibrate!",
			real_time);
		return;
	} else if (theory_time < real_time) {
		if ((real_time - theory_time) >
			(theory_time / AW_OSC_TRIM_PARAM)) {
			aw_info("(real_time - theory_time) > (theory_time/50), can't calibrate!");
			return;
		}

		if ((real_time - theory_time) <
		    (osc_cali_threshold * theory_time / 10000)) {
			aw_info("real_time: %d, theory_time: %d, no need to calibrate!",
				real_time, theory_time);
			return;
		}

		real_code = 100000 * ((real_time - theory_time)) /
			    (theory_time * AW_OSC_CALI_ACCURACY);
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = 32 + real_code;
	} else if (theory_time > real_time) {
		if ((theory_time - real_time) >
			(theory_time / AW_OSC_TRIM_PARAM)) {
			aw_info("(theory_time - real_time) > (theory_time / 50), can't calibrate!");
			return;
		}
		if ((theory_time - real_time) <
		    (osc_cali_threshold * theory_time / 10000)) {
			aw_info("real_time: %d, theory_time: %d, no need to calibrate!",
				real_time, theory_time);
			return;
		}

		real_code = (theory_time - real_time) / (theory_time / 100000) / AW_OSC_CALI_ACCURACY;
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = 32 - real_code;
	}
	if (real_code > 31)
		lra_trim_code = real_code - 32;
	else
		lra_trim_code = real_code + 32;
#ifdef AW_OSC_MULTI_CALI
	last_trim_code = aw_haptic->func->get_trim_lra(aw_haptic);
	if (last_trim_code) {
		if (lra_trim_code >= 32) {
			temp = last_trim_code - (64 - lra_trim_code);
			if (temp < 32 && temp > 0 && last_trim_code >= 32)
				temp = 32;
			if (temp < 0)
				temp = lra_trim_code + last_trim_code;
		} else if (lra_trim_code > 0 && lra_trim_code < 32) {
			temp = (last_trim_code + lra_trim_code) & 0x3f;
			if (temp >= 32 && last_trim_code <= 31)
				temp = 31;
			if ((64 - last_trim_code) <= lra_trim_code)
				temp = last_trim_code + lra_trim_code - 64;
		} else {
			temp = last_trim_code;
		}
		aw_haptic->osc_cali_data = temp;
	} else {
		aw_haptic->osc_cali_data = lra_trim_code;
	}
#else
	aw_haptic->osc_cali_data = lra_trim_code;
#endif
	aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	aw_info("real_time: %d, theory_time: %d", real_time, theory_time);
	aw_info("real_code: %d, trim_lra: 0x%02X", real_code, lra_trim_code);
#ifdef AW_OSC_MULTI_CALI
	count--;
	}
#endif
}

#ifdef AW_INPUT_FRAMEWORK
static void input_gain_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   gain_work);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	mutex_unlock(&aw_haptic->lock);
}

static void input_vib_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   input_vib_work);

	mutex_lock(&aw_haptic->lock);
	/* Enter standby mode */
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->state) {
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
		if (aw_haptic->activate_mode == AW_RAM_MODE) {
			ram_vbat_comp(aw_haptic, false);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x00,
						     aw_haptic->index);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
			aw_haptic->func->set_wav_loop(aw_haptic, 0x00, 0x00);
			ram_play(aw_haptic, AW_RAM_MODE);
		} else if (aw_haptic->activate_mode == AW_RAM_LOOP_MODE) {
			ram_vbat_comp(aw_haptic, true);
			aw_haptic->func->set_repeat_seq(aw_haptic,
							aw_haptic->index);
			ram_play(aw_haptic, AW_RAM_LOOP_MODE);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer,
				      ktime_set(aw_haptic->duration / 1000,
						(aw_haptic->duration % 1000) *
						1000000), HRTIMER_MODE_REL);
		} else {
			aw_err("activate_mode error");
		}
	}
	mutex_unlock(&aw_haptic->lock);
}

static int input_upload_effect(struct input_dev *dev, struct ff_effect *effect,
			       struct ff_effect *old)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);
	short wav_id = 0;
	int wav_id_max = 0;
	int ret = 0;

	mutex_lock(&aw_haptic->lock);
	switch (effect->type) {
	case FF_CONSTANT:
		aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
		aw_haptic->duration = effect->replay.length;
		aw_haptic->index = aw_haptic->ram.ram_num;
		aw_info("waveform id = %d", aw_haptic->index);
		break;
	case FF_PERIODIC:
		ret = copy_from_user(&wav_id, effect->u.periodic.custom_data,
				     sizeof(short));
		if (ret) {
			aw_err("copy from user error %d!!", ret);
			mutex_unlock(&aw_haptic->lock);
			return -ERANGE;
		}
		aw_info("waveform id = %d", wav_id);
		wav_id_max = sizeof(aw_rtp_name) / sizeof(*aw_rtp_name) +
			     aw_haptic->ram.ram_num - 1;
		if (wav_id > 0 && wav_id < aw_haptic->ram.ram_num) {
			aw_haptic->activate_mode = AW_RAM_MODE;
			aw_haptic->index = wav_id;
		} else if (wav_id > aw_haptic->ram.ram_num && wav_id <= wav_id_max) {
			aw_haptic->activate_mode = AW_RTP_MODE;
			aw_haptic->rtp_file_num = wav_id - aw_haptic->ram.ram_num;
		} else {
			aw_haptic->activate_mode = AW_STANDBY_MODE;
			aw_err("waveform id is error");
			mutex_unlock(&aw_haptic->lock);
			return -ERANGE;
		}
		break;
	default:
		aw_err("Unsupported effect type: %d", effect->type);
		break;
	}
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int input_playback(struct input_dev *dev, int effect_id, int val)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return -ERANGE;
	}
	if (val > 0)
		aw_haptic->state = 1;
	if (val <= 0)
		aw_haptic->state = 0;
	switch (aw_haptic->activate_mode) {
	case AW_RAM_MODE:
		schedule_work(&aw_haptic->input_vib_work);
		break;
	case AW_RAM_LOOP_MODE:
		schedule_work(&aw_haptic->input_vib_work);
		break;
	case AW_RTP_MODE:
		schedule_work(&aw_haptic->rtp_work);
		break;
	default:
		aw_err("Unsupported mode: %d", aw_haptic->activate_mode);
		break;
	}

	return 0;
}

static int input_erase(struct input_dev *dev, int effect_id)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	aw_haptic->duration = 0;
	return 0;
}

static void input_set_gain(struct input_dev *dev, uint16_t gain)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	if (gain > 0x7fff)
		gain = 0x7fff;
	aw_haptic->gain = gain * 0x80 / 0x7fff;
	schedule_work(&aw_haptic->gain_work);
	aw_info("aw_haptic->gain = 0x%02x", aw_haptic->gain);
}

static int input_framework_init(struct aw_haptic *aw_haptic)
{
	struct input_dev *input_dev;
	int ret = 0;

	input_dev = devm_input_allocate_device(aw_haptic->dev);
	if (input_dev == NULL)
		return -ENOMEM;
	input_dev->name = "aw-haptic-hv";
	input_set_drvdata(input_dev, aw_haptic);
	aw_haptic->input_dev = input_dev;
	input_set_capability(input_dev, EV_FF, FF_GAIN);
	input_set_capability(input_dev, EV_FF, FF_CONSTANT);
	input_set_capability(input_dev, EV_FF, FF_PERIODIC);
	input_set_capability(input_dev, EV_FF, FF_CUSTOM);
	ret = input_ff_create(input_dev, AW_EFFECT_NUMBER);
	if (ret < 0) {
		aw_err("create input FF device failed, rc=%d\n", ret);
		return ret;
	}
	input_dev->ff->upload = input_upload_effect;
	input_dev->ff->playback = input_playback;
	input_dev->ff->erase = input_erase;
	input_dev->ff->set_gain = input_set_gain;
	INIT_WORK(&aw_haptic->gain_work, input_gain_work_routine);
	INIT_WORK(&aw_haptic->input_vib_work, input_vib_work_routine);
	ret = input_register_device(input_dev);
	if (ret < 0) {
		aw_err("register input device failed, rc=%d\n", ret);
		input_ff_destroy(aw_haptic->input_dev);
		return ret;
	}
	return ret;
}
#endif

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct aw_haptic *aw_haptic = container_of(timer, struct aw_haptic,
						   timer);

	aw_haptic->state = 0;
	schedule_work(&aw_haptic->vibrator_work);
	return HRTIMER_NORESTART;
}

#ifdef AW_DURATION_DECIDE_WAVEFORM
static int ram_select_waveform(struct aw_haptic *aw_haptic)
{
	uint8_t wavseq = 0;
	uint8_t wavloop = 0;

	if (aw_haptic->duration <= 0) {
		aw_err("duration time %d error", aw_haptic->duration);
		return -ERANGE;
	}
	aw_haptic->activate_mode = AW_RAM_MODE;
	if ((aw_haptic->duration > 0) &&
	    (aw_haptic->duration < aw_haptic->info.duration_time[0])) {
		wavseq = 3;
	} else if ((aw_haptic->duration >= aw_haptic->info.duration_time[0]) &&
		   (aw_haptic->duration < aw_haptic->info.duration_time[1])) {
		wavseq = 2;
	} else if ((aw_haptic->duration >= aw_haptic->info.duration_time[1]) &&
		   (aw_haptic->duration < aw_haptic->info.duration_time[2])) {
		wavseq = 1;
	} else if (aw_haptic->duration >= aw_haptic->info.duration_time[2]) {
		wavseq = 4;
		wavloop = 15;
		aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
	}
	aw_info("duration %d, select index %d", aw_haptic->duration, wavseq);
	aw_haptic->func->set_wav_seq(aw_haptic, 0, wavseq);
	aw_haptic->func->set_wav_loop(aw_haptic, 0, wavloop);
	aw_haptic->func->set_wav_seq(aw_haptic, 1, 0);
	aw_haptic->func->set_wav_loop(aw_haptic, 1, 0);
	return 0;
}
#endif

static void vibrator_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   vibrator_work);

	mutex_lock(&aw_haptic->lock);
	/* Enter standby mode */
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->state) {
#ifdef AW_DURATION_DECIDE_WAVEFORM
		ram_select_waveform(aw_haptic);
#endif
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
		if (aw_haptic->activate_mode == AW_RAM_MODE) {
			ram_vbat_comp(aw_haptic, false);
			ram_play(aw_haptic, AW_RAM_MODE);
		} else if (aw_haptic->activate_mode == AW_RAM_LOOP_MODE) {
			ram_vbat_comp(aw_haptic, true);
			ram_play(aw_haptic, AW_RAM_LOOP_MODE);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer,
				      ktime_set(aw_haptic->duration / 1000,
						(aw_haptic->duration % 1000) *
						1000000), HRTIMER_MODE_REL);
		} else if (aw_haptic->activate_mode == AW_CONT_MODE) {
			aw_haptic->func->cont_config(aw_haptic);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer,
				      ktime_set(aw_haptic->duration / 1000,
						(aw_haptic->duration % 1000) *
						1000000), HRTIMER_MODE_REL);
		} else {
			aw_err("activate_mode error");
		}
	}
	mutex_unlock(&aw_haptic->lock);
}

static int rtp_play(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state_val = 0;
	uint32_t buf_len = 0;
	int ret = 0;
	struct aw_haptic_container *aw_rtp = aw_haptic->aw_rtp;

	while ((!aw_haptic->func->rtp_get_fifo_afs(aw_haptic))
	       && (aw_haptic->play_mode == AW_RTP_MODE)) {
#ifdef AW_ENABLE_RTP_PRINT_LOG
		aw_info("rtp cnt = %d", aw_haptic->rtp_cnt);
#endif
		if (!aw_rtp) {
			aw_info("aw_rtp is null, break!");
			ret = -ERANGE;
			break;
		}
		if (aw_haptic->rtp_cnt < (aw_haptic->ram.base_addr)) {
			if ((aw_rtp->len - aw_haptic->rtp_cnt) <
			    (aw_haptic->ram.base_addr)) {
				buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
			} else {
				buf_len = aw_haptic->ram.base_addr;
			}
		} else if ((aw_rtp->len - aw_haptic->rtp_cnt) <
			   (aw_haptic->ram.base_addr >> 2)) {
			buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
		} else {
			buf_len = aw_haptic->ram.base_addr >> 2;
		}
#ifdef AW_ENABLE_RTP_PRINT_LOG
		aw_info("buf_len = %d", buf_len);
#endif
#ifdef AW_DOUBLE
	if (aw_haptic->rtp_cnt == 0 && aw_haptic->dual_flag == true) {
		aw_haptic->dual_flag = false;
		if (down_trylock(&left->sema) == 0) {
			down_interruptible(&left->sema);
		} else {
			up(&left->sema);
			up(&left->sema);
		}
		aw_info("dual rtp play start");
	}
#endif
		aw_haptic->func->set_rtp_data(aw_haptic,
					      &aw_rtp->data[aw_haptic->rtp_cnt],
					      buf_len);
		aw_haptic->rtp_cnt += buf_len;
		glb_state_val = aw_haptic->func->get_glb_state(aw_haptic);
		if ((aw_haptic->rtp_cnt == aw_rtp->len)
		    || ((glb_state_val & AW_GLBRD_STATE_MASK) ==
							AW_STATE_STANDBY)) {
			if (aw_haptic->rtp_cnt != aw_rtp->len)
				aw_err("rtp play suspend!");
			else
				aw_info("rtp update complete!");
			aw_haptic->rtp_cnt = 0;
			aw_haptic->rtp_init = false;
			break;
		}
	}
	return ret;
}

static int wait_enter_rtp_mode(struct aw_haptic *aw_haptic)
{
	bool rtp_work_flag = false;
	uint8_t ret = 0;
	int cnt = 200;

	while (cnt) {
		ret = aw_haptic->func->judge_rtp_going(aw_haptic);
		if (ret) {
			rtp_work_flag = true;
			aw_info("RTP_GO!");
			break;
		}
		cnt--;
		aw_info("wait for RTP_GO, glb_state=0x%02X", ret);
		usleep_range(2000, 2500);
	}
	if (!rtp_work_flag) {
		aw_haptic->func->play_stop(aw_haptic);
		aw_err("failed to enter RTP_GO status!");
		return -ERANGE;
	}
	return 0;
}

static void rtp_work_routine(struct work_struct *work)
{
	int ret = -1;
	const struct firmware *rtp_file;
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   rtp_work);

	if (!aw_haptic->ram.base_addr) {
		aw_err("base addr is 0, not allow rtp play");
		return;
	}
	mutex_lock(&aw_haptic->lock);
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_rtp_aei(aw_haptic, false);
	aw_haptic->func->irq_clear(aw_haptic);
	if (!aw_haptic->state) {
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	mutex_unlock(&aw_haptic->lock);
	mutex_lock(&aw_haptic->rtp_lock);
	/* fw loaded */
	ret = request_firmware(&rtp_file, aw_rtp_name[aw_haptic->rtp_file_num],
			       aw_haptic->dev);
	if (ret < 0) {
		aw_err("failed to read %s",
			aw_rtp_name[aw_haptic->rtp_file_num]);
		mutex_unlock(&aw_haptic->rtp_lock);
		return;
	}
	aw_haptic->rtp_init = false;
	vfree(aw_haptic->aw_rtp);
	aw_haptic->aw_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw_haptic->aw_rtp) {
		release_firmware(rtp_file);
		aw_err("error allocating memory");
		mutex_unlock(&aw_haptic->rtp_lock);
		return;
	}
	aw_haptic->aw_rtp->len = rtp_file->size;
	aw_info("rtp file:[%s] size = %dbytes",
		aw_rtp_name[aw_haptic->rtp_file_num], aw_haptic->aw_rtp->len);
	memcpy(aw_haptic->aw_rtp->data, rtp_file->data, rtp_file->size);
	mutex_unlock(&aw_haptic->rtp_lock);
	release_firmware(rtp_file);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->rtp_init = true;

	aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	/* gain */
	ram_vbat_comp(aw_haptic, false);
	/* rtp mode config */
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	/* haptic go */
	aw_haptic->func->play_go(aw_haptic, true);
	usleep_range(2000, 2500);
	ret = wait_enter_rtp_mode(aw_haptic);
	if (ret < 0) {
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	mutex_unlock(&aw_haptic->lock);

	mutex_lock(&aw_haptic->rtp_lock);
	pm_qos_enable(aw_haptic, true);
	aw_haptic->rtp_cnt = 0;
	rtp_play(aw_haptic);
	if (aw_haptic->play_mode == AW_RTP_MODE)
		aw_haptic->func->set_rtp_aei(aw_haptic, true);
	pm_qos_enable(aw_haptic, false);
	mutex_unlock(&aw_haptic->rtp_lock);
}

#ifdef AAC_RICHTAP_SUPPORT
static void richtap_clean_buf(struct aw_haptic *aw_haptic, int status)
{
	struct mmap_buf_format *opbuf = aw_haptic->start_buf;
	int i = 0;

	for (i = 0; i < RICHTAP_MMAP_BUF_SUM; i++) {
		memset(opbuf->data, 0, RICHTAP_MMAP_BUF_SIZE);
		opbuf->status = status;
		opbuf = opbuf->kernel_next;
	}
}

static void richtap_update_fifo_data(struct aw_haptic *aw_haptic, uint32_t fifo_len)
{
	int32_t samples_left = 0, pos = 0, retry = 30;
	
	pr_err("richtap_update_fifo_data start\n");
	do
	{
		if (aw_haptic->curr_buf->status == MMAP_BUF_DATA_VALID) {
			samples_left = aw_haptic->curr_buf->length - aw_haptic->pos;
			if (samples_left <= 0)
				samples_left = 0;
			if (samples_left < fifo_len) {
				memcpy(&aw_haptic->rtp_ptr[pos], &aw_haptic->curr_buf->data[aw_haptic->pos], samples_left);
				pos += samples_left;
				fifo_len -= samples_left;
				aw_haptic->curr_buf->status = MMAP_BUF_DATA_INVALID;
				aw_haptic->curr_buf->length = 0;
				aw_haptic->curr_buf = aw_haptic->curr_buf->kernel_next;
				aw_haptic->pos = 0;
			} else {
				memcpy(&aw_haptic->rtp_ptr[pos], &aw_haptic->curr_buf->data[aw_haptic->pos], fifo_len);
				aw_haptic->pos += fifo_len;
				pos += fifo_len;
				fifo_len = 0;
			}
		} else if (aw_haptic->curr_buf->status == MMAP_BUF_DATA_FINISHED) {
			break;
		} else {
			if((retry-- <= 0) || !atomic_read(&aw_haptic->richtap_rtp_mode)) {
				atomic_set(&aw_haptic->richtap_rtp_mode, false);
				pr_err("aac richtap invalid data or stop retry %d\n", retry);
				return;
			} else {
				usleep_range(1000,1000);
			}
		}
	} while(fifo_len > 0 && atomic_read(&aw_haptic->richtap_rtp_mode));
	if (pos <= 0)
		return;
	aw_haptic->func->set_rtp_data(aw_haptic, aw_haptic->rtp_ptr, pos);
	pr_err("richtap_update_fifo_data data %d,samples_left = %d\n", pos, samples_left);
}

static bool richtap_rtp_start(struct aw_haptic *aw_haptic)
{
	int cnt = 200;
	bool rtp_work_flag = false;
	uint8_t reg_val = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	aw_haptic->func->play_go(aw_haptic, true);
	//usleep_range(2000, 2500);

	while (cnt) {
		reg_val = aw_haptic->func->get_glb_state(aw_haptic);
		if ((reg_val & AW_GLBRD_STATE_MASK) == AW_STATE_RTP) {
			cnt = 0;
			rtp_work_flag = true;
			pr_info("richtap_rtp_start:RTP_GO! glb_state=0x08\n");
			break;
		} else if (atomic_read(&aw_haptic->richtap_rtp_mode)){
			cnt--;
			pr_info("richtap_rtp_start: wait for RTP_GO, glb_state=0x%02X\n", reg_val);
			usleep_range(2000, 2500);
		}
	}

	if(rtp_work_flag == false)
		aw_haptic->func->play_stop(aw_haptic);

	mutex_unlock(&aw_haptic->lock);

	return rtp_work_flag;
}

static void richtap_rtp_work(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, richtap_rtp_work);
	uint32_t retry = 0, tmp_len = 0;
	uint8_t glb_state_val = 0;

	atomic_set(&aw_haptic->richtap_rtp_mode, true);
	aw_haptic->curr_buf = aw_haptic->start_buf;

	do {
		if(aw_haptic->curr_buf->status == MMAP_BUF_DATA_VALID) {
			if ((tmp_len + aw_haptic->curr_buf->length) > aw_haptic->ram.base_addr) {
				memcpy(&aw_haptic->rtp_ptr[tmp_len], aw_haptic->curr_buf->data, (aw_haptic->ram.base_addr - tmp_len));
				aw_haptic->pos = aw_haptic->ram.base_addr - tmp_len;
				tmp_len = aw_haptic->ram.base_addr;
				continue;
			} else {
				memcpy(&aw_haptic->rtp_ptr[tmp_len], aw_haptic->curr_buf->data, aw_haptic->curr_buf->length);
				tmp_len += aw_haptic->curr_buf->length;
				aw_haptic->curr_buf->status = MMAP_BUF_DATA_INVALID;
				aw_haptic->curr_buf->length = 0;
				aw_haptic->pos = 0;
				aw_haptic->curr_buf = aw_haptic->curr_buf->kernel_next;
				continue;
			}
		} else if (aw_haptic->curr_buf->status == MMAP_BUF_DATA_FINISHED) {
			break;
		}
		if (!atomic_read(&aw_haptic->richtap_rtp_mode))
			return;
		msleep(1);
	} while(tmp_len < aw_haptic->ram.base_addr && retry++ < 30);

	pr_info("richtap_rtp_work rtp 1837 tm_len = %d\n", tmp_len);

	if (tmp_len <= 0)
		return;
	if (richtap_rtp_start(aw_haptic)) {
		pr_info("richtap_rtp_work start richtap play\n");
		pm_qos_enable(aw_haptic, true);
		aw_haptic->func->set_rtp_data(aw_haptic, aw_haptic->rtp_ptr, tmp_len);
		aw_haptic->func->set_rtp_aei(aw_haptic, false);
		while(aw_haptic->curr_buf->status != MMAP_BUF_DATA_FINISHED) {
			if (aw_haptic->func->rtp_get_fifo_afs(aw_haptic) != 1)
				richtap_update_fifo_data(aw_haptic,(aw_haptic->ram.base_addr >> 2));
			else
				usleep_range(1000, 1001);

			if (!atomic_read(&aw_haptic->richtap_rtp_mode))
				break;

			glb_state_val = aw_haptic->func->get_glb_state(aw_haptic);
			if((glb_state_val & AW_GLBRD_STATE_MASK) == AW_STATE_STANDBY){
				pr_err("debugrichtap work break\n");
				break;
			}
		}
		atomic_set(&aw_haptic->richtap_rtp_mode, false);
		pm_qos_enable(aw_haptic, false);
	} else {
		atomic_set(&aw_haptic->richtap_rtp_mode, false);
	}
}
#ifdef AW_DOUBLE
static int haptic_left_flag(int unuse)
{
	(void)unuse;
	return LEFT_FOPS;
}
static int haptic_right_flag(int unuse)
{
	(void)unuse;
	return RIGHT_FOPS;
}
#endif

static int richtap_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;
#ifdef AW_DOUBLE
	if(file->f_op->check_flags(1) == LEFT_FOPS){
		file->private_data = (void *)left;
	}else if(file->f_op->check_flags(1) == RIGHT_FOPS){
		file->private_data = (void *)right;
	}else{
		file->private_data = (void *)NULL;
		pr_err("richtap_file_open: file private_data err!");
	}
#else
	file->private_data = (void *)g_aw_haptic;
#endif
	return 0;
}

static int richtap_file_release(struct inode *inode, struct file *file)
{
	module_put(THIS_MODULE);

	return 0;
}

static long richtap_file_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct aw_haptic *aw_haptic = (struct aw_haptic *)filp->private_data;
	int ret = 0, tmp;

	pr_info("richtap_file_unlocked_ioctl: cmd=0x%x, arg=0x%lx\n", cmd, arg);

	switch (cmd) {
		case RICHTAP_GET_HWINFO:
			tmp = RICHTAP_AW_8697;
			if(copy_to_user((void __user *)arg, &tmp, sizeof(int)))
				ret = -EFAULT;
			break;
		case RICHTAP_RTP_MODE:
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			mutex_unlock(&aw_haptic->lock);
			if (copy_from_user(aw_haptic->rtp_ptr, (void __user *)arg, RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM)) {
				ret = -EFAULT;
				break;
			}
			tmp = *((int*)aw_haptic->rtp_ptr);
			if(tmp > (RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM - 4)) {
				dev_err(aw_haptic->dev, "rtp mode date len error %d\n", tmp);
				ret = -EINVAL;
				break;
			}
			aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
			aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
			if(richtap_rtp_start(aw_haptic))
				aw_haptic->func->set_rtp_data(aw_haptic, &aw_haptic->rtp_ptr[4], tmp);
			break;
		case RICHTAP_OFF_MODE:
			break;
		case RICHTAP_GET_F0:
			tmp = aw_haptic->f0;
			if(copy_to_user((void __user *)arg, &tmp, sizeof(uint32_t)))
				ret = -EFAULT;
			break;
		case RICHTAP_SETTING_GAIN:
			if(arg > 0x80)
				arg = 0x80;
			//aw_haptic->func->enable_gain(aw_haptic, 1);
			aw_haptic->func->set_gain(aw_haptic, (uint8_t)arg);
			break;
		case RICHTAP_STREAM_MODE:
			atomic_set(&aw_haptic->richtap_rtp_mode, false);
			richtap_clean_buf(aw_haptic, MMAP_BUF_DATA_INVALID);
			cancel_work_sync(&aw_haptic->richtap_rtp_work);
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			mutex_unlock(&aw_haptic->lock);
			aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
			aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
			queue_work(aw_haptic->richtap_workqueue, &aw_haptic->richtap_rtp_work);
			break;
		case RICHTAP_STOP_MODE:
			atomic_set(&aw_haptic->richtap_rtp_mode, false);
			richtap_clean_buf(aw_haptic, MMAP_BUF_DATA_FINISHED);
			cancel_work_sync(&aw_haptic->richtap_rtp_work);
			mutex_lock(&aw_haptic->lock);
			//aw_haptic->func->irq_clear(aw_haptic);
			aw_haptic->func->play_stop(aw_haptic);
			mutex_unlock(&aw_haptic->lock);
			//aw_haptic->func->enable_gain(aw_haptic, 0);
			aw_haptic->func->set_rtp_aei(aw_haptic, false);
			break;
		default:
			dev_err(aw_haptic->dev, "%s, unknown cmd\n", __func__);
			break;
	}

	return ret;
}

static int richtap_file_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long phys;
	struct aw_haptic *aw_haptic = (struct aw_haptic *)filp->private_data;
	int ret = 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,7,0)
	//only accept PROT_READ, PROT_WRITE and MAP_SHARED from the API of mmap
	vm_flags_t vm_flags = calc_vm_prot_bits(PROT_READ|PROT_WRITE, 0) | calc_vm_flag_bits(MAP_SHARED);
	vm_flags |= current->mm->def_flags | VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC| VM_SHARED | VM_MAYSHARE;
	if(vma && (pgprot_val(vma->vm_page_prot) != pgprot_val(vm_get_page_prot(vm_flags))))
		return -EPERM;

	if(vma && ((vma->vm_end - vma->vm_start) != (PAGE_SIZE << RICHTAP_MMAP_PAGE_ORDER)))
		return -ENOMEM;
#endif
	phys = virt_to_phys(aw_haptic->start_buf);

	ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT), (vma->vm_end - vma->vm_start), vma->vm_page_prot);
	if(ret)
	{
		dev_err(aw_haptic->dev, "Error mmap failed\n");
		return ret;
	}

	return ret;
}

static ssize_t richtap_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t richtap_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	return count;
}

static struct file_operations left_fops = {
	.owner = THIS_MODULE,
	.read = richtap_read,
	.write = richtap_write,
	.mmap = richtap_file_mmap,
	.unlocked_ioctl = richtap_file_unlocked_ioctl,
	.open = richtap_file_open,
	.release = richtap_file_release,
#ifdef AW_DOUBLE
	.check_flags = haptic_left_flag,
#endif
};

#ifdef AW_DOUBLE
static struct file_operations right_fops = {
	.owner = THIS_MODULE,
	.read = richtap_read,
	.write = richtap_write,
	.mmap = richtap_file_mmap,
	.unlocked_ioctl = richtap_file_unlocked_ioctl,
	.open = richtap_file_open,
	.release = richtap_file_release,
	.check_flags = haptic_right_flag,
};
#endif

static struct miscdevice richtap_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aw86927_haptic",
	.fops = &left_fops,
};

#ifdef AW_DOUBLE
static struct miscdevice richtap_misc_x = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aw86927x_haptic",
	.fops = &right_fops,
};
#endif
#endif
static irqreturn_t irq_handle(int irq, void *data)
{
	int irq_state = 0;
	struct aw_haptic *aw_haptic = data;

	do {
		irq_state = aw_haptic->func->get_irq_state(aw_haptic);
		if (irq_state == AW_IRQ_ALMOST_EMPTY) {
			if (aw_haptic->rtp_init) {
				mutex_lock(&aw_haptic->rtp_lock);
				rtp_play(aw_haptic);
				mutex_unlock(&aw_haptic->rtp_lock);
			} else {
				aw_info("rtp_init: %d", aw_haptic->rtp_init);
			}
		}
		if (aw_haptic->play_mode != AW_RTP_MODE)
			aw_haptic->func->set_rtp_aei(aw_haptic, false);
	} while (irq_state);
	return IRQ_HANDLED;
}

static int irq_config(struct device *dev, struct aw_haptic *aw_haptic)
{
	int ret = -1;
	int irq_flags = 0;

	if (gpio_is_valid(aw_haptic->irq_gpio) &&
	    !(aw_haptic->flags & AW_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw_haptic->func->interrupt_setup(aw_haptic);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(dev,
					       gpio_to_irq(aw_haptic->irq_gpio),
					       NULL, irq_handle, irq_flags,
					       "aw_haptic", aw_haptic);
		if (ret != 0) {
			aw_err("failed to request IRQ %d: %d",
			       gpio_to_irq(aw_haptic->irq_gpio), ret);
			return ret;
		}
	} else {
		dev_info(dev, "skipping IRQ registration");
		/* disable feature support if gpio was invalid */
		aw_haptic->flags |= AW_FLAG_SKIP_INTERRUPTS;
	}
	return 0;
}

static void ram_load(const struct firmware *cont, void *context)
{
	uint16_t check_sum = 0;
	int i = 0;
	int ret = 0;
	struct aw_haptic *aw_haptic = context;
	struct aw_haptic_container *aw_fw;

#ifdef AW_READ_BIN_FLEXBALLY
	static uint8_t load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif
	if (!cont) {
		aw_err("failed to read %s", aw_ram_name);
		release_firmware(cont);
#ifdef AW_READ_BIN_FLEXBALLY
		if (load_cont <= 20) {
			schedule_delayed_work(&aw_haptic->ram_work,
					      msecs_to_jiffies(ram_timer_val));
			aw_info("start hrtimer:load_cont%d", load_cont);
		}
#endif
		return;
	}
	aw_info("loaded %s - size: %zu", aw_ram_name, cont ? cont->size : 0);
	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum != (uint16_t)((cont->data[0] << 8) | (cont->data[1]))) {
		aw_err("check sum err: check_sum=0x%04x", check_sum);
		release_firmware(cont);
		return;
	}
	aw_info("check sum pass : 0x%04x", check_sum);
	aw_haptic->ram.check_sum = check_sum;

	/* aw ram update */
	aw_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw_fw) {
		release_firmware(cont);
		aw_err("Error allocating memory");
		return;
	}
	aw_fw->len = cont->size;
	memcpy(aw_fw->data, cont->data, cont->size);
	release_firmware(cont);
	ret = aw_haptic->func->container_update(aw_haptic, aw_fw);
	if (ret) {
		aw_err("ram firmware update failed!");
	} else {
		aw_haptic->ram_init = true;
		aw_haptic->ram.len = aw_fw->len - aw_haptic->ram.ram_shift;
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->trig_init(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
		aw_info("ram firmware update complete!");
		get_ram_num(aw_haptic);
	}
	kfree(aw_fw);

#ifdef AW_BOOT_OSC_CALI
	rtp_trim_lra_cali(aw_haptic);
#endif
}

static int ram_update(struct aw_haptic *aw_haptic)
{
	aw_haptic->ram_init = false;
	aw_haptic->rtp_init = false;
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw_ram_name, aw_haptic->dev, GFP_KERNEL,
				       aw_haptic, ram_load);
}

static void ram_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic,
						   ram_work.work);

	ram_update(aw_haptic);
}

static void ram_work_init(struct aw_haptic *aw_haptic)
{
	int ram_timer_val = AW_RAM_WORK_DELAY_INTERVAL;

	INIT_DELAYED_WORK(&aw_haptic->ram_work, ram_work_routine);
	schedule_delayed_work(&aw_haptic->ram_work,
			      msecs_to_jiffies(ram_timer_val));
}

/*****************************************************
 *
 * haptic audio
 *
 *****************************************************/
static int audio_ctrl_list_ins(struct aw_haptic *aw_haptic,
			       struct aw_haptic_ctr *haptic_ctr)
{
	struct aw_haptic_ctr *p_new = NULL;
	struct aw_haptic_audio *haptic_audio = &aw_haptic->haptic_audio;

	p_new = (struct aw_haptic_ctr *)kzalloc(
		sizeof(struct aw_haptic_ctr), GFP_KERNEL);
	if (p_new == NULL) {
		aw_err("kzalloc memory fail");
		return -ENOMEM;
	}
	/* update new list info */
	p_new->cnt = haptic_ctr->cnt;
	p_new->cmd = haptic_ctr->cmd;
	p_new->play = haptic_ctr->play;
	p_new->wavseq = haptic_ctr->wavseq;
	p_new->loop = haptic_ctr->loop;
	p_new->gain = haptic_ctr->gain;
	INIT_LIST_HEAD(&(p_new->list));
	list_add(&(p_new->list), &(haptic_audio->ctr_list));
	return 0;
}

static void audio_ctrl_list_clr(struct aw_haptic_audio *haptic_audio)
{
	struct aw_haptic_ctr *p_ctr = NULL;
	struct aw_haptic_ctr *p_ctr_bak = NULL;

	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		list_del(&p_ctr->list);
		kfree(p_ctr);
	}
}

static void audio_init(struct aw_haptic *aw_haptic)
{
	aw_dbg("enter!");
	aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
}

static void audio_off(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->set_gain(aw_haptic, 0x80);
	aw_haptic->func->play_stop(aw_haptic);
	audio_ctrl_list_clr(&aw_haptic->haptic_audio);
	mutex_unlock(&aw_haptic->lock);
}

static enum hrtimer_restart audio_timer_func(struct hrtimer *timer)
{
	struct aw_haptic *aw_haptic =
	    container_of(timer, struct aw_haptic, haptic_audio.timer);

	aw_dbg("enter");
	schedule_work(&aw_haptic->haptic_audio.work);

	hrtimer_start(&aw_haptic->haptic_audio.timer,
		      ktime_set(aw_haptic->haptic_audio.timer_val / 1000000,
				(aw_haptic->haptic_audio.timer_val % 1000000) *
				1000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void audio_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic =
	    container_of(work, struct aw_haptic, haptic_audio.work);
	struct aw_haptic_audio *haptic_audio = NULL;
	struct aw_haptic_ctr *ctr = &aw_haptic->haptic_audio.ctr;
	struct aw_haptic_ctr *p_ctr = NULL;
	struct aw_haptic_ctr *p_ctr_bak = NULL;
	uint32_t ctr_list_flag = 0;
	uint32_t ctr_list_input_cnt = 0;
	uint32_t ctr_list_output_cnt = 0;
	uint32_t ctr_list_diff_cnt = 0;
	uint32_t ctr_list_del_cnt = 0;
	int rtp_is_going_on = 0;

	aw_dbg("enter");
	haptic_audio = &(aw_haptic->haptic_audio);
	mutex_lock(&aw_haptic->haptic_audio.lock);
	memset(ctr, 0, sizeof(struct aw_haptic_ctr));
	ctr_list_flag = 0;
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		ctr_list_flag = 1;
		break;
	}
	if (ctr_list_flag == 0)
		aw_dbg("ctr list empty");
	if (ctr_list_flag == 1) {
		list_for_each_entry_safe(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
			ctr_list_input_cnt = p_ctr->cnt;
			break;
		}
		list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
						 &(haptic_audio->ctr_list),
						 list) {
			ctr_list_output_cnt = p_ctr->cnt;
			break;
		}
		if (ctr_list_input_cnt > ctr_list_output_cnt) {
			ctr_list_diff_cnt =
			    ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if (ctr_list_input_cnt < ctr_list_output_cnt) {
			ctr_list_diff_cnt =
			    32 + ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if (ctr_list_diff_cnt > 2) {
			list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
							 &(haptic_audio->
							   ctr_list), list) {
				if ((p_ctr->play == 0)
				    && (AW_CMD_ENABLE ==
					(AW_CMD_HAPTIC & p_ctr->
					 cmd))) {
					list_del(&p_ctr->list);
					kfree(p_ctr);
					ctr_list_del_cnt++;
				}
				if (ctr_list_del_cnt == ctr_list_diff_cnt)
					break;
			}
		}
	}
	/* get the last data from list */
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		ctr->cnt = p_ctr->cnt;
		ctr->cmd = p_ctr->cmd;
		ctr->play = p_ctr->play;
		ctr->wavseq = p_ctr->wavseq;
		ctr->loop = p_ctr->loop;
		ctr->gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		break;
	}
	if (ctr->play) {
		aw_info("cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d",
			ctr->cnt,  ctr->cmd, ctr->play,
			ctr->wavseq, ctr->loop, ctr->gain);
	}
	if (ctr->wavseq > aw_haptic->ram.ram_num) {
		aw_err("wavseq out of range!");
		mutex_unlock(&aw_haptic->haptic_audio.lock);
		return;
	}
	rtp_is_going_on = aw_haptic->func->judge_rtp_going(aw_haptic);
	if (rtp_is_going_on) {
		mutex_unlock(&aw_haptic->haptic_audio.lock);
		return;
	}
	mutex_unlock(&aw_haptic->haptic_audio.lock);

#ifdef AW_DOUBLE
	if (ctr->play == AW_PLAY_ENABLE) {
		mutex_lock(&aw_haptic->lock);
		/* haptic config */
		if (ctr->cmd & AW_CMD_L_EN) {
			left->func->play_stop(left);
			left->func->play_mode(left, AW_RAM_MODE);
			left->func->set_wav_seq(left, 0x00, ctr->wavseq);
			left->func->set_wav_seq(left, 0x01, 0x00);
			left->func->set_wav_loop(left, 0x00, ctr->loop);
			left->func->set_gain(left, ctr->gain);
		}

		if (ctr->cmd & AW_CMD_R_EN) {
			right->func->play_stop(right);
			right->func->play_mode(right, AW_RAM_MODE);
			right->func->set_wav_seq(right, 0x00, ctr->wavseq);
			right->func->set_wav_seq(right, 0x01, 0x00);
			right->func->set_wav_loop(right, 0x00, ctr->loop);
			right->func->set_gain(right, ctr->gain);
		}
		/* play go */
		if (ctr->cmd & AW_CMD_L_EN)
			left->func->play_go(left, true);
		if (ctr->cmd & AW_CMD_R_EN)
			right->func->play_go(right, true);
		mutex_unlock(&aw_haptic->lock);
	} else if (ctr->play == AW_PLAY_STOP) {
		mutex_lock(&aw_haptic->lock);
		left->func->play_stop(left);
		right->func->play_stop(right);
		mutex_unlock(&aw_haptic->lock);
	} else if (ctr->play == AW_PLAY_GAIN) {
		mutex_lock(&aw_haptic->lock);
		left->func->set_gain(left, ctr->gain);
		right->func->set_gain(right, ctr->gain);
		mutex_unlock(&aw_haptic->lock);
	}
#else
	if (ctr->cmd == AW_CMD_ENABLE) {
		if (ctr->play == AW_PLAY_ENABLE) {
			aw_info("haptic_audio_play_start");
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			aw_haptic->func->play_mode(aw_haptic, AW_RAM_MODE);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x00,
						     ctr->wavseq);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
			aw_haptic->func->set_wav_loop(aw_haptic, 0x00,
						      ctr->loop);
			aw_haptic->func->set_gain(aw_haptic, ctr->gain);
			aw_haptic->func->play_go(aw_haptic, true);
			mutex_unlock(&aw_haptic->lock);
		} else if (ctr->play == AW_PLAY_STOP) {
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			mutex_unlock(&aw_haptic->lock);
		} else if (ctr->play == AW_PLAY_GAIN) {
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->set_gain(aw_haptic, ctr->gain);
			mutex_unlock(&aw_haptic->lock);
		}
	}
#endif

}

/*****************************************************
 *
 * node
 *
 *****************************************************/
#ifdef TIMED_OUTPUT
static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct aw_haptic *aw_haptic = container_of(dev, struct aw_haptic,
						   vib_dev);

	if (hrtimer_active(&aw_haptic->timer)) {
		ktime_t r = hrtimer_get_remaining(&aw_haptic->timer);

		return ktime_to_ms(r);
	}
	return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct aw_haptic *aw_haptic = container_of(dev, struct aw_haptic,
						   vib_dev);

	aw_info("enter");
	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return;
	}
	if (value < 0) {
		aw_err("unsupported param");
		return;
	}
	mutex_lock(&aw_haptic->lock);
	aw_haptic->state = value;
	aw_haptic->activate_mode = AW_RAM_MODE;
	mutex_unlock(&aw_haptic->lock);
	schedule_work(&aw_haptic->vibrator_work);
}
#else
static enum led_brightness brightness_get(struct led_classdev *cdev)
{
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return aw_haptic->amplitude;
}

static void brightness_set(struct led_classdev *cdev, enum led_brightness level)
{
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	aw_info("enter");
	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return;
	}
	mutex_lock(&aw_haptic->lock);
	aw_haptic->amplitude = level;
	aw_haptic->state = level;
	aw_haptic->activate_mode = AW_RAM_MODE;
	mutex_unlock(&aw_haptic->lock);
	schedule_work(&aw_haptic->vibrator_work);
}
#endif

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "state = %d\n", aw_haptic->state);
}

static ssize_t state_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	return count;
}

static ssize_t duration_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw_haptic->timer)) {
		time_rem = hrtimer_get_remaining(&aw_haptic->timer);
		time_ms = ktime_to_ms(time_rem);
	}
	return snprintf(buf, PAGE_SIZE, "duration = %lldms\n", time_ms);
}

static ssize_t duration_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;
	aw_info("duration=%d", val);
	aw_haptic->duration = val;
	return count;
}

static ssize_t activate_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "activate = %d\n", aw_haptic->state);
}

static ssize_t activate_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%d", val);
	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	mutex_lock(&aw_haptic->lock);
	aw_haptic->state = val;
	aw_haptic->activate_mode = aw_haptic->info.mode;
	mutex_unlock(&aw_haptic->lock);
	schedule_work(&aw_haptic->vibrator_work);

	return count;
}

static ssize_t activate_mode_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "activate_mode = %d\n",
			aw_haptic->activate_mode);
}

static ssize_t activate_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	aw_haptic->activate_mode = val;
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t index_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_seq(aw_haptic, 1);
	aw_haptic->index = aw_haptic->seq[0];
	mutex_unlock(&aw_haptic->lock);
	return snprintf(buf, PAGE_SIZE, "index = %d\n", aw_haptic->index);
}

static ssize_t index_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val > aw_haptic->ram.ram_num) {
		aw_err("input value out of range!");
		return count;
	}
	aw_info("value=%d", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->index = val;
	aw_haptic->func->set_repeat_seq(aw_haptic, aw_haptic->index);
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t vmax_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "vmax = %dmV\n", aw_haptic->vmax);
}

static ssize_t vmax_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%d", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->vmax = val;
	aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t gain_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "gain = 0x%02X\n", aw_haptic->gain);
}

static ssize_t gain_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=0x%02x", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->gain = val;
	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t seq_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	uint8_t i = 0;
	size_t count = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_seq(aw_haptic, AW_SEQUENCER_SIZE);
	mutex_unlock(&aw_haptic->lock);
	for (i = 0; i < AW_SEQUENCER_SIZE; i++)
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d = %d\n", i + 1, aw_haptic->seq[i]);
	return count;
}

static ssize_t seq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		if (databuf[0] >= AW_SEQUENCER_SIZE ||
		    databuf[1] > aw_haptic->ram.ram_num) {
			aw_err("input value out of range!");
			return count;
		}
		aw_info("seq%d=0x%02X", databuf[0], databuf[1]);
		mutex_lock(&aw_haptic->lock);
		aw_haptic->seq[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_seq(aw_haptic, (uint8_t)databuf[0],
					     aw_haptic->seq[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t loop_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	size_t count = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	mutex_lock(&aw_haptic->lock);
	count = aw_haptic->func->get_wav_loop(aw_haptic, buf);
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t loop_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_info("seq%d loop=0x%02X", databuf[0], databuf[1]);
		mutex_lock(&aw_haptic->lock);
		aw_haptic->loop[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_loop(aw_haptic, (uint8_t)databuf[0],
					      aw_haptic->loop[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	len = aw_haptic->func->get_reg(aw_haptic, len, buf);
	mutex_unlock(&aw_haptic->lock);
	return len;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	uint8_t val = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		val = (uint8_t)databuf[1];
		mutex_lock(&aw_haptic->lock);
		haptic_hv_i2c_writes(aw_haptic, (uint8_t)databuf[0], &val,
				     AW_I2C_BYTE_ONE);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t rtp_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp_cnt = %d\n",
			aw_haptic->rtp_cnt);
	return len;
}

static ssize_t rtp_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	uint32_t rtp_num_max = sizeof(aw_rtp_name) / AW_RTP_NAME_MAX;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		aw_err("kstrtouint fail");
		return rc;
	}
	mutex_lock(&aw_haptic->lock);
	if ((val > 0) && (val < rtp_num_max)) {
		aw_haptic->state = 1;
		aw_haptic->rtp_file_num = val;
		aw_info("aw_rtp_name[%d]: %s", val, aw_rtp_name[val]);
	} else if (val == 0) {
		aw_haptic->state = 0;
	} else {
		aw_haptic->state = 0;
		aw_err("input number error:%d", val);
	}
	mutex_unlock(&aw_haptic->lock);
	schedule_work(&aw_haptic->rtp_work);
	return count;
}

static ssize_t ram_update_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;
	int i = 0;
	uint8_t *ram_buf = NULL;

	aw_info("ram len = %d", aw_haptic->ram.len);
	ram_buf = kzalloc(aw_haptic->ram.len, GFP_KERNEL);
	if (!ram_buf) {
		aw_err("Error allocating memory");
		return len;
	}
	mutex_lock(&aw_haptic->lock);
	/* RAMINIT Enable */
	aw_haptic->func->ram_init(aw_haptic, true);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_ram_addr(aw_haptic);
	aw_haptic->func->get_ram_data(aw_haptic, ram_buf);
	for (i = 1; i <= aw_haptic->ram.len; i++) {
		len += snprintf(buf + len, PAGE_SIZE, "0x%02x,",
				*(ram_buf + i - 1));
		if (i % 16 == 0 || i == aw_haptic->ram.len) {
			len = 0;
			aw_info("%s", buf);
		}
	}
	kfree(ram_buf);
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	len = snprintf(buf, PAGE_SIZE, "Please check log\n");
	mutex_unlock(&aw_haptic->lock);
	return len;
}

static ssize_t ram_update_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val)
		ram_update(aw_haptic);
	return count;
}

static ssize_t ram_num_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	get_ram_num(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"ram_num = %d\n", aw_haptic->ram.ram_num);
	return len;
}

static ssize_t f0_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	aw_haptic->func->get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "f0 = %d\n", aw_haptic->f0);
	return len;
}

static ssize_t cali_data_show(struct device *dev, struct device_attribute *attr,
                       char *buf)
{
	uint8_t f0_calidata_to_register;
	uint32_t f0_calidata;
	uint8_t reg_val_flase=AW8692X_BIT_TMCFG_TM_LOCK;
	uint8_t reg_val_true=AW8692X_BIT_TMCFG_TM_UNLOCK;

	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						  vib_dev);
	ssize_t len = 0;

	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TMCFG, &reg_val_true,
									AW_I2C_BYTE_ONE);
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_ANACFG20, &f0_calidata_to_register,
									AW_I2C_BYTE_ONE);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TMCFG, &reg_val_flase,
									 AW_I2C_BYTE_ONE);
	f0_calidata = (uint32_t)f0_calidata_to_register;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", f0_calidata);
	return len;
}

static ssize_t ram_f0_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	aw_haptic->func->ram_get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "f0 = %d\n", aw_haptic->f0);
	return len;
}

static ssize_t cali_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	aw_haptic->func->get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "f0 = %d\n", aw_haptic->f0);
	return len;
}

static ssize_t cali_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		mutex_lock(&aw_haptic->lock);
		f0_cali(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t ram_cali_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	aw_haptic->func->ram_get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "f0 = %d\n", aw_haptic->f0);
	return len;
}

static ssize_t ram_cali_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		mutex_lock(&aw_haptic->lock);
		ram_f0_cali(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}


static ssize_t cont_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	if (val) {
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
		aw_haptic->func->cont_config(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t vbat_monitor_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_vbat(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "vbat_monitor = %d\n",
			aw_haptic->vbat);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t lra_resistance_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_lra_resistance(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "lra_resistance = %d\n",
			aw_haptic->lra);
	return len;
}

static ssize_t auto_boost_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "auto_boost = %d\n",
			aw_haptic->auto_boost);

	return len;
}

static ssize_t auto_boost_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->auto_bst_enable(aw_haptic, val);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t prct_mode_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;
	uint8_t reg_val = 0;

	mutex_lock(&aw_haptic->lock);
	reg_val = aw_haptic->func->get_prctmode(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "prctmode = %d\n", reg_val);
	return len;
}

static ssize_t prct_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t databuf[2] = { 0, 0 };
	uint32_t prtime = 0;
	uint32_t prlvl = 0;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		prtime = databuf[0];
		prlvl = databuf[1];
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->protect_config(aw_haptic, prtime, prlvl);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t ram_vbat_comp_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"ram_vbat_comp = %d\n",
			aw_haptic->ram_vbat_comp);

	return len;
}

static ssize_t ram_vbat_comp_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	if (val)
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;
	else
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_DISABLE;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t osc_cali_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw_haptic->osc_cali_data);

	return len;
}

static ssize_t osc_cali_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	if (val == 1) {
		rtp_trim_lra_cali(aw_haptic);
	} else if (val == 2) {
		aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
		rtp_osc_cali(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t haptic_audio_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n",
			aw_haptic->haptic_audio.ctr.cnt);
	return len;
}

static ssize_t haptic_audio_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	uint32_t databuf[6] = { 0 };
	struct aw_haptic_ctr *hap_ctr = NULL;

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	if (sscanf(buf, "%d %d %d %d %d %d", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5]) == 6) {
		if (databuf[2]) {
			aw_info("cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d",
				databuf[0], databuf[1], databuf[2], databuf[3],
				databuf[4], databuf[5]);
		}

		hap_ctr = (struct aw_haptic_ctr *)kzalloc(
			sizeof(struct aw_haptic_ctr), GFP_KERNEL);
		if (hap_ctr == NULL) {
			aw_err("kzalloc memory fail");
			return count;
		}
		mutex_lock(&aw_haptic->haptic_audio.lock);
		hap_ctr->cnt = (uint8_t)databuf[0];
		hap_ctr->cmd = (uint8_t)databuf[1];
		hap_ctr->play = (uint8_t)databuf[2];
		hap_ctr->wavseq = (uint8_t)databuf[3];
		hap_ctr->loop = (uint8_t)databuf[4];
		hap_ctr->gain = (uint8_t)databuf[5];
		audio_ctrl_list_ins(aw_haptic, hap_ctr);
		if (hap_ctr->cmd == 0xff) {
			aw_info("haptic_audio stop");
			if (hrtimer_active(&aw_haptic->haptic_audio.timer)) {
				aw_info("cancel haptic_audio_timer");
				hrtimer_cancel(&aw_haptic->haptic_audio.timer);
				aw_haptic->haptic_audio.ctr.cnt = 0;
				audio_off(aw_haptic);
			}
		} else {
			if (hrtimer_active(&aw_haptic->haptic_audio.timer)) {
				/* */
			} else {
				aw_info("start haptic_audio_timer");
				audio_init(aw_haptic);
				hrtimer_start(&aw_haptic->haptic_audio.timer,
					      ktime_set(aw_haptic->haptic_audio.
							delay_val / 1000000,
							(aw_haptic->haptic_audio.
							 delay_val % 1000000) *
							1000),
					      HRTIMER_MODE_REL);
			}
		}
		mutex_unlock(&aw_haptic->haptic_audio.lock);
		kfree(hap_ctr);
	}
	return count;
}

static ssize_t haptic_audio_time_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.delay_val=%dus\n",
			aw_haptic->haptic_audio.delay_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.timer_val=%dus\n",
			aw_haptic->haptic_audio.timer_val);
	return len;
}

static ssize_t haptic_audio_time_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	uint32_t databuf[2] = { 0 };

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->haptic_audio.delay_val = databuf[0];
		aw_haptic->haptic_audio.timer_val = databuf[1];
	}
	return count;
}

static ssize_t gun_type_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw_haptic->gun_type);
}

static ssize_t gun_type_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_dbg("value=%d", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->gun_type = val;
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t bullet_nr_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw_haptic->bullet_nr);
}

static ssize_t bullet_nr_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_dbg("value=%d", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->bullet_nr = val;
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t awrw_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	int i = 0;
	ssize_t len = 0;

	if (aw_haptic->i2c_info.flag != AW_SEQ_READ) {
		aw_err("no read mode");
		return -ERANGE;
	}
	if (aw_haptic->i2c_info.reg_data == NULL) {
		aw_err("awrw lack param");
		return -ERANGE;
	}
	for (i = 0; i < aw_haptic->i2c_info.reg_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"0x%02x,", aw_haptic->i2c_info.reg_data[i]);
	}
	len += snprintf(buf + len - 1, PAGE_SIZE - len, "\n");
	return len;
}

static ssize_t awrw_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	uint8_t value = 0;
	char data_buf[5] = { 0 };
	uint32_t flag = 0;
	uint32_t reg_num = 0;
	uint32_t reg_addr = 0;
	int i = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x %x %x", &flag, &reg_num, &reg_addr) == 3) {
		if (!reg_num) {
			aw_err("param error");
			return -ERANGE;
		}
		aw_haptic->i2c_info.flag = flag;
		aw_haptic->i2c_info.reg_num = reg_num;
		if (aw_haptic->i2c_info.reg_data != NULL)
			kfree(aw_haptic->i2c_info.reg_data);
		aw_haptic->i2c_info.reg_data = kmalloc(reg_num, GFP_KERNEL);
		if (flag == AW_SEQ_WRITE) {
			if ((reg_num * 5) != (strlen(buf) - 3 * 5)) {
				aw_err("param error");
				return -ERANGE;
			}
			for (i = 0; i < reg_num; i++) {
				memcpy(data_buf, &buf[15 + i * 5], 4);
				data_buf[4] = '\0';
				rc = kstrtou8(data_buf, 0, &value);
				if (rc < 0) {
					aw_err("param error");
					return -ERANGE;
				}
				aw_haptic->i2c_info.reg_data[i] = value;
			}
			mutex_lock(&aw_haptic->lock);
			haptic_hv_i2c_writes(aw_haptic, (uint8_t)reg_addr,
					 aw_haptic->i2c_info.reg_data, reg_num);
			mutex_unlock(&aw_haptic->lock);
		} else if (flag == AW_SEQ_READ) {
			mutex_lock(&aw_haptic->lock);
			haptic_hv_i2c_reads(aw_haptic, reg_addr,
					 aw_haptic->i2c_info.reg_data, reg_num);
			mutex_unlock(&aw_haptic->lock);
		}
	} else {
		aw_err("param error");
	}
	return count;
}

static ssize_t f0_save_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "f0_cali_data = 0x%02X\n",
			aw_haptic->f0_cali_data);

	return len;
}

static ssize_t f0_save_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_haptic->f0_cali_data = val;
	return count;
}

static ssize_t osc_save_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw_haptic->osc_cali_data);

	return len;
}

static ssize_t osc_save_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_haptic->osc_cali_data = val;
	return count;
}

#ifdef AW_DOUBLE
static ssize_t dual_cont_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	right->func->play_stop(right);
	left->func->play_stop(left);
	if (val) {
		right->func->upload_lra(right, AW_F0_CALI_LRA);
		left->func->upload_lra(left, AW_F0_CALI_LRA);
		right->func->cont_config(right);
		left->func->cont_config(left);
	}
	return count;
}

static ssize_t dual_index_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d %d\n",
			left->index, right->index);
	return len;
}

static ssize_t dual_index_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int index_l = 0;
	int index_r = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%d %d", &index_l, &index_r) == 2) {
		aw_info("index_l=%d index_r=%d", index_l, index_r);
		if (index_l > aw_haptic->ram.ram_num ||
		    index_r > aw_haptic->ram.ram_num) {
			aw_err("input value out of range!");
			return count;
		}
		mutex_lock(&aw_haptic->lock);
		left->index = index_l;
		left->func->set_repeat_seq(left, left->index);
		right->index = index_r;
		right->func->set_repeat_seq(right, right->index);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t dual_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "dual_mode = %d\n",
			aw_haptic->activate_mode);
}

static ssize_t dual_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	left->activate_mode = val;
	right->activate_mode = val;
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t dual_duration_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw_haptic->timer)) {
		time_rem = hrtimer_get_remaining(&aw_haptic->timer);
		time_ms = ktime_to_ms(time_rem);
	}
	return snprintf(buf, PAGE_SIZE, "duration = %lldms\n", time_ms);
}

static ssize_t dual_duration_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int duration_l = 0;
	int duration_r = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%d %d", &duration_l, &duration_r) == 2) {
		mutex_lock(&aw_haptic->lock);
		left->duration = duration_l;
		right->duration = duration_r;
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t dual_activate_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	return snprintf(buf, PAGE_SIZE, "activate = %d\n", aw_haptic->state);
}

static ssize_t dual_activate_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%d", val);
	if (!left->ram_init || !right->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	mutex_lock(&aw_haptic->lock);
	if (down_trylock(&left->sema) == 0)
		up(&left->sema);
	else
		up(&left->sema);
	left->state = val;
	right->state = val;
	left->dual_flag = true;
	right->dual_flag = true;
	mutex_unlock(&aw_haptic->lock);
	schedule_work(&left->vibrator_work);
	schedule_work(&right->vibrator_work);
	return count;
}


static ssize_t dual_rtp_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "left_rtp_cnt = %d\n",
			left->rtp_cnt);
	len += snprintf(buf + len, PAGE_SIZE - len, "right_rtp_cnt = %d\n",
			right->rtp_cnt);
	return len;
}

static ssize_t dual_rtp_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);
	int rtp_l = 0;
	int rtp_r = 0;
	uint32_t rtp_num_max = sizeof(aw_rtp_name) / AW_RTP_NAME_MAX;

	mutex_lock(&aw_haptic->lock);
	if (down_trylock(&left->sema) == 0)
		up(&left->sema);
	else
		up(&left->sema);
	if (sscanf(buf, "%d %d", &rtp_l, &rtp_r) == 2) {
		if (rtp_l > 0 && rtp_l < rtp_num_max) {
			left->state = 1;
			left->rtp_file_num = rtp_l;
		} else if (rtp_l == 0) {
			left->state = 0;
		} else {
			left->state = 0;
			aw_err("input number error:%d", rtp_l);
		}
		if (rtp_r > 0 && rtp_r < rtp_num_max) {
			right->state = 1;
			right->rtp_file_num = rtp_r;
		} else if (rtp_r == 0) {
			right->state = 0;
		} else {
			right->state = 0;
			aw_err("input number error:%d", rtp_r);
		}
	}
	left->dual_flag = true;
	right->dual_flag = true;
	mutex_unlock(&aw_haptic->lock);
	schedule_work(&left->rtp_work);
	schedule_work(&right->rtp_work);
	return count;
}
#endif

static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO, f0_show, NULL);
static DEVICE_ATTR(ram_f0, S_IWUSR | S_IRUGO, ram_f0_show, NULL);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, seq_show, seq_store);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, reg_show, reg_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, vmax_show, vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, gain_show, gain_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, loop_show, loop_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, rtp_show, rtp_store);
static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO, cali_show, cali_store);
static DEVICE_ATTR(ram_cali, S_IWUSR | S_IRUGO, ram_cali_show, ram_cali_store);
static DEVICE_ATTR(cont, S_IWUSR | S_IRUGO, NULL, cont_store);
static DEVICE_ATTR(awrw, S_IWUSR | S_IRUGO, awrw_show, awrw_store);
static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, state_show, state_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, index_show, index_store);
static DEVICE_ATTR(ram_num, S_IWUSR | S_IRUGO, ram_num_show, NULL);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, duration_show, duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, activate_show, activate_store);
static DEVICE_ATTR(osc_cali, S_IWUSR | S_IRUGO, osc_cali_show, osc_cali_store);
static DEVICE_ATTR(gun_type, S_IWUSR | S_IRUGO, gun_type_show, gun_type_store);
static DEVICE_ATTR(prctmode, S_IWUSR | S_IRUGO, prct_mode_show,
		   prct_mode_store);
static DEVICE_ATTR(bullet_nr, S_IWUSR | S_IRUGO, bullet_nr_show,
		   bullet_nr_store);
static DEVICE_ATTR(auto_boost, S_IWUSR | S_IRUGO, auto_boost_show,
		   auto_boost_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, ram_update_show,
		   ram_update_store);
static DEVICE_ATTR(haptic_audio, S_IWUSR | S_IRUGO, haptic_audio_show,
		   haptic_audio_store);
static DEVICE_ATTR(vbat_monitor, S_IWUSR | S_IRUGO, vbat_monitor_show, NULL);
static DEVICE_ATTR(activate_mode, S_IWUSR | S_IRUGO, activate_mode_show,
		   activate_mode_store);
static DEVICE_ATTR(ram_vbat_comp, S_IWUSR | S_IRUGO, ram_vbat_comp_show,
		   ram_vbat_comp_store);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO, lra_resistance_show,
		   NULL);
static DEVICE_ATTR(haptic_audio_time, S_IWUSR | S_IRUGO, haptic_audio_time_show,
		   haptic_audio_time_store);
static DEVICE_ATTR(osc_save, S_IWUSR | S_IRUGO, osc_save_show, osc_save_store);
static DEVICE_ATTR(f0_save, S_IWUSR | S_IRUGO, f0_save_show, f0_save_store);
static DEVICE_ATTR(cali_data, S_IWUSR | S_IRUGO, cali_data_show, NULL);

#ifdef AW_DOUBLE
static DEVICE_ATTR(dual_cont, S_IWUSR | S_IRUGO, NULL, dual_cont_store);
static DEVICE_ATTR(dual_index, S_IWUSR | S_IRUGO, dual_index_show, dual_index_store);
static DEVICE_ATTR(dual_mode, S_IWUSR | S_IRUGO, dual_mode_show, dual_mode_store);
static DEVICE_ATTR(dual_duration, S_IWUSR | S_IRUGO, dual_duration_show, dual_duration_store);
static DEVICE_ATTR(dual_activate, S_IWUSR | S_IRUGO, dual_activate_show, dual_activate_store);
static DEVICE_ATTR(dual_rtp, S_IWUSR | S_IRUGO, dual_rtp_show, dual_rtp_store);
#endif

static struct attribute *vibrator_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_vmax.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_reg.attr,
	&dev_attr_rtp.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_ram_num.attr,
	&dev_attr_f0.attr,
	&dev_attr_ram_f0.attr,
	&dev_attr_f0_save.attr,
	&dev_attr_cali.attr,
	&dev_attr_ram_cali.attr,
	&dev_attr_cont.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_auto_boost.attr,
	&dev_attr_prctmode.attr,
	&dev_attr_ram_vbat_comp.attr,
	&dev_attr_osc_cali.attr,
	&dev_attr_osc_save.attr,
	&dev_attr_haptic_audio.attr,
	&dev_attr_haptic_audio_time.attr,
	&dev_attr_gun_type.attr,
	&dev_attr_bullet_nr.attr,
	&dev_attr_awrw.attr,
	&dev_attr_cali_data.attr,
#ifdef AW_DOUBLE
	&dev_attr_dual_cont.attr,
	&dev_attr_dual_index.attr,
	&dev_attr_dual_mode.attr,
	&dev_attr_dual_duration.attr,
	&dev_attr_dual_activate.attr,
	&dev_attr_dual_rtp.attr,
#endif

	NULL
};

static struct attribute_group vibrator_attribute_group = {
	.attrs = vibrator_attributes
};

#ifdef AW_TIKTAP
static inline unsigned int tiktap_get_sys_msecs(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	struct timespec64 ts64;

	ktime_get_coarse_real_ts64(&ts64);
#else
	struct timespec64 ts64 = current_kernel_time64();
#endif

	return jiffies_to_msecs(timespec64_to_jiffies(&ts64));
}

static void tiktap_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic =
			  container_of(work, struct aw_haptic, tiktap_work);
	struct mmap_buf_format *tiktap_buf = aw_haptic->start_buf;
	int count = 100;
#ifdef AW_DOUBLE
	int sync_cnt = 40;
#endif
	unsigned char reg_val = 0x10;
	unsigned char glb_state_val = 0;
	unsigned int write_start;
	unsigned int buf_cnt = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->tiktap_stop_flag = false;
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	while (true && count--) {
		if (tiktap_buf->status == MMAP_BUF_DATA_VALID) {
			aw_haptic->func->play_go(aw_haptic, true);
			mdelay(1);
			break;
		} else if (aw_haptic->tiktap_stop_flag == true) {
			mutex_unlock(&aw_haptic->lock);
			return;
		}
		mdelay(1);
	}
	if (count <= 0) {
		aw_err("wait 100 ms but start_buf->status != VALID! status = 0x%02x",
		       tiktap_buf->status);
		aw_haptic->tiktap_stop_flag = true;
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	aw_haptic->tiktap_ready = true;
	mutex_unlock(&aw_haptic->lock);

#ifdef AW_DOUBLE /* double tiktap work sync */
	while (sync_cnt--) {
		if ((left->tiktap_ready == true) &&
		    (right->tiktap_ready == true)) {
			/* aw_info("double vib is ready! start write tiktap data."); */
			break;
		} else if (aw_haptic->tiktap_stop_flag == true) {
			aw_haptic->tiktap_ready = false;
			return;
		}
		udelay(500);
	}
	if (sync_cnt <= 0) {
		aw_err("wait 20ms but double vib not ready!");
		aw_haptic->tiktap_stop_flag = true;
		aw_haptic->tiktap_ready = false;
		return;
	}
#endif

	mutex_lock(&aw_haptic->rtp_lock);
	pm_qos_enable(aw_haptic, true);
	write_start = tiktap_get_sys_msecs();
	while (true) {
		if (tiktap_get_sys_msecs() > (write_start + 800)) {
			aw_err("Failed! tiktap endless loop");
			break;
		}
		reg_val = aw_haptic->func->rtp_get_fifo_aes(aw_haptic);
		glb_state_val = aw_haptic->func->get_glb_state(aw_haptic);
		if ((glb_state_val & AW_GLBRD_STATE_MASK) != AW_STATE_RTP) {
			aw_err("tiktap glb_state != RTP_GO!, glb_state = 0x%02x",
			       glb_state_val);
			break;
		}
		if ((aw_haptic->tiktap_stop_flag == true) ||
		    (tiktap_buf->status == MMAP_BUF_DATA_FINISHED) ||
		    (tiktap_buf->status == MMAP_BUF_DATA_INVALID)) {
			aw_err("tiktap exit! tiktap_buf->status = 0x%02x",
			       tiktap_buf->status);
			break;
		} else if ((
#ifdef AW_DOUBLE
			   (aw_haptic == left && (tiktap_buf->status == TIKTAP_L_VALID_R_FINISHED)) ||
			   (aw_haptic == right && (tiktap_buf->status == TIKTAP_R_VALID_L_FINISHED)) ||
#endif
			   tiktap_buf->status == MMAP_BUF_DATA_VALID) && (reg_val & 0x01)) {
			aw_info("buf_cnt = %d, bit = %d, length = %d!",
				buf_cnt, tiktap_buf->bit, tiktap_buf->length);

			aw_haptic->func->set_rtp_data(aw_haptic, tiktap_buf->data, tiktap_buf->length);
#ifdef AW_DOUBLE
			if (aw_haptic == left) {
				tiktap_buf->status &= TIkTAP_LEFT_STATUS_MASK;
				tiktap_buf->status |= TIkTAP_LEFT_FINISHED_DONE; /* left done */
			} else if (aw_haptic == right) {
				tiktap_buf->status &= TIKTAP_RIGHT_STATUS_MASK;
				tiktap_buf->status |= TIKTAP_RIGHT_FINISHED_DONE; /* right done */
			}
#else
			tiktap_buf->status = MMAP_BUF_DATA_FINISHED;
#endif
			tiktap_buf = tiktap_buf->kernel_next;
			write_start = tiktap_get_sys_msecs();
			buf_cnt++;
		} else {
			mdelay(1);
		}
	}
	pm_qos_enable(aw_haptic, false);
	aw_haptic->tiktap_stop_flag = true;
	mutex_unlock(&aw_haptic->rtp_lock);
}

static void tiktap_clean_buf(struct aw_haptic *aw_haptic, int status)
{
	struct mmap_buf_format *tiktap_buf = aw_haptic->start_buf;
	int i = 0;

	for (i = 0; i < AW_TIKTAP_MMAP_BUF_SUM; i++) {
		tiktap_buf->status = status;
		tiktap_buf = tiktap_buf->kernel_next;
	}
}

static long tiktap_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct aw_haptic *aw_haptic = g_aw_haptic;
	unsigned int tmp = 0;

	int ret = 0;

	switch (cmd) {
	case TIKTAP_GET_HWINFO:
		aw_info("cmd = TIKTAP_GET_HWINFO!");
		tmp = aw_haptic->chipid;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(unsigned int)))
			ret = -EFAULT;
		break;
	case TIKTAP_GET_F0:
		aw_info("cmd = TIKTAP_GET_F0!");
		tmp = aw_haptic->f0;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(unsigned int)))
			ret = -EFAULT;
		break;
	case TIKTAP_STOP_MODE:
		aw_info("cmd = TIKTAP_STOP_MODE!");
		tiktap_clean_buf(aw_haptic, MMAP_BUF_DATA_INVALID);
#ifdef AW_DOUBLE
		left->tiktap_stop_flag = true;
		right->tiktap_stop_flag = true;
		left->tiktap_ready = false;
		right->tiktap_ready = false;
		mutex_lock(&left->lock);
		left->func->play_stop(left);
		mutex_unlock(&left->lock);
		mutex_lock(&right->lock);
		right->func->play_stop(right);
		mutex_unlock(&right->lock);
#else
		aw_haptic->tiktap_stop_flag = true;
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->play_stop(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
#endif
		break;
	case TIKTAP_RTP_MODE:
		/* aw_info("cmd = TIKTAP_RTP_MODE!"); */
		tiktap_clean_buf(aw_haptic, MMAP_BUF_DATA_INVALID);
#ifdef AW_DOUBLE
		left->tiktap_ready = false;
		right->tiktap_ready = false;
		left->tiktap_stop_flag = true;
		right->tiktap_stop_flag = true;

		mutex_lock(&left->lock);
		left->func->play_stop(left);
		mutex_unlock(&left->lock);
		mutex_lock(&right->lock);
		right->func->play_stop(right);
		mutex_unlock(&right->lock);

		schedule_work(&right->tiktap_work);
		schedule_work(&left->tiktap_work);
#else
		aw_haptic->tiktap_stop_flag = true;
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->play_stop(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
		schedule_work(&aw_haptic->tiktap_work);
#endif
		break;
	default:
		aw_info("unknown cmd = %d", cmd);
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long tiktap_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	ret = tiktap_unlocked_ioctl(file, cmd, arg);
	return ret;
}
#endif

static int tiktap_file_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long phys;
	struct aw_haptic *aw_haptic = g_aw_haptic;
	int ret = 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 7, 0)
	vm_flags_t vm_flags = calc_vm_prot_bits(PROT_READ|PROT_WRITE, 0) |
			      calc_vm_flag_bits(MAP_SHARED);

	vm_flags |= current->mm->def_flags | VM_MAYREAD | VM_MAYWRITE |
		    VM_MAYEXEC | VM_SHARED | VM_MAYSHARE;

	if (vma && (pgprot_val(vma->vm_page_prot) !=
		   pgprot_val(vm_get_page_prot(vm_flags)))) {
		aw_err("vm_page_prot error!");
		return -EPERM;
	}

	if (vma && ((vma->vm_end - vma->vm_start) !=
		   (PAGE_SIZE << AW_TIKTAP_MMAP_PAGE_ORDER))) {
		aw_err("mmap size check err!");
		return -EPERM;
	}
#endif
	phys = virt_to_phys(aw_haptic->start_buf);

	ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT),
			      (vma->vm_end - vma->vm_start), vma->vm_page_prot);
	if (ret) {
		aw_err("mmap failed!");
		return ret;
	}

	aw_info("success!");

	return ret;
}

#ifdef KERNEL_OVER_5_10
static const struct proc_ops tiktap_proc_ops = {
	.proc_mmap = tiktap_file_mmap,
	.proc_ioctl = tiktap_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.proc_compat_ioctl = tiktap_compat_ioctl,
#endif
};
#else
static const struct file_operations tiktap_proc_ops = {
	.owner = THIS_MODULE,
	.mmap = tiktap_file_mmap,
	.unlocked_ioctl = tiktap_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tiktap_compat_ioctl,
#endif
};
#endif

static int tiktap_init(struct aw_haptic *aw_haptic)
{
	/* contrl double vib init once */
	static int tiktap_init;
	static struct mmap_buf_format *tiktap_start_buf;
	static struct proc_dir_entry *tiktap_config_proc;

	if (tiktap_init == 0) {
		/* Create proc file node */
		tiktap_config_proc = NULL;
		tiktap_config_proc = proc_create(AW_TIKTAP_PROCNAME, 0666,
							NULL, &tiktap_proc_ops);
		if (tiktap_config_proc == NULL) {
			aw_err("create proc entry %s failed",
			       AW_TIKTAP_PROCNAME);
			return -EPERM;
		}
		aw_info("create proc entry %s success", AW_TIKTAP_PROCNAME);

		/* Construct shared memory */
		tiktap_start_buf = (struct mmap_buf_format *)__get_free_pages(GFP_KERNEL, AW_TIKTAP_MMAP_PAGE_ORDER);
		if (tiktap_start_buf == NULL) {
			aw_err("Error __get_free_pages failed");
			return -ENOMEM;
		}
		SetPageReserved(virt_to_page(tiktap_start_buf));
		{
			struct mmap_buf_format *temp;
			unsigned int i = 0;

			temp = tiktap_start_buf;
			for (i = 1; i < AW_TIKTAP_MMAP_BUF_SUM; i++) {
				temp->kernel_next = (tiktap_start_buf + i);
				temp = temp->kernel_next;
			}
			temp->kernel_next = tiktap_start_buf;

			temp = tiktap_start_buf;
			for (i = 0; i < AW_TIKTAP_MMAP_BUF_SUM; i++) {
				temp->bit = i;
				temp = temp->kernel_next;
			}
		}

	}
	tiktap_init = 1;

	aw_haptic->aw_config_proc = tiktap_config_proc;
	aw_haptic->start_buf = tiktap_start_buf;
	/* init flag and work */
	aw_haptic->tiktap_stop_flag = true;
	aw_haptic->tiktap_ready = false;
	INIT_WORK(&aw_haptic->tiktap_work, tiktap_work_routine);
	return 0;
}
#endif

static int vibrator_init(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_info("enter");

#ifdef TIMED_OUTPUT
	aw_info("TIMED_OUT FRAMEWORK!");
#ifdef AW_DOUBLE
	ret = memcmp(aw_haptic->name, "left", sizeof("left"));
	if (!ret)
		aw_haptic->vib_dev.name = "vibrator_l";
	ret = memcmp(aw_haptic->name, "right", sizeof("right"));
	if (!ret)
		aw_haptic->vib_dev.name = "vibrator_r";
#else
	aw_haptic->vib_dev.name = "vibrator";
#endif
	aw_haptic->vib_dev.get_time = vibrator_get_time;
	aw_haptic->vib_dev.enable = vibrator_enable;

	ret = timed_output_dev_register(&(aw_haptic->vib_dev));
	if (ret < 0) {
		aw_err("fail to create timed output dev");
		return ret;
	}
	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &vibrator_attribute_group);
	if (ret < 0) {
		aw_err("error creating sysfs attr files");
		return ret;
	}
#else
	aw_info("loaded in leds_cdev framework!");
#ifdef AW_DOUBLE
	ret = memcmp(aw_haptic->name, "left", sizeof("left"));
	if (!ret)
		aw_haptic->vib_dev.name = "vibrator_l";
	ret = memcmp(aw_haptic->name, "right", sizeof("right"));
	if (!ret)
		aw_haptic->vib_dev.name = "vibrator_r";
#else
#ifdef KERNEL_OVER_5_10
	aw_haptic->vib_dev.name = "aw_vibrator";
#else
	aw_haptic->vib_dev.name = "vibrator";
#endif
#endif
	aw_haptic->vib_dev.brightness_get = brightness_get;
	aw_haptic->vib_dev.brightness_set = brightness_set;
	ret = devm_led_classdev_register(&aw_haptic->i2c->dev,
					 &aw_haptic->vib_dev);
	if (ret < 0) {
		aw_err("fail to create led dev");
		return ret;
	}
	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &vibrator_attribute_group);
	if (ret < 0) {
		aw_err("error creating sysfs attr files");
		return ret;
	}
#endif
	hrtimer_init(&aw_haptic->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw_haptic->timer.function = vibrator_timer_func;
	INIT_WORK(&aw_haptic->vibrator_work, vibrator_work_routine);
	INIT_WORK(&aw_haptic->rtp_work, rtp_work_routine);
	mutex_init(&aw_haptic->lock);
	mutex_init(&aw_haptic->rtp_lock);
	sema_init(&aw_haptic->sema, 1);

	return 0;
}

static void haptic_init(struct aw_haptic *aw_haptic)
{
	uint8_t f0_calidata_to_register;
	uint32_t f0_calidata;
	uint32_t uefi_f0_tmp;
	uint8_t reg_val_flase=AW8692X_BIT_TMCFG_TM_LOCK;
	uint8_t reg_val_true=AW8692X_BIT_TMCFG_TM_UNLOCK;

	if (strcmp(uefi_f0_calidata, "NA") == 0) {
		f0_calidata = 0;
	} else {
		kstrtoint(uefi_f0_calidata,10, &f0_calidata);
	}

	f0_calidata_to_register=(uint8_t)f0_calidata;

	if (0 == strcmp(uefi_f0, "NA")) {
		uefi_f0_tmp = 1700;
	} else {
		kstrtoint(uefi_f0,10, &uefi_f0_tmp);
	}

	aw_info("enter");
	/* haptic audio */
	aw_haptic->haptic_audio.delay_val = 1;
	aw_haptic->haptic_audio.timer_val = 21318;
	INIT_LIST_HEAD(&(aw_haptic->haptic_audio.ctr_list));
	hrtimer_init(&aw_haptic->haptic_audio.timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	aw_haptic->haptic_audio.timer.function = audio_timer_func;
	INIT_WORK(&aw_haptic->haptic_audio.work, audio_work_routine);
	mutex_init(&aw_haptic->haptic_audio.lock);
	INIT_LIST_HEAD(&(aw_haptic->haptic_audio.list));

	/* haptic init */
	mutex_lock(&aw_haptic->lock);
	aw_haptic->bullet_nr = 0;
	aw_haptic->gun_type = 0xff;
	aw_haptic->activate_mode = aw_haptic->info.mode;
	aw_haptic->func->play_mode(aw_haptic, AW_STANDBY_MODE);
	aw_haptic->func->set_pwm(aw_haptic, AW_PWM_24K);
	/* misc value init */
	aw_haptic->func->misc_para_init(aw_haptic);
	aw_haptic->func->set_bst_peak_cur(aw_haptic);
	aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
	aw_haptic->func->auto_bst_enable(aw_haptic,
					 aw_haptic->info.is_enabled_auto_bst);
	aw_haptic->func->offset_cali(aw_haptic);
	aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;
	/* f0 calibration */
	/* f0_cali(aw_haptic);*/
	aw_haptic->f0_cali_data = f0_calidata;
	aw_haptic->f0 = uefi_f0_tmp;

	mutex_unlock(&aw_haptic->lock);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TMCFG, &reg_val_true,
							 AW_I2C_BYTE_ONE);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_ANACFG20, &f0_calidata_to_register,
							AW_I2C_BYTE_ONE);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TMCFG, &reg_val_flase,
							AW_I2C_BYTE_ONE);
}

static int aw_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret = 0;
	struct aw_haptic *aw_haptic;
	struct device_node *np = i2c->dev.of_node;

	pr_info("<%s>%s: enter\n", AW_I2C_NAME, __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		pr_err("<%s>%s: check_functionality failed\n", AW_I2C_NAME, __func__);
		return -EIO;
	}

	aw_haptic = devm_kzalloc(&i2c->dev, sizeof(struct aw_haptic),
				 GFP_KERNEL);
	if (aw_haptic == NULL)
		return -ENOMEM;

	aw_haptic->dev = &i2c->dev;
	aw_haptic->i2c = i2c;

	i2c_set_clientdata(i2c, aw_haptic);
	dev_set_drvdata(&i2c->dev, aw_haptic);
#ifdef AW_INPUT_FRAMEWORK
	ret = input_framework_init(aw_haptic);
	if (ret < 0)
		goto err_parse_dt;
#endif
	/* aw_haptic rst & int */
	if (np) {
		ret = parse_dt_gpio(&i2c->dev, aw_haptic, np);
		if (ret) {
			aw_err("failed to parse gpio");
			goto err_parse_dt;
		}
	} else {
		aw_haptic->reset_gpio = -1;
		aw_haptic->irq_gpio = -1;
	}

	if (gpio_is_valid(aw_haptic->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw_haptic->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw_rst");
		if (ret) {
			aw_err("rst request failed");
			goto err_reset_gpio_request;
		}
	}

#ifdef AW_ENABLE_PIN_CONTROL
	aw_haptic->pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(aw_haptic->pinctrl)) {
		if (PTR_ERR(aw_haptic->pinctrl) == -EPROBE_DEFER) {
			aw_err("pinctrl not ready");
			ret = -EPROBE_DEFER;
			goto err_reset_gpio_request;
		}
		aw_err("Target does not use pinctrl");
		aw_haptic->pinctrl = NULL;
		ret = -EINVAL;
		goto err_reset_gpio_request;
	}
	aw_haptic->pinctrl_state = pinctrl_lookup_state(aw_haptic->pinctrl,
							"irq_active");
	if (IS_ERR(aw_haptic->pinctrl_state)) {
		aw_err("cannot find pinctrl state");
		ret = -EINVAL;
		goto err_reset_gpio_request;
	} else {
		pinctrl_select_state(aw_haptic->pinctrl,
				     aw_haptic->pinctrl_state);
	}
#endif
	if (gpio_is_valid(aw_haptic->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw_haptic->irq_gpio,
					    GPIOF_DIR_IN, "aw_int");
		if (ret) {
			aw_err("int request failed");
			goto err_irq_gpio_request;
		}
	}

	/* aw func ptr init */
	ret = ctrl_init(aw_haptic);
	if (ret < 0) {
		aw_err("ctrl_init failed ret=%d", ret);
		goto err_ctrl_init;
	}

	ret = aw_haptic->func->check_qualify(aw_haptic);
	if (ret < 0) {
		aw_err("qualify check failed ret=%d", ret);
		goto err_ctrl_init;
	}

	/* aw_haptic chip id */
	ret = parse_chipid(aw_haptic);
	if (ret < 0) {
		aw_err("parse chipid failed ret=%d", ret);
		goto err_id;
	}

	sw_reset(aw_haptic);
	aw_haptic->func->parse_dt(&i2c->dev, aw_haptic, np);

#ifdef AW_SND_SOC_CODEC
	aw_haptic->func->snd_soc_init(&i2c->dev);
#endif

	/* aw_haptic irq */
	ret = irq_config(&i2c->dev, aw_haptic);
	if (ret != 0) {
		aw_err("irq_config failed ret=%d", ret);
		goto err_irq_config;
	}

#ifdef AW_TIKTAP
	g_aw_haptic = aw_haptic;
	ret = tiktap_init(aw_haptic);
	if (ret) {
		aw_err("tiktap_init failed ret = %d", ret);
		goto err_irq_config;
	}
#endif

	vibrator_init(aw_haptic);
	haptic_init(aw_haptic);
	aw_haptic->func->creat_node(aw_haptic);
	ram_work_init(aw_haptic);
	aw_info("probe completed successfully!");

#ifdef AAC_RICHTAP_SUPPORT
	aw_haptic->rtp_ptr = kmalloc(RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM, GFP_KERNEL);
	if(aw_haptic->rtp_ptr == NULL) {
		dev_err(&i2c->dev, "malloc rtp memory failed\n");
		goto richtap_err1;
	}

	aw_haptic->start_buf = (struct mmap_buf_format *)__get_free_pages(GFP_KERNEL, RICHTAP_MMAP_PAGE_ORDER);
	if(aw_haptic->start_buf == NULL) {
		dev_err(&i2c->dev, "Error __get_free_pages failed\n");
		goto richtap_err2;
	}
	SetPageReserved(virt_to_page(aw_haptic->start_buf));
	{
		struct mmap_buf_format *temp;
		uint32_t i = 0;
		temp = aw_haptic->start_buf;
		for ( i = 1; i < RICHTAP_MMAP_BUF_SUM; i++)
		{
			temp->kernel_next = (aw_haptic->start_buf + i);
			temp = temp->kernel_next;
		}
		temp->kernel_next = aw_haptic->start_buf;
	}
	aw_haptic->richtap_workqueue = create_singlethread_workqueue("richtap");
	if (!aw_haptic->richtap_workqueue)
		aw_err("create richtap workqueue fail!\n");
	else
		INIT_WORK(&aw_haptic->richtap_rtp_work, richtap_rtp_work);
	atomic_set(&aw_haptic->richtap_rtp_mode, false);
#ifdef AW_DOUBLE
	if(of_device_is_compatible(np, "awinic,haptic_hv_l")) {
		misc_register(&richtap_misc);
	}

	if(of_device_is_compatible(np, "awinic,haptic_hv_r")) {
		misc_register(&richtap_misc_x);
	}
#else
	misc_register(&richtap_misc);
	//dev_set_drvdata(richtap_misc.this_device, aw_haptic);
	g_aw_haptic = aw_haptic;
#endif
#endif

	return 0;

#ifdef AAC_RICHTAP_SUPPORT
richtap_err2:
	kfree(aw_haptic->rtp_ptr);
richtap_err1:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw_haptic->irq_gpio), aw_haptic);
#endif
err_id:
err_ctrl_init:
err_irq_config:
	if (gpio_is_valid(aw_haptic->irq_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->irq_gpio);

err_irq_gpio_request:
	if (gpio_is_valid(aw_haptic->reset_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->reset_gpio);

err_parse_dt:
err_reset_gpio_request:
	devm_kfree(&i2c->dev, aw_haptic);
	aw_haptic = NULL;
	return ret;
}

static int aw_remove(struct i2c_client *i2c)
{
	struct aw_haptic *aw_haptic = i2c_get_clientdata(i2c);

	aw_info("enter");
#ifdef TIMED_OUTPUT
	timed_output_dev_unregister(&aw_haptic->vib_dev);
#else
	devm_led_classdev_unregister(&aw_haptic->i2c->dev, &aw_haptic->vib_dev);
#endif
#ifdef AW_INPUT_FRAMEWORK
	cancel_work_sync(&aw_haptic->gain_work);
	cancel_work_sync(&aw_haptic->input_vib_work);
	input_ff_destroy(aw_haptic->input_dev);
	input_unregister_device(aw_haptic->input_dev);
#endif
	cancel_delayed_work_sync(&aw_haptic->ram_work);
	cancel_work_sync(&aw_haptic->haptic_audio.work);
	hrtimer_cancel(&aw_haptic->haptic_audio.timer);
	cancel_work_sync(&aw_haptic->rtp_work);
	cancel_work_sync(&aw_haptic->vibrator_work);
	hrtimer_cancel(&aw_haptic->timer);
	mutex_destroy(&aw_haptic->lock);
	mutex_destroy(&aw_haptic->rtp_lock);
	mutex_destroy(&aw_haptic->haptic_audio.lock);
	devm_free_irq(&i2c->dev, gpio_to_irq(aw_haptic->irq_gpio), aw_haptic);
	if (gpio_is_valid(aw_haptic->irq_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->irq_gpio);
#ifdef AW_SND_SOC_CODEC
#ifdef KERNEL_OVER_4_19
	snd_soc_unregister_component(&i2c->dev);
#else
	snd_soc_unregister_codec(&i2c->dev);
#endif
#endif
#ifdef AW_TIKTAP
	proc_remove(aw_haptic->aw_config_proc);
	aw_haptic->aw_config_proc = NULL;
	free_pages((unsigned long)aw_haptic->start_buf, AW_TIKTAP_MMAP_PAGE_ORDER);
	aw_haptic->start_buf = NULL;
#endif
#ifdef AAC_RICHTAP_SUPPORT
	cancel_work_sync(&aw_haptic->richtap_rtp_work);
	kfree(aw_haptic->rtp_ptr);
	ClearPageReserved(virt_to_page(aw_haptic->start_buf));
	free_pages((unsigned long)aw_haptic->start_buf, RICHTAP_MMAP_PAGE_ORDER);
#ifdef AW_DOUBLE
	if(of_device_is_compatible(i2c->dev.of_node, "awinic,haptic_hv_l")) {
		misc_deregister(&richtap_misc);
	}
	if(of_device_is_compatible(i2c->dev.of_node, "awinic,haptic_hv_r")) {
		misc_deregister(&richtap_misc_x);
	}
#else
	misc_deregister(&richtap_misc);
#endif
#endif

	if (gpio_is_valid(aw_haptic->reset_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->reset_gpio);

	return 0;
}

static int aw_i2c_suspend(struct device *dev)
{
	int ret = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	mutex_unlock(&aw_haptic->lock);

	return ret;
}

static int aw_i2c_resume(struct device *dev)
{
	int ret = 0;
	return ret;
}

static SIMPLE_DEV_PM_OPS(aw_pm_ops, aw_i2c_suspend, aw_i2c_resume);

static const struct i2c_device_id aw_i2c_id[] = {
	{AW_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw_i2c_id);

static const struct of_device_id aw_dt_match[] = {
#ifdef AW_DOUBLE
	{.compatible = "awinic,haptic_hv_r"},
	{.compatible = "awinic,haptic_hv_l"},
#else
	{.compatible = "awinic,haptic_hv"},
#endif
	{},
};

static struct i2c_driver aw_i2c_driver = {
	.driver = {
		   .name = AW_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw_pm_ops,
#endif
		   },
	.probe = aw_i2c_probe,
	.remove = aw_remove,
	.id_table = aw_i2c_id,
};

static int __init aw_i2c_init(void)
{
	int ret = 0;

	pr_info("<%s>%s: aw_haptic driver version %s\n", AW_I2C_NAME, __func__,
		HAPTIC_HV_DRIVER_VERSION);
	ret = i2c_add_driver(&aw_i2c_driver);
	if (ret) {
		pr_err("<%s>%s: fail to add aw_haptic device into i2c\n",
		       AW_I2C_NAME, __func__);
		return ret;
	}
	return 0;
}
module_init(aw_i2c_init);

static void __exit aw_i2c_exit(void)
{
	i2c_del_driver(&aw_i2c_driver);
}
module_exit(aw_i2c_exit);

MODULE_DESCRIPTION("AWINIC Haptic Driver");
MODULE_LICENSE("GPL v2");

module_param_string(uefi_f0_calidata,uefi_f0_calidata,MAX_CMDLINE_PARAM_LEN,0600);
MODULE_PARM_DESC(uefi_f0_calidata,
		"haptic.uefi_f0_calidata");
module_param_string(uefi_f0,uefi_f0,MAX_CMDLINE_PARAM_LEN,0600);
MODULE_PARM_DESC(uefi_f0,
		"haptic.uefi_f0");