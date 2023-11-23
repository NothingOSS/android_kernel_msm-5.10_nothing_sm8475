/*
 * File: aw8692x.c
 *
 * Author: Ethan <chelvming@awinic.com>
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include "haptic_hv.h"
#include "haptic_hv_reg.h"

static void aw8692x_ram_init(struct aw_haptic *, bool);
static void aw8692x_vbat_mode_config(struct aw_haptic *, uint8_t);

static void aw8692x_reg_unlock(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t reg_val = 0;

	if (flag) {
		/* Unlock register */
		reg_val = AW8692X_BIT_TMCFG_TM_UNLOCK;
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TMCFG, &reg_val,
				     AW_I2C_BYTE_ONE);
	} else {
		/* Lock register */
		reg_val = AW8692X_BIT_TMCFG_TM_LOCK;
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TMCFG, &reg_val,
				     AW_I2C_BYTE_ONE);
	}
}

static int aw8692x_check_qualify(struct aw_haptic *aw_haptic)
{
	int ret = -1;
	uint8_t reg_val[3] = {0};

	aw_info("enter");
	ret = haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_EFCFG6, reg_val,
				  AW_I2C_BYTE_THREE);
	if (ret < 0)
		return ret;
	if (!(reg_val[0] & (~AW8692X_BIT_EFCFG6_LOCK_MASK))) {
		aw_err("unqualified chip!");
		return -ERANGE;
	}
	aw8692x_reg_unlock(aw_haptic, true);
	if ((reg_val[1] & (~AW8692X_BIT_EFCFG7_RESERVED_MASK)) ||
	    (reg_val[2] & (~AW8692X_BIT_EFCFG8_EF_TRM_BST_IPEAK_MASK))) {
		aw8692x_ram_init(aw_haptic, true);
		reg_val[0] = AW8692X_BIT_EFCFG1_INIT_VAL;
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_EFCFG1, reg_val,
				     AW_I2C_BYTE_ONE);
		usleep_range(2500, 3000);
		aw8692x_ram_init(aw_haptic, false);
	}
	ret = haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_EFCFG7, &reg_val[1],
				  AW_I2C_BYTE_TWO);
	if ((reg_val[1] & (~AW8692X_BIT_EFCFG7_RESERVED_MASK)) ||
	    (reg_val[2] & (~AW8692X_BIT_EFCFG8_EF_TRM_BST_IPEAK_MASK))) {
		return -ERANGE;
	}
	reg_val[0] = AW8692X_BIT_ANACFG2_INIT_VAL;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_ANACFG2, reg_val,
			     AW_I2C_BYTE_ONE);
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_EFCFG9, &reg_val[0],
			    AW_I2C_BYTE_ONE);
	if ((reg_val[0] & (~AW8692X_BIT_EFCFG9_EF_TRIM_OSC_MASK)) <
	    AW8692X_BOOT_DETECTION)
		return -ERANGE;
	aw8692x_reg_unlock(aw_haptic, false);
	return 0;
}

static void aw8692x_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSCTRL4,
					 AW8692X_BIT_SYSCTRL4_WAVDAT_MODE_MASK,
					 AW8692X_BIT_SYSCTRL4_WAVDAT_48K);
		break;
	case AW_PWM_24K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSCTRL4,
					 AW8692X_BIT_SYSCTRL4_WAVDAT_MODE_MASK,
					 AW8692X_BIT_SYSCTRL4_WAVDAT_24K);
		break;
	case AW_PWM_12K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSCTRL4,
					 AW8692X_BIT_SYSCTRL4_WAVDAT_MODE_MASK,
					 AW8692X_BIT_SYSCTRL4_WAVDAT_12K);
		break;
	default:
		aw_err("error param!");
		break;
	}
}

static void aw8692x_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_PLAYCFG2, &gain,
			     AW_I2C_BYTE_ONE);
}

static void aw8692x_set_bst_peak_cur(struct aw_haptic *aw_haptic)
{
	aw8692x_reg_unlock(aw_haptic, true);
	switch (aw_haptic->bst_pc) {
	case AW_BST_PC_L1:
		aw_info("bst pc = L1");
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_ANACFG13,
					AW8692X_BIT_ANACFG13_BST_PC_MASK,
					AW8692X_BIT_ANACFG13_BST_PEAKCUR_3P45A);
		break;
	case AW_BST_PC_L2:
		aw_info("bst pc = L2");
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_ANACFG13,
					 AW8692X_BIT_ANACFG13_BST_PC_MASK,
					 AW8692X_BIT_ANACFG13_BST_PEAKCUR_4A);
		break;
	default:
		aw_info("bst pc = L1");
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_ANACFG13,
					AW8692X_BIT_ANACFG13_BST_PC_MASK,
					AW8692X_BIT_ANACFG13_BST_PEAKCUR_3P45A);
		break;
	}
	aw8692x_reg_unlock(aw_haptic, false);
}

static void aw8692x_set_bst_vol(struct aw_haptic *aw_haptic, uint32_t bst_vol)
{
	uint8_t reg_val = 0;

	if (bst_vol < AW8692X_BST_VOL_MIN)
		bst_vol = AW8692X_BST_VOL_MIN;
	else if (bst_vol > AW8692X_BST_VOL_MAX)
		bst_vol = AW8692X_BST_VOL_MAX;
	reg_val = AW8692X_BST_VOL_FARMULA(bst_vol);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG1,
				 AW8692X_BIT_PLAYCFG1_BST_VOUT_VREFSET_MASK,
				 reg_val);
}

static void aw8692x_set_wav_seq(struct aw_haptic *aw_haptic, uint8_t wav,
				uint8_t seq)
{
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_WAVCFG1 + wav, &seq,
			     AW_I2C_BYTE_ONE);
}

static void aw8692x_set_wav_loop(struct aw_haptic *aw_haptic, uint8_t wav,
				 uint8_t loop)
{
	uint8_t tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_WAVCFG9 + (wav / 2),
					AW8692X_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_WAVCFG9 + (wav / 2),
					 AW8692X_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
}

static void aw8692x_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data,
				 uint32_t len)
{
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_RTPDATA, data, len);
}

static void aw8692x_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSINTM,
					 AW8692X_BIT_SYSINTM_FF_AEM_MASK,
					 AW8692X_BIT_SYSINTM_FF_AEM_ON);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSINTM,
					 AW8692X_BIT_SYSINTM_FF_AEM_MASK,
					 AW8692X_BIT_SYSINTM_FF_AEM_OFF);
	}
}

static void aw8692x_set_ram_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ram_addr[2] = {0};
	uint32_t base_addr = aw_haptic->ram.base_addr;

	ram_addr[0] = (uint8_t)AW8692X_RAMADDR_H(base_addr);
	ram_addr[1] = (uint8_t)AW8692X_RAMADDR_L(base_addr);

	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_RAMADDRH, ram_addr,
			     AW_I2C_BYTE_TWO);
}

static void aw8692x_set_base_addr(struct aw_haptic *aw_haptic)
{
	uint8_t rtp_addr[2] = {0};
	uint32_t base_addr = aw_haptic->ram.base_addr;

	rtp_addr[0] = (uint8_t)AW8692X_BASEADDR_H(base_addr);
	rtp_addr[1] = (uint8_t)AW8692X_BASEADDR_L(base_addr);

	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_RTPCFG1,
				 AW8692X_BIT_RTPCFG1_BASE_ADDR_H_MASK,
				 rtp_addr[0]);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_RTPCFG2, &rtp_addr[1],
			     AW_I2C_BYTE_ONE);
}

static void aw8692x_auto_break_mode(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
					 AW8692X_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8692X_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
					 AW8692X_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8692X_BIT_PLAYCFG3_BRK_DISABLE);
	}
}

static void aw8692x_f0_detect(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_CONTCFG1,
					 AW8692X_BIT_CONTCFG1_EN_F0_DET_MASK,
					 AW8692X_BIT_CONTCFG1_F0_DET_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_CONTCFG1,
					 AW8692X_BIT_CONTCFG1_EN_F0_DET_MASK,
					 AW8692X_BIT_CONTCFG1_F0_DET_DISABLE);
	}
}

static uint8_t aw8692x_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_GLBRD5, &state,
			    AW_I2C_BYTE_ONE);
	aw_dbg("glb state value is 0x%02X", state);
	return state;
}

static void aw8692x_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t go_on = AW8692X_BIT_PLAYCFG4_GO_ON;
	uint8_t stop_on = AW8692X_BIT_PLAYCFG4_STOP_ON;
	uint8_t reg_val = 0;

	aw_info("enter, flag = %d", flag);
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_ANACFG20, &reg_val,
			    AW_I2C_BYTE_ONE);
	if (flag && (reg_val <= AW8692X_BIT_ANACFG20_TRIM_LRA))
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_PLAYCFG4, &go_on,
				     AW_I2C_BYTE_ONE);
	else
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_PLAYCFG4, &stop_on,
				     AW_I2C_BYTE_ONE);
}

static int aw8692x_wait_enter_standby(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int count = 100;

	while (count--) {
		reg_val = aw8692x_get_glb_state(aw_haptic);
		if (reg_val == AW8692X_BIT_GLBRD5_STATE_STANDBY) {
			aw_info("entered standby!");
			return 0;
		}
		aw_dbg("wait for standby");
		usleep_range(2000, 2500);
	}
	aw_err("do not enter standby automatically");
	return -ERANGE;
}

static void aw8692x_bst_mode_config(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_BST_BOOST_MODE:
		aw_info("haptic bst mode = bst");
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG1,
					 AW8692X_BIT_PLAYCFG1_BST_MODE_MASK,
					 AW8692X_BIT_PLAYCFG1_BST_MODE);
		break;
	case AW_BST_BYPASS_MODE:
		aw_info("haptic bst mode = bypass");
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG1,
					 AW8692X_BIT_PLAYCFG1_BST_MODE_MASK,
					 AW8692X_BIT_PLAYCFG1_BST_MODE_BYPASS);
		break;
	default:
		aw_err("bst = %d error", mode);
		break;
	}
}

static void aw8692x_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_info("enter standby mode");
		aw_haptic->play_mode = AW_STANDBY_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSCTRL3,
					 AW8692X_BIT_SYSCTRL3_STANDBY_MASK,
					 AW8692X_BIT_SYSCTRL3_STANDBY_ON);
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSCTRL3,
					 AW8692X_BIT_SYSCTRL3_STANDBY_MASK,
					 AW8692X_BIT_SYSCTRL3_STANDBY_OFF);
		break;
	case AW_RAM_MODE:
		aw_info("enter ram mode");
		aw_haptic->play_mode = AW_RAM_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw8692x_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw8692x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RAM_LOOP_MODE:
		aw_info("enter ram loop mode");
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw8692x_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		aw8692x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RTP_MODE:
		aw_info("enter rtp mode");
		aw_haptic->play_mode = AW_RTP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_RTP);
		/* bst mode */
		aw8692x_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw8692x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_TRIG_MODE:
		aw_info("enter trig mode");
		aw_haptic->play_mode = AW_TRIG_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		aw8692x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_CONT_MODE:
		aw_info("enter cont mode");
		aw_haptic->play_mode = AW_CONT_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8692X_BIT_PLAYCFG3_PLAY_MODE_CONT);
		/* bst mode */
		aw8692x_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		aw8692x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
		break;
	default:
		aw_err("play mode %d error", play_mode);
		break;
	}
}

static void aw8692x_stop(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	aw_haptic->play_mode = AW_STANDBY_MODE;
	reg_val = AW8692X_BIT_PLAYCFG4_STOP_ON;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_PLAYCFG4, &reg_val,
			     AW_I2C_BYTE_ONE);
	ret = aw8692x_wait_enter_standby(aw_haptic);
	if (ret < 0) {
		aw_err("force to enter standby mode!");
		aw8692x_play_mode(aw_haptic, AW_STANDBY_MODE);
	}
}

static void aw8692x_ram_init(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSCTRL3,
					 AW8692X_BIT_SYSCTRL3_EN_RAMINIT_MASK,
					 AW8692X_BIT_SYSCTRL3_EN_RAMINIT_ON);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSCTRL3,
					 AW8692X_BIT_SYSCTRL3_EN_RAMINIT_MASK,
					 AW8692X_BIT_SYSCTRL3_EN_RAMINIT_OFF);
	}
}

static void aw8692x_upload_lra(struct aw_haptic *aw_haptic, uint32_t flag)
{
	uint8_t cali_data = 0;

	aw8692x_reg_unlock(aw_haptic, true);
	switch (flag) {
	case AW_WRITE_ZERO:
		aw_info("write zero to trim_lra!");
		break;
	case AW_F0_CALI_LRA:
		aw_info("write f0_cali_data to trim_lra = 0x%02X",
			aw_haptic->f0_cali_data);
		cali_data = (char)aw_haptic->f0_cali_data &
			    AW8692X_BIT_ANACFG20_TRIM_LRA;
		break;
	case AW_OSC_CALI_LRA:
		aw_info("write osc_cali_data to trim_lra = 0x%02X",
			aw_haptic->osc_cali_data);
		cali_data = (char)aw_haptic->osc_cali_data &
			    AW8692X_BIT_ANACFG20_TRIM_LRA;
		break;
	default:
		aw_err("error param, write f0_cali_data to trim_lra!");
		cali_data = (char)aw_haptic->f0_cali_data &
			    AW8692X_BIT_ANACFG20_TRIM_LRA;
		break;
	}
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_ANACFG20, &cali_data,
			     AW_I2C_BYTE_ONE);
	aw8692x_reg_unlock(aw_haptic, false);
}

static uint8_t aw8692x_get_trim_lra(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_ANACFG20, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW8692X_BIT_ANACFG20_TRIM_LRA;
	return reg_val;
}

static void aw8692x_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_VBATCTRL,
					 AW8692X_BIT_VBATCTRL_VBAT_MODE_MASK,
					 AW8692X_BIT_VBATCTRL_VBAT_MODE_HW);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_VBATCTRL,
					 AW8692X_BIT_VBATCTRL_VBAT_MODE_MASK,
					 AW8692X_BIT_VBATCTRL_VBAT_MODE_SW);
	}
}

static void aw8692x_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime,
				   uint8_t prlvl)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PWMCFG1,
				 AW8692X_BIT_PWMCFG1_PRC_EN_MASK,
				 AW8692X_BIT_PWMCFG1_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_info("enable protection mode");
		reg_val = AW8692X_BIT_PWMCFG3_PR_ENABLE |
			  (prlvl & (~AW8692X_BIT_PWMCFG3_PRLVL_MASK));
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_PWMCFG3, &reg_val,
				     AW_I2C_BYTE_ONE);
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_PWMCFG4, &prtime,
				     AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_info("disable protection mode");
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PWMCFG3,
					 AW8692X_BIT_PWMCFG3_PR_EN_MASK,
					 AW8692X_BIT_PWMCFG3_PR_DISABLE);
	}
}

static void aw8692x_cont_config(struct aw_haptic *aw_haptic)
{
	/* uint8_t drv1_time = 0xFF; */
	uint8_t drv2_time = 0xFF;

	/* work mode */
	aw8692x_play_mode(aw_haptic, AW_CONT_MODE);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_CONTCFG6,
				 AW8692X_BIT_CONTCFG6_TRACK_EN_MASK,
				 (aw_haptic->info.is_enabled_track_en << 7));
	/* f0 driver level */
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_CONTCFG6,
				 AW8692X_BIT_CONTCFG6_DRV1_LVL_MASK,
				 aw_haptic->info.cont_drv1_lvl);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG7,
			     &aw_haptic->info.cont_drv2_lvl, AW_I2C_BYTE_ONE);
	/* DRV1_TIME */
	/* haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG8, &drv1_time,
	 *			AW_I2C_BYTE_ONE);
	 */
	/* DRV2_TIME */
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG9, &drv2_time,
			     AW_I2C_BYTE_ONE);
	/* cont play go */
	aw8692x_play_go(aw_haptic, true);
}

static void aw8692x_one_wire_init(struct aw_haptic *aw_haptic)
{
	uint8_t trig_prio = 0x6c;
	uint8_t delay_2p5ms = AW8692X_BIT_START_DLY_2P5MS;

	aw_info("enter");
	/*if enable one-wire, trig1 priority must be less than trig2 and trig3*/
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_GLBCFG4, &trig_prio,
			     AW_I2C_BYTE_ONE);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_GLBCFG2, &delay_2p5ms,
			     AW_I2C_BYTE_ONE);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_TRGCFG8,
				 AW8692X_BIT_TRGCFG8_TRG_ONEWIRE_MASK,
				 AW8692X_BIT_TRGCFG8_TRG_ONEWIRE_ENABLE);
}

static void aw8692x_trig1_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[0].trig_level   = aw_haptic->info.trig_cfg[0];
	aw_haptic->trig[0].trig_polar   = aw_haptic->info.trig_cfg[1];
	aw_haptic->trig[0].pos_enable   = aw_haptic->info.trig_cfg[2];
	aw_haptic->trig[0].pos_sequence = aw_haptic->info.trig_cfg[3];
	aw_haptic->trig[0].neg_enable   = aw_haptic->info.trig_cfg[4];
	aw_haptic->trig[0].neg_sequence = aw_haptic->info.trig_cfg[5];
	aw_haptic->trig[0].trig_brk     = aw_haptic->info.trig_cfg[6];
	aw_haptic->trig[0].trig_bst     = aw_haptic->info.trig_cfg[7];
}

static void aw8692x_trig2_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[1].trig_level   = aw_haptic->info.trig_cfg[8];
	aw_haptic->trig[1].trig_polar   = aw_haptic->info.trig_cfg[9];
	aw_haptic->trig[1].pos_enable   = aw_haptic->info.trig_cfg[10];
	aw_haptic->trig[1].pos_sequence = aw_haptic->info.trig_cfg[11];
	aw_haptic->trig[1].neg_enable   = aw_haptic->info.trig_cfg[12];
	aw_haptic->trig[1].neg_sequence = aw_haptic->info.trig_cfg[13];
	aw_haptic->trig[1].trig_brk     = aw_haptic->info.trig_cfg[14];
	aw_haptic->trig[1].trig_bst     = aw_haptic->info.trig_cfg[15];
}

static void aw8692x_trig3_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[2].trig_level   = aw_haptic->info.trig_cfg[16];
	aw_haptic->trig[2].trig_polar   = aw_haptic->info.trig_cfg[17];
	aw_haptic->trig[2].pos_enable   = aw_haptic->info.trig_cfg[18];
	aw_haptic->trig[2].pos_sequence = aw_haptic->info.trig_cfg[19];
	aw_haptic->trig[2].neg_enable   = aw_haptic->info.trig_cfg[20];
	aw_haptic->trig[2].neg_sequence = aw_haptic->info.trig_cfg[21];
	aw_haptic->trig[2].trig_brk     = aw_haptic->info.trig_cfg[22];
	aw_haptic->trig[2].trig_bst     = aw_haptic->info.trig_cfg[23];
}

static void aw8692x_trig1_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[0].trig_level)
		trig_config |= AW8692X_BIT_TRGCFG7_TRG1_MODE_LEVEL;
	else
		trig_config |= AW8692X_BIT_TRGCFG7_TRG1_MODE_EDGE;

	if (aw_haptic->trig[0].trig_polar)
		trig_config |= AW8692X_BIT_TRGCFG7_TRG1_POLAR_NEG;
	else
		trig_config |= AW8692X_BIT_TRGCFG7_TRG1_POLAR_POS;

	if (aw_haptic->trig[0].trig_brk)
		trig_config |= AW8692X_BIT_TRGCFG7_TRG1_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRGCFG7_TRG1_AUTO_BRK_DISABLE;

	if (aw_haptic->trig[0].trig_bst)
		trig_config |= AW8692X_BIT_TRGCFG7_TRG1_BST_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRGCFG7_TRG1_BST_DISABLE;

	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_TRGCFG7,
				 (AW8692X_BIT_TRGCFG7_TRG1_MODE_MASK &
				  AW8692X_BIT_TRGCFG7_TRG1_POLAR_MASK &
				  AW8692X_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK &
				  AW8692X_BIT_TRGCFG7_TRG1_BST_MASK),
				 trig_config);

	trig_config = 0;

	/* pos config */
	if (aw_haptic->trig[0].pos_enable)
		trig_config |= AW8692X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRG_DISABLE;

	trig_config |= aw_haptic->trig[0].pos_sequence;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TRGCFG1, &trig_config,
			     AW_I2C_BYTE_ONE);

	trig_config = 0;

	/* neq config */
	if (aw_haptic->trig[0].neg_enable)
		trig_config |= AW8692X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRG_DISABLE;

	trig_config |= aw_haptic->trig[0].neg_sequence;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TRGCFG4, &trig_config,
			     AW_I2C_BYTE_ONE);
}

static void aw8692x_trig2_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[1].trig_level)
		trig_config |= AW8692X_BIT_TRGCFG7_TRG2_MODE_LEVEL;
	else
		trig_config |= AW8692X_BIT_TRGCFG7_TRG2_MODE_EDGE;

	if (aw_haptic->trig[1].trig_polar)
		trig_config |= AW8692X_BIT_TRGCFG7_TRG2_POLAR_NEG;
	else
		trig_config |= AW8692X_BIT_TRGCFG7_TRG2_POLAR_POS;

	if (aw_haptic->trig[1].trig_brk)
		trig_config |= AW8692X_BIT_TRGCFG7_TRG2_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRGCFG7_TRG2_AUTO_BRK_DISABLE;

	if (aw_haptic->trig[1].trig_bst)
		trig_config |= AW8692X_BIT_TRGCFG7_TRG2_BST_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRGCFG7_TRG2_BST_DISABLE;

	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_TRGCFG7,
				 (AW8692X_BIT_TRGCFG7_TRG2_MODE_MASK &
				  AW8692X_BIT_TRGCFG7_TRG2_POLAR_MASK &
				  AW8692X_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK &
				  AW8692X_BIT_TRGCFG7_TRG2_BST_MASK),
				 trig_config);

	trig_config = 0;

	/* pos config */
	if (aw_haptic->trig[1].pos_enable)
		trig_config |= AW8692X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRG_DISABLE;

	trig_config |= aw_haptic->trig[1].pos_sequence;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TRGCFG2, &trig_config,
			     AW_I2C_BYTE_ONE);

	trig_config = 0;

	/* neq config */
	if (aw_haptic->trig[1].neg_enable)
		trig_config |= AW8692X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRG_DISABLE;

	trig_config |= aw_haptic->trig[1].neg_sequence;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TRGCFG5, &trig_config,
			     AW_I2C_BYTE_ONE);
}

static void aw8692x_trig3_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[2].trig_level)
		trig_config |= AW8692X_BIT_TRGCFG8_TRG3_MODE_LEVEL;
	else
		trig_config |= AW8692X_BIT_TRGCFG8_TRG3_MODE_EDGE;

	if (aw_haptic->trig[2].trig_polar)
		trig_config |= AW8692X_BIT_TRGCFG8_TRG3_POLAR_NEG;
	else
		trig_config |= AW8692X_BIT_TRGCFG8_TRG3_POLAR_POS;

	if (aw_haptic->trig[2].trig_brk)
		trig_config |= AW8692X_BIT_TRGCFG8_TRG3_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRGCFG8_TRG3_AUTO_BRK_DISABLE;

	if (aw_haptic->trig[2].trig_bst)
		trig_config |= AW8692X_BIT_TRGCFG8_TRG3_BST_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRGCFG8_TRG3_BST_DISABLE;

	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_TRGCFG8,
				 (AW8692X_BIT_TRGCFG8_TRG3_MODE_MASK &
				  AW8692X_BIT_TRGCFG8_TRG3_POLAR_MASK &
				  AW8692X_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK &
				  AW8692X_BIT_TRGCFG8_TRG3_BST_MASK),
				 trig_config);

	trig_config = 0;

	/* pos config */
	if (aw_haptic->trig[2].pos_enable)
		trig_config |= AW8692X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRG_DISABLE;

	trig_config |= aw_haptic->trig[2].pos_sequence;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TRGCFG3, &trig_config,
			     AW_I2C_BYTE_ONE);

	trig_config = 0;

	/* neq config */
	if (aw_haptic->trig[2].neg_enable)
		trig_config |= AW8692X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8692X_BIT_TRG_DISABLE;

	trig_config |= aw_haptic->trig[2].neg_sequence;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_TRGCFG6, &trig_config,
			     AW_I2C_BYTE_ONE);
}

static void aw8692x_auto_bst_enable(struct aw_haptic *aw_haptic, uint8_t flag)
{
	aw_haptic->auto_boost = flag;
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
					 AW8692X_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW8692X_BIT_PLAYCFG3_AUTO_BST_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
					 AW8692X_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW8692X_BIT_PLAYCFG3_AUTO_BST_DISABLE);
	}
}

static void aw8692x_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%02X", reg_val);
	/* edge int mode */
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSCTRL4,
				 (AW8692X_BIT_SYSCTRL4_INT_MODE_MASK &
				  AW8692X_BIT_SYSCTRL4_INT_EDGE_MODE_MASK),
				 (AW8692X_BIT_SYSCTRL4_INT_MODE_EDGE |
				  AW8692X_BIT_SYSCTRL4_INT_EDGE_MODE_POS));
	/* int enable */
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSINTM,
				 (AW8692X_BIT_SYSINTM_BST_SCPM_MASK &
				  AW8692X_BIT_SYSINTM_BST_OVPM_MASK &
				  AW8692X_BIT_SYSINTM_UVLM_MASK &
				  AW8692X_BIT_SYSINTM_OCDM_MASK &
				  AW8692X_BIT_SYSINTM_OTM_MASK),
				 (AW8692X_BIT_SYSINTM_BST_SCPM_ON |
				  AW8692X_BIT_SYSINTM_BST_OVPM_OFF |
				  AW8692X_BIT_SYSINTM_UVLM_ON |
				  AW8692X_BIT_SYSINTM_OCDM_ON |
				  AW8692X_BIT_SYSINTM_OTM_ON));
}

static int aw8692x_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state = 0;
	uint8_t rtp_state = 0;

	glb_state = aw8692x_get_glb_state(aw_haptic);
	if (glb_state == AW8692X_BIT_GLBRD5_STATE_RTP_GO) {
		rtp_state = 1;  /*is going on */
		aw_info("rtp_routine_on");
	}
	return rtp_state;
}

static void aw8692x_get_ram_data(struct aw_haptic *aw_haptic, char *buf)
{
	int i = 0;
	int size = 0;

	while (i < aw_haptic->ram.len) {
		if ((aw_haptic->ram.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			size = aw_haptic->ram.len - i;
		else
			size = AW_RAMDATA_RD_BUFFER_SIZE;
		haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_RAMDATA, buf + i,
				    size);
		i += size;
	}
}

static void aw8692x_get_first_wave_addr(struct aw_haptic *aw_haptic,
					uint8_t *wave_addr)
{
	uint8_t reg_val[3] = {0};

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_RAMDATA, reg_val,
			    AW_I2C_BYTE_THREE);
	wave_addr[0] = reg_val[1];
	wave_addr[1] = reg_val[2];
}

static void aw8692x_get_wav_seq(struct aw_haptic *aw_haptic, uint32_t len)
{
	uint8_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_SIZE] = {0};

	if (len > AW_SEQUENCER_SIZE)
		len = AW_SEQUENCER_SIZE;
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_WAVCFG1, reg_val, len);
	for (i = 0; i < len; i++)
		aw_haptic->seq[i] = reg_val[i];
}

static size_t aw8692x_get_wav_loop(struct aw_haptic *aw_haptic, char *buf)
{
	uint8_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_LOOP_SIZE] = {0};
	size_t count = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_WAVCFG9, reg_val,
			    AW_SEQUENCER_LOOP_SIZE);

	for (i = 0; i < AW_SEQUENCER_LOOP_SIZE; i++) {
		aw_haptic->loop[i * 2 + 0] = (reg_val[i] >> 4) & 0x0F;
		aw_haptic->loop[i * 2 + 1] = (reg_val[i] >> 0) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 1,
				  aw_haptic->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 2,
				  aw_haptic->loop[i * 2 + 1]);
	}
	return count;
}

static void aw8692x_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSINT=0x%02X", reg_val);
}

static uint8_t aw8692x_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_PWMCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val >>= 7;
	return reg_val;
}

static int aw8692x_get_irq_state(struct aw_haptic *aw_haptic)
{
	int ret = 0;
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	//aw_dbg("reg SYSINT=0x%02X", reg_val);

	if (reg_val & AW8692X_BIT_SYSINT_BST_SCPI) {
		ret = AW_IRQ_BST_SCP;
		aw_err("chip scp int error");
	}

	if (reg_val & AW8692X_BIT_SYSINT_BST_OVPI) {
		ret = AW_IRQ_BST_OVP;
		aw_err("chip ov int error");
	}

	if (reg_val & AW8692X_BIT_SYSINT_UVLI) {
		ret = AW_IRQ_UVLO;
		aw_err("chip uvlo int error");
	}

	if (reg_val & AW8692X_BIT_SYSINT_OCDI) {
		ret = AW_IRQ_OCD;
		aw_err("chip over current int error");
	}

	if (reg_val & AW8692X_BIT_SYSINT_OTI) {
		ret = AW_IRQ_OT;
		aw_err("chip over temperature int error");
	}

	if (reg_val & AW8692X_BIT_SYSINT_DONEI) {
		ret = AW_IRQ_DONE;
		aw_info("chip playback done");
	}

	if (reg_val & AW8692X_BIT_SYSINT_FF_AFI) {
		ret = AW_IRQ_ALMOST_FULL;
		//aw_info("aw_haptic rtp mode fifo almost full!");
	}

	if (reg_val & AW8692X_BIT_SYSINT_FF_AEI)
		ret = AW_IRQ_ALMOST_EMPTY;

	return ret;
}

static int aw8692x_read_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint32_t f0_reg = 0;

#ifdef AW_LRA_F0_DEFAULT
	/* lra_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_CONTCFG14, reg_val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | reg_val[1];
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("lra_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	aw_haptic->f0 = (uint32_t)AW8692X_F0_FARMULA(f0_reg);
	aw_info("lra_f0=%d", aw_haptic->f0);
#else
	/* cont_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_CONTCFG16, reg_val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | reg_val[1];
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("cont_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	aw_haptic->f0 = (uint32_t)AW8692X_F0_FARMULA(f0_reg);
	aw_info("cont_f0=%d", aw_haptic->f0);
#endif
	return 0;
}

static int aw8692x_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
	uint8_t d2s_gain_default = 0;
	uint8_t cont_config[3] = {0};
	int drv_width = 0;
	int ret = 0;

	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw8692x_stop(aw_haptic);
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_DETCFG2, &reg_val,
			    AW_I2C_BYTE_ONE);
	d2s_gain_default = reg_val & AW8692X_BIT_DETCFG2_D2S_GAIN;
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG2,
				 AW8692X_BIT_DETCFG2_D2S_GAIN_MASK,
				 AW8692X_BIT_DETCFG2_D2S_GAIN_40);
	/* f0 calibrate work mode */
	aw8692x_play_mode(aw_haptic, AW_CONT_MODE);
	/* enable f0 detect */
	aw8692x_f0_detect(aw_haptic, true);
	/* cont config */
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_CONTCFG6,
				 AW8692X_BIT_CONTCFG6_TRACK_EN_MASK,
				 (aw_haptic->info.is_enabled_track_en << 7));
	/* enable auto break */
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_PLAYCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	brk_en_default = 0x04 & reg_val;
	aw8692x_auto_break_mode(aw_haptic, true);
	/* f0 driver level & time */
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_CONTCFG6,
				 AW8692X_BIT_CONTCFG6_DRV1_LVL_MASK,
				 aw_haptic->info.cont_drv1_lvl);
	cont_config[0] = aw_haptic->info.cont_drv2_lvl;
	cont_config[1] = aw_haptic->info.cont_drv1_time;
	cont_config[2] = aw_haptic->info.cont_drv2_time;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG7, cont_config,
			     AW_I2C_BYTE_THREE);
	/* TRACK_MARGIN */
	if (!aw_haptic->info.cont_track_margin) {
		aw_err("aw_haptic->info.cont_track_margin = 0");
	} else {
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG11,
				     &aw_haptic->info.cont_track_margin,
				     AW_I2C_BYTE_ONE);
	}
	/* DRV_WIDTH */
	if (!aw_haptic->info.f0_pre)
		return -ERANGE;
	drv_width = AW_DRV_WIDTH_FARMULA(aw_haptic->info.f0_pre,
					 aw_haptic->info.cont_brk_gain,
					 aw_haptic->info.cont_track_margin);
	if (drv_width < AW_DRV_WIDTH_MIN)
		drv_width = AW_DRV_WIDTH_MIN;
	else if (drv_width > AW_DRV_WIDTH_MAX)
		drv_width = AW_DRV_WIDTH_MAX;
	cont_config[0] = (uint8_t)drv_width;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG3, &cont_config[0],
			     AW_I2C_BYTE_ONE);
	/* play go */
	aw8692x_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw8692x_wait_enter_standby(aw_haptic);
	ret = aw8692x_read_f0(aw_haptic);
	/* restore default config */
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG2,
				 AW8692X_BIT_DETCFG2_D2S_GAIN_MASK,
				 d2s_gain_default);
	aw8692x_f0_detect(aw_haptic, false);
	/* recover auto break config */
	if (brk_en_default)
		aw8692x_auto_break_mode(aw_haptic, true);
	else
		aw8692x_auto_break_mode(aw_haptic, false);
	return ret;
}

static int aw8692x_ram_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
	uint8_t d2s_gain_default = 0;
	int ret = 0;

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return -ERANGE;
	}
	if (aw_haptic->ram.ram_num < AW_RAM_GET_F0_SEQ) {
		aw_err("miss ram get f0 waveform!");
		return -ERANGE;
	}
	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw8692x_stop(aw_haptic);
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_DETCFG2, &reg_val,
			    AW_I2C_BYTE_ONE);
	d2s_gain_default = reg_val & AW8692X_BIT_DETCFG2_D2S_GAIN;
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG2,
				 AW8692X_BIT_DETCFG2_D2S_GAIN_MASK,
				 AW8692X_BIT_DETCFG2_D2S_GAIN_40);
	/* f0 calibrate work mode */
	aw8692x_play_mode(aw_haptic, AW_RAM_MODE);
	/* enable f0 detect */
	aw8692x_f0_detect(aw_haptic, true);
	/* enable auto break */
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_PLAYCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	brk_en_default = 0x04 & reg_val;
	aw8692x_auto_break_mode(aw_haptic, true);
	aw8692x_set_bst_vol(aw_haptic, 8000);
	aw8692x_set_wav_seq(aw_haptic, 0x00, AW_RAM_GET_F0_SEQ);
	aw8692x_set_wav_seq(aw_haptic, 0x01, 0x00);
	aw8692x_set_wav_loop(aw_haptic, 0x00, 0x02);
	/* play go */
	aw8692x_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw8692x_wait_enter_standby(aw_haptic);
	ret = aw8692x_read_f0(aw_haptic);
	/* restore default config */
	aw8692x_set_bst_vol(aw_haptic, aw_haptic->vmax);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG2,
				 AW8692X_BIT_DETCFG2_D2S_GAIN_MASK,
				 d2s_gain_default);
	aw8692x_f0_detect(aw_haptic, false);
	/* recover auto break config */
	if (brk_en_default)
		aw8692x_auto_break_mode(aw_haptic, true);
	else
		aw8692x_auto_break_mode(aw_haptic, false);
	return ret;
}

static uint8_t aw8692x_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW8692X_BIT_SYSST_FF_AFS;
	reg_val >>= 3;
	return reg_val;
}

static uint8_t aw8692x_rtp_get_fifo_aes(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW8692X_BIT_SYSST_FF_AES;
	reg_val >>= 4;
	return reg_val;
}

static uint8_t aw8692x_get_osc_status(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_SYSST2, &state,
			    AW_I2C_BYTE_ONE);
	state &= AW8692X_BIT_SYSST2_FF_EMPTY;
	return state;
}

static int aw8692x_select_d2s_gain(uint8_t reg)
{
	int d2s_gain = 0;

	switch (reg) {
	case AW8692X_BIT_DETCFG2_D2S_GAIN_1:
		d2s_gain = 1;
		break;
	case AW8692X_BIT_DETCFG2_D2S_GAIN_2:
		d2s_gain = 2;
		break;
	case AW8692X_BIT_DETCFG2_D2S_GAIN_4:
		d2s_gain = 4;
		break;
	case AW8692X_BIT_DETCFG2_D2S_GAIN_8:
		d2s_gain = 8;
		break;
	case AW8692X_BIT_DETCFG2_D2S_GAIN_10:
		d2s_gain = 10;
		break;
	case AW8692X_BIT_DETCFG2_D2S_GAIN_16:
		d2s_gain = 16;
		break;
	case AW8692X_BIT_DETCFG2_D2S_GAIN_20:
		d2s_gain = 20;
		break;
	case AW8692X_BIT_DETCFG2_D2S_GAIN_40:
		d2s_gain = 40;
		break;
	default:
		d2s_gain = -1;
		break;
	}
	return d2s_gain;
}

static void aw8692x_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t reg_array[2] = {0};
	uint8_t adc_fs_default = 0;
	uint8_t d2s_gain = 0;
	uint32_t lra_code = 0;

	aw8692x_ram_init(aw_haptic, true);
	aw8692x_stop(aw_haptic);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG2,
				 AW8692X_BIT_DETCFG2_DET_SEQ0_MASK,
				 AW8692X_BIT_DETCFG2_DET_SEQ0_RL);
	haptic_hv_i2c_reads(aw_haptic,  AW8692X_REG_DETCFG1, &reg_val,
			    AW_I2C_BYTE_ONE);
	adc_fs_default = reg_val & AW8692X_BIT_DETCFG1_ADC_FS;
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG1,
				 (AW8692X_BIT_DETCFG1_ADC_FS_MASK &
				  AW8692X_BIT_DETCFG1_DET_GO_MASK),
				 (AW8692X_BIT_DETCFG1_ADC_FS_96KHZ |
				  AW8692X_BIT_DETCFG1_DET_GO_DET_SEQ0));
	usleep_range(3000, 3500);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG1,
				 AW8692X_BIT_DETCFG1_DET_GO_MASK,
				 AW8692X_BIT_DETCFG1_DET_GO_NA);
	/* restore default config*/
	aw8692x_ram_init(aw_haptic, false);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG1,
				 AW8692X_BIT_DETCFG1_ADC_FS_MASK,
				 adc_fs_default);
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_DETCFG2, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW8692X_BIT_DETCFG2_D2S_GAIN;
	d2s_gain = aw8692x_select_d2s_gain(reg_val);
	if (d2s_gain < 0) {
		aw_err("d2s_gain is error");
		return;
	}
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_DETRD1, reg_array,
			    AW_I2C_BYTE_TWO);
	lra_code = ((reg_array[0] & AW8692X_BIT_DETRD1_AVG_DATA) << 8) +
								   reg_array[1];
	aw_haptic->lra = AW8692X_LRA_FORMULA(lra_code, d2s_gain);
}


static void aw8692x_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw8692x_set_wav_seq(aw_haptic, 0x00, seq);
	aw8692x_set_wav_loop(aw_haptic, 0x00, AW8692X_WAVLOOP_INIFINITELY);
}

static void aw8692x_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_array[2] = {0};
	uint32_t vbat_code = 0;

	aw8692x_stop(aw_haptic);
	aw8692x_ram_init(aw_haptic, true);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG2,
				 AW8692X_BIT_DETCFG2_DET_SEQ0_MASK,
				 AW8692X_BIT_DETCFG2_DET_SEQ0_VBAT);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG1,
				 AW8692X_BIT_DETCFG1_DET_GO_MASK,
				 AW8692X_BIT_DETCFG1_DET_GO_DET_SEQ0);
	usleep_range(3000, 3500);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG1,
				 AW8692X_BIT_DETCFG1_DET_GO_MASK,
				 AW8692X_BIT_DETCFG1_DET_GO_NA);
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_DETRD1, reg_array,
			    AW_I2C_BYTE_TWO);
	vbat_code = ((reg_array[0] & AW8692X_BIT_DETRD1_AVG_DATA) << 8) +
								   reg_array[1];
	aw_haptic->vbat = AW8692X_VBAT_FORMULA(vbat_code);

	if (aw_haptic->vbat > AW_VBAT_MAX) {
		aw_haptic->vbat = AW_VBAT_MAX;
		aw_info("vbat max limit = %d", aw_haptic->vbat);
	}
	if (aw_haptic->vbat < AW_VBAT_MIN) {
		aw_haptic->vbat = AW_VBAT_MIN;
		aw_info("vbat min limit = %d", aw_haptic->vbat);
	}
	aw_info("awinic->vbat=%dmV, vbat_code=0x%02X",
		aw_haptic->vbat, vbat_code);
	aw8692x_ram_init(aw_haptic, false);
}

static ssize_t aw8692x_get_reg(struct aw_haptic *aw_haptic, ssize_t len,
			       char *buf)
{
	uint8_t i = 0;
	uint8_t reg_array[AW8692X_REG_ANACFG22 + 1] = {0};

	for (i = 0; i < AW8692X_REG_RTPDATA; i++)
		haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_RSTCFG, reg_array,
				    AW8692X_REG_RTPDATA);
	for (i = AW8692X_REG_RTPDATA + 1; i < AW8692X_REG_RAMDATA; i++)
		haptic_hv_i2c_reads(aw_haptic, (AW8692X_REG_RTPDATA + 1),
			       &reg_array[AW8692X_REG_RTPDATA + 1],
			       (AW8692X_REG_RAMDATA - AW8692X_REG_RTPDATA - 1));
	for (i = AW8692X_REG_RAMDATA + 1; i <= AW8692X_REG_ANACFG22; i++)
		haptic_hv_i2c_reads(aw_haptic, (AW8692X_REG_RAMDATA + 1),
			      &reg_array[AW8692X_REG_RAMDATA + 1],
			      (AW8692X_REG_ANACFG22 - AW8692X_REG_RAMDATA - 1));
	for (i = 0; i <= AW8692X_REG_ANACFG22; i++)
		if ((i != AW8692X_REG_RTPDATA) && (i != AW8692X_REG_RAMDATA))
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02X=0x%02X\n", i, reg_array[i]);
	return len;
}

static void aw8692x_offset_cali(struct aw_haptic *aw_haptic)
{

}

static void aw8692x_trig_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter!");
	if (aw_haptic->info.is_enabled_one_wire) {
		aw_info("one wire is enabled!");
		aw8692x_one_wire_init(aw_haptic);
	} else {
		aw8692x_trig1_param_init(aw_haptic);
		aw8692x_trig1_param_config(aw_haptic);
	}
	aw8692x_trig2_param_init(aw_haptic);
	aw8692x_trig3_param_init(aw_haptic);
	aw8692x_trig2_param_config(aw_haptic);
	aw8692x_trig3_param_config(aw_haptic);
}

#ifdef AW_CHECK_RAM_DATA
static int aw8692x_check_ram_data(struct aw_haptic *aw_haptic,
				  uint8_t *cont_data,
				  uint8_t *ram_data, uint32_t len)
{
	int i = 0;

	for (i = 0; i < len; i++) {
		if (ram_data[i] != cont_data[i]) {
			aw_err("check ramdata error, addr=0x%04x, ram_data=0x%02x, file_data=0x%02x",
				i, ram_data[i], cont_data[i]);
			return -ERANGE;
		}
	}
	return 0;
}
#endif

static int aw8692x_container_update(struct aw_haptic *aw_haptic,
				    struct aw_haptic_container *awinic_cont)
{
	uint8_t ae_addr_h = 0;
	uint8_t af_addr_h = 0;
	uint8_t ae_addr_l = 0;
	uint8_t af_addr_l = 0;
	uint8_t reg_array[3] = {0};
	uint32_t base_addr = 0;
	uint32_t shift = 0;
	int i = 0;
	int len = 0;
	int ret = 0;

#ifdef AW_CHECK_RAM_DATA
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};
#endif
	mutex_lock(&aw_haptic->lock);
	aw_haptic->ram.baseaddr_shift = 2;
	aw_haptic->ram.ram_shift = 4;
	/* RAMINIT Enable */
	aw8692x_ram_init(aw_haptic, true);
	/* Enter standby mode */
	aw8692x_stop(aw_haptic);
	/* base addr */
	shift = aw_haptic->ram.baseaddr_shift;
	aw_haptic->ram.base_addr =
	    (uint32_t)((awinic_cont->data[0 + shift] << 8) |
		       (awinic_cont->data[1 + shift]));
	base_addr = aw_haptic->ram.base_addr;
	aw_info("base_addr = %d", aw_haptic->ram.base_addr);

	/* set FIFO_AE and FIFO_AF addr */
	ae_addr_h = (uint8_t)AW8692X_FIFO_AE_ADDR_H(base_addr);
	af_addr_h = (uint8_t)AW8692X_FIFO_AF_ADDR_H(base_addr);
	reg_array[0] = ae_addr_h | af_addr_h;
	reg_array[1] = (uint8_t)AW8692X_FIFO_AE_ADDR_L(base_addr);
	reg_array[2] = (uint8_t)AW8692X_FIFO_AF_ADDR_L(base_addr);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_RTPCFG3, reg_array,
			     AW_I2C_BYTE_THREE);

	/* get FIFO_AE and FIFO_AF addr */
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_RTPCFG3, reg_array,
			    AW_I2C_BYTE_THREE);
	ae_addr_h = ((reg_array[0]) & AW8692X_BIT_RTPCFG3_FIFO_AEH) >> 4;
	ae_addr_l = reg_array[1];
	aw_info("almost_empty_threshold = %d",
		(uint16_t)((ae_addr_h << 8) | ae_addr_l));
	af_addr_h = ((reg_array[0]) & AW8692X_BIT_RTPCFG3_FIFO_AFH);
	af_addr_l = reg_array[2];
	aw_info("almost_full_threshold = %d",
		(uint16_t)((af_addr_h << 8) | af_addr_l));

	aw8692x_set_base_addr(aw_haptic);
	aw8692x_set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;

		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_RAMDATA,
				     &awinic_cont->data[i], len);
		i += len;
	}

#ifdef AW_CHECK_RAM_DATA
	aw8692x_set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_RD_BUFFER_SIZE;

		haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_RAMDATA,
				    ram_data, len);
		ret = aw8692x_check_ram_data(aw_haptic, &awinic_cont->data[i],
					     ram_data, len);
		if (ret < 0)
			break;
		i += len;
	}
	if (ret)
		aw_err("ram data check sum error");
	else
		aw_info("ram data check sum pass");
#endif
	/* RAMINIT Disable */
	aw8692x_ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);
	return ret;
}

static uint64_t aw8692x_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	uint64_t theory_time = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_SYSCTRL2, &reg_val,
			    AW_I2C_BYTE_ONE);
	fre_val = reg_val & AW8692X_BIT_SYSCTRL2_RCK_FRE;
	if (fre_val == AW8692X_RCK_FRE_24K)
		theory_time = (aw_haptic->rtp_len / 24000) * 1000000;	/*24K*/
	if (fre_val == AW8692X_RCK_FRE_48K)
		theory_time = (aw_haptic->rtp_len / 48000) * 1000000;	/*48K*/
	aw_info("microsecond:%llu  theory_time = %llu",
		aw_haptic->microsecond, theory_time);
	return theory_time;
}

static void aw8692x_parse_dt(struct device *dev, struct aw_haptic *aw_haptic,
			     struct device_node *np)
{
	uint8_t duration_time[3];
	uint8_t trig_config_temp[24];
	uint32_t val = 0;

	val = of_property_read_u8(np, "aw8692x_gain_bypass",
				  &aw_haptic->info.gain_bypass);
	if (val != 0)
		aw_info("aw8692x_gain_bypass not found");
	val = of_property_read_u32(np, "f0_pre", &aw_haptic->info.f0_pre);
	if (val != 0)
		aw_info("vib_f0_pre not found");
	val = of_property_read_u8(np, "f0_cali_percent",
				  &aw_haptic->info.f0_cali_percent);
	if (val != 0)
		aw_info("vib_f0_cali_percent not found");
	val = of_property_read_u8(np, "aw8692x_cont_drv1_lvl",
				  &aw_haptic->info.cont_drv1_lvl);
	if (val != 0)
		aw_info("vib_cont_drv1_lvl not found");
	val = of_property_read_u32(np, "aw8692x_cont_lra_vrms",
				   &aw_haptic->info.cont_lra_vrms);
	if (val != 0)
		aw_info("vib_cont_lra_vrms not found");
	val = of_property_read_u8(np, "aw8692x_cont_drv1_time",
				  &aw_haptic->info.cont_drv1_time);
	if (val != 0)
		aw_info("vib_cont_drv1_time not found");
	val = of_property_read_u8(np, "aw8692x_cont_drv2_time",
				  &aw_haptic->info.cont_drv2_time);
	if (val != 0)
		aw_info("vib_cont_drv2_time not found");
	val = of_property_read_u8(np, "aw8692x_cont_wait_num",
				  &aw_haptic->info.cont_wait_num);
	if (val != 0)
		aw_info("vib_cont_wait_num not found");
	val = of_property_read_u8(np, "aw8692x_cont_brk_time",
				  &aw_haptic->info.cont_brk_time);
	if (val != 0)
		aw_info("vib_cont_brk_time not found");
	val = of_property_read_u8(np, "aw8692x_cont_track_margin",
				  &aw_haptic->info.cont_track_margin);
	if (val != 0)
		aw_info("vib_cont_track_margin not found");
	val = of_property_read_u8(np, "aw8692x_brk_bst_md",
				  &aw_haptic->info.brk_bst_md);
	if (val != 0)
		aw_info("vib_brk_bst_md not found");
	val = of_property_read_u8(np, "aw8692x_cont_tset",
				  &aw_haptic->info.cont_tset);
	if (val != 0)
		aw_info("vib_cont_tset not found");
	val = of_property_read_u8(np, "aw8692x_cont_bemf_set",
				  &aw_haptic->info.cont_bemf_set);
	if (val != 0)
		aw_info("vib_cont_bemf_set not found");
	val = of_property_read_u8(np, "aw8692x_cont_bst_brk_gain",
				  &aw_haptic->info.cont_bst_brk_gain);
	if (val != 0)
		aw_info("vib_cont_bst_brk_gain not found");
	val = of_property_read_u8(np, "aw8692x_cont_brk_gain",
				  &aw_haptic->info.cont_brk_gain);
	if (val != 0)
		aw_info("vib_cont_brk_gain not found");
	val = of_property_read_u8(np, "aw8692x_d2s_gain",
				  &aw_haptic->info.d2s_gain);
	if (val != 0)
		aw_info("vib_d2s_gain not found");
	val = of_property_read_u8_array(np, "aw8692x_duration_time",
					duration_time,
					ARRAY_SIZE(duration_time));
	if (val != 0)
		aw_info("aw8692x_duration_time not found");
	else
		memcpy(aw_haptic->info.duration_time, duration_time,
		       sizeof(duration_time));
	val = of_property_read_u8_array(np, "aw8692x_trig_config",
					trig_config_temp,
					ARRAY_SIZE(trig_config_temp));
	if (val != 0)
		aw_info("vib_trig_config not found");
	else
		memcpy(aw_haptic->info.trig_cfg, trig_config_temp,
		       sizeof(trig_config_temp));
	val = of_property_read_u32(np, "aw8692x_bst_vol_default",
				   &aw_haptic->info.bst_vol_default);
	if (val != 0)
		aw_info("vib_bst_vol_default not found");
	aw_haptic->info.is_enabled_track_en =
		of_property_read_bool(np, "aw8692x_is_enabled_track_en");
	aw_info("aw_haptic->info.is_enabled_track_en = %d",
		aw_haptic->info.is_enabled_track_en);
	aw_haptic->info.is_enabled_auto_bst =
		of_property_read_bool(np, "aw8692x_is_enabled_auto_bst");
	aw_info("aw_haptic->info.is_enabled_auto_bst = %d",
		aw_haptic->info.is_enabled_auto_bst);
	aw_haptic->info.is_enabled_one_wire = of_property_read_bool(np,
						 "aw8692x_is_enabled_one_wire");
	aw_info("aw_haptic->info.is_enabled_one_wire = %d",
		aw_haptic->info.is_enabled_one_wire);
}

static void aw8692x_misc_para_init(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[8] = {0};
	uint32_t drv2_lvl = 0;

	/* Set I2C broadcast addr */
	reg_val[0] = (uint8_t)aw_haptic->i2c->addr;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_SYSCTRL5, reg_val,
			     AW_I2C_BYTE_ONE);
	/* Cont drv2 lvl */
	drv2_lvl = AW8692X_DRV2_LVL_FARMULA(aw_haptic->info.f0_pre,
					    aw_haptic->info.cont_lra_vrms);
	if (drv2_lvl > AW_DRV2_LVL_MAX)
		aw_haptic->info.cont_drv2_lvl = AW_DRV2_LVL_MAX;
	else
		aw_haptic->info.cont_drv2_lvl = (uint8_t)drv2_lvl;
	/* Get vmax */
	if (aw_haptic->info.bst_vol_default > 0)
		aw_haptic->vmax = aw_haptic->info.bst_vol_default;
	/* Get gain */
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_PLAYCFG2, reg_val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->gain = reg_val[0];
	/* Get wave_seq */
	haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_WAVCFG1, reg_val,
			    AW_I2C_BYTE_EIGHT);
	aw_haptic->index = reg_val[0];
	memcpy(aw_haptic->seq, reg_val, AW_SEQUENCER_SIZE);
	/* Set gain_bypass */
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_SYSCTRL4,
				 AW8692X_BIT_SYSCTRL4_GAIN_BYPASS_MASK,
				 aw_haptic->info.gain_bypass);
	aw8692x_reg_unlock(aw_haptic, true);
	reg_val[0] = AW8692X_BIT_ANACFG11_INIT_VAL;
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_ANACFG11, reg_val,
			     AW_I2C_BYTE_ONE);
	/* Close boost skip */
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_ANACFG12,
				 AW8692X_BIT_ANACFG12_BST_SKIP_MASK,
				 AW8692X_BIT_ANACFG12_BST_SKIP_SHUTDOWN);
	/* Open adaptive ipeak current limiting */
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_ANACFG15,
				 AW8692X_BIT_ANACFG15_BST_PEAK_MODE_MASK,
				 AW8692X_BIT_ANACFG15_BST_PEAK_BACK);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_ANACFG16,
				 AW8692X_BIT_ANACFG16_BST_SRC_MASK,
				 AW8692X_BIT_ANACFG16_BST_SRC_3NS);
	aw8692x_reg_unlock(aw_haptic, false);

	if (!aw_haptic->info.brk_bst_md)
		aw_err("aw_haptic->info.brk_bst_md = 0!");
	if (!aw_haptic->info.cont_brk_time)
		aw_err("aw_haptic->info.cont_brk_time = 0!");
	if (!aw_haptic->info.cont_tset)
		aw_err("aw_haptic->info.cont_tset = 0!");
	if (!aw_haptic->info.cont_bemf_set)
		aw_err("aw_haptic->info.cont_bemf_set = 0!");
	if (!aw_haptic->info.cont_bst_brk_gain)
		aw_err("aw_haptic->info.cont_bst_brk_gain = 0");
	if (!aw_haptic->info.cont_brk_gain)
		aw_err("aw_haptic->info.cont_brk_gain = 0!");
	if (!aw_haptic->info.d2s_gain)
		aw_err("aw_haptic->info.d2s_gain = 0!");

	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_CONTCFG1,
				 AW8692X_BIT_CONTCFG1_BRK_BST_MD_MASK,
				 (aw_haptic->info.brk_bst_md << 6));
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_CONTCFG5,
				 AW8692X_BIT_CONTCFG5_BST_BRK_GAIN_MASK &
				 AW8692X_BIT_CONTCFG5_BRK_GAIN_MASK,
				 (aw_haptic->info.cont_bst_brk_gain << 4) |
				  aw_haptic->info.cont_brk_gain);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_CONTCFG13,
				 AW8692X_BIT_CONTCFG13_TSET_MASK &
				 AW8692X_BIT_CONTCFG13_BEME_SET_MASK,
				 (aw_haptic->info.cont_tset << 4) |
				 aw_haptic->info.cont_bemf_set);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_DETCFG2,
				 AW8692X_BIT_DETCFG2_D2S_GAIN_MASK,
				 aw_haptic->info.d2s_gain);
	aw8692x_protect_config(aw_haptic, AW8692X_BIT_PWMCFG4_PRTIME_DEFAULT_VALUE,
			       AW8692X_BIT_PWMCFG3_PRLVL_DEFAULT_VALUE);
	haptic_hv_i2c_write_bits(aw_haptic, AW8692X_REG_PLAYCFG3,
				 AW8692X_BIT_PLAYCFG3_BRK_EN_MASK,
				 AW8692X_BIT_PLAYCFG3_BRK_DISABLE);
}

/******************************************************
 *
 * Extern function : sysfs attr
 *
 ******************************************************/
static ssize_t aw8692x_cont_wait_num_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_wait_num = 0x%02X\n",
			aw_haptic->info.cont_wait_num);
	return len;
}

static ssize_t aw8692x_cont_wait_num_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtou8(buf, 0, &aw_haptic->info.cont_wait_num);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG4,
			     &aw_haptic->info.cont_wait_num, AW_I2C_BYTE_ONE);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t aw8692x_cont_drv_lvl_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_lvl = 0x%02X, cont_drv2_lvl = 0x%02X\n",
			aw_haptic->info.cont_drv1_lvl,
			aw_haptic->info.cont_drv2_lvl);
	return len;
}

static ssize_t aw8692x_cont_drv_lvl_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	uint32_t databuf[2] = {0};
	uint8_t reg_array[2] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_lvl = databuf[0];
		aw_haptic->info.cont_drv2_lvl = databuf[1];
		mutex_lock(&aw_haptic->lock);
		haptic_hv_i2c_reads(aw_haptic, AW8692X_REG_CONTCFG6, reg_array,
				    AW_I2C_BYTE_ONE);
		reg_array[0] &= AW8692X_BIT_CONTCFG6_DRV1_LVL_MASK;
		reg_array[0] |= aw_haptic->info.cont_drv1_lvl;
		reg_array[1] = aw_haptic->info.cont_drv2_lvl;
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG6, reg_array,
				     AW_I2C_BYTE_TWO);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t aw8692x_cont_drv_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_time = 0x%02X, cont_drv2_time = 0x%02X\n",
			aw_haptic->info.cont_drv1_time,
			aw_haptic->info.cont_drv2_time);
	return len;
}

static ssize_t aw8692x_cont_drv_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	uint8_t reg_array[2] = {0};
	uint32_t databuf[2] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_time = databuf[0];
		aw_haptic->info.cont_drv2_time = databuf[1];
		reg_array[0] = (uint8_t)aw_haptic->info.cont_drv1_time;
		reg_array[1] = (uint8_t)aw_haptic->info.cont_drv2_time;
		mutex_lock(&aw_haptic->lock);
		haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG8, reg_array,
				     AW_I2C_BYTE_TWO);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static ssize_t aw8692x_cont_brk_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "cont_brk_time = 0x%02X\n",
			aw_haptic->info.cont_brk_time);
	return len;
}

static ssize_t aw8692x_cont_brk_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtou8(buf, 0, &aw_haptic->info.cont_brk_time);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	haptic_hv_i2c_writes(aw_haptic, AW8692X_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);
	mutex_unlock(&aw_haptic->lock);
	return count;
}

static ssize_t aw8692x_trig_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	for (i = 0; i < AW_TRIG_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: trig_level=%d, trig_polar=%d, pos_enable=%d, pos_sequence=%d, neg_enable=%d, neg_sequence=%d trig_brk=%d, trig_bst=%d\n",
				i + 1,
				aw_haptic->trig[i].trig_level,
				aw_haptic->trig[i].trig_polar,
				aw_haptic->trig[i].pos_enable,
				aw_haptic->trig[i].pos_sequence,
				aw_haptic->trig[i].neg_enable,
				aw_haptic->trig[i].neg_sequence,
				aw_haptic->trig[i].trig_brk,
				aw_haptic->trig[i].trig_bst);
	}

	return len;
}

static ssize_t aw8692x_trig_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	uint32_t databuf[9] = { 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%d %d %d %d %d %d %d %d %d", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5],
		   &databuf[6], &databuf[7], &databuf[8]) == 9) {
		aw_info("%d, %d, %d, %d, %d, %d, %d, %d, %d",
			databuf[0], databuf[1], databuf[2], databuf[3],
			databuf[4], databuf[5], databuf[6], databuf[7],
			databuf[8]);
		if (databuf[0] < 1 || databuf[0] > 3) {
			aw_info("input trig_num out of range!");
			return count;
		}
		if (databuf[0] == 1 && aw_haptic->info.is_enabled_one_wire) {
			aw_info("trig1 pin used for one wire!");
			return count;
		}
		if (!aw_haptic->ram_init) {
			aw_err("ram init failed, not allow to play!");
			return count;
		}
		if (databuf[4] > aw_haptic->ram.ram_num ||
		    databuf[6] > aw_haptic->ram.ram_num) {
			aw_err("input seq value out of range!");
			return count;
		}
		databuf[0] -= 1;

		aw_haptic->trig[databuf[0]].trig_level = databuf[1];
		aw_haptic->trig[databuf[0]].trig_polar = databuf[2];
		aw_haptic->trig[databuf[0]].pos_enable = databuf[3];
		aw_haptic->trig[databuf[0]].pos_sequence = databuf[4];
		aw_haptic->trig[databuf[0]].neg_enable = databuf[5];
		aw_haptic->trig[databuf[0]].neg_sequence = databuf[6];
		aw_haptic->trig[databuf[0]].trig_brk = databuf[7];
		aw_haptic->trig[databuf[0]].trig_bst = databuf[8];
		mutex_lock(&aw_haptic->lock);
		switch (databuf[0]) {
		case AW8692X_TRIG1:
			aw8692x_trig1_param_config(aw_haptic);
			break;
		case AW8692X_TRIG2:
			aw8692x_trig2_param_config(aw_haptic);
			break;
		case AW8692X_TRIG3:
			aw8692x_trig3_param_config(aw_haptic);
			break;
		}
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static DEVICE_ATTR(cont_wait_num, S_IWUSR | S_IRUGO, aw8692x_cont_wait_num_show,
		   aw8692x_cont_wait_num_store);
static DEVICE_ATTR(cont_drv_lvl, S_IWUSR | S_IRUGO, aw8692x_cont_drv_lvl_show,
		   aw8692x_cont_drv_lvl_store);
static DEVICE_ATTR(cont_drv_time, S_IWUSR | S_IRUGO, aw8692x_cont_drv_time_show,
		   aw8692x_cont_drv_time_store);
static DEVICE_ATTR(cont_brk_time, S_IWUSR | S_IRUGO, aw8692x_cont_brk_time_show,
		   aw8692x_cont_brk_time_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw8692x_trig_show,
		   aw8692x_trig_store);

static struct attribute *aw8692x_vibrator_attributes[] = {
	&dev_attr_cont_wait_num.attr,
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	&dev_attr_cont_brk_time.attr,
	&dev_attr_trig.attr,
	NULL
};

static struct attribute_group aw8692x_vibrator_attribute_group = {
	.attrs = aw8692x_vibrator_attributes
};

static int aw8692x_creat_node(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &aw8692x_vibrator_attribute_group);
	if (ret < 0) {
		aw_err("error create aw8692x sysfs attr files");
		return ret;
	}
	return 0;
}

struct aw_haptic_func aw8692x_func_list = {
	.play_stop = aw8692x_stop,
	.ram_init = aw8692x_ram_init,
	.get_vbat = aw8692x_get_vbat,
	.creat_node = aw8692x_creat_node,
	.get_f0 = aw8692x_get_f0,
	.ram_get_f0 = aw8692x_ram_get_f0,
	.cont_config = aw8692x_cont_config,
	.offset_cali = aw8692x_offset_cali,
	.get_irq_state = aw8692x_get_irq_state,
	.check_qualify = aw8692x_check_qualify,
	.judge_rtp_going = aw8692x_judge_rtp_going,
	.set_bst_peak_cur = aw8692x_set_bst_peak_cur,
	.get_theory_time = aw8692x_get_theory_time,
	.get_lra_resistance = aw8692x_get_lra_resistance,
	.set_pwm = aw8692x_set_pwm,
	.play_mode = aw8692x_play_mode,
	.set_bst_vol = aw8692x_set_bst_vol,
	.interrupt_setup = aw8692x_interrupt_setup,
	.set_repeat_seq = aw8692x_set_repeat_seq,
	.auto_bst_enable = aw8692x_auto_bst_enable,
	.vbat_mode_config = aw8692x_vbat_mode_config,
	.set_wav_seq = aw8692x_set_wav_seq,
	.set_wav_loop = aw8692x_set_wav_loop,
	.set_ram_addr = aw8692x_set_ram_addr,
	.set_rtp_data = aw8692x_set_rtp_data,
	.container_update = aw8692x_container_update,
	.protect_config = aw8692x_protect_config,
	.parse_dt = aw8692x_parse_dt,
	.trig_init = aw8692x_trig_init,
	.irq_clear = aw8692x_irq_clear,
	.get_wav_loop = aw8692x_get_wav_loop,
	.play_go = aw8692x_play_go,
	.misc_para_init = aw8692x_misc_para_init,
	.set_rtp_aei = aw8692x_set_rtp_aei,
	.set_gain = aw8692x_set_gain,
	.upload_lra = aw8692x_upload_lra,
	.bst_mode_config = aw8692x_bst_mode_config,
	.get_reg = aw8692x_get_reg,
	.get_prctmode = aw8692x_get_prctmode,
	.get_trim_lra = aw8692x_get_trim_lra,
	.get_ram_data = aw8692x_get_ram_data,
	.get_first_wave_addr = aw8692x_get_first_wave_addr,
	.get_glb_state = aw8692x_get_glb_state,
	.get_osc_status = aw8692x_get_osc_status,
	.rtp_get_fifo_afs = aw8692x_rtp_get_fifo_afs,
	.rtp_get_fifo_aes = aw8692x_rtp_get_fifo_aes,
	.get_wav_seq = aw8692x_get_wav_seq,
};
