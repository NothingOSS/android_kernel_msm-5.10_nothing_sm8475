/*
 * File: aw869x.c
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

#include "haptic_hv_reg.h"
#include "haptic_hv.h"

static void aw869x_vbat_mode_config(struct aw_haptic *, uint8_t);

static int aw869x_check_qualify(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	return 0;
}

static void aw869x_ram_init(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag == true) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 AW869X_BIT_SYSCTRL_RAMINIT_MASK,
					 AW869X_BIT_SYSCTRL_RAMINIT_EN);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 AW869X_BIT_SYSCTRL_RAMINIT_MASK,
					 AW869X_BIT_SYSCTRL_RAMINIT_OFF);
	}
}

static void aw869x_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	aw_info("enter, flag = %d", flag);
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_GO,
					 AW869X_BIT_GO_MASK,
					 AW869X_BIT_GO_ENABLE);
		aw_haptic->kpre_enter_time = ktime_get();
	} else {
		aw_haptic->kcurrent_time = ktime_get();
		aw_haptic->interval_us = ktime_to_us(
					ktime_sub(aw_haptic->kcurrent_time,
						  aw_haptic->kpre_enter_time));
		if (aw_haptic->interval_us < 2000) {
			aw_info("aw_haptic->interval_us=%d < 2000",
				aw_haptic->interval_us);
			usleep_range(2000, 2500);
		}
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_GO,
					 AW869X_BIT_GO_MASK,
					 AW869X_BIT_GO_DISABLE);
	}
}

static uint8_t aw869x_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_GLB_STATE, &state,
			    AW_I2C_BYTE_ONE);
	aw_dbg("glb state value is 0x%02X", state);
	return state;
}

#ifdef AW_CHECK_RAM_DATA
static int aw869x_check_ram_data(struct aw_haptic *aw_haptic,
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

static int aw869x_container_update(struct aw_haptic *aw_haptic,
				   struct aw_haptic_container *awinic_cont)
{
	uint8_t fifo_adr[4] = {0};
	uint16_t ae_thr = 0;
	uint16_t af_thr = 0;
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
	aw869x_ram_init(aw_haptic, true);
	/* base addr */
	shift = aw_haptic->ram.baseaddr_shift;
	aw_haptic->ram.base_addr =
		   (uint32_t)((awinic_cont->data[0 + shift] << 8) |
		   (awinic_cont->data[1 + shift]));
	fifo_adr[0] = (uint8_t)AW869X_FIFO_AE_ADDR_H(aw_haptic->ram.base_addr);
	fifo_adr[1] = (uint8_t)AW869X_FIFO_AE_ADDR_L(aw_haptic->ram.base_addr);
	fifo_adr[2] = (uint8_t)AW869X_FIFO_AF_ADDR_H(aw_haptic->ram.base_addr);
	fifo_adr[3] = (uint8_t)AW869X_FIFO_AF_ADDR_L(aw_haptic->ram.base_addr);
	aw_info("base_addr=0x%04x", aw_haptic->ram.base_addr);
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_BASE_ADDRH,
			     &awinic_cont->data[shift], AW_I2C_BYTE_TWO);
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_FIFO_AEH, fifo_adr,
			     AW_I2C_BYTE_FOUR);
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_FIFO_AEH, fifo_adr,
			    AW_I2C_BYTE_FOUR);
	ae_thr = (((fifo_adr[0] & 0x0f) << 8) | fifo_adr[1]);
	aw_info("almost_empty_threshold = %d", ae_thr);
	af_thr = (((fifo_adr[2] & 0x0f) << 8) | fifo_adr[3]);
	aw_info("almost_full_threshold = %d", af_thr);
	/* ram */
	shift = aw_haptic->ram.baseaddr_shift;
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_RAMADDRH,
			     &awinic_cont->data[shift], AW_I2C_BYTE_TWO);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;
		haptic_hv_i2c_writes(aw_haptic, AW869X_REG_RAMDATA,
				     &awinic_cont->data[i], len);
		i += len;
	}
#ifdef AW_CHECK_RAM_DATA
	/* ram check */
	shift = aw_haptic->ram.baseaddr_shift;
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_RAMADDRH,
			     &awinic_cont->data[shift], AW_I2C_BYTE_TWO);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_RD_BUFFER_SIZE;
		haptic_hv_i2c_reads(aw_haptic, AW869X_REG_RAMDATA,
				    ram_data, len);
		ret = aw869x_check_ram_data(aw_haptic, &awinic_cont->data[i],
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
	aw869x_ram_init(aw_haptic, false);

	mutex_unlock(&aw_haptic->lock);

	return ret;
}

/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
static void aw869x_bst_mode_config(struct aw_haptic *aw_haptic,
				   uint8_t boost_mode)
{
	switch (boost_mode) {
	case AW_BST_BOOST_MODE:
		aw_info("haptic boost mode = boost");
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 AW869X_BIT_SYSCTRL_BST_MODE_MASK,
					 AW869X_BIT_SYSCTRL_BST_MODE_BOOST);
		break;
	case AW_BST_BYPASS_MODE:
		aw_info("haptic boost mode = bypass");
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 AW869X_BIT_SYSCTRL_BST_MODE_MASK,
					 AW869X_BIT_SYSCTRL_BST_MODE_BYPASS);
		break;
	default:
		aw_err("boost_mode = %d error", boost_mode);
		break;
	}
}

static void aw869x_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSINT=0x%x", reg_val);
}

static void aw869x_active(struct aw_haptic *aw_haptic)
{
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
				 AW869X_BIT_SYSCTRL_WORK_MODE_MASK,
				 AW869X_BIT_SYSCTRL_ACTIVE);
	aw869x_irq_clear(aw_haptic);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSINTM,
				 AW869X_BIT_SYSINTM_UVLO_MASK,
				 AW869X_BIT_SYSINTM_UVLO_EN);
}

static void aw869x_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_info("enter standby mode");
		aw_haptic->play_mode = AW_STANDBY_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSINTM,
					 AW869X_BIT_SYSINTM_UVLO_MASK,
					 AW869X_BIT_SYSINTM_UVLO_OFF);
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 AW869X_BIT_SYSCTRL_WORK_MODE_MASK,
					 AW869X_BIT_SYSCTRL_STANDBY);
		break;
	case AW_RAM_MODE:
		aw_info("enter ram mode");
		aw_haptic->play_mode = AW_RAM_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_MASK &
					  AW869X_BIT_SYSCTRL_BST_MODE_MASK),
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_RAM |
					  AW869X_BIT_SYSCTRL_BST_MODE_BYPASS));
		if (aw_haptic->auto_boost) {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
					   AW869X_BIT_BST_AUTO_BST_RAM_MASK,
					   AW869X_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw869x_active(aw_haptic);
		if (aw_haptic->auto_boost) {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
					    AW869X_BIT_BST_AUTO_BST_RAM_MASK,
					    AW869X_BIT_BST_AUTO_BST_RAM_ENABLE);
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					    (AW869X_BIT_SYSCTRL_BST_MODE_MASK &
					     AW869X_BIT_SYSCTRL_WORK_MODE_MASK),
					    (AW869X_BIT_SYSCTRL_BST_MODE_BOOST |
					     AW869X_BIT_SYSCTRL_STANDBY));
			aw869x_active(aw_haptic);
		} else {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					     AW869X_BIT_SYSCTRL_BST_MODE_MASK,
					     AW869X_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		usleep_range(2000, 2500);
		aw869x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RAM_LOOP_MODE:
		aw_info("enter ram loop mode");
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_MASK &
					  AW869X_BIT_SYSCTRL_BST_MODE_MASK),
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_RAM |
					  AW869X_BIT_SYSCTRL_BST_MODE_BYPASS));

		if (aw_haptic->auto_boost) {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
					   AW869X_BIT_BST_AUTO_BST_RAM_MASK,
					   AW869X_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw869x_active(aw_haptic);
		aw869x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RTP_MODE:
		aw_info("enter rtp mode");
		aw_haptic->play_mode = AW_RTP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_MASK &
					  AW869X_BIT_SYSCTRL_BST_MODE_MASK),
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_RTP |
					  AW869X_BIT_SYSCTRL_BST_MODE_BYPASS));

		if (aw_haptic->auto_boost) {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
					   AW869X_BIT_BST_AUTO_BST_RTP_MASK,
					   AW869X_BIT_BST_AUTO_BST_RTP_DISABLE);
		}
		aw869x_active(aw_haptic);
		if (aw_haptic->auto_boost) {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
					    AW869X_BIT_BST_AUTO_BST_RTP_MASK,
					    AW869X_BIT_BST_AUTO_BST_RTP_ENABLE);
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					    (AW869X_BIT_SYSCTRL_BST_MODE_MASK &
					     AW869X_BIT_SYSCTRL_WORK_MODE_MASK),
					    (AW869X_BIT_SYSCTRL_BST_MODE_BOOST |
					     AW869X_BIT_SYSCTRL_STANDBY));
			aw869x_active(aw_haptic);
		} else {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					     AW869X_BIT_SYSCTRL_BST_MODE_MASK,
					     AW869X_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		usleep_range(2000, 2500);
		aw869x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_TRIG_MODE:
		aw_info("enter trig mode");
		aw_haptic->play_mode = AW_TRIG_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_MASK &
					  AW869X_BIT_SYSCTRL_BST_MODE_MASK),
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_RAM |
					  AW869X_BIT_SYSCTRL_BST_MODE_BYPASS));

		if (aw_haptic->auto_boost) {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
					   AW869X_BIT_BST_AUTO_BST_RAM_MASK,
					   AW869X_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw869x_active(aw_haptic);
		if (aw_haptic->auto_boost) {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
					    AW869X_BIT_BST_AUTO_BST_RAM_MASK,
					    AW869X_BIT_BST_AUTO_BST_RAM_ENABLE);
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					    (AW869X_BIT_SYSCTRL_BST_MODE_MASK &
					     AW869X_BIT_SYSCTRL_WORK_MODE_MASK),
					    (AW869X_BIT_SYSCTRL_BST_MODE_BOOST |
					     AW869X_BIT_SYSCTRL_STANDBY));
			aw869x_active(aw_haptic);
		} else {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					     AW869X_BIT_SYSCTRL_BST_MODE_MASK,
					     AW869X_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		usleep_range(2000, 2500);
		aw869x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_CONT_MODE:
		aw_info("enter cont mode");
		aw_haptic->play_mode = AW_CONT_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_MASK &
					  AW869X_BIT_SYSCTRL_BST_MODE_MASK),
					 (AW869X_BIT_SYSCTRL_PLAY_MODE_CONT |
					  AW869X_BIT_SYSCTRL_BST_MODE_BYPASS));
		if (aw_haptic->auto_boost) {
			haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
					   AW869X_BIT_BST_AUTO_BST_RAM_MASK,
					   AW869X_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw869x_active(aw_haptic);
		aw869x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
		break;
	default:
		aw_err("play mode %d err", play_mode);
		break;
	}
}

static int aw869x_wait_enter_standby(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int count = 100;

	while (count--) {
		reg_val = aw869x_get_glb_state(aw_haptic);
		if ((reg_val & 0x0f) == AW869X_BIT_GLB_STATE_STANDBY) {
			aw_info("entered standby!");
			return 0;
		}
		aw_dbg("wait for standby");
		usleep_range(2000, 2500);
	}
	aw_err("do not enter standby automatically");
	return -ERANGE;
}

static void aw869x_stop(struct aw_haptic *aw_haptic)
{
	aw_haptic->play_mode = AW_STANDBY_MODE;
	aw869x_play_go(aw_haptic, false);
	aw869x_wait_enter_standby(aw_haptic);
	aw869x_play_mode(aw_haptic, AW_STANDBY_MODE);
}

static int aw869x_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t rtp_state = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_SYSCTRL, &reg_val,
			    AW_I2C_BYTE_ONE);
	if ((reg_val & AW869X_BIT_SYSCTRL_PLAY_MODE_RTP) &&
	    (!(reg_val & AW869X_BIT_SYSCTRL_STANDBY))) {
		rtp_state = 1;	/*is going on */
		aw_info("rtp_routine_on");
	}
	return rtp_state;
}

static void aw869x_set_wav_seq(struct aw_haptic *aw_haptic, uint8_t wav,
			       uint8_t seq)
{
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_WAVSEQ1 + wav, &seq,
			     AW_I2C_BYTE_ONE);
}

static void aw869x_get_wav_seq(struct aw_haptic *aw_haptic, uint32_t len)
{
	uint8_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_SIZE] = {0};

	if (len > AW_SEQUENCER_SIZE)
		len = AW_SEQUENCER_SIZE;
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_WAVSEQ1, reg_val, len);
	for (i = 0; i < len; i++)
		aw_haptic->seq[i] = reg_val[i];
}

static void aw869x_get_ram_data(struct aw_haptic *aw_haptic, char *buf)
{
	int i = 0;
	int size = 0;

	while (i < aw_haptic->ram.len) {
		if ((aw_haptic->ram.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			size = aw_haptic->ram.len - i;
		else
			size = AW_RAMDATA_RD_BUFFER_SIZE;
		haptic_hv_i2c_reads(aw_haptic, AW869X_REG_RAMDATA, buf + i,
				    size);
		i += size;
	}
}

static void aw869x_get_first_wave_addr(struct aw_haptic *aw_haptic,
					uint8_t *wave_addr)
{
	uint8_t reg_array[3] = {0};

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_RAMDATA, reg_array,
			    AW_I2C_BYTE_THREE);
	wave_addr[0] = reg_array[1];
	wave_addr[1] = reg_array[2];
}

static void aw869x_set_ram_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ram_addr[2] = {0};

	ram_addr[0] = (uint8_t)AW869X_RAM_ADDR_H(aw_haptic->ram.base_addr);
	ram_addr[1] = (uint8_t)AW869X_RAM_ADDR_L(aw_haptic->ram.base_addr);
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_RAMADDRH, ram_addr,
			     AW_I2C_BYTE_TWO);
}

static void aw869x_set_wav_loop(struct aw_haptic *aw_haptic, uint8_t wav,
				uint8_t loop)
{
	uint8_t tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		haptic_hv_i2c_write_bits(aw_haptic,
					 AW869X_REG_WAVLOOP1 + (wav / 2),
					 AW869X_BIT_WAVLOOP_SEQNP1_MASK, tmp);
	} else {
		tmp = loop << 4;
		haptic_hv_i2c_write_bits(aw_haptic,
					 AW869X_REG_WAVLOOP1 + (wav / 2),
					 AW869X_BIT_WAVLOOP_SEQN_MASK, tmp);
	}
}

static size_t aw869x_get_wav_loop(struct aw_haptic *aw_haptic, char *buf)
{
	uint8_t i = 0;
	uint8_t reg_val[4] = {0};
	size_t count = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_WAVLOOP1, reg_val,
			    AW_I2C_BYTE_FOUR);
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

static uint8_t aw869x_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_PRLVL, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val >>= 7;
	return reg_val;
}

static void aw869x_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw869x_set_wav_seq(aw_haptic, 0x00, seq);
	aw869x_set_wav_loop(aw_haptic, 0x00, AW869X_BIT_WAVLOOP_INIFINITELY);
}

static void aw869x_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data,
				uint32_t len)
{
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_RTP_DATA, data, len);
}

static void aw869x_set_bst_vol(struct aw_haptic *aw_haptic, uint32_t bst_vol)
{
	uint8_t reg_val = 0;

	if (bst_vol < AW869X_BST_VOL_MIN)
		bst_vol = AW869X_BST_VOL_MIN;
	else if (bst_vol > AW869X_BST_VOL_MAX)
		bst_vol = AW869X_BST_VOL_MAX;
	reg_val = AW869X_BST_VOL_FARMULA(bst_vol);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BSTDBG4,
				 AW869X_BIT_BSTDBG4_BSTVOL_MASK, reg_val);
}

static void aw869x_set_bst_peak_cur(struct aw_haptic *aw_haptic)
{
	switch (aw_haptic->bst_pc) {
	case AW_BST_PC_L1:
		aw_info("bst pc = L1");
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BSTCFG,
					 AW869X_BIT_BSTCFG_PEAKCUR_MASK,
					 AW869X_BIT_BSTCFG_PEAKCUR_2A);
		break;
	case AW_BST_PC_L2:
		aw_info("bst pc = L2");
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BSTCFG,
					 AW869X_BIT_BSTCFG_PEAKCUR_MASK,
					 AW869X_BIT_BSTCFG_PEAKCUR_3P5A);
		break;
	default:
		aw_info("bst pc = L1");
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BSTCFG,
					 AW869X_BIT_BSTCFG_PEAKCUR_MASK,
					 AW869X_BIT_BSTCFG_PEAKCUR_2A);
		break;
	}
}

static void aw869x_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_DATDBG, &gain,
			     AW_I2C_BYTE_ONE);
}

static void aw869x_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_PWMDBG,
					 AW869X_BIT_PWMDBG_PWM_MODE_MASK,
					 AW869X_BIT_PWMDBG_PWM_48K);
		break;
	case AW_PWM_24K:
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_PWMDBG,
					 AW869X_BIT_PWMDBG_PWM_MODE_MASK,
					 AW869X_BIT_PWMDBG_PWM_24K);
		break;
	case AW_PWM_12K:
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_PWMDBG,
					 AW869X_BIT_PWMDBG_PWM_MODE_MASK,
					 AW869X_BIT_PWMDBG_PWM_12K);
		break;
	default:
		break;
	}
}

/*****************************************************
 *
 * motor protect
 *
 *****************************************************/
static void aw869x_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime,
				  uint8_t prlvl)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_PWMPRC,
				 AW869X_BIT_PWMPRC_PRC_MASK,
				 AW869X_BIT_PWMPRC_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_info("enable protection mode");
		reg_val = AW869X_BIT_PRLVL_PR_ENABLE |
			  (prlvl & (~AW869X_BIT_PRLVL_PRLVL_MASK));
		haptic_hv_i2c_writes(aw_haptic, AW869X_REG_PRLVL, &reg_val,
				     AW_I2C_BYTE_ONE);
		haptic_hv_i2c_writes(aw_haptic, AW869X_REG_PRTIME, &prtime,
				     AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_info("disable protection mode");
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_PRLVL,
					 AW869X_BIT_PRLVL_PR_MASK,
					 AW869X_BIT_PRLVL_PR_DISABLE);
	}
}

/*****************************************************
 *
 * offset calibration
 *
 *****************************************************/
static void aw869x_offset_cali(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t cnt = 2000;

	aw869x_ram_init(aw_haptic, true);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_DETCTRL,
				 AW869X_BIT_DETCTRL_DIAG_GO_MASK,
				 AW869X_BIT_DETCTRL_DIAG_GO_ENABLE);
	while (cnt--) {
		haptic_hv_i2c_reads(aw_haptic, AW869X_REG_DETCTRL, &reg_val,
				    AW_I2C_BYTE_ONE);
		if ((reg_val & 0x01) == 0) {
			aw869x_ram_init(aw_haptic, false);
			return;
		}
	}
	aw_err("calibration offset failed!");
	aw869x_ram_init(aw_haptic, false);
}

/*****************************************************
 *
 * trig config
 *
 *****************************************************/
static void aw869x_trig_param_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->trig[0].enable = aw_haptic->info.trig_cfg[0];
	aw_haptic->trig[0].default_level = aw_haptic->info.trig_cfg[1];
	aw_haptic->trig[0].dual_edge = aw_haptic->info.trig_cfg[2];
	aw_haptic->trig[0].frist_seq = aw_haptic->info.trig_cfg[3];
	aw_haptic->trig[0].second_seq = aw_haptic->info.trig_cfg[4];
	aw_haptic->trig[1].enable = aw_haptic->info.trig_cfg[5];
	aw_haptic->trig[1].default_level = aw_haptic->info.trig_cfg[6];
	aw_haptic->trig[1].dual_edge = aw_haptic->info.trig_cfg[7];
	aw_haptic->trig[1].frist_seq = aw_haptic->info.trig_cfg[8];
	aw_haptic->trig[1].second_seq = aw_haptic->info.trig_cfg[9];
	aw_haptic->trig[2].enable = aw_haptic->info.trig_cfg[10];
	aw_haptic->trig[2].default_level = aw_haptic->info.trig_cfg[11];
	aw_haptic->trig[2].dual_edge = aw_haptic->info.trig_cfg[12];
	aw_haptic->trig[2].frist_seq = aw_haptic->info.trig_cfg[13];
	aw_haptic->trig[2].second_seq = aw_haptic->info.trig_cfg[14];
}

static void aw869x_trig_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;
	uint8_t trig_seq[6] = {0};

	if (aw_haptic->trig[0].default_level)
		trig_config |= AW869X_BIT_TRGCFG1_TRG1_POLAR_NEG;
	else
		trig_config |= AW869X_BIT_TRGCFG1_TRG1_POLAR_POS;
	if (aw_haptic->trig[1].default_level)
		trig_config |= AW869X_BIT_TRGCFG1_TRG2_POLAR_NEG;
	else
		trig_config |= AW869X_BIT_TRGCFG1_TRG2_POLAR_POS;
	if (aw_haptic->trig[2].default_level)
		trig_config |= AW869X_BIT_TRGCFG1_TRG3_POLAR_NEG;
	else
		trig_config |= AW869X_BIT_TRGCFG1_TRG3_POLAR_POS;
	if (aw_haptic->trig[0].dual_edge)
		trig_config |= AW869X_BIT_TRGCFG1_TRG1_EDGE_POS_NEG;
	else
		trig_config |= AW869X_BIT_TRGCFG1_TRG1_EDGE_POS;
	if (aw_haptic->trig[1].dual_edge)
		trig_config |= AW869X_BIT_TRGCFG1_TRG2_EDGE_POS_NEG;
	else
		trig_config |= AW869X_BIT_TRGCFG1_TRG2_EDGE_POS;
	if (aw_haptic->trig[2].dual_edge)
		trig_config |= AW869X_BIT_TRGCFG1_TRG3_EDGE_POS_NEG;
	else
		trig_config |= AW869X_BIT_TRGCFG1_TRG3_EDGE_POS;
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_TRG_CFG1,
				 (AW869X_BIT_TRGCFG1_TRG1_POLAR_MASK &
				  AW869X_BIT_TRGCFG1_TRG2_POLAR_MASK &
				  AW869X_BIT_TRGCFG1_TRG3_POLAR_MASK &
				  AW869X_BIT_TRGCFG1_TRG1_EDGE_MASK &
				  AW869X_BIT_TRGCFG1_TRG2_EDGE_MASK &
				  AW869X_BIT_TRGCFG1_TRG3_EDGE_MASK),
				  trig_config);

	trig_seq[0] = aw_haptic->trig[0].frist_seq;
	trig_seq[1] = aw_haptic->trig[1].frist_seq;
	trig_seq[2] = aw_haptic->trig[2].frist_seq;
	trig_seq[3] = aw_haptic->trig[0].second_seq;
	trig_seq[4] = aw_haptic->trig[1].second_seq;
	trig_seq[5] = aw_haptic->trig[2].second_seq;
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_TRG1_WAV_P, trig_seq,
			     AW_I2C_BYTE_SIX);
}

static void aw869x_trig_enable_config(struct aw_haptic *aw_haptic)
{
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_TRG_CFG2,
				 (AW869X_BIT_TRGCFG2_TRG1_ENABLE_MASK &
				  AW869X_BIT_TRGCFG2_TRG2_ENABLE_MASK &
				  AW869X_BIT_TRGCFG2_TRG3_ENABLE_MASK),
				 ((aw_haptic->trig[0].enable << 0) |
				  (aw_haptic->trig[1].enable << 1) |
				  (aw_haptic->trig[2].enable << 2)));
}

static void aw869x_auto_bst_enable(struct aw_haptic *aw_haptic, uint8_t flag)
{
	aw_haptic->auto_boost = flag;
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
				       AW869X_BIT_BST_AUTO_BST_AUTOSW_MASK,
				       AW869X_BIT_BST_AUTO_BST_AUTOMATIC_BOOST);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BST_AUTO,
					 AW869X_BIT_BST_AUTO_BST_AUTOSW_MASK,
					 AW869X_BIT_BST_AUTO_BST_MANUAL_BOOST);
	}
}

static uint8_t aw869x_get_trim_lra(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_TRIM_LRA, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= 0x3F;
	return reg_val;
}

/*****************************************************
 *
 * vbat mode
 *
 *****************************************************/
static void aw869x_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_ADCTEST,
					 AW869X_BIT_ADCTEST_VBAT_MODE_MASK,
					 AW869X_BIT_ADCTEST_VBAT_HW_COMP);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_ADCTEST,
					 AW869X_BIT_ADCTEST_VBAT_MODE_MASK,
					 AW869X_BIT_ADCTEST_VBAT_SW_COMP);
	}
}

static void aw869x_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t cont = 2000;

	aw869x_stop(aw_haptic);
	aw869x_ram_init(aw_haptic, true);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_DETCTRL,
				 AW869X_BIT_DETCTRL_VBAT_GO_MASK,
				 AW869X_BIT_DETCTRL_VABT_GO_ENABLE);

	while (1) {
		haptic_hv_i2c_reads(aw_haptic, AW869X_REG_DETCTRL, &reg_val,
				    AW_I2C_BYTE_ONE);
		if ((reg_val & 0x02) == 0 || cont == 0)
			break;
		cont--;
	}

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_VBATDET, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->vbat = AW869X_VBAT_FORMULA(reg_val);
	if (aw_haptic->vbat > AW_VBAT_MAX) {
		aw_haptic->vbat = AW_VBAT_MAX;
		aw_dbg("vbat max limit = %d", aw_haptic->vbat);
	}
	if (aw_haptic->vbat < AW_VBAT_MIN) {
		aw_haptic->vbat = AW_VBAT_MIN;
		aw_dbg("vbat min limit = %d", aw_haptic->vbat);
	}
	aw_info("aw_haptic->vbat=%dmV", aw_haptic->vbat);
	aw869x_ram_init(aw_haptic, false);
}

static ssize_t aw869x_get_reg(struct aw_haptic *aw_haptic, ssize_t len,
			      char *buf)
{
	uint8_t i = 0;
	uint8_t reg_array[AW869X_REG_NUM_F0_3 + 1] = {0};

	for (i = 0; i < AW869X_REG_RTP_DATA; i++)
		haptic_hv_i2c_reads(aw_haptic, AW869X_REG_ID, reg_array,
				    AW869X_REG_RTP_DATA);
	for (i = AW869X_REG_RTP_DATA + 1; i < AW869X_REG_RAMDATA; i++)
		haptic_hv_i2c_reads(aw_haptic, (AW869X_REG_RTP_DATA + 1),
				&reg_array[AW869X_REG_RTP_DATA + 1],
				(AW869X_REG_RAMDATA - AW869X_REG_RTP_DATA - 1));
	for (i = AW869X_REG_RAMDATA + 1; i <= AW869X_REG_NUM_F0_3; i++)
		haptic_hv_i2c_reads(aw_haptic, (AW869X_REG_RAMDATA + 1),
				&reg_array[AW869X_REG_RAMDATA + 1],
				(AW869X_REG_NUM_F0_3 - AW869X_REG_RAMDATA - 1));
	for (i = 0; i <= AW869X_REG_NUM_F0_3; i++)
		if ((i != AW869X_REG_RTP_DATA) && (i != AW869X_REG_RAMDATA))
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02X=0x%02X\n", i, reg_array[i]);
	return len;
}

static void aw869x_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	aw869x_stop(aw_haptic);
	aw869x_ram_init(aw_haptic, true);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
				 AW869X_BIT_SYSCTRL_BST_MODE_MASK,
				 AW869X_BIT_SYSCTRL_BST_MODE_BYPASS);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_ANACTRL,
				 AW869X_BIT_ANACTRL_HD_PD_MASK,
				 AW869X_BIT_ANACTRL_HD_HZ_EN);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_D2SCFG,
				 AW869X_BIT_D2SCFG_CLK_ADC_MASK,
				 AW869X_BIT_D2SCFG_CLK_ASC_1P5MHZ);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_DETCTRL,
				 (AW869X_BIT_DETCTRL_RL_OS_MASK &
				  AW869X_BIT_DETCTRL_DIAG_GO_MASK),
				 (AW869X_BIT_DETCTRL_RL_DETECT |
				  AW869X_BIT_DETCTRL_DIAG_GO_ENABLE));
	usleep_range(3000, 3500);
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_RLDET, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->lra = AW869X_LRA_FORMULA(reg_val);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_ANACTRL,
				 AW869X_BIT_ANACTRL_HD_PD_MASK,
				 AW869X_BIT_ANACTRL_HD_PD_EN);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_D2SCFG,
				 AW869X_BIT_D2SCFG_CLK_ADC_MASK,
				 AW869X_BIT_D2SCFG_CLK_ASC_6MHZ);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSCTRL,
				 AW869X_BIT_SYSCTRL_BST_MODE_MASK,
				 AW869X_BIT_SYSCTRL_BST_MODE_BOOST);
	aw869x_ram_init(aw_haptic, false);
}

/*****************************************************
 *
 * f0
 *
 *****************************************************/
static int aw869x_read_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint32_t f0_reg = 0;

#ifdef AW_LRA_F0_DEFAULT
	/* lra_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_F_LRA_F0_H, reg_val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | (reg_val[1] << 0);
	if (!(f0_reg * aw_haptic->info.f0_coeff)) {
		aw_haptic->f0 = 0;
		aw_err("lra_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	aw_haptic->f0 = (uint32_t)AW869X_F0_FARMULA(f0_reg,
						    aw_haptic->info.f0_coeff);
	aw_info("lra_f0=%d", aw_haptic->f0);
#else
	/* cont_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_F_LRA_CONT_H, reg_val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | (reg_val[1] << 0);
	if (!(f0_reg * aw_haptic->info.f0_coeff)) {
		aw_haptic->f0 = 0;
		aw_err("cont_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	aw_haptic->f0 = (uint32_t)AW869X_F0_FARMULA(f0_reg,
						    aw_haptic->info.f0_coeff);
	aw_info("cont_f0=%d", aw_haptic->f0);
#endif
	return 0;
}

static void aw869x_read_beme(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	ret = haptic_hv_i2c_reads(aw_haptic, AW869X_REG_WAIT_VOL_MP, &reg_val,
				  AW_I2C_BYTE_ONE);
	aw_haptic->max_pos_beme = (reg_val << 0);
	ret = haptic_hv_i2c_reads(aw_haptic, AW869X_REG_WAIT_VOL_MN, &reg_val,
				  AW_I2C_BYTE_ONE);
	aw_haptic->max_neg_beme = (reg_val << 0);
	aw_info("max_pos_beme=%d", aw_haptic->max_pos_beme);
	aw_info("max_neg_beme=%d", aw_haptic->max_neg_beme);
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static void aw869x_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSINTM,
					 AW869X_BIT_SYSINTM_FF_AE_MASK,
					 AW869X_BIT_SYSINTM_FF_AE_EN);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSINTM,
					 AW869X_BIT_SYSINTM_FF_AE_MASK,
					 AW869X_BIT_SYSINTM_FF_AE_OFF);
	}
}

static uint8_t aw869x_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW869X_BIT_SYSST_FF_AFS;
	reg_val >>= 3;
	return reg_val;
}

static uint8_t aw869x_rtp_get_fifo_aes(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW869X_BIT_SYSST_FF_AES;
	reg_val >>= 4;
	return reg_val;
}

static void aw869x_upload_lra(struct aw_haptic *aw_haptic, uint32_t flag)
{
	uint8_t zero = 0x00;
	uint8_t f0 = 0;
	uint8_t osc = 0;

	switch (flag) {
	case AW_WRITE_ZERO:
		aw_info("write zero to trim_lra!");
		haptic_hv_i2c_writes(aw_haptic, AW869X_REG_TRIM_LRA, &zero,
				     AW_I2C_BYTE_ONE);
		break;
	case AW_F0_CALI_LRA:
		aw_info("f0_cali_lra=%d", aw_haptic->f0_cali_data);
		f0 = (uint8_t)aw_haptic->f0_cali_data;
		haptic_hv_i2c_writes(aw_haptic, AW869X_REG_TRIM_LRA, &f0,
				     AW_I2C_BYTE_ONE);
		break;
	case AW_OSC_CALI_LRA:
		aw_info("rtp_cali_lra=%d", aw_haptic->osc_cali_data);
		osc = (uint8_t)aw_haptic->osc_cali_data;
		haptic_hv_i2c_writes(aw_haptic, AW869X_REG_TRIM_LRA, &osc,
				     AW_I2C_BYTE_ONE);
		break;
	default:
		break;
	}
}

static uint64_t  aw869x_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	uint64_t  theory_time = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_PWMDBG, &reg_val,
			    AW_I2C_BYTE_ONE);
	fre_val = (reg_val & 0x006f) >> 5;
	if (fre_val == 3)
		theory_time = (aw_haptic->rtp_len / 12000) * 1000000; /*12K */
	if (fre_val == 2)
		theory_time = (aw_haptic->rtp_len / 24000) * 1000000; /*24K */
	if (fre_val == 1 || fre_val == 0)
		theory_time = (aw_haptic->rtp_len / 48000) * 1000000; /*48K */

	aw_info("microsecond:%llu theory_time = %llu",
		aw_haptic->microsecond, theory_time);

	return theory_time;
}

static uint8_t aw869x_osc_read_status(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_DBGSTAT, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW869X_BIT_DBGSTAT_FF_EMPTY;
	return reg_val;
}

/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
static void aw869x_cont_config(struct aw_haptic *aw_haptic)
{
	uint8_t time_nzc = 0x23;
	uint8_t cont_td[2] = {0};
	uint8_t cont_zc[2] = {0};
	uint8_t drv_lvl[2] = {0};

	/* work mode */
	aw869x_play_mode(aw_haptic, AW_CONT_MODE);
	/* lpf */
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_DATCTRL,
				 (AW869X_BIT_DATCTRL_FC_MASK &
				  AW869X_BIT_DATCTRL_LPF_ENABLE_MASK),
				 (AW869X_BIT_DATCTRL_FC_1000HZ |
				  AW869X_BIT_DATCTRL_LPF_ENABLE));
	/* cont config */
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_CONT_CTRL,
				 (AW869X_BIT_CONT_CTRL_ZC_DETEC_MASK &
				  AW869X_BIT_CONT_CTRL_WAIT_PERIOD_MASK &
				  AW869X_BIT_CONT_CTRL_MODE_MASK &
				  AW869X_BIT_CONT_CTRL_EN_CLOSE_MASK &
				  AW869X_BIT_CONT_CTRL_F0_DETECT_MASK &
				  AW869X_BIT_CONT_CTRL_O2C_MASK &
				  AW869X_BIT_CONT_CTRL_AUTO_BRK_MASK),
				 (AW869X_BIT_CONT_CTRL_ZC_DETEC_ENABLE |
				  AW869X_BIT_CONT_CTRL_WAIT_1PERIOD |
				  AW869X_BIT_CONT_CTRL_BY_GO_SIGNAL |
				  AW869X_BIT_CONT_CTRL_CLOSE_PLAYBACK |
				  AW869X_BIT_CONT_CTRL_F0_DETECT_DISABLE |
				  AW869X_BIT_CONT_CTRL_O2C_DISABLE |
				  AW869X_BIT_CONT_CTRL_AUTO_BRK_ENABLE));
	/* TD time */
	cont_td[0] = (uint8_t)(aw_haptic->info.cont_td >> 8); /* cont_td_h */
	cont_td[1] = (uint8_t)(aw_haptic->info.cont_td >> 0); /* cont_td_l */
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_TD_H, cont_td,
			     AW_I2C_BYTE_TWO);
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_TSET, &aw_haptic->info.tset,
			     AW_I2C_BYTE_ONE);
	/* zero cross */
	cont_zc[0] = (uint8_t)(aw_haptic->info.cont_zc_thr >> 8);/* cont_zc_h */
	cont_zc[1] = (uint8_t)(aw_haptic->info.cont_zc_thr >> 0);/* cont_zc_l */
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_ZC_THRSH_H, cont_zc,
			     AW_I2C_BYTE_TWO);

	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BEMF_NUM,
				 AW869X_BIT_BEMF_NUM_BRK_MASK,
				 aw_haptic->info.cont_num_brk);
	/* 35*171us=5.985ms */
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_TIME_NZC, &time_nzc,
			     AW_I2C_BYTE_ONE);
	/* f0 driver level */
	drv_lvl[0] = aw_haptic->info.cont_drv_lvl;
	drv_lvl[1] = aw_haptic->info.cont_drv_lvl_ov;
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_DRV_LVL, drv_lvl,
			     AW_I2C_BYTE_TWO);
	/* cont play go */
	aw869x_play_go(aw_haptic, true);
}

/*****************************************************
 *
 * haptic f0 cali
 *
 *****************************************************/
static int aw869x_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t f0_param[5] = {0};
	uint32_t t_f0_ms = 0;
	uint32_t t_f0_trace_ms = 0;
	int ret = 0;

	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* f0 calibrate work mode */
	aw869x_stop(aw_haptic);
	aw869x_play_mode(aw_haptic, AW_CONT_MODE);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_CONT_CTRL,
				 (AW869X_BIT_CONT_CTRL_EN_CLOSE_MASK &
				  AW869X_BIT_CONT_CTRL_F0_DETECT_MASK),
				 (AW869X_BIT_CONT_CTRL_OPEN_PLAYBACK |
				  AW869X_BIT_CONT_CTRL_F0_DETECT_ENABLE));
	/* LPF */
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_DATCTRL,
				 (AW869X_BIT_DATCTRL_FC_MASK &
				  AW869X_BIT_DATCTRL_LPF_ENABLE_MASK),
				 (AW869X_BIT_DATCTRL_FC_1000HZ |
				  AW869X_BIT_DATCTRL_LPF_ENABLE));
	/* f0 driver level */
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_DRV_LVL,
			     &aw_haptic->info.cont_drv_lvl, AW_I2C_BYTE_ONE);
	/* f0 trace parameter */
	f0_param[1] = aw_haptic->info.f0_trace_parameter[2]; /* f0_repeat_num */
	f0_param[2] = aw_haptic->info.f0_trace_parameter[3]; /* f0_trace_num */
	f0_param[3] = aw_haptic->info.f0_trace_parameter[0]; /* f0_pre_num */
	f0_param[4] = aw_haptic->info.f0_trace_parameter[1]; /* f0_wait_num */
	f0_param[0] = (f0_param[3] << 4) | (f0_param[4] << 0); /* f0_1 */
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_NUM_F0_1, f0_param,
			     AW_I2C_BYTE_THREE);
	/* clear aw_haptic interrupt */
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	/* play go */
	aw869x_play_go(aw_haptic, true);
	/* f0 trace time */
	if (!aw_haptic->pre_f0) {
		aw_err("aw_haptic->pre_f0=0!!");
		return -ERANGE;
	}
	t_f0_ms = 1000 * 10 / aw_haptic->pre_f0;
	t_f0_trace_ms = t_f0_ms * (f0_param[3] + f0_param[4] +
		(f0_param[2] + f0_param[4]) * (f0_param[1] - 1));
	usleep_range(t_f0_trace_ms * 1000, t_f0_trace_ms * 1000 + 500);
	aw869x_wait_enter_standby(aw_haptic);
	ret = aw869x_read_f0(aw_haptic);
	aw869x_read_beme(aw_haptic);
	/* restore default config */
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_CONT_CTRL,
				 (AW869X_BIT_CONT_CTRL_EN_CLOSE_MASK &
				  AW869X_BIT_CONT_CTRL_F0_DETECT_MASK),
				 (AW869X_BIT_CONT_CTRL_CLOSE_PLAYBACK |
				  AW869X_BIT_CONT_CTRL_F0_DETECT_DISABLE));
	return ret;
}

static int aw869x_ram_get_f0(struct aw_haptic *aw_haptic)
{
	aw_err("aw869x no support ram get f0");
	return -ERANGE;
}

static int aw869x_set_pref0(struct aw_haptic *aw_haptic, uint32_t f0_pre)
{
	uint8_t reg_val[2] = {0};
	uint32_t f0_reg = 0;

	if (!(f0_pre && aw_haptic->info.f0_coeff)) {
		aw_err("Divisor is 0! f0_pre=%d, f0_coeff=%d",
			f0_pre, aw_haptic->info.f0_coeff);
		return -EPERM;
	}
	f0_reg = 1000000000 / (f0_pre * aw_haptic->info.f0_coeff);
	reg_val[0] = (uint8_t)((f0_reg >> 8) & 0xff);
	reg_val[1] = (uint8_t)((f0_reg >> 0) & 0xff);
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_F_PRE_H, reg_val,
			     AW_I2C_BYTE_TWO);
	return 0;
}

#ifdef AW869X_MUL_GET_F0
static int aw869x_multiple_get_f0(struct aw_haptic *aw_haptic)
{
	int f0_max = aw_haptic->info.f0_pre + AW869X_MUL_GET_F0_RANGE;
	int f0_min = aw_haptic->info.f0_pre - AW869X_MUL_GET_F0_RANGE;
	int i = 0;
	int ret = 0;

	aw_haptic->pre_f0 = aw_haptic->info.f0_pre;
	for (i = 0; i < AW869X_MUL_GET_F0_NUM; i++) {
		aw869x_set_pref0(aw_haptic, aw_haptic->pre_f0);
		ret = aw869x_get_f0(aw_haptic);
		if (ret)
			return ret;
		aw_haptic->pre_f0 = aw_haptic->f0;
		if (aw_haptic->f0 >= f0_max || aw_haptic->f0 <= f0_min)
			break;
		usleep_range(4000, 4500);
	}
	return 0;
}
#endif

static void aw869x_misc_para_init(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[8] = {0};

	/* Get vmax */
	if (aw_haptic->info.bst_vol_default > 0)
		aw_haptic->vmax = aw_haptic->info.bst_vol_default;
	/* Get gain */
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_DATDBG, reg_val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->gain = reg_val[0];
	/* Get wave_seq */
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_WAVSEQ1, reg_val,
			    AW_I2C_BYTE_EIGHT);
	aw_haptic->index = reg_val[0];
	memcpy(aw_haptic->seq, reg_val, AW_SEQUENCER_SIZE);
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_BSTDBG1,
			     aw_haptic->info.bstdbg, AW_I2C_BYTE_THREE);
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_TSET, &aw_haptic->info.tset,
			     AW_I2C_BYTE_ONE);
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_R_SPARE,
			     &aw_haptic->info.r_spare, AW_I2C_BYTE_ONE);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_ANADBG,
				 AW869X_BIT_ANADBG_IOC_MASK,
				 AW869X_BIT_ANADBG_IOC_4P65A);
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_ANACTRL,
				 AW869X_BIT_ANACTRL_LRA_SRC_MASK,
				 AW869X_BIT_ANACTRL_LRA_SRC_REG);
	/* beme config */
	haptic_hv_i2c_writes(aw_haptic, AW869X_REG_BEMF_VTHH_H,
			     aw_haptic->info.bemf_config, AW_I2C_BYTE_FOUR);
	aw_haptic->pre_f0 = aw_haptic->info.f0_pre;
	aw869x_set_pref0(aw_haptic, aw_haptic->info.f0_pre);
	aw869x_protect_config(aw_haptic, AW869X_BIT_PRTIME_DEFAULT_VALUE,
			      AW869X_BIT_PRLVL_PRLVL_DEFAULT_VALUE);
}

static void aw869x_trig_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter!");
	aw869x_trig_param_init(aw_haptic);
	aw869x_trig_param_config(aw_haptic);
	aw869x_trig_enable_config(aw_haptic);
}

static void aw869x_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%x", reg_val);
	/* edge int mode */
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_DBGCTRL,
				 AW869X_BIT_DBGCTRL_INT_MODE_MASK,
				 AW869X_BIT_DBGCTRL_INT_MODE_EDGE);
	/* int enable */
	haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_SYSINTM,
				 (AW869X_BIT_SYSINTM_BSTERR_MASK &
				  AW869X_BIT_SYSINTM_OV_MASK &
				  AW869X_BIT_SYSINTM_UVLO_MASK &
				  AW869X_BIT_SYSINTM_OCD_MASK &
				  AW869X_BIT_SYSINTM_OT_MASK),
				 (AW869X_BIT_SYSINTM_BSTERR_OFF |
				  AW869X_BIT_SYSINTM_OV_OFF |
				  AW869X_BIT_SYSINTM_UVLO_EN |
				  AW869X_BIT_SYSINTM_OCD_EN |
				  AW869X_BIT_SYSINTM_OT_EN));
}

static int aw869x_get_irq_state(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t dbg_val = 0;
	uint8_t int_sts = 0;
	int ret = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_SYSST, &int_sts,
			    AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSST=0x%x", int_sts);
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSINT=0x%x", reg_val);
	haptic_hv_i2c_reads(aw_haptic, AW869X_REG_DBGSTAT, &dbg_val,
			    AW_I2C_BYTE_ONE);
	aw_dbg("reg DBGSTAT=0x%x", dbg_val);
	if (reg_val & AW869X_BIT_SYSINT_OVI) {
		ret = AW_IRQ_OV;
		aw_err("chip ov int error");
	}
	if (reg_val & AW869X_BIT_SYSINT_UVLI & int_sts) {
		ret = AW_IRQ_UVLO;
		aw_err("chip uvlo int error");
	}
	if (reg_val & AW869X_BIT_SYSINT_OCDI) {
		ret = AW_IRQ_OCD;
		aw_err("chip over current int error");
	}
	if (reg_val & AW869X_BIT_SYSINT_OTI) {
		ret = AW_IRQ_OT;
		aw_err("chip over temperature int error");
	}
	if (reg_val & AW869X_BIT_SYSINT_DONEI) {
		ret = AW_IRQ_DONE;
		aw_info("chip playback done");
	}
	if (reg_val & AW869X_BIT_SYSINT_FF_AFI) {
		ret = AW_IRQ_ALMOST_FULL;
		aw_info("aw_haptic rtp mode fifo full empty");
	}
	if (reg_val & AW869X_BIT_SYSINT_FF_AEI)
		ret = AW_IRQ_ALMOST_EMPTY;
	return ret;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static void aw869x_parse_dt(struct device *dev, struct aw_haptic *aw_haptic,
			    struct device_node *np)
{
	uint8_t bstdbg[6];
	uint8_t f0_trace_parameter[4];
	uint8_t bemf_config[4];
	uint8_t duration_time[3];
	uint8_t trig_config_temp[15];
	uint32_t val = 0;

	val = of_property_read_u32(np, "f0_pre", &aw_haptic->info.f0_pre);
	if (val != 0)
		aw_info("f0_pre not found");
	val = of_property_read_u8(np, "f0_cali_percent",
				  &aw_haptic->info.f0_cali_percent);
	if (val != 0)
		aw_info("f0_cali_percent not found");
	val = of_property_read_u8(np, "aw869x_cont_drv_lev",
				  &aw_haptic->info.cont_drv_lvl);
	if (val != 0)
		aw_info("aw869x_cont_drv_lev not found");
	val = of_property_read_u8(np, "aw869x_cont_drv_lvl_ov",
				  &aw_haptic->info.cont_drv_lvl_ov);
	if (val != 0)
		aw_info("aw869x_cont_drv_lvl_ov not found");
	val = of_property_read_u32(np, "aw869x_cont_td",
				   &aw_haptic->info.cont_td);
	if (val != 0)
		aw_info("aw869x_cont_td not found");
	val = of_property_read_u32(np, "aw869x_cont_zc_thr",
				   &aw_haptic->info.cont_zc_thr);
	if (val != 0)
		aw_info("aw869x_cont_zc_thr not found");
	val = of_property_read_u8(np, "aw869x_cont_num_brk",
				  &aw_haptic->info.cont_num_brk);
	if (val != 0)
		aw_info("aw869x_cont_num_brk not found");
	val = of_property_read_u32(np, "aw869x_f0_coeff",
				   &aw_haptic->info.f0_coeff);
	if (val != 0)
		aw_info("aw869x_f0_coeff not found");
	val = of_property_read_u8_array(np, "aw869x_duration_time",
					duration_time,
					ARRAY_SIZE(duration_time));
	if (val != 0)
		aw_info("aw869x_duration_time not found");
	else
		memcpy(aw_haptic->info.duration_time, duration_time,
		       sizeof(duration_time));
	val = of_property_read_u8(np, "aw869x_tset", &aw_haptic->info.tset);
	if (val != 0)
		aw_info("aw869x_tset not found");
	val = of_property_read_u8(np, "aw869x_r_spare",
				  &aw_haptic->info.r_spare);
	if (val != 0)
		aw_info("aw869x_r_spare not found");
	val = of_property_read_u8_array(np, "aw869x_bstdbg", bstdbg,
					ARRAY_SIZE(bstdbg));
	if (val != 0)
		aw_info("aw869x_bstdbg not found");
	else
		memcpy(aw_haptic->info.bstdbg, bstdbg, sizeof(bstdbg));
	val = of_property_read_u8_array(np, "aw869x_f0_trace_parameter",
					f0_trace_parameter,
					ARRAY_SIZE(f0_trace_parameter));
	if (val != 0)
		aw_info("aw869x_f0_trace_parameter not found");
	else
		memcpy(aw_haptic->info.f0_trace_parameter, f0_trace_parameter,
		       sizeof(f0_trace_parameter));
	val = of_property_read_u8_array(np, "aw869x_bemf_config", bemf_config,
					ARRAY_SIZE(bemf_config));
	if (val != 0)
		aw_info("aw869x_bemf_config not found");
	else
		memcpy(aw_haptic->info.bemf_config, bemf_config,
		       sizeof(bemf_config));
	val = of_property_read_u8_array(np, "aw869x_trig_config",
					trig_config_temp,
					ARRAY_SIZE(trig_config_temp));
	if (val != 0)
		aw_info("vib_trig_config not found");
	else
		memcpy(aw_haptic->info.trig_cfg, trig_config_temp,
		       sizeof(trig_config_temp));
	val = of_property_read_u32(np, "aw869x_bst_vol_default",
				   &aw_haptic->info.bst_vol_default);
	if (val != 0)
		aw_info("aw869x_bst_vol_default not found");
}

static ssize_t aw869x_cont_td_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw869x cont delay time = 0x%04x\n",
			aw_haptic->info.cont_td);
	return len;
}

static ssize_t aw869x_cont_td_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	uint8_t cont_td[2] = {0};
	uint32_t databuf[1] = { 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw_haptic->info.cont_td = databuf[0];
		cont_td[0] = (uint8_t)(databuf[0] >> 8);
		cont_td[1] = (uint8_t)(databuf[0] >> 0);
		haptic_hv_i2c_writes(aw_haptic, AW869X_REG_TD_H, cont_td,
				     AW_I2C_BYTE_TWO);
	}
	return count;
}

static ssize_t aw869x_cont_drv_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw869x cont drv level = %d\n",
			aw_haptic->info.cont_drv_lvl);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw869x cont drv level overdrive= %d\n",
			aw_haptic->info.cont_drv_lvl_ov);
	return len;
}

static ssize_t aw869x_cont_drv_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	uint8_t drv_lvl[2] = {0};
	uint32_t databuf[2] = { 0, 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv_lvl = (uint8_t)databuf[0];
		aw_haptic->info.cont_drv_lvl_ov = (uint8_t)databuf[1];
		drv_lvl[0] = (uint8_t)databuf[0];
		drv_lvl[1] = (uint8_t)databuf[1];
		haptic_hv_i2c_writes(aw_haptic, AW869X_REG_DRV_LVL, drv_lvl,
				     AW_I2C_BYTE_TWO);
	}
	return count;
}

static ssize_t aw869x_cont_num_brk_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw869x cont break num = %d\n",
			aw_haptic->info.cont_num_brk);
	return len;
}

static ssize_t aw869x_cont_num_brk_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	uint32_t databuf[1] = { 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		aw_haptic->info.cont_num_brk = databuf[0];
		if (aw_haptic->info.cont_num_brk > 7)
			aw_haptic->info.cont_num_brk = 7;
		haptic_hv_i2c_write_bits(aw_haptic, AW869X_REG_BEMF_NUM,
					 AW869X_BIT_BEMF_NUM_BRK_MASK,
					 aw_haptic->info.cont_num_brk);
	}
	return count;
}

static ssize_t aw869x_cont_zc_thr_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw869x cont zero cross thr = 0x%04x\n",
			aw_haptic->info.cont_zc_thr);
	return len;
}

static ssize_t aw869x_cont_zc_thr_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	uint8_t cont_zc[2] = {0};
	uint32_t databuf[1] = { 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw_haptic->info.cont_zc_thr = databuf[0];
		cont_zc[0] = (uint8_t)(databuf[0] >> 8);
		cont_zc[1] = (uint8_t)(databuf[0] >> 0);
		haptic_hv_i2c_writes(aw_haptic, AW869X_REG_ZC_THRSH_H, cont_zc,
				     AW_I2C_BYTE_TWO);
	}
	return count;
}

static ssize_t aw869x_trig_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	for (i = 0; i < AW_TRIG_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: enable=%d, default_level=%d, dual_edge=%d, frist_seq=%d, second_seq=%d\n",
				i + 1, aw_haptic->trig[i].enable,
				aw_haptic->trig[i].default_level,
				aw_haptic->trig[i].dual_edge,
				aw_haptic->trig[i].frist_seq,
				aw_haptic->trig[i].second_seq);
	}

	return len;
}

static ssize_t aw869x_trig_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	uint32_t databuf[6] = { 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%d %d %d %d %d %d",
		&databuf[0], &databuf[1], &databuf[2], &databuf[3],
		&databuf[4], &databuf[5]) == 6) {
		aw_info("%d, %d, %d, %d, %d, %d", databuf[0],
			databuf[1], databuf[2], databuf[3], databuf[4],
			databuf[5]);
		if (databuf[0] > 3)
			databuf[0] = 3;
		if (databuf[0] > 0)
			databuf[0] -= 1;
		if (!aw_haptic->ram_init) {
			aw_err("ram init failed, not allow to play!");
			return count;
		}
		if (databuf[4] > aw_haptic->ram.ram_num ||
			databuf[5] > aw_haptic->ram.ram_num) {
			aw_err("input wave num out of range!");
			return count;
		}
		aw_haptic->trig[databuf[0]].enable = databuf[1];
		aw_haptic->trig[databuf[0]].default_level = databuf[2];
		aw_haptic->trig[databuf[0]].dual_edge = databuf[3];
		aw_haptic->trig[databuf[0]].frist_seq = databuf[4];
		aw_haptic->trig[databuf[0]].second_seq = databuf[5];
		mutex_lock(&aw_haptic->lock);
		aw869x_trig_param_config(aw_haptic);
		aw869x_trig_enable_config(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static DEVICE_ATTR(cont_td, S_IWUSR | S_IRUGO, aw869x_cont_td_show,
		   aw869x_cont_td_store);
static DEVICE_ATTR(cont_drv, S_IWUSR | S_IRUGO, aw869x_cont_drv_show,
		   aw869x_cont_drv_store);
static DEVICE_ATTR(cont_num_brk, S_IWUSR | S_IRUGO, aw869x_cont_num_brk_show,
		   aw869x_cont_num_brk_store);
static DEVICE_ATTR(cont_zc_thr, S_IWUSR | S_IRUGO, aw869x_cont_zc_thr_show,
		   aw869x_cont_zc_thr_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw869x_trig_show,
		   aw869x_trig_store);

static struct attribute *aw869x_vibrator_attributes[] = {
	&dev_attr_cont_td.attr,
	&dev_attr_cont_drv.attr,
	&dev_attr_cont_num_brk.attr,
	&dev_attr_cont_zc_thr.attr,
	&dev_attr_trig.attr,
	NULL
};

static struct attribute_group aw869x_vibrator_attribute_group = {
	.attrs = aw869x_vibrator_attributes
};

static int aw869x_creat_node(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &aw869x_vibrator_attribute_group);
	if (ret < 0)
		aw_err("error creating sysfs attr files");
	return ret;
}

struct aw_haptic_func aw869x_func_list = {
	.play_stop = aw869x_stop,
	.ram_init = aw869x_ram_init,
	.get_vbat = aw869x_get_vbat,
	.creat_node = aw869x_creat_node,
	.cont_config = aw869x_cont_config,
	.offset_cali = aw869x_offset_cali,
	.check_qualify = aw869x_check_qualify,
	.get_irq_state = aw869x_get_irq_state,
	.judge_rtp_going = aw869x_judge_rtp_going,
	.set_bst_peak_cur = aw869x_set_bst_peak_cur,
	.get_theory_time = aw869x_get_theory_time,
	.get_lra_resistance = aw869x_get_lra_resistance,
	.set_pwm = aw869x_set_pwm,
	.play_mode = aw869x_play_mode,
	.ram_get_f0 = aw869x_ram_get_f0,
	.set_bst_vol = aw869x_set_bst_vol,
	.interrupt_setup = aw869x_interrupt_setup,
	.set_repeat_seq = aw869x_set_repeat_seq,
	.auto_bst_enable = aw869x_auto_bst_enable,
	.vbat_mode_config = aw869x_vbat_mode_config,
	.set_wav_seq = aw869x_set_wav_seq,
	.set_wav_loop = aw869x_set_wav_loop,
	.set_ram_addr = aw869x_set_ram_addr,
	.set_rtp_data = aw869x_set_rtp_data,
	.container_update = aw869x_container_update,
	.protect_config = aw869x_protect_config,
	.parse_dt = aw869x_parse_dt,
	.trig_init = aw869x_trig_init,
	.irq_clear = aw869x_irq_clear,
	.get_wav_loop = aw869x_get_wav_loop,
	.play_go = aw869x_play_go,
	.misc_para_init = aw869x_misc_para_init,
	.set_rtp_aei = aw869x_set_rtp_aei,
	.set_gain = aw869x_set_gain,
	.upload_lra = aw869x_upload_lra,
	.bst_mode_config = aw869x_bst_mode_config,
	.get_reg = aw869x_get_reg,
	.get_prctmode = aw869x_get_prctmode,
	.get_trim_lra = aw869x_get_trim_lra,
	.get_ram_data = aw869x_get_ram_data,
	.get_first_wave_addr = aw869x_get_first_wave_addr,
	.get_glb_state = aw869x_get_glb_state,
	.get_osc_status = aw869x_osc_read_status,
	.rtp_get_fifo_afs = aw869x_rtp_get_fifo_afs,
	.rtp_get_fifo_aes = aw869x_rtp_get_fifo_aes,
	.get_wav_seq = aw869x_get_wav_seq,
#ifdef AW869X_MUL_GET_F0
	.get_f0 = aw869x_multiple_get_f0,
#else
	.get_f0 = aw869x_get_f0,
#endif
};
