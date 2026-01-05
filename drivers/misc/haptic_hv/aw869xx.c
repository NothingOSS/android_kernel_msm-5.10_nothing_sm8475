/*
 * File: aw869xx.c
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

#include "haptic_hv.h"
#include "haptic_hv_reg.h"

static uint8_t aw869xx_get_glb_state(struct aw_haptic *aw_haptic);
static void aw869xx_vbat_mode_config(struct aw_haptic *, uint8_t);

/******************************************************
 *
 * aw869xx codec
 *
 ******************************************************/
#ifdef AW_SND_SOC_CODEC

#ifdef KERNEL_OVER_4_19
static const struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_component,
	.aw_snd_soc_codec_get_drvdata = snd_soc_component_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_component_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_component,
	.aw_snd_soc_register_codec = snd_soc_register_component,
};
#else
static const struct aw_componet_codec_ops aw_componet_codec_ops = {
	.aw_snd_soc_kcontrol_codec = snd_soc_kcontrol_codec,
	.aw_snd_soc_codec_get_drvdata = snd_soc_codec_get_drvdata,
	.aw_snd_soc_add_codec_controls = snd_soc_add_codec_controls,
	.aw_snd_soc_unregister_codec = snd_soc_unregister_codec,
	.aw_snd_soc_register_codec = snd_soc_register_codec,
};
#endif

static aw_snd_soc_codec_t *aw_get_codec(struct snd_soc_dai *dai)
{
#ifdef KERNEL_OVER_4_19
	return dai->component;
#else
	return dai->codec;
#endif
}

/******************************************************
 *
 * Digital Audio Interface
 *
 ******************************************************/
static void aw869xx_i2s_enable(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_I2SCFG2,
					 AW869XX_BIT_I2SCFG2_I2S_EN_MASK,
					 AW869XX_BIT_I2SCFG2_I2S_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_I2SCFG2,
					 AW869XX_BIT_I2SCFG2_I2S_EN_MASK,
					 AW869XX_BIT_I2SCFG2_I2S_DISABLE);
	}
}

static int aw869xx_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("enter");
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mutex_lock(&aw_haptic->lock);
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
					 AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
					 AW869XX_BIT_SYSCTRL2_STANDBY_OFF);
		mutex_unlock(&aw_haptic->lock);
	}

	return 0;
}

static int aw869xx_set_fmt(struct snd_soc_dai *dai, uint32_t fmt)
{
	aw_info("fmt=0x%02X", fmt);
	/* supported mode: regular I2S, slave, or PDM */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) !=
			SND_SOC_DAIFMT_CBS_CFS) {
			aw_err("invalid codec master mode");
			return -EINVAL;
		}
		break;
	default:
		aw_err("unsupported DAI format %d",
		       fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}
	return 0;
}

static int aw869xx_set_dai_sysclk(struct snd_soc_dai *dai,
				  int clk_id, uint32_t freq, int dir)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("freq=%d", freq);

	aw_haptic->sysclk = freq;
	return 0;
}

static int aw869xx_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	uint8_t i2sfs_val = 0;
	uint8_t i2sbck_val = 0;
	uint8_t tmp_val = 0;
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_info("steam is capture");
		return 0;
	}

	aw_info("requested rate: %d, sample size: %d", params_rate(params),
		snd_pcm_format_width(params_format(params)));

	/* get rate param */
	aw_haptic->rate = params_rate(params);

	mutex_lock(&aw_haptic->lock);

	/* get bit width */
	aw_haptic->width = params_width(params);
	aw_info("width = %d", aw_haptic->width);
	i2sbck_val = params_width(params) * params_channels(params);
	/* match bit width */
	switch (aw_haptic->width) {
	case 16:
		i2sfs_val = AW869XX_BIT_I2SCFG1_I2SFS_16BIT;
		break;
	case 24:
		i2sfs_val = AW869XX_BIT_I2SCFG1_I2SFS_24BIT;
		i2sbck_val = 32 * params_channels(params);
		break;
	case 32:
		i2sfs_val = AW869XX_BIT_I2SCFG1_I2SFS_32BIT;
		break;
	default:
		i2sfs_val = AW869XX_BIT_I2SCFG1_I2SFS_16BIT;
		i2sbck_val = 16 * params_channels(params);
		aw_err("width [%d]can not support", aw_haptic->width);
		break;
	}
	aw_info("i2sfs_val=0x%02X", i2sfs_val);
	/* set width */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_I2SCFG1,
				 AW869XX_BIT_I2SCFG1_I2SFS_MASK, i2sfs_val);

	/* match bck mode */
	switch (i2sbck_val) {
	case 32:
		i2sbck_val = AW869XX_BIT_I2SCFG1_I2SBCK_32FS;
		break;
	case 48:
		i2sbck_val = AW869XX_BIT_I2SCFG1_I2SBCK_48FS;
		break;
	case 64:
		i2sbck_val = AW869XX_BIT_I2SCFG1_I2SBCK_64FS;
		break;
	default:
		aw_err("i2sbck [%d] can not support", i2sbck_val);
		i2sbck_val = AW869XX_BIT_I2SCFG1_I2SBCK_32FS;
		break;
	}
	/* set bck mode */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_I2SCFG1,
				 AW869XX_BIT_I2SCFG1_I2SBCK_MASK, i2sbck_val);

	/* check i2s cfg */
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_I2SCFG1, &tmp_val,
			    AW_I2C_BYTE_ONE);
	aw_info("i2scfg1=0x%02X", tmp_val);

	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int aw869xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	uint8_t reg_val = 0;
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		aw_info("steam is capture");
		return 0;
	}
	aw_info("mute state=%d", mute);
	if (mute) {
		mutex_lock(&aw_haptic->lock);
		aw869xx_i2s_enable(aw_haptic, false);
		mutex_unlock(&aw_haptic->lock);
	} else {
		mutex_lock(&aw_haptic->lock);
		aw869xx_i2s_enable(aw_haptic, true);
		usleep_range(2000, 3000);
		haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSST, &reg_val,
				    AW_I2C_BYTE_ONE);
		aw_info("sysst=0x%02X", reg_val);
		haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSST2, &reg_val,
				    AW_I2C_BYTE_ONE);
		aw_info("sysst2=0x%02X", reg_val);
		haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSER, &reg_val,
				    AW_I2C_BYTE_ONE);
		aw_info("syser=0x%02X", reg_val);
		reg_val = aw869xx_get_glb_state(aw_haptic);
		if (reg_val != 0x0a) {
			aw_err("i2s config err, glb_state=0x%02X", reg_val);
			aw869xx_i2s_enable(aw_haptic, false);
		} else {
			aw_info("i2s config pass, glb_state=0x%02X", reg_val);
		}
		mutex_unlock(&aw_haptic->lock);
	}
	return 0;
}

static void aw869xx_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("enter");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aw_haptic->rate = 0;
		mutex_lock(&aw_haptic->lock);
		/* aw869xx_i2s_enable(aw_haptic, false); */
		mutex_unlock(&aw_haptic->lock);
	}
}

static const struct snd_soc_dai_ops aw869xx_dai_ops = {
	.startup = aw869xx_startup,
	.set_fmt = aw869xx_set_fmt,
	.set_sysclk = aw869xx_set_dai_sysclk,
	.hw_params = aw869xx_hw_params,
	.mute_stream = aw869xx_mute,
	.shutdown = aw869xx_shutdown,
};

static struct snd_soc_dai_driver aw869xx_dai[] = {
	{
	 .name = "aw869xx-aif",
	 .id = 1,
	 .playback = {
		      .stream_name = "Speaker_Playback",
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = AW869XX_RATES,
		      .formats = AW869XX_FORMATS,
		      },
	 .capture = {
		     .stream_name = "Speaker_Capture",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = AW869XX_RATES,
		     .formats = AW869XX_FORMATS,
		     },
	 .ops = &aw869xx_dai_ops,
	 .symmetric_rates = 1,
	 },
};

/*****************************************************
 *
 * codec driver
 *
 *****************************************************/
static void aw869xx_add_codec_controls(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
}

static int aw869xx_codec_probe(aw_snd_soc_codec_t *codec)
{
	int ret = -1;
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("enter");

	aw_haptic->codec = codec;

	aw869xx_add_codec_controls(aw_haptic);

	aw_info("exit");

	ret = 0;
	return ret;
}

#ifdef KERNEL_OVER_4_19
static void aw869xx_codec_remove(struct snd_soc_component *component)
{
	/*
	 * struct aw_haptic *aw_haptic =
	 *     aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	 */
	aw_info("enter");
}
#else
static int aw869xx_codec_remove(aw_snd_soc_codec_t *codec)
{
	/*
	 * struct aw_haptic *aw_haptic =
	 *     aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	 */
	aw_info("enter");

	return 0;
}
#endif

static uint32_t aw869xx_codec_read(aw_snd_soc_codec_t *codec, uint32_t reg)
{
	uint8_t value = 0;
	int ret = -1;
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("enter");

	ret = haptic_hv_i2c_reads(aw_haptic, reg, &value, AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_err("read register failed");
		return ret;
	}
	return ret;
}

static int aw869xx_codec_write(aw_snd_soc_codec_t *codec,
			       uint32_t reg, uint32_t value)
{
	uint8_t val = (uint8_t)value;
	int ret = -1;
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	aw_info("enter ,reg:0x%02X = 0x%02X", reg, value);

	ret = haptic_hv_i2c_writes(aw_haptic, (uint8_t)reg, &val, AW_I2C_BYTE_ONE);

	return ret;
}

#ifdef KERNEL_OVER_4_19
static struct snd_soc_component_driver soc_codec_dev_aw869xx = {
	.probe = aw869xx_codec_probe,
	.remove = aw869xx_codec_remove,
	.read = aw869xx_codec_read,
	.write = aw869xx_codec_write,
};
#else
static struct snd_soc_codec_driver soc_codec_dev_aw869xx = {
	.probe = aw869xx_codec_probe,
	.remove = aw869xx_codec_remove,
	.read = aw869xx_codec_read,
	.write = aw869xx_codec_write,
	.reg_cache_size = AW_HAPTIC_REG_MAX,
	.reg_word_size = 2,
};
#endif

static int aw869xx_snd_soc_init(struct device *dev)
{
	int ret = 0;
	struct snd_soc_dai_driver *dai;

	dev_set_name(dev, "%s", AW_I2C_NAME);
	/* register codec */
	dai = devm_kzalloc(dev, sizeof(aw869xx_dai), GFP_KERNEL);
	if (!dai) {
		aw_err("failed to register aw869xx");
		return -ENOMEM;
	}
	memcpy(dai, aw869xx_dai, sizeof(aw869xx_dai));
	aw_info("dai->name(%s)", dai->name);

	ret =
	aw_componet_codec_ops.aw_snd_soc_register_codec(dev,
							&soc_codec_dev_aw869xx,
							dai,
							ARRAY_SIZE
							(aw869xx_dai));
	if (ret < 0) {
		aw_err("failed to register aw869xx: %d", ret);
		return ret;
	}
	return 0;
}

#endif

static int aw869xx_check_qualify(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = -1;

	aw_info("enter");
	/* chip qualify */
	ret = haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_RDATA_A, &reg_val,
				  AW_I2C_BYTE_ONE);
	if (ret < 0)
		return ret;
	if (!(reg_val & 0x80)) {
		aw_err("unqualified chip!");
		return -ERANGE;
	}
	return 0;
}

static void aw869xx_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
					 AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW869XX_BIT_SYSCTRL2_RATE_48K);
		break;
	case AW_PWM_24K:
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
					 AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW869XX_BIT_SYSCTRL2_RATE_24K);
		break;
	case AW_PWM_12K:
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
					 AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW869XX_BIT_SYSCTRL2_RATE_12K);
		break;
	default:
		break;
	}
}

static void aw869xx_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_PLAYCFG2, &gain,
			     AW_I2C_BYTE_ONE);
}

static void aw869xx_set_bst_peak_cur(struct aw_haptic *aw_haptic)
{
	switch (aw_haptic->bst_pc) {
	case AW_BST_PC_L1:
		aw_info("bst pc = L1");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_BSTCFG1,
					 AW869XX_BIT_BSTCFG1_BST_PC_MASK,
					 AW869XX_BIT_BSTCFG1_PEAKCUR_2P75A);
		return;
	case AW_BST_PC_L2:
		aw_info("bst pc = L2");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_BSTCFG1,
					 AW869XX_BIT_BSTCFG1_BST_PC_MASK,
					 AW869XX_BIT_BSTCFG1_PEAKCUR_4A);
		return;
	default:
		aw_info("bst pc = L1");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_BSTCFG1,
					 AW869XX_BIT_BSTCFG1_BST_PC_MASK,
					 AW869XX_BIT_BSTCFG1_PEAKCUR_2P75A);
		break;
	}
}

static void aw869xx_set_bst_vol(struct aw_haptic *aw_haptic, uint32_t bst_vol)
{
	uint8_t reg_val = 0;

	if (bst_vol < AW869XX_BST_VOL_MIN)
		bst_vol = AW869XX_BST_VOL_MIN;
	else if (bst_vol > AW869XX_BST_VOL_MAX)
		bst_vol = AW869XX_BST_VOL_MAX;
	reg_val = AW869XX_BST_VOL_FARMULA(bst_vol);
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG1,
				 AW869XX_BIT_PLAYCFG1_BST_VOUT_RDA_MASK,
				 reg_val);
}

static void aw869xx_set_wav_seq(struct aw_haptic *aw_haptic, uint8_t wav,
				uint8_t seq)
{
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_WAVCFG1 + wav, &seq,
			     AW_I2C_BYTE_ONE);
}

static void aw869xx_set_wav_loop(struct aw_haptic *aw_haptic, uint8_t wav,
				 uint8_t loop)
{
	uint8_t tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_WAVCFG9 + (wav / 2),
					AW869XX_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_WAVCFG9 + (wav / 2),
					 AW869XX_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
}

static void aw869xx_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data,
				 uint32_t len)
{
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_RTPDATA, data, len);
}

static void aw869xx_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSINTM,
					 AW869XX_BIT_SYSINTM_FF_AEM_MASK,
					 AW869XX_BIT_SYSINTM_FF_AEM_ON);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSINTM,
					 AW869XX_BIT_SYSINTM_FF_AEM_MASK,
					 AW869XX_BIT_SYSINTM_FF_AEM_OFF);
	}
}

static void aw869xx_set_ram_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ram_addr[2] = {0};

	ram_addr[0] = (uint8_t)AW869XX_RAM_ADDR_H(aw_haptic->ram.base_addr);
	ram_addr[1] = (uint8_t)AW869XX_RAM_ADDR_L(aw_haptic->ram.base_addr);
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_RAMADDRH, ram_addr,
			     AW_I2C_BYTE_TWO);
}

static void aw869xx_auto_break_mode(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
					 AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW869XX_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
					 AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW869XX_BIT_PLAYCFG3_BRK_DISABLE);
	}
}

static uint8_t aw869xx_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_GLBRD5, &state,
			    AW_I2C_BYTE_ONE);
	aw_dbg("glb state value is 0x%02X", state);
	return state;
}

static void aw869xx_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t delay_20us = AW869XX_BIT_START_DLY_20US;
	uint8_t delay_2p5ms = AW869XX_BIT_START_DLY_2P5MS;
	uint8_t go_on = AW869XX_BIT_PLAYCFG4_GO_ON;
	uint8_t stop_on = AW869XX_BIT_PLAYCFG4_STOP_ON;

	aw_info("enter, flag = %d", flag);
	if (flag) {
		if (aw_haptic->info.is_enabled_one_wire) {
			haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_GLBCFG2,
					     &delay_20us, AW_I2C_BYTE_ONE);
			haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_PLAYCFG4,
					     &go_on, AW_I2C_BYTE_ONE);
			usleep_range(1000, 1500);
			haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_GLBCFG2,
					     &delay_2p5ms, AW_I2C_BYTE_ONE);
		} else {
			haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_PLAYCFG4,
					     &go_on, AW_I2C_BYTE_ONE);
		}
	} else {
		haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_PLAYCFG4, &stop_on,
				     AW_I2C_BYTE_ONE);
	}
}

static void aw869xx_bst_mode_config(struct aw_haptic *aw_haptic,
				    uint8_t boost_mode)
{
	switch (boost_mode) {
	case AW_BST_BOOST_MODE:
		aw_info("haptic boost mode = boost");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG1,
					 AW869XX_BIT_PLAYCFG1_BST_MODE_MASK,
					 AW869XX_BIT_PLAYCFG1_BST_MODE_BOOST);
		break;
	case AW_BST_BYPASS_MODE:
		aw_info("haptic boost mode = bypass");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG1,
					 AW869XX_BIT_PLAYCFG1_BST_MODE_MASK,
					 AW869XX_BIT_PLAYCFG1_BST_MODE_BYPASS);
		break;
	default:
		aw_err("boost_mode = %d error", boost_mode);
		break;
	}
}

static int aw869xx_wait_enter_standby(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int count = 100;

	while (count--) {
		reg_val = aw869xx_get_glb_state(aw_haptic);
		if ((reg_val & 0x0f) == AW869XX_BIT_GLBRD5_STATE_STANDBY ||
		    (reg_val & 0x0f) == AW869XX_BIT_GLBRD5_STATE_I2S_GO) {
			aw_info("entered standby!");
			return 0;
		}
		aw_dbg("wait for standby");
		usleep_range(2000, 2500);
	}
	aw_err("do not enter standby automatically");
	return -ERANGE;
}

static void aw869xx_stop(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	aw_haptic->play_mode = AW_STANDBY_MODE;
	reg_val = AW869XX_BIT_PLAYCFG4_STOP_ON;
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_PLAYCFG4, &reg_val,
			     AW_I2C_BYTE_ONE);
	ret = aw869xx_wait_enter_standby(aw_haptic);
	if (ret < 0) {
		aw_err("force to enter standby mode!");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
					 AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
					 AW869XX_BIT_SYSCTRL2_STANDBY_ON);
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
					 AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
					 AW869XX_BIT_SYSCTRL2_STANDBY_OFF);
	}
}

static void aw869xx_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_info("enter standby mode");
		aw_haptic->play_mode = AW_STANDBY_MODE;
		aw869xx_stop(aw_haptic);
		break;
	case AW_RAM_MODE:
		aw_info("enter ram mode");
		aw_haptic->play_mode = AW_RAM_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw869xx_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw869xx_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RAM_LOOP_MODE:
		aw_info("enter ram loop mode");
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw869xx_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		aw869xx_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RTP_MODE:
		aw_info("enter rtp mode");
		aw_haptic->play_mode = AW_RTP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_RTP);
		/* bst mode */
		aw869xx_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw869xx_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_TRIG_MODE:
		aw_info("enter trig mode");
		aw_haptic->play_mode = AW_TRIG_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		aw869xx_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_CONT_MODE:
		aw_info("enter cont mode");
		aw_haptic->play_mode = AW_CONT_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW869XX_BIT_PLAYCFG3_PLAY_MODE_CONT);
		/* bst mode */
		aw869xx_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		aw869xx_vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
		break;
	default:
		aw_err("play mode %d error", play_mode);
		break;
	}
}

static void aw869xx_ram_init(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL1,
					 AW869XX_BIT_SYSCTRL1_RAMINIT_MASK,
					 AW869XX_BIT_SYSCTRL1_RAMINIT_ON);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL1,
					 AW869XX_BIT_SYSCTRL1_RAMINIT_MASK,
					 AW869XX_BIT_SYSCTRL1_RAMINIT_OFF);
	}
}

static void aw869xx_upload_lra(struct aw_haptic *aw_haptic, uint32_t flag)
{
	switch (flag) {
	case AW_WRITE_ZERO:
		aw_info("write zero to trim_lra!");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRIMCFG3,
					 AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
					 0x00);
		break;
	case AW_F0_CALI_LRA:
		aw_info("write f0_cali_data to trim_lra = 0x%02X",
			aw_haptic->f0_cali_data);
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRIMCFG3,
					 AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
					 (char)aw_haptic->f0_cali_data);
		break;
	case AW_OSC_CALI_LRA:
		aw_info("write osc_cali_data to trim_lra = 0x%02X",
			aw_haptic->osc_cali_data);
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRIMCFG3,
					 AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
					 (char)aw_haptic->osc_cali_data);
		break;
	default:
		break;
	}
}

static uint8_t aw869xx_get_trim_lra(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_TRIMCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= 0x3F;
	return reg_val;
}

static void aw869xx_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	uint8_t reg_val = 0;

	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		reg_val = AW869XX_BIT_START_DLY_250US;
		haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_GLBCFG2, &reg_val,
				     AW_I2C_BYTE_ONE);
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL1,
					 AW869XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
					 AW869XX_BIT_SYSCTRL1_VBAT_MODE_HW);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL1,
					 AW869XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
					 AW869XX_BIT_SYSCTRL1_VBAT_MODE_SW);
	}
}

static void aw869xx_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime,
				   uint8_t prlvl)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PWMCFG1,
				 AW869XX_BIT_PWMCFG1_PRC_EN_MASK,
				 AW869XX_BIT_PWMCFG1_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_info("enable protection mode");
		reg_val = AW869XX_BIT_PWMCFG3_PR_ENABLE |
			  (prlvl & (~AW869XX_BIT_PWMCFG3_PRLVL_MASK));
		haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_PWMCFG3, &reg_val,
				     AW_I2C_BYTE_ONE);
		haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_PWMCFG4, &prtime,
				     AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_info("disable protection mode");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PWMCFG3,
					 AW869XX_BIT_PWMCFG3_PR_EN_MASK,
					 AW869XX_BIT_PWMCFG3_PR_DISABLE);
	}
}

static void aw869xx_cont_config(struct aw_haptic *aw_haptic)
{
	/* uint8_t drv1_time = 0xFF; */
	uint8_t drv2_time = 0xFF;

	/* work mode */
	aw869xx_play_mode(aw_haptic, AW_CONT_MODE);
	/* cont config */
	/* haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG1,
	 *			    AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
	 *			    AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	 */

	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG6,
				 (AW869XX_BIT_CONTCFG6_TRACK_EN_MASK &
				  AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK),
				 ((aw_haptic->info.is_enabled_track_en << 7) |
				  aw_haptic->info.cont_drv1_lvl));

	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG7,
			     &aw_haptic->info.cont_drv2_lvl, AW_I2C_BYTE_ONE);
	/* DRV1_TIME */
	/* haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG8, &drv1_time,
	 *			AW_I2C_BYTE_ONE);
	 */
	/* DRV2_TIME */
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG9, &drv2_time,
			     AW_I2C_BYTE_ONE);
	/* cont play go */
	aw869xx_play_go(aw_haptic, true);
}

static void aw869xx_one_wire_init(struct aw_haptic *aw_haptic)
{
	uint8_t trig_prio = 0x6c;
	uint8_t delay_2p5ms = AW869XX_BIT_START_DLY_2P5MS;

	aw_info("enter");
	/*if enable one-wire, trig1 priority must be less than trig2 and trig3*/
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_GLBCFG4, &trig_prio,
			     AW_I2C_BYTE_ONE);
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_GLBCFG2, &delay_2p5ms,
			     AW_I2C_BYTE_ONE);
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG8,
				 AW869XX_BIT_TRGCFG8_TRG_ONEWIRE_MASK,
				 AW869XX_BIT_TRGCFG8_TRG_ONEWIRE_ENABLE);
}

static void aw869xx_i2s_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
				 AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
				 AW869XX_BIT_SYSCTRL2_I2S_PIN_I2S);
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_IOCFG1,
				 AW869XX_BIT_IOCFG1_IO_FAST_MASK,
				 AW869XX_BIT_IOCFG1_IIS_IO_FAST_ENABLE);
}

static void aw869xx_trig1_param_init(struct aw_haptic *aw_haptic)
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

static void aw869xx_trig2_param_init(struct aw_haptic *aw_haptic)
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

static void aw869xx_trig3_param_init(struct aw_haptic *aw_haptic)
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

static void aw869xx_trig1_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[0].trig_level)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_MODE_LEVEL;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_MODE_EDGE;
	if (aw_haptic->trig[0].trig_polar)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_POLAR_NEG;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_POLAR_POS;
	if (aw_haptic->trig[0].pos_enable) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG1,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG1,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[0].neg_enable) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG4,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG4,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[0].pos_sequence) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG1,
					 AW869XX_BIT_TRG_SEQ_MASK,
					 aw_haptic->trig[0].pos_sequence);
	}
	if (aw_haptic->trig[0].neg_sequence) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG4,
					 AW869XX_BIT_TRG_SEQ_MASK,
					 aw_haptic->trig[0].neg_sequence);
	}
	if (aw_haptic->trig[0].trig_brk)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[0].trig_bst)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_BST_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_BST_DISABLE;
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG7,
				 (AW869XX_BIT_TRGCFG7_TRG1_MODE_MASK &
				  AW869XX_BIT_TRGCFG7_TRG1_POLAR_MASK &
				  AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK &
				  AW869XX_BIT_TRGCFG7_TRG1_BST_MASK),
				 trig_config);
}

static void aw869xx_trig2_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[1].trig_level)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_MODE_LEVEL;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_MODE_EDGE;
	if (aw_haptic->trig[1].trig_polar)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_POLAR_NEG;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_POLAR_POS;
	if (aw_haptic->trig[1].pos_enable) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG2,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG2,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[1].neg_enable) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG5,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG5,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[1].pos_sequence) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG2,
					 AW869XX_BIT_TRG_SEQ_MASK,
					 aw_haptic->trig[1].pos_sequence);
	}
	if (aw_haptic->trig[1].neg_sequence) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG5,
					 AW869XX_BIT_TRG_SEQ_MASK,
					 aw_haptic->trig[1].neg_sequence);
	}
	if (aw_haptic->trig[1].trig_brk)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[1].trig_bst)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_BST_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_BST_DISABLE;
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG7,
				 (AW869XX_BIT_TRGCFG7_TRG2_MODE_MASK &
				  AW869XX_BIT_TRGCFG7_TRG2_POLAR_MASK &
				  AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK &
				  AW869XX_BIT_TRGCFG7_TRG2_BST_MASK),
				 trig_config);
}

static void aw869xx_trig3_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[2].trig_level)
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_MODE_LEVEL;
	else
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_MODE_EDGE;
	if (aw_haptic->trig[2].trig_polar)
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_POLAR_NEG;
	else
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_POLAR_POS;
	if (aw_haptic->trig[2].pos_enable) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG3,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG3,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[2].neg_enable) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG6,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG6,
					 AW869XX_BIT_TRG_ENABLE_MASK,
					 AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[2].pos_sequence) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG3,
					 AW869XX_BIT_TRG_SEQ_MASK,
					 aw_haptic->trig[2].pos_sequence);
	}
	if (aw_haptic->trig[2].neg_sequence) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG6,
					 AW869XX_BIT_TRG_SEQ_MASK,
					 aw_haptic->trig[2].neg_sequence);
	}
	if (aw_haptic->trig[2].trig_brk)
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[2].trig_bst)
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_BST_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_BST_DISABLE;
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_TRGCFG8,
				 (AW869XX_BIT_TRGCFG8_TRG3_MODE_MASK &
				  AW869XX_BIT_TRGCFG8_TRG3_POLAR_MASK &
				  AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK &
				  AW869XX_BIT_TRGCFG8_TRG3_BST_MASK),
				 trig_config);
}

static void aw869xx_auto_bst_enable(struct aw_haptic *aw_haptic, uint8_t flag)
{
	aw_haptic->auto_boost = flag;
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
					 AW869XX_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW869XX_BIT_PLAYCFG3_AUTO_BST_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
					 AW869XX_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW869XX_BIT_PLAYCFG3_AUTO_BST_DISABLE);
	}
}

static void aw869xx_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%02X", reg_val);
	/* edge int mode */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL7,
				 (AW869XX_BIT_SYSCTRL7_INT_MODE_MASK &
				  AW869XX_BIT_SYSCTRL7_INT_EDGE_MODE_MASK),
				 (AW869XX_BIT_SYSCTRL7_INT_MODE_EDGE |
				  AW869XX_BIT_SYSCTRL7_INT_EDGE_MODE_POS));
	/* int enable */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSINTM,
				 (AW869XX_BIT_SYSINTM_BST_SCPM_MASK &
				  AW869XX_BIT_SYSINTM_BST_OVPM_MASK &
				  AW869XX_BIT_SYSINTM_UVLM_MASK &
				  AW869XX_BIT_SYSINTM_OCDM_MASK &
				  AW869XX_BIT_SYSINTM_OTM_MASK),
				 (AW869XX_BIT_SYSINTM_BST_SCPM_ON |
				  AW869XX_BIT_SYSINTM_BST_OVPM_OFF |
				  AW869XX_BIT_SYSINTM_UVLM_ON |
				  AW869XX_BIT_SYSINTM_OCDM_ON |
				  AW869XX_BIT_SYSINTM_OTM_ON));
}

static int aw869xx_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state = 0;
	uint8_t rtp_state = 0;

	glb_state = aw869xx_get_glb_state(aw_haptic);
	if (glb_state == AW869XX_BIT_GLBRD5_STATE_RTP_GO) {
		rtp_state = 1;	/*is going on */
		aw_info("rtp_routine_on");
	}
	return rtp_state;
}

static void aw869xx_get_ram_data(struct aw_haptic *aw_haptic, char *buf)
{
	int i = 0;
	int size = 0;

	while (i < aw_haptic->ram.len) {
		if ((aw_haptic->ram.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			size = aw_haptic->ram.len - i;
		else
			size = AW_RAMDATA_RD_BUFFER_SIZE;
		haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_RAMDATA, buf + i,
				    size);
		i += size;
	}
}

static void aw869xx_get_first_wave_addr(struct aw_haptic *aw_haptic,
					uint8_t *wave_addr)
{
	uint8_t reg_array[3] = {0};

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_RAMDATA, reg_array,
			    AW_I2C_BYTE_THREE);
	wave_addr[0] = reg_array[1];
	wave_addr[1] = reg_array[2];
}

static void aw869xx_get_wav_seq(struct aw_haptic *aw_haptic, uint32_t len)
{
	uint8_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_SIZE] = {0};

	if (len > AW_SEQUENCER_SIZE)
		len = AW_SEQUENCER_SIZE;
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_WAVCFG1, reg_val, len);
	for (i = 0; i < len; i++)
		aw_haptic->seq[i] = reg_val[i];
}

static size_t aw869xx_get_wav_loop(struct aw_haptic *aw_haptic, char *buf)
{
	uint8_t i = 0;
	uint8_t reg_val[4] = {0};
	size_t count = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_WAVCFG9, reg_val,
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

static void aw869xx_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%02X", reg_val);
}

static uint8_t aw869xx_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_PWMCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val >>= 7;
	return reg_val;
}

static int aw869xx_get_irq_state(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSINT=0x%02X", reg_val);
	if (reg_val & AW869XX_BIT_SYSINT_BST_SCPI) {
		ret = AW_IRQ_BST_SCP;
		aw_err("chip scp int error");
	}
	if (reg_val & AW869XX_BIT_SYSINT_BST_OVPI) {
		ret = AW_IRQ_BST_OVP;
		aw_err("chip ov int error");
	}
	if (reg_val & AW869XX_BIT_SYSINT_UVLI) {
		ret = AW_IRQ_UVLO;
		aw_err("chip uvlo int error");
	}
	if (reg_val & AW869XX_BIT_SYSINT_OCDI) {
		ret = AW_IRQ_OCD;
		aw_err("chip over current int error");
	}
	if (reg_val & AW869XX_BIT_SYSINT_OTI) {
		ret = AW_IRQ_OT;
		aw_err("chip over temperature int error");
	}
	if (reg_val & AW869XX_BIT_SYSINT_DONEI) {
		ret = AW_IRQ_DONE;
		aw_info("chip playback done");
	}
	if (reg_val & AW869XX_BIT_SYSINT_FF_AFI) {
		ret = AW_IRQ_ALMOST_FULL;
		aw_info("aw_haptic rtp mode fifo almost full!");
	}
	if (reg_val & AW869XX_BIT_SYSINT_FF_AEI)
		ret = AW_IRQ_ALMOST_EMPTY;
	return ret;
}

static int aw869xx_read_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint32_t f0_reg = 0;

#ifdef AW_LRA_F0_DEFAULT
	/* lra_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_CONTRD14, reg_val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | (reg_val[1] << 0);
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("lra_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	aw_haptic->f0 = (uint32_t)AW869XX_F0_FARMULA(f0_reg);
	aw_info("lra_f0=%d", aw_haptic->f0);
#else
	/* cont_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_CONTRD16, reg_val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | (reg_val[1] << 0);
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("cont_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	aw_haptic->f0 = (uint32_t)AW869XX_F0_FARMULA(f0_reg);
	aw_info("cont_f0=%d", aw_haptic->f0);
#endif
	return 0;
}

static int aw869xx_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
	uint8_t cont_config[3] = {0};
	int drv_width = 0;
	int ret = 0;

	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw869xx_stop(aw_haptic);
	/* f0 calibrate work mode */
	aw869xx_play_mode(aw_haptic, AW_CONT_MODE);
	/* enable f0 detect */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG1,
				 AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
				 AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	/* cont config */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG6,
				 AW869XX_BIT_CONTCFG6_TRACK_EN_MASK,
				 (aw_haptic->info.is_enabled_track_en << 7));
	/* enable auto break */
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_PLAYCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	brk_en_default = 0x04 & reg_val;
	aw869xx_auto_break_mode(aw_haptic, true);
	/* f0 driver level & time */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG6,
				 AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK,
				 aw_haptic->info.cont_drv1_lvl);
	cont_config[0] = aw_haptic->info.cont_drv2_lvl;
	cont_config[1] = aw_haptic->info.cont_drv1_time;
	cont_config[2] = aw_haptic->info.cont_drv2_time;
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG7, cont_config,
			     AW_I2C_BYTE_THREE);
	/* TRACK_MARGIN */
	if (!aw_haptic->info.cont_track_margin) {
		aw_err("aw_haptic->info.cont_track_margin = 0!");
	} else {
		haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG11,
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
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG3, &cont_config[0],
			     AW_I2C_BYTE_ONE);
	/* play go */
	aw869xx_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw869xx_wait_enter_standby(aw_haptic);
	ret = aw869xx_read_f0(aw_haptic);
	/* restore default config */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG1,
				 AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
				 AW869XX_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	if (brk_en_default)
		aw869xx_auto_break_mode(aw_haptic, true);
	else
		aw869xx_auto_break_mode(aw_haptic, false);
	return ret;
}

static int aw869xx_ram_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
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
	aw869xx_stop(aw_haptic);
	/* f0 calibrate work mode */
	aw869xx_play_mode(aw_haptic, AW_RAM_MODE);
	/* enable f0 detect */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG1,
				 AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
				 AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	/* enable auto break */
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_PLAYCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	brk_en_default = 0x04 & reg_val;
	aw869xx_auto_break_mode(aw_haptic, true);
	aw869xx_set_bst_vol(aw_haptic, 8000);
	aw869xx_set_wav_seq(aw_haptic, 0x00, AW_RAM_GET_F0_SEQ);
	aw869xx_set_wav_seq(aw_haptic, 0x01, 0x00);
	aw869xx_set_wav_loop(aw_haptic, 0x00, 0x02);
	/* play go */
	aw869xx_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw869xx_wait_enter_standby(aw_haptic);
	ret = aw869xx_read_f0(aw_haptic);
	/* restore default config */
	aw869xx_set_bst_vol(aw_haptic, aw_haptic->vmax);
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG1,
				 AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
				 AW869XX_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	if (brk_en_default)
		aw869xx_auto_break_mode(aw_haptic, true);
	else
		aw869xx_auto_break_mode(aw_haptic, false);
	return ret;
}

static uint8_t aw869xx_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW869XX_BIT_SYSST_FF_AFS;
	reg_val >>= 3;
	return reg_val;
}

static uint8_t aw869xx_rtp_get_fifo_aes(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW869XX_BIT_SYSST_FF_AES;
	reg_val >>= 4;
	return reg_val;
}

static uint8_t aw869xx_osc_read_status(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSST2, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW869XX_BIT_SYSST2_FF_EMPTY;
	return reg_val;
}

static void aw869xx_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t lra_code = 0;

	aw869xx_stop(aw_haptic);

	aw869xx_ram_init(aw_haptic, true);
	/* enter standby mode */
	aw869xx_stop(aw_haptic);
	usleep_range(2000, 2500);
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
				 AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
				 AW869XX_BIT_SYSCTRL2_STANDBY_OFF);
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_DETCFG1,
				 AW869XX_BIT_DETCFG1_RL_OS_MASK,
				 AW869XX_BIT_DETCFG1_RL);
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_DETCFG2,
				 AW869XX_BIT_DETCFG2_DIAG_GO_MASK,
				 AW869XX_BIT_DETCFG2_DIAG_GO_ON);
	usleep_range(30000, 35000);
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_DET_RL, &reg_val,
			    AW_I2C_BYTE_ONE);
	lra_code = (lra_code | reg_val) << 2;
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_DET_LO, &reg_val,
			    AW_I2C_BYTE_ONE);
	lra_code = lra_code | (reg_val & 0x03);
	aw_haptic->lra = AW869XX_LRA_FORMULA(lra_code);
	aw869xx_ram_init(aw_haptic, false);
}

static void aw869xx_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw869xx_set_wav_seq(aw_haptic, 0x00, seq);
	aw869xx_set_wav_loop(aw_haptic, 0x00, AW869XX_BIT_WAVLOOP_INIFINITELY);
}

static void aw869xx_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t vbat_code = 0;
	uint32_t cont = 2000;

	aw869xx_stop(aw_haptic);
	aw869xx_ram_init(aw_haptic, true);
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_DETCFG2,
				 AW869XX_BIT_DETCFG2_VBAT_GO_MASK,
				 AW869XX_BIT_DETCFG2_VABT_GO_ON);

	while (cont--) {
		haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_DETCFG2, &reg_val,
			    AW_I2C_BYTE_ONE);
		if ((reg_val & 0x02) == 0 || cont == 0)
			break;
	}

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_DET_VBAT, &reg_val,
			    AW_I2C_BYTE_ONE);
	vbat_code = (vbat_code | reg_val) << 2;
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_DET_LO, &reg_val,
			    AW_I2C_BYTE_ONE);
	vbat_code = vbat_code | ((reg_val & 0x30) >> 4);
	aw_haptic->vbat = AW869XX_VBAT_FORMULA(vbat_code);
	if (aw_haptic->vbat > AW_VBAT_MAX) {
		aw_haptic->vbat = AW_VBAT_MAX;
		aw_info("vbat max limit = %d", aw_haptic->vbat);
	}
	if (aw_haptic->vbat < AW_VBAT_MIN) {
		aw_haptic->vbat = AW_VBAT_MIN;
		aw_info("vbat min limit = %d", aw_haptic->vbat);
	}
	aw_info("aw_haptic->vbat=%dmV, vbat_code=0x%02X",
		aw_haptic->vbat, vbat_code);
	aw869xx_ram_init(aw_haptic, false);
}

static ssize_t aw869xx_get_reg(struct aw_haptic *aw_haptic, ssize_t len,
			       char *buf)
{
	uint8_t i = 0;
	uint8_t reg_array[AW869XX_REG_D2SCFG1 + 1] = {0};

	for (i = 0; i < AW869XX_REG_RTPDATA; i++)
		haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_ID, reg_array,
				    AW869XX_REG_RTPDATA);
	for (i = AW869XX_REG_RTPDATA + 1; i < AW869XX_REG_RAMDATA; i++)
		haptic_hv_i2c_reads(aw_haptic, (AW869XX_REG_RTPDATA + 1),
			       &reg_array[AW869XX_REG_RTPDATA + 1],
			       (AW869XX_REG_RAMDATA - AW869XX_REG_RTPDATA - 1));
	for (i = AW869XX_REG_RAMDATA + 1; i <= AW869XX_REG_D2SCFG1; i++)
		haptic_hv_i2c_reads(aw_haptic, (AW869XX_REG_RAMDATA + 1),
			       &reg_array[AW869XX_REG_RAMDATA + 1],
			       (AW869XX_REG_D2SCFG1 - AW869XX_REG_RAMDATA - 1));
	for (i = 0; i <= AW869XX_REG_D2SCFG1; i++)
		if ((i != AW869XX_REG_RTPDATA) && (i != AW869XX_REG_RAMDATA))
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02X=0x%02X\n", i, reg_array[i]);
	return len;
}

static void aw869xx_offset_cali(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t cnt = 2000;

	aw869xx_ram_init(aw_haptic, true);

	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_DETCFG2,
				 AW869XX_BIT_DETCFG2_DIAG_GO_MASK,
				 AW869XX_BIT_DETCFG2_DIAG_GO_ON);
	while (cnt--) {
		haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_DETCFG2, &reg_val,
				    AW_I2C_BYTE_ONE);
		if ((reg_val & 0x01) == 0) {
			aw869xx_ram_init(aw_haptic, false);
			return;
		}
	}
	aw_err("calibration offset failed!");
	aw869xx_ram_init(aw_haptic, false);
}

static void aw869xx_trig_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter!");
	/* i2s config */
	if (aw_haptic->info.is_enabled_i2s && aw_haptic->i2s_config) {
		aw_info("i2s is enabled!");
		aw869xx_i2s_init(aw_haptic);
	} else {
		aw869xx_trig2_param_init(aw_haptic);
		aw869xx_trig3_param_init(aw_haptic);
		aw869xx_trig2_param_config(aw_haptic);
		aw869xx_trig3_param_config(aw_haptic);
	}
	/* one wire config */
	if (aw_haptic->info.is_enabled_one_wire) {
		aw_info("one wire is enabled!");
		aw869xx_one_wire_init(aw_haptic);
	} else {
		aw869xx_trig1_param_init(aw_haptic);
		aw869xx_trig1_param_config(aw_haptic);
	}
}

#ifdef AW_CHECK_RAM_DATA
#ifdef AW869XX_CHECK_RAM_DATA
static int aw869xx_check_ram_data(struct aw_haptic *aw_haptic,
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
#endif

static int aw869xx_container_update(struct aw_haptic *aw_haptic,
				    struct aw_haptic_container *awinic_cont)
{
	uint8_t reg_val[3] = {0};
	uint8_t fifo_adr[4] = {0};
	uint32_t temp = 0;
	uint32_t shift = 0;
	int i = 0;
	int ret = 0;
	int len = 0;

#ifdef AW_CHECK_RAM_DATA
#ifdef AW869XX_CHECK_RAM_DATA
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};
#endif
#endif

	mutex_lock(&aw_haptic->lock);
	aw_haptic->ram.baseaddr_shift = 2;
	aw_haptic->ram.ram_shift = 4;
	/* RAMINIT Enable */
	aw869xx_ram_init(aw_haptic, true);
	/* Enter standby mode */
	aw869xx_stop(aw_haptic);
	/* base addr */
	shift = aw_haptic->ram.baseaddr_shift;
	aw_haptic->ram.base_addr =
		(uint32_t)((awinic_cont->data[0 + shift] << 8) |
			   (awinic_cont->data[1 + shift]));
	/* fifo_ael */
	fifo_adr[0] = (uint8_t)AW869XX_FIFO_AE_ADDR_L(aw_haptic->ram.base_addr);
	/* fifo_afl */
	fifo_adr[1] = (uint8_t)AW869XX_FIFO_AF_ADDR_L(aw_haptic->ram.base_addr);
	/* fifo_aeh */
	fifo_adr[2] = (uint8_t)AW869XX_FIFO_AE_ADDR_H(aw_haptic->ram.base_addr);
	/* fifo_afh */
	fifo_adr[3] = (uint8_t)AW869XX_FIFO_AF_ADDR_H(aw_haptic->ram.base_addr);

	aw_info("base_addr = %d", aw_haptic->ram.base_addr);
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_RTPCFG1, &awinic_cont->data[shift],
			     AW_I2C_BYTE_TWO);
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_RTPCFG3,
				 (AW869XX_BIT_RTPCFG3_FIFO_AEH_MASK &
				  AW869XX_BIT_RTPCFG3_FIFO_AFH_MASK),
				 (fifo_adr[2] | fifo_adr[3]));
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_RTPCFG4, fifo_adr,
			     AW_I2C_BYTE_TWO);
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_RTPCFG3, reg_val,
			    AW_I2C_BYTE_THREE);
	temp = ((reg_val[0] & 0x0f) << 24) | ((reg_val[0] & 0xf0) << 4);
	temp = temp | reg_val[1];
	aw_info("almost_empty_threshold = %d", (uint16_t)temp);
	temp = temp | (reg_val[2] << 16);
	aw_info("almost_full_threshold = %d", temp >> 16);
	/* ram */
	shift = aw_haptic->ram.baseaddr_shift;
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_RAMADDRH,
			     &awinic_cont->data[shift], AW_I2C_BYTE_TWO);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;
		haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_RAMDATA,
				     &awinic_cont->data[i], len);
		i += len;
	}
#ifdef AW_CHECK_RAM_DATA
#ifdef AW869XX_CHECK_RAM_DATA
	shift = aw_haptic->ram.baseaddr_shift;
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_RAMADDRH,
			     &awinic_cont->data[shift], AW_I2C_BYTE_TWO);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_RD_BUFFER_SIZE;
		haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_RAMDATA,
				    ram_data, len);
		ret = aw869xx_check_ram_data(aw_haptic, &awinic_cont->data[i],
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
#endif
	/* RAMINIT Disable */
	aw869xx_ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);
	return ret;
}

static uint64_t aw869xx_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	uint64_t theory_time = 0;

	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_SYSCTRL2, &reg_val,
			    AW_I2C_BYTE_ONE);
	fre_val = (reg_val & 0x03) >> 0;

	if (fre_val == 2 || fre_val == 3)
		theory_time = (aw_haptic->rtp_len / 12000) * 1000000;	/*12K */
	if (fre_val == 0)
		theory_time = (aw_haptic->rtp_len / 24000) * 1000000;	/*24K */
	if (fre_val == 1)
		theory_time = (aw_haptic->rtp_len / 48000) * 1000000;	/*48K */

	aw_info("microsecond:%llu theory_time = %llu", aw_haptic->microsecond,
		theory_time);

	return theory_time;
}

static void aw869xx_parse_dt(struct device *dev, struct aw_haptic *aw_haptic,
			     struct device_node *np)
{
	uint8_t duration_time[3];
	uint8_t sine_array_temp[4];
	uint8_t bstcfg_temp[5];
	uint8_t trig_config_temp[24];
	uint32_t val = 0;

	val = of_property_read_u8(np, "aw869xx_gain_bypass",
				  &aw_haptic->info.gain_bypass);
	if (val != 0)
		aw_info("aw869xx_gain_bypass not found");
	val = of_property_read_u8(np, "aw869xx_brk_bst_md",
				  &aw_haptic->info.brk_bst_md);
	if (val != 0)
		aw_info("aw869xx_brk_bst_md not found");
	val = of_property_read_u32(np, "f0_pre", &aw_haptic->info.f0_pre);
	if (val != 0)
		aw_info("f0_pre not found");
	val = of_property_read_u8(np, "f0_cali_percent",
				  &aw_haptic->info.f0_cali_percent);
	if (val != 0)
		aw_info("f0_cali_percent not found");
	val = of_property_read_u8(np, "aw869xx_cont_drv1_lvl",
				  &aw_haptic->info.cont_drv1_lvl);
	if (val != 0)
		aw_info("aw869xx_cont_drv1_lvl not found");
	val = of_property_read_u32(np, "aw869xx_cont_lra_vrms",
				   &aw_haptic->info.cont_lra_vrms);
	if (val != 0)
		aw_info("aw869xx_cont_lra_vrms not found");
	val = of_property_read_u8(np, "aw869xx_cont_drv1_time",
				  &aw_haptic->info.cont_drv1_time);
	if (val != 0)
		aw_info("aw869xx_cont_drv1_time not found");
	val = of_property_read_u8(np, "aw869xx_cont_drv2_time",
				  &aw_haptic->info.cont_drv2_time);
	if (val != 0)
		aw_info("aw869xx_cont_drv2_time not found");
	val = of_property_read_u8(np, "aw869xx_cont_wait_num",
				  &aw_haptic->info.cont_wait_num);
	if (val != 0)
		aw_info("aw869xx_cont_wait_num not found");
	val = of_property_read_u8(np, "aw869xx_cont_bst_brk_gain",
				  &aw_haptic->info.cont_bst_brk_gain);
	if (val != 0)
		aw_info("aw869xx_cont_bst_brk_gain not found");
	val = of_property_read_u8(np, "aw869xx_cont_brk_gain",
				  &aw_haptic->info.cont_brk_gain);
	if (val != 0)
		aw_info("aw869xx_cont_brk_gain not found");
	val = of_property_read_u8(np, "aw869xx_cont_tset",
				  &aw_haptic->info.cont_tset);
	if (val != 0)
		aw_info("aw869xx_cont_tset not found");
	val = of_property_read_u8(np, "aw869xx_cont_bemf_set",
				  &aw_haptic->info.cont_bemf_set);
	if (val != 0)
		aw_info("aw869xx_cont_bemf_set not found");
	val = of_property_read_u8(np, "aw869xx_d2s_gain",
				  &aw_haptic->info.d2s_gain);
	if (val != 0)
		aw_info("aw869xx_d2s_gain not found");
	val = of_property_read_u8(np, "aw869xx_cont_brk_time",
				  &aw_haptic->info.cont_brk_time);
	if (val != 0)
		aw_info("aw869xx_cont_brk_time not found");
	val = of_property_read_u8(np, "aw869xx_cont_track_margin",
				  &aw_haptic->info.cont_track_margin);
	if (val != 0)
		aw_info("aw869xx_cont_track_margin not found");
	aw_haptic->info.is_enabled_track_en =
		of_property_read_bool(np, "aw869xx_is_enabled_track_en");
	aw_info("aw_haptic->info.is_enabled_track_en = %d",
		aw_haptic->info.is_enabled_track_en);
	aw_haptic->info.is_enabled_auto_bst =
		of_property_read_bool(np, "aw869xx_is_enabled_auto_bst");
	aw_info("aw_haptic->info.is_enabled_auto_bst = %d",
		aw_haptic->info.is_enabled_auto_bst);
	aw_haptic->info.is_enabled_i2s =
			of_property_read_bool(np, "aw869xx_is_enabled_i2s");
	aw_info("aw_haptic->info.is_enabled_i2s = %d",
		aw_haptic->info.is_enabled_i2s);
	aw_haptic->info.is_enabled_one_wire = of_property_read_bool(np,
						 "aw869xx_is_enabled_one_wire");
	aw_info("aw_haptic->info.is_enabled_one_wire = %d",
		aw_haptic->info.is_enabled_one_wire);
	val = of_property_read_u32(np, "aw869xx_bst_vol_default",
				   &aw_haptic->info.bst_vol_default);
	if (val != 0)
		aw_info("aw869xx_bst_vol_default not found");
	val = of_property_read_u8_array(np, "aw869xx_duration_time",
					duration_time,
					ARRAY_SIZE(duration_time));
	if (val != 0)
		aw_info("aw869xx_duration_time not found");
	else
		memcpy(aw_haptic->info.duration_time, duration_time,
		       sizeof(duration_time));
	val = of_property_read_u8_array(np, "aw869xx_bstcfg", bstcfg_temp,
					ARRAY_SIZE(bstcfg_temp));
	if (val != 0)
		aw_info("aw869xx_bstcfg not found");
	else
		memcpy(aw_haptic->info.bstcfg, bstcfg_temp,
		       sizeof(bstcfg_temp));
	val = of_property_read_u8_array(np, "aw869xx_sine_array",
				  sine_array_temp, ARRAY_SIZE(sine_array_temp));
	if (val != 0)
		aw_info("aw869xx_sine_array not found");
	else
		memcpy(aw_haptic->info.sine_array, sine_array_temp,
		       sizeof(sine_array_temp));
	val = of_property_read_u8_array(np, "aw869xx_trig_config",
					trig_config_temp,
					ARRAY_SIZE(trig_config_temp));
	if (val != 0)
		aw_info("aw869xx_trig_config not found");
	else
		memcpy(aw_haptic->info.trig_cfg, trig_config_temp,
		       sizeof(trig_config_temp));
	aw_dbg("aw_haptic->info.brk_bst_md: %d", aw_haptic->info.brk_bst_md);
	aw_dbg("aw_haptic->info.bst_vol_default: %d",
		aw_haptic->info.bst_vol_default);
}

static void aw869xx_misc_para_init(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[8] = {0};
	uint32_t drv2_lvl = 0;

	/* Cont drv2 lvl */
	drv2_lvl = AW869XX_DRV2_LVL_FARMULA(aw_haptic->info.f0_pre,
					    aw_haptic->info.cont_lra_vrms);
	if (drv2_lvl > AW_DRV2_LVL_MAX)
		aw_haptic->info.cont_drv2_lvl = AW_DRV2_LVL_MAX;
	else
		aw_haptic->info.cont_drv2_lvl = (uint8_t)drv2_lvl;
	/* Get vmax */
	if (aw_haptic->info.bst_vol_default > 0)
		aw_haptic->vmax = aw_haptic->info.bst_vol_default;
	/* Get gain */
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_PLAYCFG2, reg_val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->gain = reg_val[0];
	/* Get wave_seq */
	haptic_hv_i2c_reads(aw_haptic, AW869XX_REG_WAVCFG1, reg_val,
			    AW_I2C_BYTE_EIGHT);
	aw_haptic->index = reg_val[0];
	memcpy(aw_haptic->seq, reg_val, AW_SEQUENCER_SIZE);
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_BSTCFG1,
			     aw_haptic->info.bstcfg, AW_I2C_BYTE_FIVE);
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_SYSCTRL3,
			     aw_haptic->info.sine_array, AW_I2C_BYTE_FOUR);
	/* Set gain_bypass */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL7,
				 AW869XX_BIT_SYSCTRL7_GAIN_BYPASS_MASK,
				 aw_haptic->info.gain_bypass << 6);

	/* brk_bst_md */
	if (!aw_haptic->info.brk_bst_md)
		aw_err("aw_haptic->info.brk_bst_md = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG1,
				 AW869XX_BIT_CONTCFG1_BRK_BST_MD_MASK,
				 aw_haptic->info.brk_bst_md << 1);
	/* d2s_gain */
	if (!aw_haptic->info.d2s_gain)
		aw_err("aw_haptic->info.d2s_gain = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL7,
				 AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
				 aw_haptic->info.d2s_gain);
	/* cont_tset */
	if (!aw_haptic->info.cont_tset)
		aw_err("aw_haptic->info.cont_tset = 0!");
	/* cont_bemf_set */
	if (!aw_haptic->info.cont_bemf_set)
		aw_err("aw_haptic->info.cont_bemf_set = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG13,
				 (AW869XX_BIT_CONTCFG13_TSET_MASK &
				  AW869XX_BIT_CONTCFG13_BEME_SET_MASK),
				 ((aw_haptic->info.cont_tset << 4) |
				  aw_haptic->info.cont_bemf_set));
	/* cont_brk_time */
	if (!aw_haptic->info.cont_brk_time)
		aw_err("aw_haptic->info.cont_brk_time = 0!");
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);
	/* cont_bst_brk_gain */
	if (!aw_haptic->info.cont_bst_brk_gain)
		aw_err("aw_haptic->info.cont_bst_brk_gain = 0!");
	/* cont_brk_gain */
	if (!aw_haptic->info.cont_brk_gain)
		aw_err("aw_haptic->info.cont_brk_gain = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG5,
				 (AW869XX_BIT_CONTCFG5_BST_BRK_GAIN_MASK &
				  AW869XX_BIT_CONTCFG5_BRK_GAIN_MASK),
				 ((aw_haptic->info.cont_bst_brk_gain << 4) |
				  aw_haptic->info.cont_brk_gain));
	/* i2s enbale */
	if (aw_haptic->info.is_enabled_i2s && aw_haptic->i2s_config) {
		aw_info("i2s enabled!");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
					 AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
					 AW869XX_BIT_SYSCTRL2_I2S_PIN_I2S);
	} else {
		aw_info("i2s disabled!");
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
					 AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
					 AW869XX_BIT_SYSCTRL2_I2S_PIN_TRIG);
	}
	/* set BST_ADJ */
	haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_BSTCFG5,
				 AW869XX_BIT_BSTCFG5_BST_ADJ_MASK,
				 AW869XX_BIT_BSTCFG5_BST_ADJ_LOW);
	aw869xx_protect_config(aw_haptic, AW869XX_BIT_PWMCFG4_PRTIME_DEFAULT_VALUE,
			       AW869XX_BIT_PWMCFG3_PRLVL_DEFAULT_VALUE);
}

static ssize_t aw869xx_cont_wait_num_show(struct device *dev,
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

static ssize_t aw869xx_cont_wait_num_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	uint8_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtou8(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_haptic->info.cont_wait_num = val;
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG4, &val,
			     AW_I2C_BYTE_ONE);

	return count;
}

static ssize_t aw869xx_cont_drv_lvl_show(struct device *dev,
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

static ssize_t aw869xx_cont_drv_lvl_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	uint32_t databuf[2] = { 0, 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_lvl = databuf[0];
		aw_haptic->info.cont_drv2_lvl = databuf[1];
		haptic_hv_i2c_write_bits(aw_haptic, AW869XX_REG_CONTCFG6,
					 AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK,
					 aw_haptic->info.cont_drv1_lvl);
		haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG7,
			       &aw_haptic->info.cont_drv2_lvl, AW_I2C_BYTE_ONE);
	}
	return count;
}

static ssize_t aw869xx_cont_drv_time_show(struct device *dev,
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

static ssize_t aw869xx_cont_drv_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	uint8_t cont_time[2] = {0};
	uint32_t databuf[2] = { 0, 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		cont_time[0] = aw_haptic->info.cont_drv1_time = databuf[0];
		cont_time[1] = aw_haptic->info.cont_drv2_time = databuf[1];
		haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG8, cont_time,
				     AW_I2C_BYTE_TWO);
	}
	return count;
}

static ssize_t aw869xx_cont_brk_time_show(struct device *dev,
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

static ssize_t aw869xx_cont_brk_time_store(struct device *dev,
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
	haptic_hv_i2c_writes(aw_haptic, AW869XX_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);
	return count;
}

static ssize_t aw869xx_trig_show(struct device *dev,
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

static ssize_t aw869xx_trig_store(struct device *dev,
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

		if ((databuf[0] == 2 || databuf[0] == 3) &&
		     aw_haptic->info.is_enabled_i2s && aw_haptic->i2s_config) {
			aw_info("trig2 and trig3 pin used for i2s!");
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
		case 0:
			aw869xx_trig1_param_config(aw_haptic);
			break;
		case 1:
			aw869xx_trig2_param_config(aw_haptic);
			break;
		case 2:
			aw869xx_trig3_param_config(aw_haptic);
			break;
		}
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static DEVICE_ATTR(cont_wait_num, S_IWUSR | S_IRUGO, aw869xx_cont_wait_num_show,
		   aw869xx_cont_wait_num_store);
static DEVICE_ATTR(cont_drv_lvl, S_IWUSR | S_IRUGO, aw869xx_cont_drv_lvl_show,
		   aw869xx_cont_drv_lvl_store);
static DEVICE_ATTR(cont_drv_time, S_IWUSR | S_IRUGO, aw869xx_cont_drv_time_show,
		   aw869xx_cont_drv_time_store);
static DEVICE_ATTR(cont_brk_time, S_IWUSR | S_IRUGO, aw869xx_cont_brk_time_show,
		   aw869xx_cont_brk_time_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw869xx_trig_show,
		   aw869xx_trig_store);

static struct attribute *aw869xx_vibrator_attributes[] = {
	&dev_attr_trig.attr,
	&dev_attr_cont_wait_num.attr,
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	&dev_attr_cont_brk_time.attr,
	NULL
};

static struct attribute_group aw869xx_vibrator_attribute_group = {
	.attrs = aw869xx_vibrator_attributes
};

static int aw869xx_creat_node(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &aw869xx_vibrator_attribute_group);
	if (ret < 0)
		aw_err("error creating sysfs attr files");
	return ret;
}

struct aw_haptic_func aw869xx_func_list = {
	.play_stop = aw869xx_stop,
	.ram_init = aw869xx_ram_init,
	.get_vbat = aw869xx_get_vbat,
	.creat_node = aw869xx_creat_node,
	.get_f0 = aw869xx_get_f0,
	.ram_get_f0 = aw869xx_ram_get_f0,
	.cont_config = aw869xx_cont_config,
	.offset_cali = aw869xx_offset_cali,
	.check_qualify = aw869xx_check_qualify,
	.get_irq_state = aw869xx_get_irq_state,
	.judge_rtp_going = aw869xx_judge_rtp_going,
	.set_bst_peak_cur = aw869xx_set_bst_peak_cur,
	.get_theory_time = aw869xx_get_theory_time,
	.get_lra_resistance = aw869xx_get_lra_resistance,
	.set_pwm = aw869xx_set_pwm,
	.play_mode = aw869xx_play_mode,
	.set_bst_vol = aw869xx_set_bst_vol,
	.interrupt_setup = aw869xx_interrupt_setup,
	.set_repeat_seq = aw869xx_set_repeat_seq,
	.auto_bst_enable = aw869xx_auto_bst_enable,
	.vbat_mode_config = aw869xx_vbat_mode_config,
	.set_wav_seq = aw869xx_set_wav_seq,
	.set_wav_loop = aw869xx_set_wav_loop,
	.set_ram_addr = aw869xx_set_ram_addr,
	.set_rtp_data = aw869xx_set_rtp_data,
	.container_update = aw869xx_container_update,
	.protect_config = aw869xx_protect_config,
	.parse_dt = aw869xx_parse_dt,
	.trig_init = aw869xx_trig_init,
	.irq_clear = aw869xx_irq_clear,
	.get_wav_loop = aw869xx_get_wav_loop,
	.play_go = aw869xx_play_go,
	.misc_para_init = aw869xx_misc_para_init,
	.set_rtp_aei = aw869xx_set_rtp_aei,
	.set_gain = aw869xx_set_gain,
	.upload_lra = aw869xx_upload_lra,
	.bst_mode_config = aw869xx_bst_mode_config,
	.get_reg = aw869xx_get_reg,
	.get_prctmode = aw869xx_get_prctmode,
	.get_trim_lra = aw869xx_get_trim_lra,
	.get_ram_data = aw869xx_get_ram_data,
	.get_first_wave_addr = aw869xx_get_first_wave_addr,
	.get_glb_state = aw869xx_get_glb_state,
	.get_osc_status = aw869xx_osc_read_status,
	.rtp_get_fifo_afs = aw869xx_rtp_get_fifo_afs,
	.rtp_get_fifo_aes = aw869xx_rtp_get_fifo_aes,
	.get_wav_seq = aw869xx_get_wav_seq,
#ifdef AW_SND_SOC_CODEC
	.snd_soc_init = aw869xx_snd_soc_init,
#endif
};
