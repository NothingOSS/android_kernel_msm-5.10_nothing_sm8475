/*
 * File: aw8671x.c
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

static uint8_t aw8671x_get_glb_state(struct aw_haptic *aw_haptic);
static void aw8671x_vbat_mode_config(struct aw_haptic *, uint8_t);

/******************************************************
 *
 * aw8671x codec
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
static void aw8671x_i2s_enable(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_I2SCFG1,
					 AW8671X_BIT_I2SCFG1_I2S_EN_MASK,
					 AW8671X_BIT_I2SCFG1_I2S_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_I2SCFG1,
					 AW8671X_BIT_I2SCFG1_I2S_EN_MASK,
					 AW8671X_BIT_I2SCFG1_I2S_DISABLE);
	}
}

static int aw8671x_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("enter");
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mutex_lock(&aw_haptic->lock);
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL2,
					 AW8671X_BIT_SYSCTRL2_STANDBY_MASK,
					 AW8671X_BIT_SYSCTRL2_STANDBY_OFF);
		mutex_unlock(&aw_haptic->lock);
	}

	return 0;
}

static int aw8671x_set_fmt(struct snd_soc_dai *dai, uint32_t fmt)
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

static int aw8671x_set_dai_sysclk(struct snd_soc_dai *dai,
				  int clk_id, uint32_t freq, int dir)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("freq=%d", freq);
	aw_haptic->sysclk = freq;
	return 0;
}

static int aw8671x_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	uint8_t tmp_val = 0;
	uint8_t i2sfs_val = 0;
	uint8_t i2sbck_val = 0;
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
		i2sfs_val = AW8671X_BIT_I2SCFG3_I2SFS_16BITS;
		break;
	case 24:
		i2sfs_val = AW8671X_BIT_I2SCFG3_I2SFS_24BITS;
		i2sbck_val = 32 * params_channels(params);
		break;
	case 32:
		i2sfs_val = AW8671X_BIT_I2SCFG3_I2SFS_32BITS;
		break;
	default:
		i2sfs_val = AW8671X_BIT_I2SCFG3_I2SFS_16BITS;
		i2sbck_val = 16 * params_channels(params);
		aw_err("width [%d]can not support", aw_haptic->width);
		break;
	}
	aw_info("i2sfs_val=0x%02X", i2sfs_val);
	/* set width */
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_I2SCFG3,
				 AW8671X_BIT_I2SCFG3_I2SFS_MASK, i2sfs_val);

	/* match bck mode */
	switch (i2sbck_val) {
	case 32:
		i2sbck_val = AW8671X_BIT_I2SCFG3_I2SBCK_32FS;
		break;
	case 48:
		i2sbck_val = AW8671X_BIT_I2SCFG3_I2SBCK_48FS;
		break;
	case 64:
		i2sbck_val = AW8671X_BIT_I2SCFG3_I2SBCK_64FS;
		break;
	default:
		aw_err("i2sbck [%d] can not support", i2sbck_val);
		i2sbck_val = AW8671X_BIT_I2SCFG3_I2SBCK_32FS;
		break;
	}
	/* set bck mode */
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_I2SCFG3,
				 AW8671X_BIT_I2SCFG3_I2SBCK_MASK, i2sbck_val);

	/* check i2s cfg */
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_I2SCFG3, &tmp_val,
			    AW_I2C_BYTE_ONE);
	aw_info("i2scfg1=0x%02X", tmp_val);

	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int aw8671x_mute(struct snd_soc_dai *dai, int mute, int stream)
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
		aw8671x_i2s_enable(aw_haptic, false);
		mutex_unlock(&aw_haptic->lock);
	} else {
		mutex_lock(&aw_haptic->lock);
		aw8671x_i2s_enable(aw_haptic, true);
		usleep_range(2000, 3000);
		haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSST, &reg_val,
				    AW_I2C_BYTE_ONE);
		aw_info("sysst=0x%02X", reg_val);
		haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSST2, &reg_val,
				    AW_I2C_BYTE_ONE);
		aw_info("sysst2=0x%02X", reg_val);
		haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSER, &reg_val,
				    AW_I2C_BYTE_ONE);
		aw_info("syser=0x%02X", reg_val);
		reg_val = aw8671x_get_glb_state(aw_haptic);
		if (reg_val != 0x0a) {
			aw_err("i2s config err, glb_state=0x%02X", reg_val);
			aw8671x_i2s_enable(aw_haptic, false);
		} else {
			aw_info("i2s config pass, glb_state=0x%02X", reg_val);
		}
		mutex_unlock(&aw_haptic->lock);
	}

	return 0;
}

static void aw8671x_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	aw_snd_soc_codec_t *codec = aw_get_codec(dai);
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("enter");
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aw_haptic->rate = 0;
		mutex_lock(&aw_haptic->lock);
		/* aw8671x_i2s_enable(aw_haptic, false); */
		mutex_unlock(&aw_haptic->lock);
	}
}

static const struct snd_soc_dai_ops aw8671x_dai_ops = {
	.startup = aw8671x_startup,
	.set_fmt = aw8671x_set_fmt,
	.set_sysclk = aw8671x_set_dai_sysclk,
	.hw_params = aw8671x_hw_params,
	.mute_stream = aw8671x_mute,
	.shutdown = aw8671x_shutdown,
};

static struct snd_soc_dai_driver aw8671x_dai[] = {
	{
	 .name = "aw8671x-aif",
	 .id = 1,
	 .playback = {
		      .stream_name = "Speaker_Playback",
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = AW8671X_RATES,
		      .formats = AW8671X_FORMATS,
		      },
	 .capture = {
		     .stream_name = "Speaker_Capture",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = AW8671X_RATES,
		     .formats = AW8671X_FORMATS,
		     },
	 .ops = &aw8671x_dai_ops,
	 .symmetric_rates = 1,
	 },
};

/*****************************************************
 *
 * codec driver
 *
 *****************************************************/
static void aw8671x_add_codec_controls(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
}

static int aw8671x_codec_probe(aw_snd_soc_codec_t *codec)
{
	int ret = -1;
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("enter");
	aw_haptic->codec = codec;
	aw8671x_add_codec_controls(aw_haptic);
	aw_info("exit");
	ret = 0;
	return ret;
}

#ifdef KERNEL_OVER_4_19
static void aw8671x_codec_remove(struct snd_soc_component *component)
{
	/*
	 * struct aw_haptic *aw_haptic =
	 *     aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	 */
	aw_info("enter");
}
#else
static int aw8671x_codec_remove(aw_snd_soc_codec_t *codec)
{
	/*
	 * struct aw_haptic *aw_haptic =
	 *     aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);
	 */
	aw_info("enter");

	return 0;
}
#endif

static uint32_t aw8671x_codec_read(aw_snd_soc_codec_t *codec, uint32_t reg)
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

static int aw8671x_codec_write(aw_snd_soc_codec_t *codec,
			       uint32_t reg, uint32_t value)
{
	uint8_t val = (uint8_t)value;
	int ret = -1;
	struct aw_haptic *aw_haptic =
	    aw_componet_codec_ops.aw_snd_soc_codec_get_drvdata(codec);

	aw_info("enter ,reg:0x%02X = 0x%02X", reg, value);
	ret = haptic_hv_i2c_writes(aw_haptic, (uint8_t) reg, &val,
				   AW_I2C_BYTE_ONE);

	return ret;
}

#ifdef KERNEL_OVER_4_19
static struct snd_soc_component_driver soc_codec_dev_aw8671x = {
	.probe = aw8671x_codec_probe,
	.remove = aw8671x_codec_remove,
	.read = aw8671x_codec_read,
	.write = aw8671x_codec_write,
};
#else
static struct snd_soc_codec_driver soc_codec_dev_aw8671x = {
	.probe = aw8671x_codec_probe,
	.remove = aw8671x_codec_remove,
	.read = aw8671x_codec_read,
	.write = aw8671x_codec_write,
	.reg_cache_size = AW_HAPTIC_REG_MAX,
	.reg_word_size = 2,
};
#endif

static int aw8671x_snd_soc_init(struct device *dev)
{
	int ret = 0;
	struct snd_soc_dai_driver *dai;

	dev_set_name(dev, "%s", AW_I2C_NAME);
	/* register codec */
	dai = devm_kzalloc(dev, sizeof(aw8671x_dai), GFP_KERNEL);
	if (!dai) {
		aw_err("failed to register aw8671x");
		return -ENOMEM;
	}
	memcpy(dai, aw8671x_dai, sizeof(aw8671x_dai));
	aw_info("dai->name(%s)", dai->name);

	ret =
	aw_componet_codec_ops.aw_snd_soc_register_codec(dev,
							&soc_codec_dev_aw8671x,
							dai,
							ARRAY_SIZE
							(aw8671x_dai));
	if (ret < 0) {
		aw_err("failed to register aw8671x: %d", ret);
		return ret;
	}
	return 0;
}

#endif

static int aw8671x_check_qualify(struct aw_haptic *aw_haptic)
{
	int ret = -1;
	uint8_t reg_val = 0;

	aw_info("enter");
	ret = haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_EFCFG5, &reg_val,
				  AW_I2C_BYTE_ONE);
	if (ret < 0)
		return ret;
	if (!(reg_val & 0x80)) {
		aw_err("unqualified chip!");
		return -ERANGE;
	}
	return 0;
}

static void aw8671x_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL2,
					 AW8671X_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW8671X_BIT_SYSCTRL2_RATE_48K);
		break;
	case AW_PWM_24K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL2,
					 AW8671X_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW8671X_BIT_SYSCTRL2_RATE_24K);
		break;
	case AW_PWM_12K:
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL2,
					 AW8671X_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
					 AW8671X_BIT_SYSCTRL2_RATE_12K);
		break;
	default:
		break;
	}
}

static void aw8671x_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_PLAYCFG2, &gain,
			     AW_I2C_BYTE_ONE);
}

static void aw8671x_set_bst_peak_cur(struct aw_haptic *aw_haptic)
{

}

static void aw8671x_set_bst_vol(struct aw_haptic *aw_haptic, uint32_t bst_vol)
{
	uint8_t reg_val = 0;

	if (bst_vol < AW8671X_CHARGEPUMP_CODE1)
		reg_val = 0x00;
	else if (bst_vol < AW8671X_CHARGEPUMP_CODE2)
		reg_val = 0x01;
	else if (bst_vol < AW8671X_CHARGEPUMP_CODE3)
		reg_val = 0x02;
	else if (bst_vol < AW8671X_CHARGEPUMP_CODE4)
		reg_val = 0x03;
	else
		reg_val = 0x04;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG1,
				 AW8671X_BIT_PLAYCFG1_CP_CODE_MASK, reg_val);
}

static void aw8671x_set_wav_seq(struct aw_haptic *aw_haptic, uint8_t wav,
				uint8_t seq)
{
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_WAVCFG1 + wav, &seq,
			     AW_I2C_BYTE_ONE);
}

static void aw8671x_set_wav_loop(struct aw_haptic *aw_haptic, uint8_t wav,
				 uint8_t loop)
{
	uint8_t tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_WAVCFG9 + (wav / 2),
					 AW8671X_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_WAVCFG9 + (wav / 2),
					 AW8671X_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
}

static void aw8671x_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data,
				 uint32_t len)
{
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_RTPDATA, data, len);
}

static void aw8671x_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSINTM,
					 AW8671X_BIT_SYSINTM_FF_AEM_MASK,
					 AW8671X_BIT_SYSINTM_FF_AEM_ON);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSINTM,
					 AW8671X_BIT_SYSINTM_FF_AEM_MASK,
					 AW8671X_BIT_SYSINTM_FF_AEM_OFF);
	}
}

static void aw8671x_set_ram_addr(struct aw_haptic *aw_haptic)
{
	uint8_t ram_addr_l = (uint8_t)(aw_haptic->ram.base_addr & 0x00ff);

	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_RAMADDRH,
				 AW8671X_BIT_RAMADDRH_MASK,
				 (uint8_t)(aw_haptic->ram.base_addr >> 8));
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_RAMADDRL, &ram_addr_l,
			     AW_I2C_BYTE_ONE);
}

static void aw8671x_auto_break_mode(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG3,
					 AW8671X_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8671X_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG3,
					 AW8671X_BIT_PLAYCFG3_BRK_EN_MASK,
					 AW8671X_BIT_PLAYCFG3_BRK_DISABLE);
	}
}

static void aw8671x_f0_detect(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_CONTCFG1,
					 AW8671X_BIT_CONTCFG1_EN_F0_DET_MASK,
					 AW8671X_BIT_CONTCFG1_F0_DET_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_CONTCFG1,
					 AW8671X_BIT_CONTCFG1_EN_F0_DET_MASK,
					 AW8671X_BIT_CONTCFG1_F0_DET_DISABLE);
	}
}

static uint8_t aw8671x_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_GLBRD5, &state,
			    AW_I2C_BYTE_ONE);
	aw_dbg("glb state value is 0x%02X", state);
	return state;
}

static void aw8671x_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t delay_20us = AW8671X_BIT_START_DLY_20US;
	uint8_t delay_2p5ms = AW8671X_BIT_START_DLY_2P5MS;
	uint8_t go_on = AW8671X_BIT_PLAYCFG4_GO_ON;
	uint8_t stop_on = AW8671X_BIT_PLAYCFG4_STOP_ON;

	aw_dbg("enter, flag = %d", flag);
	if (flag) {
		if (aw_haptic->info.is_enabled_one_wire) {
			haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_GLBCFG2,
					     &delay_20us, AW_I2C_BYTE_ONE);
			haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_PLAYCFG4,
					     &go_on, AW_I2C_BYTE_ONE);
			usleep_range(1000, 1500);
			haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_GLBCFG2,
					     &delay_2p5ms, AW_I2C_BYTE_ONE);
		} else {
			haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_PLAYCFG4,
					     &go_on, AW_I2C_BYTE_ONE);
		}
	} else {
		haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_PLAYCFG4,
				     &stop_on, AW_I2C_BYTE_ONE);
	}
}

static int aw8671x_wait_enter_standby(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int count = 100;

	while (count--) {
		reg_val = aw8671x_get_glb_state(aw_haptic);
		if (reg_val == AW8671X_BIT_GLBRD5_STATE_STANDBY) {
			aw_info("entered standby!");
			return 0;
		}
		aw_dbg("wait for standby");
		usleep_range(2000, 2500);
	}
	aw_err("do not enter standby automatically");
	return -ERANGE;
}

static void aw8671x_bst_mode_config(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_BST_BOOST_MODE:
		aw_info("haptic bst mode = boost");
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG1,
				       AW8671X_BIT_PLAYCFG1_CP_MODE_MASK,
				       AW8671X_BIT_PLAYCFG1_CP_MODE_CHARGEPUMP);
		break;
	case AW_BST_BYPASS_MODE:
		aw_info("haptic bst mode = bypass");
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG1,
					 AW8671X_BIT_PLAYCFG1_CP_MODE_MASK,
					 AW8671X_BIT_PLAYCFG1_CP_MODE_BYPASS);
		break;
	default:
		aw_err("error mode %d", mode);
		break;
	}
}

static void aw8671x_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_info("enter standby mode");
		aw_haptic->play_mode = AW_STANDBY_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL2,
					 AW8671X_BIT_SYSCTRL2_STANDBY_MASK,
					 AW8671X_BIT_SYSCTRL2_STANDBY_ON);
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL2,
					 AW8671X_BIT_SYSCTRL2_STANDBY_MASK,
					 AW8671X_BIT_SYSCTRL2_STANDBY_OFF);
		break;
	case AW_RAM_MODE:
		aw_info("enter ram mode");
		aw_haptic->play_mode = AW_RAM_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG3,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw8671x_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw8671x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RAM_LOOP_MODE:
		aw_info("enter ram loop mode");
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG3,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw8671x_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		aw8671x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_RTP_MODE:
		aw_info("enter rtp mode");
		aw_haptic->play_mode = AW_RTP_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG3,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_RTP);
		/* bst mode */
		aw8671x_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		aw8671x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_TRIG_MODE:
		aw_info("enter trig mode");
		aw_haptic->play_mode = AW_TRIG_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG3,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_RAM);
		aw8671x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_SW_COMP_MODE);
		break;
	case AW_CONT_MODE:
		aw_info("enter cont mode");
		aw_haptic->play_mode = AW_CONT_MODE;
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG3,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_MASK,
					 AW8671X_BIT_PLAYCFG3_PLAY_MODE_CONT);
		/* bst mode */
		aw8671x_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		aw8671x_vbat_mode_config(aw_haptic, AW_CONT_VBAT_HW_COMP_MODE);
		break;
	default:
		aw_err("play mode %d error", play_mode);
		break;
	}
}

static void aw8671x_stop(struct aw_haptic *aw_haptic)
{
	aw8671x_play_go(aw_haptic, false);
	aw8671x_wait_enter_standby(aw_haptic);
	aw8671x_play_mode(aw_haptic, AW_STANDBY_MODE);
}

static void aw8671x_ram_init(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL1,
					 AW8671X_BIT_SYSCTRL1_RAMINIT_MASK,
					 AW8671X_BIT_SYSCTRL1_RAMINIT_ON);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL1,
					 AW8671X_BIT_SYSCTRL1_RAMINIT_MASK,
					 AW8671X_BIT_SYSCTRL1_RAMINIT_OFF);
	}
}

static void aw8671x_upload_lra(struct aw_haptic *aw_haptic, uint32_t flag)
{
	switch (flag) {
	case AW_WRITE_ZERO:
		aw_info("write zero to trim_lra!");
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRIMCFG2,
					 AW8671X_BIT_TRIMCFG2_TRIM_LRA_MASK,
					 0x00);
		break;
	case AW_F0_CALI_LRA:
		aw_info("write f0_cali_data to trim_lra = 0x%02X",
			aw_haptic->f0_cali_data);
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRIMCFG2,
					 AW8671X_BIT_TRIMCFG2_TRIM_LRA_MASK,
					 (char)aw_haptic->f0_cali_data);
		break;
	case AW_OSC_CALI_LRA:
		aw_info("write osc_cali_data to trim_lra = 0x%02X",
			aw_haptic->osc_cali_data);
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRIMCFG2,
					 AW8671X_BIT_TRIMCFG2_TRIM_LRA_MASK,
					 (char)aw_haptic->osc_cali_data);
		break;
	default:
		aw_err("error param!");
		break;
	}
}

static uint8_t aw8671x_get_trim_lra(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_TRIMCFG2, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= 0x3F;
	return reg_val;
}

static void aw8671x_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_VBATCTRL,
					 AW8671X_BIT_VBATCTRL_VBAT_MODE_MASK,
					 AW8671X_BIT_VBATCTRL_VBAT_MODE_HW);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_VBATCTRL,
					 AW8671X_BIT_VBATCTRL_VBAT_MODE_MASK,
					 AW8671X_BIT_VBATCTRL_VBAT_MODE_SW);
	}
}

static void aw8671x_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime,
				   uint8_t prlvl)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PWMCFG1,
				 AW8671X_BIT_PWMCFG1_PRC_EN_MASK,
				 AW8671X_BIT_PWMCFG1_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_info("enable protection mode");
		reg_val = AW8671X_BIT_PWMCFG3_PR_ENABLE |
			  (prlvl & (~AW8671X_BIT_PWMCFG3_PRLVL_MASK));
		haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_PWMCFG3, &reg_val,
				     AW_I2C_BYTE_ONE);
		haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_PWMCFG4, &prtime,
				     AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_info("disable protection mode");
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PWMCFG3,
					 AW8671X_BIT_PWMCFG3_PR_EN_MASK,
					 AW8671X_BIT_PWMCFG3_PR_DISABLE);
	}
}

static void aw8671x_cont_config(struct aw_haptic *aw_haptic)
{
	/* uint8_t drv1_time = 0xFF; */
	uint8_t drv2_time = 0xFF;

	/* work mode */
	aw8671x_play_mode(aw_haptic, AW_CONT_MODE);
	/* f0 driver level */
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_CONTCFG6,
				 (AW8671X_BIT_CONTCFG6_TRACK_EN_MASK &
				  AW8671X_BIT_CONTCFG6_DRV1_LVL_MASK),
				 ((aw_haptic->info.is_enabled_track_en << 7) |
				  aw_haptic->info.cont_drv1_lvl));

	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG7,
			     &aw_haptic->info.cont_drv2_lvl, AW_I2C_BYTE_ONE);
	/* DRV1_TIME */
	/* haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG8, &drv1_time,
				AW_I2C_BYTE_ONE); */
	/* DRV2_TIME */
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG9, &drv2_time,
			     AW_I2C_BYTE_ONE);
	/* cont play go */
	aw8671x_play_go(aw_haptic, true);
}

static void aw8671x_one_wire_init(struct aw_haptic *aw_haptic)
{
	uint8_t trig_prio = 0x6c;
	uint8_t delay_2p5ms = AW8671X_BIT_START_DLY_2P5MS;

	aw_info("enter");
	/*if enable one-wire, trig1 priority must be less than trig2 and trig3*/
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_GLBCFG4, &trig_prio,
			     AW_I2C_BYTE_ONE);
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_GLBCFG2, &delay_2p5ms,
			     AW_I2C_BYTE_ONE);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG8,
				 AW8671X_BIT_TRGCFG8_TRG_ONEWIRE_MASK,
				 AW8671X_BIT_TRGCFG8_TRG_ONEWIRE_ENABLE);
}

static void aw8671x_i2s_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter");

	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL2,
				 AW8671X_BIT_SYSCTRL2_I2S_PIN_MASK,
				 AW8671X_BIT_SYSCTRL2_I2S_PIN_I2S);
}

static void aw8671x_trig1_param_init(struct aw_haptic *aw_haptic)
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

static void aw8671x_trig2_param_init(struct aw_haptic *aw_haptic)
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

static void aw8671x_trig3_param_init(struct aw_haptic *aw_haptic)
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

static void aw8671x_trig1_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[0].trig_level)
		trig_config |= AW8671X_BIT_TRGCFG7_TRG1_MODE_LEVEL;
	else
		trig_config |= AW8671X_BIT_TRGCFG7_TRG1_MODE_EDGE;
	if (aw_haptic->trig[0].trig_polar)
		trig_config |= AW8671X_BIT_TRGCFG7_TRG1_POLAR_NEG;
	else
		trig_config |= AW8671X_BIT_TRGCFG7_TRG1_POLAR_POS;
	if (aw_haptic->trig[0].trig_brk)
		trig_config |= AW8671X_BIT_TRGCFG7_TRG1_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRGCFG7_TRG1_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[0].trig_bst)
		trig_config |= AW8671X_BIT_TRGCFG7_TRG1_BST_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRGCFG7_TRG1_BST_DISABLE;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG7,
				 (AW8671X_BIT_TRGCFG7_TRG1_MODE_MASK &
				  AW8671X_BIT_TRGCFG7_TRG1_POLAR_MASK &
				  AW8671X_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK &
				  AW8671X_BIT_TRGCFG7_TRG1_BST_MASK),
				 trig_config);

	trig_config = 0;
	if (aw_haptic->trig[0].pos_enable)
		trig_config |= AW8671X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[0].pos_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG1,
				 (AW8671X_BIT_TRG_ENABLE_MASK &
				  AW8671X_BIT_TRG_SEQ_MASK), trig_config);

	trig_config = 0;
	if (aw_haptic->trig[0].neg_enable)
		trig_config |= AW8671X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[0].neg_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG4,
				 (AW8671X_BIT_TRG_ENABLE_MASK &
				  AW8671X_BIT_TRG_SEQ_MASK), trig_config);
}

static void aw8671x_trig2_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[1].trig_level)
		trig_config |= AW8671X_BIT_TRGCFG7_TRG2_MODE_LEVEL;
	else
		trig_config |= AW8671X_BIT_TRGCFG7_TRG2_MODE_EDGE;
	if (aw_haptic->trig[1].trig_polar)
		trig_config |= AW8671X_BIT_TRGCFG7_TRG2_POLAR_NEG;
	else
		trig_config |= AW8671X_BIT_TRGCFG7_TRG2_POLAR_POS;
	if (aw_haptic->trig[1].trig_brk)
		trig_config |= AW8671X_BIT_TRGCFG7_TRG2_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRGCFG7_TRG2_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[1].trig_bst)
		trig_config |= AW8671X_BIT_TRGCFG7_TRG2_BST_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRGCFG7_TRG2_BST_DISABLE;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG7,
				 (AW8671X_BIT_TRGCFG7_TRG2_MODE_MASK &
				  AW8671X_BIT_TRGCFG7_TRG2_POLAR_MASK &
				  AW8671X_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK &
				  AW8671X_BIT_TRGCFG7_TRG2_BST_MASK),
				 trig_config);

	trig_config = 0;
	if (aw_haptic->trig[1].pos_enable)
		trig_config |= AW8671X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[1].pos_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG2,
				 (AW8671X_BIT_TRG_ENABLE_MASK &
				  AW8671X_BIT_TRG_SEQ_MASK), trig_config);

	trig_config = 0;
	if (aw_haptic->trig[1].neg_enable)
		trig_config |= AW8671X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[1].neg_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG5,
				 (AW8671X_BIT_TRG_ENABLE_MASK &
				  AW8671X_BIT_TRG_SEQ_MASK), trig_config);
}

static void aw8671x_trig3_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[2].trig_level)
		trig_config |= AW8671X_BIT_TRGCFG8_TRG3_MODE_LEVEL;
	else
		trig_config |= AW8671X_BIT_TRGCFG8_TRG3_MODE_EDGE;
	if (aw_haptic->trig[2].trig_polar)
		trig_config |= AW8671X_BIT_TRGCFG8_TRG3_POLAR_NEG;
	else
		trig_config |= AW8671X_BIT_TRGCFG8_TRG3_POLAR_POS;
	if (aw_haptic->trig[2].trig_brk)
		trig_config |= AW8671X_BIT_TRGCFG8_TRG3_AUTO_BRK_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRGCFG8_TRG3_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[2].trig_bst)
		trig_config |= AW8671X_BIT_TRGCFG8_TRG3_BST_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRGCFG8_TRG3_BST_DISABLE;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG8,
				 (AW8671X_BIT_TRGCFG8_TRG3_MODE_MASK &
				  AW8671X_BIT_TRGCFG8_TRG3_POLAR_MASK &
				  AW8671X_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK &
				  AW8671X_BIT_TRGCFG8_TRG3_BST_MASK),
				 trig_config);

	trig_config = 0;
	if (aw_haptic->trig[2].pos_enable)
		trig_config |= AW8671X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[2].pos_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG3,
				 (AW8671X_BIT_TRG_ENABLE_MASK &
				  AW8671X_BIT_TRG_SEQ_MASK), trig_config);

	trig_config = 0;
	if (aw_haptic->trig[2].neg_enable)
		trig_config |= AW8671X_BIT_TRG_ENABLE;
	else
		trig_config |= AW8671X_BIT_TRG_DISABLE;
	trig_config |= aw_haptic->trig[2].neg_sequence;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_TRGCFG6,
				 (AW8671X_BIT_TRG_ENABLE_MASK &
				  AW8671X_BIT_TRG_SEQ_MASK), trig_config);
}

static void aw8671x_auto_bst_enable(struct aw_haptic *aw_haptic, uint8_t flag)
{
	aw_haptic->auto_boost = flag;

	if (flag) {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG3,
					 AW8671X_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW8671X_BIT_PLAYCFG3_AUTO_BST_ENABLE);
	} else {
		haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_PLAYCFG3,
					 AW8671X_BIT_PLAYCFG3_AUTO_BST_MASK,
					 AW8671X_BIT_PLAYCFG3_AUTO_BST_DISABLE);
	}
}

static void aw8671x_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%02X", reg_val);
	/* edge int mode */
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL3,
				 (AW8671X_BIT_SYSCTRL3_INT_MODE_MASK &
				  AW8671X_BIT_SYSCTRL3_INT_EDGE_MODE_MASK),
				 (AW8671X_BIT_SYSCTRL3_INT_MODE_EDGE |
				  AW8671X_BIT_SYSCTRL3_INT_EDGE_MODE_POS));
	/* int enable */
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSINTM,
				 (AW8671X_BIT_SYSINTM_BST_OVPM_MASK &
				  AW8671X_BIT_SYSINTM_UVLM_MASK &
				  AW8671X_BIT_SYSINTM_OCDM_MASK &
				  AW8671X_BIT_SYSINTM_OTM_MASK),
				 (AW8671X_BIT_SYSINTM_BST_OVPM_OFF |
				  AW8671X_BIT_SYSINTM_UVLM_ON |
				  AW8671X_BIT_SYSINTM_OCDM_ON |
				  AW8671X_BIT_SYSINTM_OTM_ON));
}

static int aw8671x_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state = 0;
	uint8_t rtp_state = 0;

	glb_state = aw8671x_get_glb_state(aw_haptic);
	if (glb_state == AW8671X_BIT_GLBRD5_STATE_RTP_GO) {
		rtp_state = 1;  /*is going on */
		aw_info("rtp_routine_on");
	}
	return rtp_state;
}

static void aw8671x_get_ram_data(struct aw_haptic *aw_haptic, char *buf)
{
	int i = 0;
	int size = 0;

	while (i < aw_haptic->ram.len) {
		if ((aw_haptic->ram.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			size = aw_haptic->ram.len - i;
		else
			size = AW_RAMDATA_RD_BUFFER_SIZE;
		haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_RAMDATA, buf + i,
				    size);
		i += size;
	}
}

static void aw8671x_get_first_wave_addr(struct aw_haptic *aw_haptic,
					uint8_t *wave_addr)
{
	uint8_t reg_array[3] = {0};

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_RAMDATA, reg_array,
			    AW_I2C_BYTE_THREE);
	wave_addr[0] = reg_array[1];
	wave_addr[1] = reg_array[2];
}

static void aw8671x_get_wav_seq(struct aw_haptic *aw_haptic, uint32_t len)
{
	uint8_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_SIZE] = {0};

	if (len > AW_SEQUENCER_SIZE)
		len = AW_SEQUENCER_SIZE;
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_WAVCFG1, reg_val, len);
	for (i = 0; i < len; i++)
		aw_haptic->seq[i] = reg_val[i];
}

static size_t aw8671x_get_wav_loop(struct aw_haptic *aw_haptic, char *buf)
{
	uint8_t i = 0;
	uint8_t reg_val[AW_SEQUENCER_LOOP_SIZE] = {0};
	size_t count = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_WAVCFG9, reg_val,
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

static void aw8671x_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_info("reg SYSINT=0x%02X", reg_val);
}

static uint8_t aw8671x_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_PWMCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val >>= 7;
	return reg_val;
}

static int aw8671x_get_irq_state(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSINT, &reg_val,
			    AW_I2C_BYTE_ONE);
	aw_dbg("reg SYSINT=0x%02X", reg_val);

	if (reg_val & AW8671X_BIT_SYSINT_LOW_VBATI) {
		ret = AW_IRQ_LOW_VBAT;
		aw_err("chip low_vbat int error");
	}

	if (reg_val & AW8671X_BIT_SYSINT_UVLI) {
		ret = AW_IRQ_UVLO;
		aw_err("chip uvlo int error");
	}

	if (reg_val & AW8671X_BIT_SYSINT_OCDI) {
		ret = AW_IRQ_OCD;
		aw_err("chip over current int error");
	}

	if (reg_val & AW8671X_BIT_SYSINT_OTI) {
		ret = AW_IRQ_OT;
		aw_err("chip over temperature int error");
	}

	if (reg_val & AW8671X_BIT_SYSINT_DONEI) {
		ret = AW_IRQ_DONE;
		aw_info("chip playback done");
	}
	if (reg_val & AW8671X_BIT_SYSINT_FF_AFI) {
		ret = AW_IRQ_ALMOST_FULL;
		aw_info("aw_haptic rtp mode fifo almost full!");
	}

	if (reg_val & AW8671X_BIT_SYSINT_FF_AEI)
		ret = AW_IRQ_ALMOST_EMPTY;

	return ret;
}

static int aw8671x_read_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint32_t f0_reg = 0;
	uint64_t f0_tmp = 0;

#ifdef AW_LRA_F0_DEFAULT
	/* lra_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_CONTCFG14, reg_val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | reg_val[1];
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("lra_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	f0_tmp = AW8671X_F0_FARMULA(f0_reg);
	aw_haptic->f0 = (uint32_t)f0_tmp;
	aw_info("lra_f0=%d", aw_haptic->f0);
#else
	/* cont_f0 */
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_CONTCFG16, reg_val,
			    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | reg_val[1];
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_err("cont_f0 is error, f0_reg=0");
		return -ERANGE;
	}
	f0_tmp = AW8671X_F0_FARMULA(f0_reg);
	aw_haptic->f0 = (uint32_t)f0_tmp;
	aw_info("cont_f0=%d", aw_haptic->f0);
#endif
	return 0;
}

static int aw8671x_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
	uint8_t d2s_gain_default = 0;
	uint8_t cont_config[3] = {0};
	int drv_width = 0;
	int ret = 0;

	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw8671x_stop(aw_haptic);
	/* config max d2s_gain */
	haptic_hv_i2c_reads(aw_haptic,  AW8671X_REG_DETCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	d2s_gain_default = reg_val & AW8671X_BIT_DETCFG3_D2S_GAIN;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG3,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_MASK,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_40);
	/* f0 calibrate work mode */
	aw8671x_play_mode(aw_haptic, AW_CONT_MODE);
	/* enable f0 detect */
	aw8671x_f0_detect(aw_haptic, true);
	/* cont config */
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_CONTCFG6,
				 AW8671X_BIT_CONTCFG6_TRACK_EN_MASK,
				 (aw_haptic->info.is_enabled_track_en << 7));
	/* enable auto break */
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_PLAYCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	brk_en_default = reg_val & AW8671X_BIT_PLAYCFG3_BRK_ENABLE;
	aw8671x_auto_break_mode(aw_haptic, true);
	/* f0 driver level & time */
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_CONTCFG6,
				 AW8671X_BIT_CONTCFG6_DRV1_LVL_MASK,
				 aw_haptic->info.cont_drv1_lvl);
	cont_config[0] = aw_haptic->info.cont_drv2_lvl;
	cont_config[1] = aw_haptic->info.cont_drv1_time;
	cont_config[2] = aw_haptic->info.cont_drv2_time;
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG7, cont_config,
			     AW_I2C_BYTE_THREE);
	/* TRACK_MARGIN */
	if (!aw_haptic->info.cont_track_margin) {
		aw_err("awinic->info.cont_track_margin = 0!");
	} else {
		haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG11,
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
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG3, &cont_config[0],
			     AW_I2C_BYTE_ONE);
	/* play go */
	aw8671x_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw8671x_wait_enter_standby(aw_haptic);
	ret = aw8671x_read_f0(aw_haptic);
	/* restore default config */
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG3,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_MASK,
				 d2s_gain_default);
	aw8671x_f0_detect(aw_haptic, false);
	/* recover auto break config */
	if (brk_en_default)
		aw8671x_auto_break_mode(aw_haptic, true);
	else
		aw8671x_auto_break_mode(aw_haptic, false);
	return ret;
}

static int aw8671x_ram_get_f0(struct aw_haptic *aw_haptic)
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
	aw8671x_stop(aw_haptic);
	/* config max d2s_gain */
	haptic_hv_i2c_reads(aw_haptic,  AW8671X_REG_DETCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	d2s_gain_default = reg_val & AW8671X_BIT_DETCFG3_D2S_GAIN;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG3,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_MASK,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_40);
	/* f0 calibrate work mode */
	aw8671x_play_mode(aw_haptic, AW_RAM_MODE);
	/* enable f0 detect */
	aw8671x_f0_detect(aw_haptic, true);
	/* enable auto break */
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_PLAYCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
	brk_en_default = reg_val & AW8671X_BIT_PLAYCFG3_BRK_ENABLE;
	aw8671x_auto_break_mode(aw_haptic, true);
	aw8671x_set_bst_vol(aw_haptic, 6558);
	aw8671x_set_wav_seq(aw_haptic, 0x00, AW_RAM_GET_F0_SEQ);
	aw8671x_set_wav_seq(aw_haptic, 0x01, 0x00);
	aw8671x_set_wav_loop(aw_haptic, 0x00, 0x02);
	/* play go */
	aw8671x_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw8671x_wait_enter_standby(aw_haptic);
	ret = aw8671x_read_f0(aw_haptic);
	/* restore default config */
	aw8671x_set_bst_vol(aw_haptic, aw_haptic->vmax);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG3,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_MASK,
				 d2s_gain_default);
	aw8671x_f0_detect(aw_haptic, false);
	/* recover auto break config */
	if (brk_en_default)
		aw8671x_auto_break_mode(aw_haptic, true);
	else
		aw8671x_auto_break_mode(aw_haptic, false);
	return ret;
}

static uint8_t aw8671x_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW8671X_BIT_SYSST_FF_AFS;
	reg_val >>= 3;
	return reg_val;
}

static uint8_t aw8671x_rtp_get_fifo_aes(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSST, &reg_val,
			    AW_I2C_BYTE_ONE);
	reg_val &= AW8671X_BIT_SYSST_FF_AES;
	reg_val >>= 4;
	return reg_val;
}

static uint8_t aw8671x_get_osc_status(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSST2, &state,
			    AW_I2C_BYTE_ONE);
	state &= AW8671X_BIT_SYSST2_FF_EMPTY;
	return state;
}

static int aw8671x_select_d2s_gain(uint8_t reg)
{
	int d2s_gain = 0;

	switch (reg) {
	case AW8671X_BIT_DETCFG3_D2S_GAIN_1:
		d2s_gain = 1;
		break;
	case AW8671X_BIT_DETCFG3_D2S_GAIN_2:
		d2s_gain = 2;
		break;
	case AW8671X_BIT_DETCFG3_D2S_GAIN_4:
		d2s_gain = 4;
		break;
	case AW8671X_BIT_DETCFG3_D2S_GAIN_8:
		d2s_gain = 8;
		break;
	case AW8671X_BIT_DETCFG3_D2S_GAIN_10:
		d2s_gain = 10;
		break;
	case AW8671X_BIT_DETCFG3_D2S_GAIN_16:
		d2s_gain = 16;
		break;
	case AW8671X_BIT_DETCFG3_D2S_GAIN_20:
		d2s_gain = 20;
		break;
	case AW8671X_BIT_DETCFG3_D2S_GAIN_40:
		d2s_gain = 40;
		break;
	default:
		d2s_gain = -1;
		break;
	}
	return d2s_gain;
}

static void aw8671x_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint8_t adc_fs_default = 0;
	uint8_t d2s_gain_default = 0;
	uint8_t d2s_gain = 0;
	uint32_t lra_code = 0;

	aw8671x_ram_init(aw_haptic, true);
	aw8671x_stop(aw_haptic);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG3,
				 AW8671X_BIT_DETCFG3_DET_SEQ0_MASK,
				 AW8671X_BIT_DETCFG3_DET_SEQ0_RL);
	/* ADC_FS */
	haptic_hv_i2c_reads(aw_haptic,  AW8671X_REG_DETCFG1, &reg_val[0],
			    AW_I2C_BYTE_ONE);
	adc_fs_default = reg_val[0] & AW8671X_BIT_DETCFG1_ADC_FS;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG1,
				 AW8671X_BIT_DETCFG1_ADC_FS_MASK,
				 AW8671X_BIT_DETCFG1_ADC_FS_12KHZ);
	/* D2S_GAIN*/
	haptic_hv_i2c_reads(aw_haptic,  AW8671X_REG_DETCFG3, &reg_val[0],
			    AW_I2C_BYTE_ONE);
	d2s_gain_default = reg_val[0] & AW8671X_BIT_DETCFG3_D2S_GAIN;
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG3,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_MASK,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_10);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG1,
				 AW8671X_BIT_DETCFG1_DET_GO_MASK,
				 AW8671X_BIT_DETCFG1_DET_GO_DET_SEQ0);
	usleep_range(3000, 3500);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG1,
				 AW8671X_BIT_DETCFG1_DET_GO_MASK,
				 AW8671X_BIT_DETCFG1_DET_GO_NA);

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_DETCFG3, &reg_val[0],
			    AW_I2C_BYTE_ONE);
	d2s_gain = reg_val[0] & AW8671X_BIT_DETCFG3_D2S_GAIN;
	d2s_gain = aw8671x_select_d2s_gain(d2s_gain);
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_DETRD1, reg_val,
			    AW_I2C_BYTE_TWO);
	lra_code = ((reg_val[0] & AW8671X_BIT_DETRD1_AVG_DATA) << 8) +
								     reg_val[1];
	aw_haptic->lra = AW8671X_LRA_FORMULA(lra_code, d2s_gain);
	/* restore default config */
	aw8671x_ram_init(aw_haptic, false);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG1,
				 AW8671X_BIT_DETCFG1_ADC_FS_MASK,
				 adc_fs_default);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG3,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_MASK,
				 d2s_gain_default);
}

static void aw8671x_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw8671x_set_wav_seq(aw_haptic, 0x00, seq);
	aw8671x_set_wav_loop(aw_haptic, 0x00, AW8671X_BIT_WAVLOOP_INIFINITELY);
}

static void aw8671x_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint32_t vbat_code = 0;

	aw8671x_stop(aw_haptic);
	aw8671x_ram_init(aw_haptic, true);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG3,
				 AW8671X_BIT_DETCFG3_DET_SEQ0_MASK,
				 AW8671X_BIT_DETCFG3_DET_SEQ0_VBAT);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG1,
				 AW8671X_BIT_DETCFG1_DET_GO_MASK,
				 AW8671X_BIT_DETCFG1_DET_GO_DET_SEQ0);
	usleep_range(3000, 3500);
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG1,
				 AW8671X_BIT_DETCFG1_DET_GO_MASK,
				 AW8671X_BIT_DETCFG1_DET_GO_NA);
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_DETRD1, reg_val,
			    AW_I2C_BYTE_TWO);
	vbat_code = ((reg_val[0] & AW8671X_BIT_DETRD1_AVG_DATA) << 8) +
								     reg_val[1];
	aw_haptic->vbat = AW8671X_VBAT_FORMULA(vbat_code);
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
	aw8671x_ram_init(aw_haptic, false);
}

static ssize_t aw8671x_get_reg(struct aw_haptic *aw_haptic, ssize_t len,
			       char *buf)
{
	uint8_t i = 0;
	uint8_t reg_array[AW8671X_REG_ANACFG14 + 1] = {0};

	for (i = 0; i < AW8671X_REG_RTPDATA; i++)
		haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_RSTCFG, reg_array,
				    AW8671X_REG_RTPDATA);
	for (i = AW8671X_REG_RTPDATA + 1; i < AW8671X_REG_RAMDATA; i++)
		haptic_hv_i2c_reads(aw_haptic, (AW8671X_REG_RTPDATA + 1),
			       &reg_array[AW8671X_REG_RTPDATA + 1],
			       (AW8671X_REG_RAMDATA - AW8671X_REG_RTPDATA - 1));
	for (i = AW8671X_REG_RAMDATA + 1; i <= AW8671X_REG_ANACFG14; i++)
		haptic_hv_i2c_reads(aw_haptic, (AW8671X_REG_RAMDATA + 1),
			      &reg_array[AW8671X_REG_RAMDATA + 1],
			      (AW8671X_REG_ANACFG14 - AW8671X_REG_RAMDATA - 1));
	for (i = 0; i <= AW8671X_REG_ANACFG14; i++)
		if ((i != AW8671X_REG_RTPDATA) && (i != AW8671X_REG_RAMDATA))
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02X=0x%02X\n", i, reg_array[i]);
	return len;
}

static void aw8671x_offset_cali(struct aw_haptic *aw_haptic)
{

}

static void aw8671x_trig_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter!");
	if (aw_haptic->info.is_enabled_one_wire) {
		aw_info("one wire is enabled!");
		aw8671x_one_wire_init(aw_haptic);
	} else {
		aw8671x_trig1_param_init(aw_haptic);
		aw8671x_trig1_param_config(aw_haptic);
	}
	if (aw_haptic->info.is_enabled_i2s && aw_haptic->i2s_config) {
		aw_info("i2s is enabled!");
		aw8671x_i2s_init(aw_haptic);
	} else {
		aw8671x_trig2_param_init(aw_haptic);
		aw8671x_trig3_param_init(aw_haptic);
		aw8671x_trig2_param_config(aw_haptic);
		aw8671x_trig3_param_config(aw_haptic);
	}
}

#ifdef AW_CHECK_RAM_DATA
static int aw8671x_check_ram_data(struct aw_haptic *aw_haptic,
				  uint8_t *cont_data, uint8_t *ram_data,
				  uint32_t len)
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

static int aw8671x_container_update(struct aw_haptic *aw_haptic,
				    struct aw_haptic_container *awinic_cont)
{
	uint8_t reg_val[3] = {0};
	uint8_t rtp_addr[5] = {0};
#ifdef AW_CHECK_RAM_DATA
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};
#endif
	int i = 0;
	int ret = 0;
	int len = 0;
	uint32_t temp = 0;
	uint32_t shift = 0;
	uint32_t base_addr = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->ram.baseaddr_shift = 2;
	aw_haptic->ram.ram_shift = 4;
	/* RAMINIT Enable */
	aw8671x_ram_init(aw_haptic, true);
	/* Enter standby mode */
	aw8671x_stop(aw_haptic);
	/* base addr */
	shift = aw_haptic->ram.baseaddr_shift;
	aw_haptic->ram.base_addr =
			    (uint32_t)((awinic_cont->data[0 + shift] << 8) |
			    (awinic_cont->data[1 + shift]));
	base_addr = aw_haptic->ram.base_addr;
	/* ADDRH */
	rtp_addr[0] = awinic_cont->data[0 + shift];
	/* ADDRL */
	rtp_addr[1] = awinic_cont->data[1 + shift];
	/* FIFO AEH FIFO AFH */
	rtp_addr[2] = (uint8_t)(AW8671X_FIFO_AE_ADDR_H(base_addr) |
				AW8671X_FIFO_AF_ADDR_H(base_addr));
	/* FIFO AEL */
	rtp_addr[3] = (uint8_t)(AW8671X_FIFO_AE_ADDR_L(base_addr));
	/* FIFO AFL */
	rtp_addr[4] = (uint8_t)(AW8671X_FIFO_AF_ADDR_L(base_addr));
	aw_info("base_addr = %d", base_addr);
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_RTPCFG1, rtp_addr,
			     AW_I2C_BYTE_FIVE);
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_RTPCFG3, reg_val,
			    AW_I2C_BYTE_THREE);
	temp = ((reg_val[0] & AW8671X_BIT_RTPCFG3_FIFO_AFH) << 24) |
	       ((reg_val[0] & AW8671X_BIT_RTPCFG3_FIFO_AEH) << 4) | reg_val[1];
	aw_info("almost_empty_threshold = %d", (uint16_t)temp);
	temp = temp | (reg_val[2] << 16);
	aw_info("almost_full_threshold = %d", temp >> 16);
	/* ram */
	aw8671x_set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;

		haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_RAMDATA,
				     &awinic_cont->data[i], len);
		i += len;
	}
#ifdef AW_CHECK_RAM_DATA
	aw8671x_set_ram_addr(aw_haptic);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_RD_BUFFER_SIZE;

		haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_RAMDATA,
				    ram_data, len);
		ret = aw8671x_check_ram_data(aw_haptic, &awinic_cont->data[i],
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
	aw8671x_ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	return ret;
}

static uint64_t aw8671x_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	uint64_t theory_time = 0;

	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_SYSCTRL2, &reg_val,
			    AW_I2C_BYTE_ONE);
	fre_val = reg_val & AW8671X_BIT_SYSCTRL2_WAVDAT_MODE;
	if (fre_val == AW8671X_BIT_SYSCTRL2_RATE_24K)
		theory_time = (aw_haptic->rtp_len / 24000) * 1000000;	/*24K */
	else if (fre_val == AW8671X_BIT_SYSCTRL2_RATE_48K)
		theory_time = (aw_haptic->rtp_len / 48000) * 1000000;	/*48K */
	else
		theory_time = (aw_haptic->rtp_len / 12000) * 1000000;	/*12K */
	aw_info("microsecond:%llu  theory_time = %llu",
		aw_haptic->microsecond, theory_time);
	return theory_time;
}

static void aw8671x_parse_dt(struct device *dev, struct aw_haptic *aw_haptic,
			    struct device_node *np)
{
	uint8_t duration_time[3];
	uint8_t trig_config_temp[24];
	uint32_t val = 0;

	val = of_property_read_u8(np, "aw8671x_gain_bypass",
				  &aw_haptic->info.gain_bypass);
	if (val != 0)
		aw_info("aw8671x_gain_bypass not found");
	val = of_property_read_u8(np, "aw8671x_brk_bst_md",
				  &aw_haptic->info.brk_bst_md);
	if (val != 0)
		aw_info("aw8671x_brk_bst_md not found");
	val = of_property_read_u32(np, "f0_pre", &aw_haptic->info.f0_pre);
	if (val != 0)
		aw_info("f0_pre not found");
	val = of_property_read_u8(np, "f0_cali_percent",
				  &aw_haptic->info.f0_cali_percent);
	if (val != 0)
		aw_info("f0_cali_percent not found");

	val = of_property_read_u8(np, "aw8671x_cont_drv1_lvl",
				  &aw_haptic->info.cont_drv1_lvl);
	if (val != 0)
		aw_info("aw8671x_cont_drv1_lvl not found");
	val = of_property_read_u32(np, "aw8671x_cont_lra_vrms",
				   &aw_haptic->info.cont_lra_vrms);
	if (val != 0)
		aw_info("aw8671x_cont_lra_vrms not found");
	val = of_property_read_u8(np, "aw8671x_cont_drv1_time",
				  &aw_haptic->info.cont_drv1_time);
	if (val != 0)
		aw_info("aw8671x_cont_drv1_time not found");
	val = of_property_read_u8(np, "aw8671x_cont_drv2_time",
				  &aw_haptic->info.cont_drv2_time);
	if (val != 0)
		aw_info("aw8671x_cont_drv2_time not found");
	val = of_property_read_u8(np, "aw8671x_cont_wait_num",
				  &aw_haptic->info.cont_wait_num);
	if (val != 0)
		aw_info("aw8671x_cont_wait_num not found");
	val = of_property_read_u8(np, "aw8671x_cont_bst_brk_gain",
				  &aw_haptic->info.cont_bst_brk_gain);
	if (val != 0)
		aw_info("aw8671x_cont_bst_brk_gain not found");
	val = of_property_read_u8(np, "aw8671x_cont_brk_gain",
				  &aw_haptic->info.cont_brk_gain);
	if (val != 0)
		aw_info("aw8671x_cont_brk_gain not found");
	val = of_property_read_u8(np, "aw8671x_cont_tset",
				  &aw_haptic->info.cont_tset);
	if (val != 0)
		aw_info("aw8671x_cont_tset not found");
	val = of_property_read_u8(np, "aw8671x_cont_bemf_set",
				  &aw_haptic->info.cont_bemf_set);
	if (val != 0)
		aw_info("aw8671x_cont_bemf_set not found");
	val = of_property_read_u8(np, "aw8671x_d2s_gain",
				  &aw_haptic->info.d2s_gain);
	if (val != 0)
		aw_info("aw8671x_d2s_gain not found");
	val = of_property_read_u8(np, "aw8671x_cont_brk_time",
				  &aw_haptic->info.cont_brk_time);
	if (val != 0)
		aw_info("aw8671x_cont_brk_time not found");
	val = of_property_read_u8(np, "aw8671x_cont_track_margin",
				  &aw_haptic->info.cont_track_margin);
	if (val != 0)
		aw_info("aw8671x_cont_track_margin not found");
	aw_haptic->info.is_enabled_track_en =
		of_property_read_bool(np, "aw8671x_is_enabled_track_en");
	aw_info("aw_haptic->info.is_enabled_track_en = %d",
		aw_haptic->info.is_enabled_track_en);
	aw_haptic->info.is_enabled_auto_bst =
		of_property_read_bool(np, "aw8671x_is_enabled_auto_bst");
	aw_info("aw_haptic->info.is_enabled_auto_bst = %d",
		aw_haptic->info.is_enabled_auto_bst);
	aw_haptic->info.is_enabled_i2s =
			of_property_read_bool(np, "aw8671x_is_enabled_i2s");
	aw_info("aw_haptic->info.is_enabled_i2s = %d",
		aw_haptic->info.is_enabled_i2s);
	aw_haptic->info.is_enabled_one_wire = of_property_read_bool(np,
						 "aw8671x_is_enabled_one_wire");
	aw_info("aw_haptic->info.is_enabled_one_wire = %d",
		aw_haptic->info.is_enabled_one_wire);
	val = of_property_read_u8_array(np, "aw8671x_duration_time",
					duration_time,
					ARRAY_SIZE(duration_time));
	if (val != 0)
		aw_info("aw8671x_duration_time not found");
	else
		memcpy(aw_haptic->info.duration_time, duration_time,
		       sizeof(duration_time));
	val = of_property_read_u32(np, "aw8671x_bst_vol_default",
				   &aw_haptic->info.bst_vol_default);
	if (val != 0)
		aw_info("aw8671x_bst_vol_default not found");
	val = of_property_read_u8_array(np, "aw8671x_trig_config",
					trig_config_temp,
					ARRAY_SIZE(trig_config_temp));
	if (val != 0)
		aw_info("aw8671x_trig_config not found");
	else
		memcpy(aw_haptic->info.trig_cfg, trig_config_temp,
		       sizeof(trig_config_temp));
	aw_dbg("aw_haptic->info.brk_bst_md: %d", aw_haptic->info.brk_bst_md);
	aw_dbg("aw_haptic->info.bst_vol_default: %d",
		aw_haptic->info.bst_vol_default);
}

static void aw8671x_misc_para_init(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[8] = {0};
	uint32_t drv2_lvl = 0;

	/* Set I2C broadcast addr */
	reg_val[0] = (uint8_t)aw_haptic->i2c->addr;
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_SYSCTRL4, reg_val,
			     AW_I2C_BYTE_ONE);
	/* Cont drv2 lvl */
	drv2_lvl = AW8671X_DRV2_LVL_FARMULA(aw_haptic->info.f0_pre,
					    aw_haptic->info.cont_lra_vrms);
	if (drv2_lvl > AW_DRV2_LVL_MAX)
		aw_haptic->info.cont_drv2_lvl = AW_DRV2_LVL_MAX;
	else
		aw_haptic->info.cont_drv2_lvl = (uint8_t)drv2_lvl;
	/* Get vmax */
	if (aw_haptic->info.bst_vol_default > 0)
		aw_haptic->vmax = aw_haptic->info.bst_vol_default;
	/* Get gain */
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_PLAYCFG2, reg_val,
			    AW_I2C_BYTE_ONE);
	aw_haptic->gain = reg_val[0];
	/* Get wave_seq */
	haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_WAVCFG1, reg_val,
			    AW_I2C_BYTE_EIGHT);
	aw_haptic->index = reg_val[0];
	memcpy(aw_haptic->seq, reg_val, AW_SEQUENCER_SIZE);
	/* Set gain_bypass */
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_SYSCTRL3,
				 AW8671X_BIT_SYSCTRL3_GAIN_BYPASS_MASK,
				 aw_haptic->info.gain_bypass << 1);

	/* brk_bst_md */
	if (!aw_haptic->info.brk_bst_md)
		aw_err("aw_haptic->info.brk_bst_md = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_CONTCFG1,
				 AW8671X_BIT_CONTCFG1_BRK_BST_MD,
				 (aw_haptic->info.brk_bst_md << 6));

	/* d2s_gain */
	if (!aw_haptic->info.d2s_gain)
		aw_err("aw_haptic->info.d2s_gain = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_DETCFG3,
				 AW8671X_BIT_DETCFG3_D2S_GAIN_MASK,
				 aw_haptic->info.d2s_gain);

	/* cont_tset */
	if (!aw_haptic->info.cont_tset)
		aw_err("aw_haptic->info.cont_tset = 0!");
	/* cont_bemf_set */
	if (!aw_haptic->info.cont_bemf_set)
		aw_err("aw_haptic->info.cont_bemf_set = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_CONTCFG13,
				 (AW8671X_BIT_CONTCFG13_TSET_MASK &
				  AW8671X_BIT_CONTCFG13_BEME_SET_MASK),
				 (aw_haptic->info.cont_tset << 4 |
				  aw_haptic->info.cont_bemf_set));

	/* cont_brk_time */
	if (!aw_haptic->info.cont_brk_time)
		aw_err("aw_haptic->info.cont_brk_time = 0!");
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);

	/* cont_bst_brk_gain */
	if (!aw_haptic->info.cont_bst_brk_gain)
		aw_err("aw_haptic->info.cont_bst_brk_gain = 0");
	/* cont_brk_gain */
	if (!aw_haptic->info.cont_brk_gain)
		aw_err("aw_haptic->info.cont_brk_gain = 0!");
	haptic_hv_i2c_write_bits(aw_haptic, AW8671X_REG_CONTCFG5,
				 (AW8671X_BIT_CONTCFG5_BST_BRK_GAIN_MASK &
				  AW8671X_BIT_CONTCFG5_BRK_GAIN_MASK),
				 (aw_haptic->info.cont_bst_brk_gain << 4 |
				  aw_haptic->info.cont_brk_gain));
	aw8671x_protect_config(aw_haptic, AW8671X_BIT_PWMCFG4_PRTIME_DEFAULT_VALUE,
			       AW8671X_BIT_PWMCFG3_PRLVL_DEFAULT_VALUE);
}

static ssize_t aw8671x_cont_wait_num_show(struct device *dev,
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

static ssize_t aw8671x_cont_wait_num_store(struct device *dev,
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
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG4,
			     &aw_haptic->info.cont_wait_num, AW_I2C_BYTE_ONE);

	return count;
}

static ssize_t aw8671x_cont_drv_lvl_show(struct device *dev,
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

static ssize_t aw8671x_cont_drv_lvl_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	uint8_t reg_array[2] = {0};
	uint32_t databuf[2] = {0};
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_lvl = databuf[0];
		aw_haptic->info.cont_drv2_lvl = databuf[1];
		haptic_hv_i2c_reads(aw_haptic, AW8671X_REG_CONTCFG6, reg_array,
				    AW_I2C_BYTE_ONE);
		reg_array[0] &= AW8671X_BIT_CONTCFG6_DRV1_LVL_MASK;
		reg_array[0] |= aw_haptic->info.cont_drv1_lvl;
		reg_array[1] = aw_haptic->info.cont_drv2_lvl;
		haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG6, reg_array,
				     AW_I2C_BYTE_TWO);
	}
	return count;
}

static ssize_t aw8671x_cont_drv_time_show(struct device *dev,
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

static ssize_t aw8671x_cont_drv_time_store(struct device *dev,
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
		haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG8, reg_array,
				     AW_I2C_BYTE_TWO);
	}
	return count;
}

static ssize_t aw8671x_cont_brk_time_show(struct device *dev,
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

static ssize_t aw8671x_cont_brk_time_store(struct device *dev,
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
	haptic_hv_i2c_writes(aw_haptic, AW8671X_REG_CONTCFG10,
			     &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);
	return count;
}

static ssize_t aw8671x_trig_show(struct device *dev,
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

static ssize_t aw8671x_trig_store(struct device *dev,
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
			aw8671x_trig1_param_config(aw_haptic);
			break;
		case 1:
			aw8671x_trig2_param_config(aw_haptic);
			break;
		case 2:
			aw8671x_trig3_param_config(aw_haptic);
			break;
		}
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static DEVICE_ATTR(cont_wait_num, S_IWUSR | S_IRUGO, aw8671x_cont_wait_num_show,
		   aw8671x_cont_wait_num_store);
static DEVICE_ATTR(cont_drv_lvl, S_IWUSR | S_IRUGO, aw8671x_cont_drv_lvl_show,
		   aw8671x_cont_drv_lvl_store);
static DEVICE_ATTR(cont_drv_time, S_IWUSR | S_IRUGO, aw8671x_cont_drv_time_show,
		   aw8671x_cont_drv_time_store);
static DEVICE_ATTR(cont_brk_time, S_IWUSR | S_IRUGO, aw8671x_cont_brk_time_show,
		   aw8671x_cont_brk_time_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw8671x_trig_show,
		   aw8671x_trig_store);

static struct attribute *aw8671x_vibrator_attributes[] = {
	&dev_attr_cont_wait_num.attr,
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	&dev_attr_cont_brk_time.attr,
	&dev_attr_trig.attr,
	NULL
};

static struct attribute_group aw8671x_vibrator_attribute_group = {
	.attrs = aw8671x_vibrator_attributes
};

static int aw8671x_creat_node(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &aw8671x_vibrator_attribute_group);
	if (ret < 0) {
		aw_err("error create aw8671x sysfs attr files");
		return ret;
	}
	return 0;
}

struct aw_haptic_func aw8671x_func_list = {
	.play_stop = aw8671x_stop,
	.ram_init = aw8671x_ram_init,
	.get_vbat = aw8671x_get_vbat,
	.creat_node = aw8671x_creat_node,
	.get_f0 = aw8671x_get_f0,
	.ram_get_f0 = aw8671x_ram_get_f0,
	.cont_config = aw8671x_cont_config,
	.offset_cali = aw8671x_offset_cali,
	.check_qualify = aw8671x_check_qualify,
	.get_irq_state = aw8671x_get_irq_state,
	.judge_rtp_going = aw8671x_judge_rtp_going,
	.set_bst_peak_cur = aw8671x_set_bst_peak_cur,
	.get_theory_time = aw8671x_get_theory_time,
	.get_lra_resistance = aw8671x_get_lra_resistance,
	.set_pwm = aw8671x_set_pwm,
	.play_mode = aw8671x_play_mode,
	.set_bst_vol = aw8671x_set_bst_vol,
	.interrupt_setup = aw8671x_interrupt_setup,
	.set_repeat_seq = aw8671x_set_repeat_seq,
	.auto_bst_enable = aw8671x_auto_bst_enable,
	.vbat_mode_config = aw8671x_vbat_mode_config,
	.set_wav_seq = aw8671x_set_wav_seq,
	.set_wav_loop = aw8671x_set_wav_loop,
	.set_ram_addr = aw8671x_set_ram_addr,
	.set_rtp_data = aw8671x_set_rtp_data,
	.container_update = aw8671x_container_update,
	.protect_config = aw8671x_protect_config,
	.parse_dt = aw8671x_parse_dt,
	.trig_init = aw8671x_trig_init,
	.irq_clear = aw8671x_irq_clear,
	.get_wav_loop = aw8671x_get_wav_loop,
	.play_go = aw8671x_play_go,
	.misc_para_init = aw8671x_misc_para_init,
	.set_rtp_aei = aw8671x_set_rtp_aei,
	.set_gain = aw8671x_set_gain,
	.upload_lra = aw8671x_upload_lra,
	.bst_mode_config = aw8671x_bst_mode_config,
	.get_reg = aw8671x_get_reg,
	.get_prctmode = aw8671x_get_prctmode,
	.get_trim_lra = aw8671x_get_trim_lra,
	.get_ram_data = aw8671x_get_ram_data,
	.get_first_wave_addr = aw8671x_get_first_wave_addr,
	.get_glb_state = aw8671x_get_glb_state,
	.get_osc_status = aw8671x_get_osc_status,
	.rtp_get_fifo_afs = aw8671x_rtp_get_fifo_afs,
	.rtp_get_fifo_aes = aw8671x_rtp_get_fifo_aes,
	.get_wav_seq = aw8671x_get_wav_seq,
#ifdef AW_SND_SOC_CODEC
	.snd_soc_init = aw8671x_snd_soc_init,
#endif
};
