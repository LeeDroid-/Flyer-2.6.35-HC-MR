/* linux/arch/arm/mach-msm/board-flyer-audio.c
 *
 * Copyright (C) 2010-2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/android_pmem.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/dal.h>
#include "board-flyer.h"
#include <mach/qdsp5v2/snddev_icodec.h>
#include <mach/qdsp5v2/snddev_ecodec.h>
#include <mach/qdsp5v2/audio_def.h>
#include <mach/qdsp5v2/voice.h>
#include <mach/htc_acoustic_7x30.h>
#include <mach/htc_acdb_7x30.h>
#include <linux/spi/spi_aic3254.h>
#include <mach/qdsp5v2/snddev_mi2s.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>

static struct mutex bt_sco_lock;
static struct mutex mic_lock;
static int curr_rx_mode;
static int board_ver;
static int hsed_mic_en;
static atomic_t aic3254_ctl = ATOMIC_INIT(0);
void flyer_back_mic_enable(int);
void flyer_usb_headset_on(int en);
#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)
#define BIT_USB_HS	(1 << 5)

#define FLYER_ACDB_RADIO_BUFFER_SIZE (1024 * 3072)

static struct q5v2_hw_info q5v2_audio_hw[Q5V2_HW_COUNT] = {
	[Q5V2_HW_HANDSET] = {
		.max_gain[VOC_NB_INDEX] = 400,
		.min_gain[VOC_NB_INDEX] = -1600,
		.max_gain[VOC_WB_INDEX] = 400,
		.min_gain[VOC_WB_INDEX] = -1600,
	},
	[Q5V2_HW_HEADSET] = {
		.max_gain[VOC_NB_INDEX] = 900,
		.min_gain[VOC_NB_INDEX] = -1100,
		.max_gain[VOC_WB_INDEX] = 900,
		.min_gain[VOC_WB_INDEX] = -1100,
	},
	[Q5V2_HW_SPEAKER] = {
		.max_gain[VOC_NB_INDEX] = 200,
		.min_gain[VOC_NB_INDEX] = -1300,
		.max_gain[VOC_WB_INDEX] = 200,
		.min_gain[VOC_WB_INDEX] = -1300,
	},
	[Q5V2_HW_BT_SCO] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = -1500,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = -1500,
	},
	[Q5V2_HW_TTY] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = 0,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = 0,
	},
	[Q5V2_HW_HS_SPKR] = {
		.max_gain[VOC_NB_INDEX] = -500,
		.min_gain[VOC_NB_INDEX] = -2000,
		.max_gain[VOC_WB_INDEX] = -500,
		.min_gain[VOC_WB_INDEX] = -2000,
	},
	[Q5V2_HW_USB_HS] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
	[Q5V2_HW_HAC] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
};

static unsigned aux_pcm_gpio_off[] = {
	PCOM_GPIO_CFG(FLYER_GPIO_BT_PCM_OUT, 0, GPIO_OUTPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_GPIO_BT_PCM_IN, 0, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_GPIO_BT_PCM_SYNC, 0, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_GPIO_BT_PCM_CLK, 0, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
};

static unsigned aux_pcm_gpio_on[] = {
	PCOM_GPIO_CFG(FLYER_GPIO_BT_PCM_OUT, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_GPIO_BT_PCM_IN, 1, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_GPIO_BT_PCM_SYNC, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_GPIO_BT_PCM_CLK, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
};

static unsigned mi2s_clk_gpios_on[] = {
	PCOM_GPIO_CFG(FLYER_WCF_I2S_CLK_XC, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCF_I2S_WS_XC, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCA_MCLK_XC, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static unsigned mi2s_clk_gpios_off[] = {
	PCOM_GPIO_CFG(FLYER_WCF_I2S_CLK_XC, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCF_I2S_WS_XC, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCA_MCLK_XC, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
};

static unsigned mi2s_rx_data_lines_gpios_on[] = {
	PCOM_GPIO_CFG(FLYER_WCA_DATA_SD0_XC, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCA_DATA_SD1_XC, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCA_DATA_SD2_XC, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCF_I2S_DATA_XC, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static unsigned mi2s_rx_data_lines_gpios_off[] = {
	PCOM_GPIO_CFG(FLYER_WCA_DATA_SD0_XC, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCA_DATA_SD1_XC, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCA_DATA_SD2_XC, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(FLYER_WCF_I2S_DATA_XC, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
};


void flyer_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		msleep(80);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_SPKR_EN), 1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_SPKL_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_SPEAKER;
	} else {
		msleep(10);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_SPKR_EN), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_SPKL_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_SPEAKER;
	}
}

void flyer_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		msleep(80);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_HEADSET;
	} else {
		msleep(10);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_HEADSET;
	}
}

void flyer_snddev_hs_spk_pamp_on(int en)
{
	flyer_snddev_poweramp_on(en);
	flyer_snddev_hsed_pamp_on(en);
}

void flyer_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
#if 0
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_EP_EN), 1);
		mdelay(5);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_RECEIVER;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_EP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_RECEIVER;
	}
#endif
}

void flyer_snddev_bt_sco_pamp_on(int en)
{
	static int bt_sco_refcount;
	pr_aud_info("%s %d\n", __func__, en);
	mutex_lock(&bt_sco_lock);
	if (en) {
		if (++bt_sco_refcount == 1) {
			config_gpio_table(aux_pcm_gpio_on,
					ARRAY_SIZE(aux_pcm_gpio_on));
		}
	} else {
		if (--bt_sco_refcount == 0) {
			config_gpio_table(aux_pcm_gpio_off,
					ARRAY_SIZE(aux_pcm_gpio_off));
			gpio_set_value(FLYER_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(FLYER_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(FLYER_GPIO_BT_PCM_CLK, 0);
		}
	}
	mutex_unlock(&bt_sco_lock);
}

void flyer_xd03mic_enable(int en)
{
	struct vreg *vreg_l2;
	int ret = 0;
	pr_aud_info("%s %d\n", __func__, en);
	if (hsed_mic_en && !en) {
		pr_aud_info("ignore disable mic-bias\n");
		return;
	}
	vreg_l2 = vreg_get(NULL, "xo_out");
	if (IS_ERR(vreg_l2)) {
		pr_aud_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "xo_out", PTR_ERR(vreg_l2));
		return;
	}

	if (en)
		ret = vreg_enable(vreg_l2);
	else
		ret = vreg_disable(vreg_l2);

	if (ret)
		pr_aud_err("%s: %d failed (%d)\n", __func__, en, ret);
}

/* power up internal/externnal mic shared GPIO */
void flyer_mic_enable(int en, int shift)
{
	static int flag = 0;
	pr_aud_info("%s: %d, shift %d\n", __func__, en, shift);
	mutex_lock(&mic_lock);
	if (en)
		flag |= 1 << shift;
	else
		flag &= ~(1 << shift);

	if (flag) {
		if (board_ver < 30)
			gpio_set_value(FLYER_AUD_MIC_BIAS, 1);
		else if (board_ver >=43)
			flyer_xd03mic_enable(en);
	} else {
		if (board_ver < 30)
			gpio_set_value(FLYER_AUD_MIC_BIAS, 0);
		else if (board_ver >=43)
			flyer_xd03mic_enable(en);
	}

	mutex_unlock(&mic_lock);
}

void hsed_mic_enable(int en, int shift) {
	pr_aud_info("%s: %d, shift %d\n", __func__, en, shift);
	hsed_mic_en = en;
	flyer_mic_enable(en, shift);
}

void flyer_snddev_imic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	flyer_mic_enable(en, 0);
	flyer_back_mic_enable(en);
}

void flyer_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		if (board_ver >= 30 && board_ver < 43)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_MICPATH_SEL_XC), 1);
		else if (board_ver < 30)
			gpio_set_value(FLYER_AUD_MICPATH_SEL, 1);
		else if (board_ver >=43)
			flyer_xd03mic_enable(en);
	} else
		if (board_ver >= 30 && board_ver < 43)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_MICPATH_SEL_XC), 0);
		else if (board_ver < 30)
			gpio_set_value(FLYER_AUD_MICPATH_SEL, 0);

}

void flyer_snddev_fmspk_pamp_on(int en)
{
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_SPKR_EN), 1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_SPKL_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_SPK;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_SPKR_EN), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_SPKL_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_SPK;
	}
}

void flyer_snddev_fmhs_pamp_on(int en)
{
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_HS;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_HS;
	}
}

int flyer_get_rx_vol(uint8_t hw, int network, int level)
{
	struct q5v2_hw_info *info;
	int vol, maxv, minv;

	info = &q5v2_audio_hw[hw];
	maxv = info->max_gain[network];
	minv = info->min_gain[network];
	vol = minv + ((maxv - minv) * level) / 100;
	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);
	return vol;
}

void flyer_rx_amp_enable(int en)
{
	if (curr_rx_mode != 0) {
		atomic_set(&aic3254_ctl, 1);
		pr_aud_info("%s: curr_rx_mode 0x%x, en %d\n",
			__func__, curr_rx_mode, en);
		if (curr_rx_mode & BIT_SPEAKER)
			flyer_snddev_poweramp_on(en);
		if (curr_rx_mode & BIT_HEADSET)
			flyer_snddev_hsed_pamp_on(en);
		if (curr_rx_mode & BIT_RECEIVER)
			flyer_snddev_receiver_pamp_on(en);
		if (curr_rx_mode & BIT_FM_SPK)
			flyer_snddev_fmspk_pamp_on(en);
		if (curr_rx_mode & BIT_FM_HS)
			flyer_snddev_fmhs_pamp_on(en);
		if (curr_rx_mode & BIT_USB_HS)
			flyer_usb_headset_on(en);
		atomic_set(&aic3254_ctl, 0);;
	}
}

uint32_t flyer_get_acdb_radio_buffer_size(void)
{
	return FLYER_ACDB_RADIO_BUFFER_SIZE;
}

int flyer_support_aic3254(void)
{
	return 1;
}

int flyer_support_back_mic(void)
{
	return 1;
}

void flyer_get_acoustic_tables(struct acoustic_tables *tb)
{
	bool xd03 = false;
	xd03 = gpio_get_value(PM8058_GPIO_PM_TO_SYS(FLYER_NC_PMIC13)) == 0 &&
		gpio_get_value(PM8058_GPIO_PM_TO_SYS(FLYER_NC_PMIC14)) == 1 && system_rev >= 3;
	strcpy(tb->adie, "AdieHWCodec.csv\0");

	if (board_ver < 30) {
		strcpy(tb->aic3254,
			"AIC3254_REG.csv\0");
                 /* aftet XD03 */
	} else if (board_ver >= 43) {
		strcpy(tb->aic3254,
			"AIC3254_REG_XD.csv\0");
			strcpy(tb->aic3254_dsp,
				"CodecDSPID_XD.txt\0");
		/* XC ~ XD02 */
	} else {
		strcpy(tb->aic3254,
			"AIC3254_REG_XC.csv\0");
			strcpy(tb->aic3254_dsp,
				"CodecDSPID_XC.txt\0");
	}
}

static struct acdb_ops acdb = {
	.get_acdb_radio_buffer_size = flyer_get_acdb_radio_buffer_size,
};

void flyer_back_mic_enable(int en)
{
        pr_aud_info("%s %d\n", __func__, en);
        if (en) {
		if (board_ver >= 30 && board_ver < 43)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_MICPATH_SEL_XC), 0);
		else if ((board_ver) >= 43)
			flyer_xd03mic_enable(en);
	}
}

void flyer_usb_headset_on(int en)
{
	pr_aud_info("%s %d\n",__func__, en);
	if (en)	{
		msleep(80);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_LO_EN_XD), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_USB_HS;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_AUD_LO_EN_XD), 0);
		msleep(10);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_USB_HS;
	}
}

int flyer_mi2s_clk_enable(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (board_ver < 30)
		return -1;
	if (en) {
		config_gpio_table(mi2s_clk_gpios_on,
				ARRAY_SIZE(mi2s_clk_gpios_on));
	} else {
		config_gpio_table(mi2s_clk_gpios_off,
				ARRAY_SIZE(mi2s_clk_gpios_off));
	}
	return 0;
}

int flyer_mi2s_data_enable(int en, u32 direction, u8 sd_line_mask)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (board_ver < 30)
		return -1;
	if (en) {
		config_gpio_table(mi2s_rx_data_lines_gpios_on,
				ARRAY_SIZE(mi2s_rx_data_lines_gpios_on));
	} else {
		config_gpio_table(mi2s_rx_data_lines_gpios_on,
				ARRAY_SIZE(mi2s_rx_data_lines_gpios_off));
	}
	return 0;
}

int flyer_support_receiver(void)
{
	return 0;
}

static struct q5v2audio_analog_ops ops = {
	.speaker_enable	= flyer_snddev_poweramp_on,
	.headset_enable	= flyer_snddev_hsed_pamp_on,
	.handset_enable	= flyer_snddev_receiver_pamp_on,
	.headset_speaker_enable	= flyer_snddev_hs_spk_pamp_on,
	.bt_sco_enable	= flyer_snddev_bt_sco_pamp_on,
	.int_mic_enable = flyer_snddev_imic_pamp_on,
	.ext_mic_enable = flyer_snddev_emic_pamp_on,
	.fm_headset_enable = flyer_snddev_fmhs_pamp_on,
	.fm_speaker_enable = flyer_snddev_fmspk_pamp_on,
	.usb_headset_enable = flyer_usb_headset_on
};

static struct q5v2audio_ecodec_ops eops = {
	.bt_sco_enable  = flyer_snddev_bt_sco_pamp_on,
};

static struct q5v2voice_ops vops = {
	.get_rx_vol = flyer_get_rx_vol,
};

static struct q5v2audio_mi2s_ops mi2sops = {
	.mi2s_clk_enable = flyer_mi2s_clk_enable,
	.mi2s_data_enable = flyer_mi2s_data_enable,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = hsed_mic_enable,
	.support_aic3254 = flyer_support_aic3254,
	.support_back_mic = flyer_support_back_mic,
	.enable_back_mic =  flyer_back_mic_enable,
	.support_receiver = flyer_support_receiver,
	.get_acoustic_tables = flyer_get_acoustic_tables,
};

static struct aic3254_ctl_ops cops = {
	.rx_amp_enable = flyer_rx_amp_enable,
	.tx_amp_enable = flyer_rx_amp_enable,
};

void __init flyer_audio_init(void)
{
	struct pm8058_gpio audio_pwr = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.vin_sel        = 6,
	};

	mutex_init(&bt_sco_lock);
	mutex_init(&mic_lock);
	board_ver = (system_rev + 1)* 10;
	if (gpio_get_value(PM8058_GPIO_PM_TO_SYS(FLYER_NC_PMIC13)) == 0 && system_rev == 3)
	board_ver += 3;
	pr_aud_info("%s board_ver = %d\n", __func__, board_ver);
#ifdef CONFIG_MSM7KV2_AUDIO
	htc_7x30_register_analog_ops(&ops);
	htc_7x30_register_ecodec_ops(&eops);
	htc_7x30_register_voice_ops(&vops);
	htc_7x30_register_mi2s_ops(&mi2sops);
	acoustic_register_ops(&acoustic);
	acdb_register_ops(&acdb);
#endif
        aic3254_register_ctl_ops(&cops);

	/* Init PMIC GPIO */
	pm8058_gpio_config(FLYER_AUD_SPKL_EN, &audio_pwr);
	pm8058_gpio_config(FLYER_AUD_SPKR_EN, &audio_pwr);
	pm8058_gpio_config(FLYER_AUD_HP_EN, &audio_pwr);
	if (board_ver >= 30 && board_ver < 43)
		pm8058_gpio_config(FLYER_AUD_MICPATH_SEL_XC, &audio_pwr);
	/* Rest AIC3254 */
	gpio_set_value(FLYER_AUD_CODEC_RST, 0);
	mdelay(1);
	gpio_set_value(FLYER_AUD_CODEC_RST, 1);
	audio_pwr.vin_sel = 2;
	pm8058_gpio_config(FLYER_AUD_LO_EN_XD, &audio_pwr);
	mutex_lock(&bt_sco_lock);
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(FLYER_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(FLYER_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(FLYER_GPIO_BT_PCM_CLK, 0);
	if (board_ver >=43)
		flyer_xd03mic_enable(0);
	mutex_unlock(&bt_sco_lock);
	hsed_mic_en = 0;
}
