/* linux/arch/arm/mach-msm/board-flyer.c
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
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/android_pmem.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/spi/spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <linux/akm8975.h>
#include <linux/bma150.h>
#include <linux/ntrig.h>
#include <linux/leds-pm8058.h>
#include <linux/proc_fs.h>
#include <linux/ds2746_battery.h>
#include <linux/cap_sense.h>
#include <linux/fb.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <mach/msm_flashlight.h>

#include <mach/system.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_serial_hs.h>
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
#include <mach/bcm_bt_lpm.h>
#endif

#include <mach/msm_fb.h>
#include <mach/htc_usb.h>
#include <mach/hardware.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/dma.h>
#include <mach/msm_iomap.h>
#include <mach/perflock.h>
#include <mach/msm_serial_debugger.h>
#include <mach/rpc_pmapp.h>
#include <mach/remote_spinlock.h>
#include <mach/msm_panel.h>
#include <mach/vreg.h>
#include <mach/atmega_microp.h>
#include <mach/htc_battery.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_microp.h>
#include <mach/htc_headset_pmic.h>
#include <mach/htc_headset_misc.h>
#include <mach/htc_mhl.h>

#include "board-flyer.h"
#include "devices.h"
#include "proc_comm.h"
#include "smd_private.h"
#include "spm.h"
#include "pm.h"
#include "socinfo.h"
#ifdef CONFIG_MSM_SSBI
#include <mach/msm_ssbi.h>
#endif

#define XC 2
#define XD 3

static unsigned engineerid;

static int flyer_get_PMIC_GPIO_INT(void)
{
	return PMIC_GPIO_INT;
}

static int get_thermal_id(void)
{
	return THERMAL_300_47_3440;
}

static int get_battery_id(void)
{
	if (system_rev >= XD)
		return -1; /* means need battery id detection */
	else
		return 3;
}

UINT32 flyer_battery_id_tbl[] =
{
	/* id resister range = [min, max)*/
	7000, 50000,	/* ID = 1 ATL  (22k) 7k ~ 50k */
	600000, -1,	/* ID = 2 LG  (>600k) */
	-1, /* end of table */
};


static void flyer_poweralg_config_init(struct poweralg_config_type *config)
{
	pr_info("batt: %s() is used\n",__func__);
	config->full_charging_mv = 4110;
	config->full_charging_ma = 150;
	config->full_pending_ma = 0;		/* disabled*/
	config->full_charging_timeout_sec = 30 * 60;
	config->voltage_recharge_mv = 0; /* disabled */
	config->capacity_recharge_p = 98;
	config->voltage_exit_full_mv = 3800;
	config->min_taper_current_mv = 0;	/* disabled */
	config->min_taper_current_ma = 0;	/* disabled */
	config->wait_votlage_statble_sec = 1 * 60;
	config->predict_timeout_sec = 10;
	/* TODO: doesn't be used. use program instead. (FAST_POLL/SLOW_POLL) */
	config->polling_time_in_charging_sec = 30;
	/* TODO: doesn't be used. use program instead. (FAST_POLL/SLOW_POLL) */
	config->polling_time_in_discharging_sec = 30;

	config->enable_full_calibration = TRUE;
	config->enable_weight_percentage = TRUE;
	config->software_charger_timeout_sec = 0;   	 /* disabled*/
	config->superchg_software_charger_timeout_sec = 16 * 60 * 60;	/* 16 hrs */
	config->charger_hw_safety_timer_watchdog_sec =  4 * 60 * 60;	/* 4 hrs */

	config->debug_disable_shutdown = FALSE;
	config->debug_fake_room_temp = FALSE;
	config->debug_disable_hw_timer = FALSE;
	config->debug_always_predict = FALSE;
	config->full_level = 0;
}

static int flyer_update_charging_protect_flag(int ibat_ma, int vbat_mv, int temp_01c, BOOL* chg_allowed, BOOL* hchg_allowed, BOOL *temp_fault)
{
	static int pState = 0;
	int old_pState = pState;
	/* pStates:
		0: initial (temp detection)
		1: temp < 0 degree c
		2: 0 <= temp <= 48 degree c
		3: 48 < temp <= 55 degree c
		4: 55 < temp
	*/
	enum {
		PSTAT_DETECT=0,
		PSTAT_LOW_STOP,
		PSTAT_NORMAL,
		PSTAT_LIMITED,
		PSTAT_HIGH_STOP
	};
	/* generally we assumed that pState implies last temp.
		it won't hold if temp changes faster than sample rate */

	/* step 1. check if change state condition is hit */
	/* pr_info("batt: %s(i=%d, v=%d, t=%d, %d, %d)\n",__func__, ibat_ma, vbat_mv, temp_01c, *chg_allowed, *hchg_allowed); */
	switch(pState) {
		default:
			pr_info("error: unexpected pState\n");
		case PSTAT_DETECT:
			if (temp_01c < 0)
				pState = PSTAT_LOW_STOP;
			if ((0 <= temp_01c) && (temp_01c <= 480))
				pState = PSTAT_NORMAL;
			if ((480 < temp_01c) && (temp_01c <= 550))
				pState = PSTAT_LIMITED;
			if (550 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			break;
		case PSTAT_LOW_STOP:
			if (30 <= temp_01c)
				pState = PSTAT_NORMAL;
			/* suppose never jump to LIMITED/HIGH_STOP from here */
			break;
		case PSTAT_NORMAL:
			if (temp_01c < 0)
				pState = PSTAT_LOW_STOP;
			else if (550 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			else if (480 < temp_01c) /* also implies t <= 550 */
				pState = PSTAT_LIMITED;
			break;
		case PSTAT_LIMITED:
			if (temp_01c < 470)
				pState = PSTAT_NORMAL;
			if (550 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			/* suppose never jump to LOW_STOP from here */
			break;
		case PSTAT_HIGH_STOP:
			if (temp_01c < 450)
				pState = PSTAT_NORMAL;
			else if ((temp_01c < 530) && (vbat_mv < 3800))
				pState = PSTAT_LIMITED;
			/* suppose never jump to LOW_STOP from here */
			break;
	}
	if (old_pState != pState)
		pr_info("batt: Protect pState changed from %d to %d\n", old_pState, pState);

	/* step 2. check state protect condition */
	/* chg_allowed = TRUE:only means it's allowed no matter it has charger.
		same as hchg_allowed. */
	switch(pState) {
		default:
		case PSTAT_DETECT:
			pr_info("batt: error: unexpected pState\n");
			break;
		case PSTAT_LOW_STOP:
			*chg_allowed = FALSE;
			*hchg_allowed = FALSE;
			break;
		case PSTAT_NORMAL:
			*chg_allowed = TRUE;
			*hchg_allowed = TRUE;
			break;
		case PSTAT_LIMITED:
			if (PSTAT_LIMITED != old_pState)
				*chg_allowed = TRUE;
			if (4000 < vbat_mv)
				*chg_allowed = FALSE;
			else if (vbat_mv < 3800)
				*chg_allowed = TRUE;
			*hchg_allowed = FALSE;
			break;
		case PSTAT_HIGH_STOP:
			*chg_allowed = FALSE;
			*hchg_allowed = FALSE;
			break;
	}

	/* update temp_fault */
	if (PSTAT_NORMAL == pState)
		*temp_fault = FALSE;
	else
		*temp_fault = TRUE;

	return pState;
}

static int is_support_super_charger(void)
{
	return TRUE;
}

static uint opt_disable_uart2;

/* default GPIO setting of Camera */
static int gCAM_RST_GPIO = FLYER_CAM_RST;
static int gCAM2_PWN_GPIO = FLYER_CAM2_PWD;
static int gCAM_GPIO_SEL = FLYER_CLK_SWITCH;

module_param_named(disable_uart2, opt_disable_uart2, uint, 0);

static const unsigned int get_flyer_gpio_usb_id_pin(void)
{
	return (system_rev >= XC) ? FLYER_GPIO_USB_ID_PIN_XC : FLYER_GPIO_USB_ID_PIN;
}

static uint32_t usb_ID_PIN_input_table[] = {
	PCOM_GPIO_CFG(FLYER_GPIO_USB_ID_PIN, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	PCOM_GPIO_CFG(FLYER_GPIO_USB_ID_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
};

static uint32_t usb_ID_PIN_ouput_table_XC[] = {
	PCOM_GPIO_CFG(FLYER_GPIO_USB_ID_PIN_XC, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
};

static uint32_t usb_ID_PIN_input_table_XC[] = {
	PCOM_GPIO_CFG(FLYER_GPIO_USB_ID_PIN_XC, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
};

void config_flyer_usb_id_gpios(bool output)
{
	if (system_rev >= XC) {
		if (output) {
			config_gpio_table(usb_ID_PIN_ouput_table_XC,
				ARRAY_SIZE(usb_ID_PIN_ouput_table_XC));
			gpio_set_value(FLYER_GPIO_USB_ID_PIN_XC, 1);
		} else
			config_gpio_table(usb_ID_PIN_input_table_XC,
				ARRAY_SIZE(usb_ID_PIN_input_table_XC));
	} else {
		if (output) {
			config_gpio_table(usb_ID_PIN_ouput_table,
				ARRAY_SIZE(usb_ID_PIN_ouput_table));
			gpio_set_value(FLYER_GPIO_USB_ID_PIN, 1);
		} else
			config_gpio_table(usb_ID_PIN_input_table,
				ARRAY_SIZE(usb_ID_PIN_input_table));
	}
}

#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	static int vbus_is_on;

	/* If VBUS is already on (or off), do nothing. */
	if (unlikely(on == vbus_is_on))
	return;

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
	.power_budget	= 200, /* FIXME: 390, */
};
#endif

#if defined(CONFIG_BATTERY_MSM8X60) && !defined(CONFIG_USB_EHCI_MSM)
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	if (init)
		msm_charger_register_vbus_sn(callback);
	else
		msm_charger_unregister_vbus_sn(callback);
	return 0;
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, def_vol);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage = 3400;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return vreg_set_level(vreg_3p3, mV);
}
#endif

#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct msm_otg_platform_data msm_otg_pdata = {
	/* if usb link is in sps there is no need for
	 * usb pclk as dayatona fabric clock will be
	 * used instead
	 */
	.pemp_level		= PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		= CDR_AUTO_RESET_DISABLE,
	.se1_gating		= SE1_GATING_DISABLE,
#ifdef CONFIG_USB_EHCI_MSM
	.vbus_power		= msm_hsusb_vbus_power,
#endif
#if defined(CONFIG_BATTERY_MSM8X60) && !defined(CONFIG_USB_EHCI_MSM)
	.pmic_notif_init	= msm_hsusb_pmic_notif_init,
#endif
	.ldo_init		= msm_hsusb_ldo_init,
	.ldo_enable		= msm_hsusb_ldo_enable,
	.ldo_set_voltage	= msm_hsusb_ldo_set_voltage,
#ifdef CONFIG_BATTERY_MSM8X60
	.chg_vbus_draw		= msm_charger_vbus_draw,
#endif

	.rpc_connect		= hsusb_rpc_connect,
	.core_clk		= 1,
	.drv_ampl		= HS_DRV_AMPLITUDE_DEFAULT,
	.phy_reset		= (void *) msm_hsusb_phy_reset,
	.usb_oc_pin		= PM8058_GPIO_PM_TO_SYS(FLYER_H2W_CABLE_IN1),
	.usb_oc_irq             = MSM_GPIO_TO_INT(
				PM8058_GPIO_PM_TO_SYS(FLYER_H2W_CABLE_IN1)),
};
#endif

#ifdef CONFIG_USB_ANDROID
static void flyer_disable_usb_charger(void)
{
	printk(KERN_INFO "%s\n", __func__);
	htc_battery_charger_disable();
}

static int phy_init_seq[] = { 0x06, 0x36, 0x0C, 0x31, 0x31, 0x32, 0x1, 0x0E, 0x1, 0x11, -1 };
#if 1
static uint32_t mhl_reset_pin_ouput_table[] = {
	GPIO_CFG(FLYER_GPIO_MHL_RESET_XC, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static uint32_t mhl_usb_switch_ouput_table[] = {
	GPIO_CFG(FLYER_GPIO_USB_MHL_SEL_XC, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static uint32_t mhl_power_ouput_table[] = {
	GPIO_CFG(FLYER_GPIO_MHL_POWER_XD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

void config_pyramid_mhl_gpios(void)
{
	if (system_rev >= XC) {
		config_gpio_table(mhl_usb_switch_ouput_table, ARRAY_SIZE(mhl_usb_switch_ouput_table));
		config_gpio_table(mhl_reset_pin_ouput_table, ARRAY_SIZE(mhl_reset_pin_ouput_table));
	}
	if (system_rev >= XD)
		config_gpio_table(mhl_power_ouput_table, ARRAY_SIZE(mhl_usb_switch_ouput_table));
}

static void flyer_mhl_usb_switch(bool mhl)
{
	printk(KERN_INFO "%s: %d\n", __func__, mhl);

	if (system_rev >= XC) {
		if (mhl) {
			if (system_rev >= XD) {
				gpio_set_value(FLYER_GPIO_MHL_POWER_XD, 1);
				mdelay(10);
			}
			pm8058_gpio_cfg(FLYER_UART_EN, PM_GPIO_DIR_OUT, 0, 0, PM_GPIO_PULL_NO,
					PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_UART_EN), 1);
			gpio_set_value(FLYER_GPIO_USB_MHL_SEL_XC, 1);
/*			gpio_set_value(FLYER_GPIO_MHL_RESET_XC, 1);*/
#ifdef CONFIG_MSM_HDMI_MHL
			mhl_device_wakeup();
#endif
		} else {
/*			gpio_set_value(FLYER_GPIO_MHL_RESET_XC, 0);*/
			pm8058_gpio_cfg(FLYER_UART_EN, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO, PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
			gpio_set_value(FLYER_GPIO_USB_MHL_SEL_XC, 0);
			if (system_rev >= XD) {
				gpio_set_value(FLYER_GPIO_MHL_POWER_XD, 0);
			}
		}
	}
}
#endif

static void flyer_host_usb_switch(int host)
{
	printk(KERN_INFO "%s: %d (rev:%d)\n", __func__, host, system_rev);

	if (system_rev < XC)
		return;

	if (host) {
		if (system_rev >= XD) {
			gpio_set_value(FLYER_GPIO_MHL_POWER_XD, 1);
			mdelay(10);
		}
		pm8058_gpio_cfg(FLYER_UART_EN, PM_GPIO_DIR_OUT, 0, 0, PM_GPIO_PULL_NO,
				PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_UART_EN), 1);
	} else {
		pm8058_gpio_cfg(FLYER_UART_EN, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO,
				PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
		if (system_rev >= XD)
			gpio_set_value(FLYER_GPIO_MHL_POWER_XD, 0);
	}
}

static uint32_t usb_gpio_id2_pin_table[] = {
	PCOM_GPIO_CFG(FLYER_GPIO_USB_ID2_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
};

static void flyer_config_9v_gpio(int input)
{
	printk(KERN_INFO "%s: %d\n", __func__, input);

	if (input)
		pm8058_gpio_cfg(FLYER_9V_AC_DETECT, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM_GPIO_VIN_L6, 0, PM_GPIO_FUNC_NORMAL, 0);
	else
		pm8058_gpio_cfg(FLYER_9V_AC_DETECT, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 0, PM_GPIO_PULL_DN, PM_GPIO_VIN_L6, 0, PM_GPIO_FUNC_NORMAL, 0);
}

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= phy_init_seq,
	.phy_reset		= (void *) msm_hsusb_phy_reset,
	.self_powered		= 1,
	.accessory_detect	= 2, /* detect by PMIC adc */
	.usb_mhl_switch		= flyer_mhl_usb_switch,
	.usb_id_pin_gpio	= FLYER_GPIO_USB_ID_PIN,
	.config_usb_id_gpios	= config_flyer_usb_id_gpios,
	.usb_id2_pin_gpio	= FLYER_GPIO_USB_ID2_PIN,
	.usb_host_switch	= flyer_host_usb_switch,
	.ac_9v_gpio             = PM8058_GPIO_PM_TO_SYS(FLYER_9V_AC_DETECT),
	.configure_ac_9v_gpio = flyer_config_9v_gpio,
	.disable_usb_charger	= flyer_disable_usb_charger,
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 2,
	.vendor		= "HTC",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x0bb4,
	.product_id		= 0x0ca9,
	.version		= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products 		= ARRAY_SIZE(usb_products),
	.products 		= usb_products,
	.num_functions		= ARRAY_SIZE(usb_functions_all),
	.functions		= usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name		= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

void flyer_add_usb_devices(void)
{
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	msm_hsusb_pdata.usb_id_pin_gpio = get_flyer_gpio_usb_id_pin();
	msm_hsusb_pdata.serial_number = board_serialno();
	android_usb_pdata.serial_number = board_serialno();
	config_flyer_usb_id_gpios(0);
	config_pyramid_mhl_gpios();
	config_gpio_table(usb_gpio_id2_pin_table, ARRAY_SIZE(usb_gpio_id2_pin_table));
#if defined(CONFIG_USB_OTG)
	msm_otg_pdata.idgnd_gpio = get_flyer_gpio_usb_id_pin();
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_hsusb_host.dev.platform_data = &msm_usb_host_pdata;

	platform_device_register(&msm_device_otg);
	platform_device_register(&msm_device_hsusb_host);
#endif
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;

	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
}
#endif

int flyer_pm8058_gpios_init(struct pm8058_chip *pm_chip)
{
	if (system_rev >= 2)
		pm8058_gpio_cfg(FLYER_CSA_INTz_XC, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);

	pm8058_gpio_cfg(FLYER_TP_RSTz, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_DN, PM_GPIO_VIN_BB, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8058_gpio_cfg(FLYER_SDMC_CD_N, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8058_gpio_cfg(FLYER_VOL_UP, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8058_gpio_cfg(FLYER_VOL_DN, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8058_gpio_cfg(FLYER_AUD_HP_DETz, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8058_gpio_cfg(FLYER_H2W_CABLE_IN1, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO, PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8058_gpio_cfg(FLYER_H2W_CABLE_IN2, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO, PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8058_gpio_cfg(FLYER_H2W_IO1_CLK, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO, PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
	pm8058_gpio_cfg(FLYER_H2W_IO2_DAT, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO, PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);
	if (system_rev >= 2)
		pm8058_gpio_cfg(FLYER_UART_EN, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO, PM_GPIO_VIN_S3, 0, PM_GPIO_FUNC_NORMAL, 0);

	if (system_rev >= 3){/* for Led XD board */
		pm8058_gpio_cfg(FLYER_PEN_LED3, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_NO, PM_GPIO_VIN_L6, 0, PM_GPIO_FUNC_NORMAL, 0);
	}

	pm8058_gpio_cfg(FLYER_9V_AC_DETECT, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 0, PM_GPIO_PULL_DN, PM_GPIO_VIN_L6, 0, PM_GPIO_FUNC_NORMAL, 0);
	return 0;
}

/* HTC_HEADSET_MICROP Driver */
static struct htc_headset_microp_platform_data htc_headset_microp_data = {
	.remote_int		= 1 << 13,
	.remote_irq		= MSM_uP_TO_INT(13),
	.remote_enable_pin	= 0,
	.adc_channel		= 0x01,
	.adc_remote		= {0, 33, 43, 73, 110, 172},
};

static struct platform_device htc_headset_microp = {
	.name	= "HTC_HEADSET_MICROP",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_microp_data,
	},
};

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data = {
	.hpin_gpio	= PM8058_GPIO_PM_TO_SYS(FLYER_AUD_HP_DETz),
	.hpin_irq	= MSM_GPIO_TO_INT(
			  PM8058_GPIO_PM_TO_SYS(FLYER_AUD_HP_DETz)),
};

static struct platform_device htc_headset_pmic = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data,
	},
};

/* HTC_HEADSET_MISC Driver */
static struct htc_headset_misc_platform_data htc_headset_misc_data = {
	.driver_flag		= DRIVER_HS_MISC_EXT_HP_DET,
	.ext_hpin_gpio		= PM8058_GPIO_PM_TO_SYS(FLYER_H2W_CABLE_IN1),
	.ext_accessory_type	= USB_AUDIO_OUT,
};

static struct platform_device htc_headset_misc = {
	.name	= "HTC_HEADSET_MISC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_misc_data,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
	&htc_headset_microp,
	&htc_headset_misc,
	&htc_headset_pmic,
	/* Please put the headset detection driver on the last */
};

static struct headset_adc_config htc_headset_mgr_config[] = {
	{
		.type = HEADSET_MIC,
		.adc_max = 1023,
		.adc_min = 512,
	},
	{
		.type = HEADSET_BEATS,
		.adc_max = 512,
		.adc_min = 200,
	},
	{
		.type = HEADSET_NO_MIC,
		.adc_max = 200,
		.adc_min = 0,
	},
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
	.headset_config_num	= ARRAY_SIZE(htc_headset_mgr_config),
	.headset_config		= htc_headset_mgr_config,
};

static int flyer_battery_charging_ctrl(enum batt_ctl_t ctl)
{
	int result = 0;

	switch (ctl) {
	case DISABLE:
		pr_info("batt: flyer charger OFF\n");
		if (system_rev >= XC)
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N_XC, 1);
		else
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N, 1);
		break;
	case ENABLE_SLOW_CHG:
		if (1 <= engineerid) {
			pr_info("batt: flyer charger ON (SLOW) - adapter mode\n");
			result = gpio_direction_output(FLYER_GPIO_ISET_XC, 1);
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N_XC, 0);
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_ADP_9V), 0);
		} else if (system_rev >= XC) {
			pr_info("batt: flyer charger ON (SLOW)\n");
			result = gpio_direction_output(FLYER_GPIO_ISET_XC, 0);
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N_XC, 0);
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_ADP_9V), 0);
		} else {
			pr_info("batt: flyer charger ON (SLOW)\n");
			result = gpio_direction_output(FLYER_GPIO_ISET, 0);
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N, 0);
			result = gpio_direction_output(FLYER_GPIO_ADP_9V, 0);
		}
		break;
	case ENABLE_FAST_CHG:
		pr_info("batt: flyer charger ON (FAST)\n");
		if (system_rev >= XC) {
			result = gpio_direction_output(FLYER_GPIO_ISET_XC, 1);
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N_XC, 0);
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_ADP_9V), 0);
		} else {
			result = gpio_direction_output(FLYER_GPIO_ISET, 1);
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N, 0);
			result = gpio_direction_output(FLYER_GPIO_ADP_9V, 0);
		}
		break;
	case ENABLE_SUPER_CHG:
		pr_info("batt: flyer charger ON (SUPER)\n");
		if (system_rev >= XC) {
			result = gpio_direction_output(FLYER_GPIO_ISET_XC, 1);
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N_XC, 0);
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(FLYER_ADP_9V), 1);
		} else {
			result = gpio_direction_output(FLYER_GPIO_ISET, 1);
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N, 0);
			result = gpio_direction_output(FLYER_GPIO_ADP_9V, 1);
		}
		break;
	case TOGGLE_CHARGER:
		pr_info("batt: flyer toggle charger(->OFF->ON)\n");
		if (system_rev >= XC)
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N_XC, 1);
		else
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N, 1);
		udelay(200);
		if (system_rev >= XC)
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N_XC, 0);
		else
			result = gpio_direction_output(FLYER_GPIO_MCHG_EN_N, 0);
		break;
	case ENABLE_MIN_TAPER: /* express only */
	case DISABLE_MIN_TAPER: /* express ondly */
	default:
		pr_info("%s: Not supported battery ctr(%d) called.!\n", __func__, ctl);
		result = -EINVAL;
		break;
	}

	return result;
}

static int flyer_battery_gpio_init(void)
{

	if (system_rev >= XC) {
		gpio_request(FLYER_GPIO_MCHG_EN_N_XC, "charger_en");
		gpio_request(FLYER_GPIO_ISET_XC, "charge_current");
		pm8058_gpio_cfg(FLYER_ADP_9V, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 1, PM_GPIO_PULL_NO,
		PM_GPIO_VIN_BB, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_NORMAL, 0);
	} else {
		gpio_request(FLYER_GPIO_MCHG_EN_N, "charger_en");
		gpio_request(FLYER_GPIO_ISET, "charge_current");
		gpio_request(FLYER_GPIO_ADP_9V, "super_charge_current");
	}
	return 0;
}


static struct htc_battery_platform_data htc_battery_pdev_data = {
	.func_show_batt_attr = htc_battery_show_attr,
	.func_is_support_super_charger = is_support_super_charger,
	.func_battery_charging_ctrl = flyer_battery_charging_ctrl,
	.func_battery_gpio_init = flyer_battery_gpio_init,
	.guage_driver = GUAGE_DS2746,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

/* battery parameters */
UINT32 m_parameter_unknown_3700mah[] = {
	/* capacity (in 0.01%) -> voltage (in mV)*/
	/* ex: 100% (in 0.01%) -> 4.132 (in mV) for the first two elements.*/
	10000, 4132, 9000 ,4104, 8000, 4019, 7000, 3949, 6000, 3884, 5000, 3827,
	4000, 3796, 3000, 3775, 2000, 3751, 1000, 3685, 0, 3572
};
UINT32 m_parameter_atl_3840mah[] = {
	10000, 4132, 9000 ,4088, 8000, 4007, 7000, 3939, 6000, 3874, 5000, 3821,
	4000, 3789, 3000, 3766, 2000, 3739, 1000, 3682, 0, 3575
};
UINT32 m_parameter_lg_3850mah[] = {
	10000, 4132, 9000 ,4104, 8000, 4019, 7000, 3949, 6000, 3884, 5000, 3827,
	4000, 3796, 3000, 3775, 2000, 3751, 1000, 3685, 0, 3572
};
UINT32 m_parameter_old_lg_3260mah[] =
{
	10000, 4135, 7500, 3960, 4700, 3800, 1700,
	3727, 900, 3674, 300, 3640, 0, 3575,
};

/* arrays' size listed below should be the same =
	num of batteryID + 1 (for unknown batt) */
/* note: flyer only has 2 battery ids, why ary size is 4:
	to compatible to old battery before XD device,
	we define ID=3 for old device battery param. */
static UINT32* m_param_tbl[] = {
	m_parameter_unknown_3700mah,
	m_parameter_atl_3840mah,
	m_parameter_lg_3850mah,
	m_parameter_old_lg_3260mah,
};
static UINT32 fl_25[] = {3700, 3840, 3850, 3260};
static UINT32 pd_m_coef[] = {14, 14, 14, 30};
static UINT32 pd_m_resl[] = {100, 100, 100, 100};
static UINT32 pd_t_coef[] = {70, 70, 70, 250};
static INT32 padc[] = {200, 200, 200, 200};
static INT32 pw[] = {5, 5, 5, 5};

/* arrays' size listed below should be the same =
	num of temp_index level
	(only one level for flyer) */
static UINT32* pd_m_coef_tbl[] = {pd_m_coef,};
static UINT32* pd_m_resl_tbl[] = {pd_m_resl,};
static UINT32 capacity_deduction_tbl_01p[] = {0,};

static struct battery_parameter flyer_battery_parameter = {
	.fl_25 = fl_25,
	.pd_m_coef_tbl = pd_m_coef_tbl,
	.pd_m_coef_tbl_boot = pd_m_coef_tbl,
	.pd_m_resl_tbl = pd_m_resl_tbl,
	.pd_m_resl_tbl_boot = pd_m_resl_tbl,
	.pd_t_coef = pd_t_coef,
	.padc = padc,
	.pw = pw,
	.capacity_deduction_tbl_01p = capacity_deduction_tbl_01p,
	.id_tbl = flyer_battery_id_tbl,
	.temp_index_tbl = NULL,
	.m_param_tbl = m_param_tbl,
	.m_param_tbl_size = sizeof(m_param_tbl)/sizeof(UINT32*),

	.voltage_adc_to_mv_coef = 244,
	.voltage_adc_to_mv_resl = 100,
	.current_adc_to_mv_coef = 625,
	.current_adc_to_mv_resl = 1580,
	.discharge_adc_to_mv_coef = 625,
	.discharge_adc_to_mv_resl = 1580,
	.acr_adc_to_mv_coef = 625,
	.acr_adc_to_mv_resl = 1580,
	.charge_counter_zero_base_mAh = 500,
	.id_adc_overflow = 3067,
	.id_adc_resl = 2047,
	.temp_adc_resl = 2047,
};

static ds2746_platform_data ds2746_pdev_data = {
	.func_get_thermal_id = get_thermal_id,
	.func_get_battery_id = get_battery_id,
	.func_poweralg_config_init = flyer_poweralg_config_init,
	.func_update_charging_protect_flag = flyer_update_charging_protect_flag,
	.r2_kohm = 300,
	.batt_param = &flyer_battery_parameter,
};

static struct platform_device ds2746_battery_pdev = {
	.name = "ds2746-battery",
	.id = -1,
	.dev = {
		.platform_data = &ds2746_pdev_data,
	},
};

static struct microp_function_config microp_functions[] = {
	{
		.name   = "microp_intrrupt",
		.category = MICROP_FUNCTION_INTR,
	},
	{
		.name   = "reset-int",
		.category = MICROP_FUNCTION_RESET_INT,
		.int_pin = 1 << 8,
	},
};

static int __capella_cm3602_power(int on)
{
	uint8_t data[3];
	int ret;

	data[0] = 0;
	data[1] = 0;
	data[2] = 0x10;

	printk(KERN_DEBUG "%s: :Pull the ALS_SHDN %s\n",
		__func__, (on) ? "LOW" : "HIGH");
	if (on) {
		ret = microp_i2c_write(MICROP_I2C_WCMD_GPO_LED_STATUS_EN,
			data, 3);
		if (ret)
			printk(KERN_WARNING "%s: Pull PD4 DOWN fail!\n",
				__func__);
	} else {
		ret = microp_i2c_write(MICROP_I2C_WCMD_GPO_LED_STATUS_DIS,
			data, 3);
		if (ret)
			printk(KERN_WARNING "%s: Pull PD4 UP fail!\n",
				__func__);
	}
	return 0;
}

static DEFINE_MUTEX(capella_cm3602_lock);
static int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	/* TODO eolsen Add Voltage reg control */
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
}

static struct microp_function_config microp_lightsensor_function = {
	.name = "light_sensor",
	.category = MICROP_FUNCTION_LSENSOR,
	.levels = { 5, 22, 41, 275, 366, 427, 488, 549,610, 4095 },
	.channel = 3,
	.int_pin = 1 << 9,
	.golden_adc = 0xB1,
	.ls_power = capella_cm3602_power,
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_lightsensor_function,
	.irq = MSM_uP_TO_INT(9),
};

static struct microp_led_config up_led_config[] = {
	{
		.name = "amber",
		.type = LED_RGB,
	},
	{
		.name = "green",
		.type = LED_RGB,
	},
	{
		.name = "button-backlight-portrait",
		.type = LED_PWM,
		.led_pin = 1 << 0,
		.init_value = 0xFF,
		.fade_time = 5,
	},
	{
		.name = "button-backlight-landscape",
		.type = LED_PWM,
		.led_pin = 1 << 1,
		.init_value = 0xFF,
		.fade_time = 5,
	},
};

static struct microp_led_config up_led_config_XD2[] = {
	{
		.name = "amber",
		.type = LED_RGB,
	},
	{
		.name = "green",
		.type = LED_RGB,
	},
	{
		.name = "button-backlight-portrait",
		.type = LED_PWM,
		.led_pin = 1 << 0,
		.fade_time = 5,
	},
	{
		.name = "button-backlight-landscape",
		.type = LED_PWM,
		.led_pin = 1 << 1,
		.fade_time = 5,
	},
};

static struct microp_led_platform_data microp_leds_data = {
	.num_leds	= ARRAY_SIZE(up_led_config),
	.led_config	= up_led_config,
};

static struct bma150_platform_data flyer_g_sensor_pdata = {
	.microp_new_cmd = 1,
	.chip_layout = 1,
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},
	},
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &flyer_g_sensor_pdata,
		},
	},
	{
		.name	= "HTC_HEADSET_MGR",
		.id	= -1,
		.dev	= {
			.platform_data	= &htc_headset_mgr_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions   = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = FLYER_GPIO_UP_RESET_N,
	.spi_devices = SPI_GSENSOR,
};

static struct gpio_led gpio_led_config[] = {
	{
		.name = "amber-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED1),
		.active_low = 1,
	},
	{
		.name = "green-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED2),
		.active_low = 1,
	},
	{
		.name = "blue-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED3),
		.active_low = 1,
	},
	{
		.name = "amber-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED4),
		.active_low = 1,
	},
	{
		.name = "green-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED5),
		.active_low = 1,
	},
	{
		.name = "blue-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED6),
		.active_low = 1,
	},
};

static struct gpio_led gpio_led_config_XC[] = {
	{
		.name = "amber-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED1),
		.active_low = 1,
	},
	{
		.name = "green-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED2),
		.active_low = 1,
	},
	{
		.name = "blue-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED3),
		.active_low = 1,
	},
	{
		.name = "amber-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED1),
		.active_low = 1,
	},
	{
		.name = "green-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED2),
		.active_low = 1,
	},
	{
		.name = "blue-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED3),
		.active_low = 1,
	},
	{
		.name = "green-camera",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_2ND_CAM_LED),
		.active_low = 0,
	},
};


static struct gpio_led gpio_led_config_XD[] = {
	{
		.name = "amber-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED1),
		.active_low = 1,
	},
	{
		.name = "green-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED2),
		.active_low = 1,
	},
	{
		.name = "blue-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED3),
		.active_low = 1,
	},
	{
		.name = "amber-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED1),
		.active_low = 1,
	},
	{
		.name = "green-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED2),
		.active_low = 1,
	},
	{
		.name = "blue-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED3),
		.active_low = 1,
	},
	{
		.name = "green-camera",
		.gpio = FLYER_2ND_CAM_LED_XD,
		.active_low = 0,
	},
};


static struct gpio_led gpio_led_config_XD2[] = {
	{
		.name = "amber-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED1),
		.active_low = 0,
	},
	{
		.name = "green-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED2),
		.active_low = 0,
	},
	{
		.name = "white-portrait",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED3),
		.active_low = 0,
	},
	{
		.name = "amber-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED1),
		.active_low = 0,
	},
	{
		.name = "green-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED2),
		.active_low = 0,
	},
	{
		.name = "white-landscape",
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED3),
		.active_low = 0,
	},
	{
		.name = "green-camera",
		.gpio = FLYER_2ND_CAM_LED_XD,
		.active_low = 0,
	},
};

static struct gpio_led_platform_data gpio_leds_data = {
	.num_leds = ARRAY_SIZE(gpio_led_config),
	.leds = gpio_led_config,
};

static struct platform_device gpio_leds = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &gpio_leds_data,
	},
};

/* Compass platform data */

static struct akm8975_platform_data compass_platform_data_XA_XB = {
	.layouts = FLYER_COMPASS_LAYOUTS_XA_XB,
	.irq_trigger = 1,
};

static struct akm8975_platform_data compass_platform_data_XC = {
	.layouts = FLYER_COMPASS_LAYOUTS_XC,
	.irq_trigger = 1,
};

static struct akm8975_platform_data compass_platform_data_XD = {
	.layouts = FLYER_COMPASS_LAYOUTS_XC,
	.irq_trigger = 1,
};

static struct akm8975_platform_data compass_platform_data_VER_A = {
	.layouts = FLYER_COMPASS_LAYOUTS_VER_A,
	.irq_trigger = 1,
};

#ifdef CONFIG_SENSORS_CAPSENSE
/* CapSense platform data */
static struct capsense_platform_data capsense_data = {
	.intr = PM8058_GPIO_PM_TO_SYS(FLYER_CSA_INTz_XC),
};

static struct i2c_board_info i2c_capsense_devices_XC[] = {
	{
		I2C_BOARD_INFO(CAPSENSE_NAME, 0x40 >> 1),
		.platform_data = &capsense_data,
		.irq = MSM_GPIO_TO_INT(
			PM8058_GPIO_PM_TO_SYS(FLYER_CSA_INTz_XC))
	},
};
#endif

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(FLYER_GPIO_UP_INT_N)
	},
};

static struct i2c_board_info i2c_compass_devices_XA_XB[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x1A >> 1),
		.platform_data = &compass_platform_data_XA_XB,
		.irq = MSM_GPIO_TO_INT(FLYER_GPIO_COMPASS_INT_XA_XB),
	},
};

static struct i2c_board_info i2c_compass_devices_XC[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x1A >> 1),
		.platform_data = &compass_platform_data_XC,
		.irq = MSM_GPIO_TO_INT(FLYER_GPIO_COMPASS_INT_XC),
	},
};

static struct i2c_board_info i2c_compass_devices_XD[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x1A >> 1),
		.platform_data = &compass_platform_data_XD,
		.irq = MSM_GPIO_TO_INT(
			PM8058_GPIO_PM_TO_SYS(FLYER_GPIO_COMPASS_INT_XD)),
	},
};

static struct i2c_board_info i2c_compass_devices_VER_A[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x1A >> 1),
		.platform_data = &compass_platform_data_VER_A,
		.irq = MSM_GPIO_TO_INT(
			PM8058_GPIO_PM_TO_SYS(FLYER_GPIO_COMPASS_INT_XD)),
	},
};

static struct vreg *vreg_marimba_1;
static struct vreg *vreg_marimba_2;

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}

out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
};

/*
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc;
	uint32_t irqcfg;
	const char *id = "FMPW";

	pdata->vreg_s2 = vreg_get(NULL, "s2");
	if (IS_ERR(pdata->vreg_s2)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(pdata->vreg_s2));
		return -1;
	}

	rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 1300);
	if (rc < 0) {
		printk(KERN_ERR "%s: voltage level vote failed (%d)\n",
			__func__, rc);
		return rc;
	}

	rc = vreg_enable(pdata->vreg_s2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		return rc;
	}

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_ON);
	if (rc < 0) {
		printk(KERN_ERR "%s: clock vote failed (%d)\n",
			__func__, rc);
		goto fm_clock_vote_fail;
	}
	irqcfg = PCOM_GPIO_CFG(147, 0, GPIO_INPUT, GPIO_NO_PULL,
					GPIO_2MA);
	rc = gpio_tlmm_config(irqcfg, GPIO_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
		rc = -EIO;
		goto fm_gpio_config_fail;

	}
	return 0;
fm_gpio_config_fail:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
				  PMAPP_CLOCK_VOTE_OFF);
fm_clock_vote_fail:
	vreg_disable(pdata->vreg_s2);
	return rc;

};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";
	uint32_t irqcfg = PCOM_GPIO_CFG(147, 0, GPIO_INPUT, GPIO_PULL_UP,
					GPIO_2MA);
	rc = gpio_tlmm_config(irqcfg, GPIO_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
	}
	rc = vreg_disable(pdata->vreg_s2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0)
		printk(KERN_ERR "%s: clock_vote return val: %d \n",
						__func__, rc);
	rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 0);
	if (rc < 0)
		printk(KERN_ERR "%s: vreg level vote return val: %d \n",
						__func__, rc);
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup =  fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(FLYER_GPIO_WIFI_IRQ),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
};
*/

/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
/*#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A*/
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

static const char *tsadc_id = "MADC";
static const char *vregs_tsadc_name[] = {
	"gp12",
	"s2",
};
static struct vreg *vregs_tsadc[ARRAY_SIZE(vregs_tsadc_name)];

static int marimba_tsadc_power(int vreg_on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
		if (!vregs_tsadc[i]) {
			printk(KERN_ERR "%s: vreg_get %s failed (%d)\n",
				__func__, vregs_tsadc_name[i], rc);
			goto vreg_fail;
		}

		rc = vreg_on ? vreg_enable(vregs_tsadc[i]) :
			  vreg_disable(vregs_tsadc[i]);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg %s %s failed (%d)\n",
				__func__, vregs_tsadc_name[i],
			       vreg_on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
	}
	/* vote for D0 buffer */
	rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
		vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
	if (rc)	{
		printk(KERN_ERR "%s: unable to %svote for d0 clk\n",
			__func__, vreg_on ? "" : "de-");
		goto do_vote_fail;
	}

	mdelay(5); /* ensure power is stable */

	return 0;

do_vote_fail:
vreg_fail:
	while (i)
		vreg_disable(vregs_tsadc[--i]);
	return rc;
}

static int marimba_tsadc_vote(int vote_on)
{
	int rc, level;

	level = vote_on ? 1300 : 0;

	rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, level);
	if (rc < 0)
		printk(KERN_ERR "%s: vreg level %s failed (%d)\n",
			__func__, vote_on ? "on" : "off", rc);

	return rc;
}

static int marimba_tsadc_init(void)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
		vregs_tsadc[i] = vreg_get(NULL, vregs_tsadc_name[i]);
		if (IS_ERR(vregs_tsadc[i])) {
			printk(KERN_ERR "%s: vreg get %s failed (%ld)\n",
			       __func__, vregs_tsadc_name[i],
			       PTR_ERR(vregs_tsadc[i]));
			rc = PTR_ERR(vregs_tsadc[i]);
			goto vreg_get_fail;
		}
	}

	return rc;

vreg_get_fail:
	while (i)
		vreg_put(vregs_tsadc[--i]);
	return rc;
}

static int marimba_tsadc_exit(void)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
		if (vregs_tsadc[i])
			vreg_put(vregs_tsadc[i]);
	}

	rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, 0);
	if (rc < 0)
		printk(KERN_ERR "%s: vreg level off failed (%d)\n",
			__func__, rc);

	return rc;
}

static struct marimba_tsadc_platform_data marimba_tsadc_pdata = {
	.marimba_tsadc_power = marimba_tsadc_power,
	.init		     =  marimba_tsadc_init,
	.exit		     =  marimba_tsadc_exit,
	.level_vote	     =  marimba_tsadc_vote,
	.tsadc_prechg_en = true,
	.setup = {
		.pen_irq_en	=	true,
		.tsadc_en	=	true,
	},
	.params2 = {
		.input_clk_khz		=	2400,
		.sample_prd		=	TSADC_CLK_3,
	},
	.params3 = {
		.prechg_time_nsecs	=	6400,
		.stable_time_nsecs	=	6400,
		.tsadc_test_mode	=	0,
	},
};

static struct vreg *vreg_codec_s4;
static int msm_marimba_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_codec_s4) {

		vreg_codec_s4 = vreg_get(NULL, "s4");

		if (IS_ERR(vreg_codec_s4)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_codec_s4));
			rc = PTR_ERR(vreg_codec_s4);
			goto  vreg_codec_s4_fail;
		}
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	} else {
		rc = vreg_disable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	}

vreg_codec_s4_fail:
	return rc;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power = msm_marimba_codec_power,
};

static struct marimba_platform_data marimba_pdata = {
	/*.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,*/
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	/*.fm = &marimba_fm_pdata,*/
	.tsadc = &marimba_tsadc_pdata,
	.codec = &mariba_codec_pdata,
};

static void __init msm7x30_init_marimba(void)
{
	vreg_marimba_1 = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	vreg_marimba_2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba_2)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_2));
		return;
	}
}
#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

#if 1
static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};
#endif

static struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

static struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

static struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0, 0, 0, 0,
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};
#if 1
static unsigned aux_pcm_gpio_on[] = {
	PCOM_GPIO_CFG(138, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* PCM_DOUT */
	PCOM_GPIO_CFG(139, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),   /* PCM_DIN  */
	PCOM_GPIO_CFG(140, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),   /* PCM_SYNC */
	PCOM_GPIO_CFG(141, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),   /* PCM_CLK  */
};
static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}
#endif
#endif /* CONFIG_MSM7KV2_AUDIO */

static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	}
};

static int flyer_ts_ntrig_power(int on)
{
	pr_info("[ts]%s(%d):\n", __func__, on);

	if (on) {
		gpio_set_value(FLYER_GPIO_TP_3V3_ENABLE, 1);
		mdelay(1);
		pm8058_gpio_cfg(FLYER_TP_ATT_PMIC,
						PM_GPIO_DIR_IN,
						0,
						0,
						PM_GPIO_PULL_UP_31P5,
						PM_GPIO_VIN_L6,
						0,
						PM_GPIO_FUNC_PAIRED,
						0);
	} else {
		gpio_set_value(FLYER_GPIO_TP_3V3_ENABLE, 0);
		system_rev >= XC ? gpio_set_value(FLYER_GPIO_SPI_ENABLE_XC, 0) :
							gpio_set_value(FLYER_GPIO_SPI_ENABLE, 0);

		pm8058_gpio_cfg(FLYER_TP_ATT_PMIC,
						PM_GPIO_DIR_IN,
						0,
						0,
						PM_GPIO_PULL_DN,
						0,
						PM_GPIO_STRENGTH_NO,
						PM_GPIO_FUNC_NORMAL,
						0);
	}

	return 0;
}

static struct ntrig_spi_platform_data flyer_ts_ntrig_data[] = {
	{
		.abs_x_min = 600,
		.abs_x_max = 7200,
		.abs_y_min = 0,
		.abs_y_max = 9150,
		.fwtwozero = 0x1620,
		.abs_width_min = 0,
		.abs_width_max = 100,
		.orientate = 0x03,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.spi_enable = FLYER_GPIO_SPI_ENABLE,
		.irq_gpio = FLYER_GPIO_TP_ATT_N,
		.power = flyer_ts_ntrig_power,
		.esdFlag = true,
	},
};

static struct spi_board_info msm_spi_board_info[] __initdata = {
	{
		.modalias	= "spi_aic3254",
		.mode           = SPI_MODE_1,
		.bus_num        = 0,
		.chip_select    = 3,
		.max_speed_hz   = 9963243,
	},
	{
		.modalias	= NTRIG_NAME,
		.mode           = SPI_MODE_0,
		.bus_num        = 0,
		.chip_select    = 2,
		.max_speed_hz   = 9963243,
		.platform_data  = &flyer_ts_ntrig_data,
		.irq	= MSM_GPIO_TO_INT(FLYER_GPIO_TP_ATT_N),
	}
};


static int msm_qsd_spi_gpio_config(void)
{
	unsigned id;
	printk(KERN_INFO "%s\n", __func__);
	id = PCOM_GPIO_CFG(45, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_6MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	id = PCOM_GPIO_CFG(47, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_6MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	id = PCOM_GPIO_CFG(48, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_6MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	id = PCOM_GPIO_CFG(87, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	/* SPI GPIO for AIC3254 */
	id = PCOM_GPIO_CFG(89, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_6MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	return 0;
}

static void msm_qsd_spi_gpio_release(void)
{
	unsigned id;
	printk(KERN_INFO "%s\n", __func__);
	id = PCOM_GPIO_CFG(45, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	id = PCOM_GPIO_CFG(47, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	id = PCOM_GPIO_CFG(48, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	id = PCOM_GPIO_CFG(87, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	id = PCOM_GPIO_CFG(89, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26000000,
	.clk_name = "spi_clk",
	.pclk_name = "spi_pclk",
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
//	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsdnew_device_spi.dev.platform_data = &qsd_spi_pdata;
}
#ifndef CONFIG_MSM_SSBI
static struct pm8058_platform_data pm8058_flyer_data = {
	.pm_irqs = {
		[PM8058_IRQ_KEYPAD - PM8058_FIRST_IRQ] = 74,
		[PM8058_IRQ_KEYSTUCK - PM8058_FIRST_IRQ] = 75,
	},
	.init = &flyer_pm8058_gpios_init,

	.num_subdevs = 4,
	.sub_devices = {
		{	.name = "pm8058-gpio",
		},
		{	.name = "pm8058-mpp",
		},
		{	.name = "pm8058-pwm",
		},
		{	.name = "pm8058-nfc",
		},
	},
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0),
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data = &pm8058_flyer_data,
	},
};
#endif

#ifdef CONFIG_I2C_SSBI
static struct msm_i2c_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI"
};

static struct msm_i2c_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI"
};
#endif

#define LCM_GPIO_CFG(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)

#define LCM_GPIO_CFG_8MA(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA)

#define LCM_GPIO_CFG_6MA(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_6MA)

#define LCM_GPIO_CFG_4MA(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)


#define LCM_GPIO_CFG_14MA(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_14MA)


/* below 2 display on/off structure are not used */
#if 0
static uint32_t display_on_gpio_table[] = {
	LCM_GPIO_CFG(FLYER_LCD_PCLK, 1),
	LCM_GPIO_CFG(FLYER_LCD_DE, 1),
	LCM_GPIO_CFG(FLYER_LCD_VSYNC, 1),
	LCM_GPIO_CFG(FLYER_LCD_HSYNC, 1),
	LCM_GPIO_CFG(FLYER_LCD_G2, 1),
	LCM_GPIO_CFG(FLYER_LCD_G3, 1),
	LCM_GPIO_CFG(FLYER_LCD_G4, 1),
	LCM_GPIO_CFG(FLYER_LCD_G5, 1),
	LCM_GPIO_CFG(FLYER_LCD_G6, 1),
	LCM_GPIO_CFG(FLYER_LCD_G7, 1),
	LCM_GPIO_CFG(FLYER_LCD_B3, 1),
	LCM_GPIO_CFG(FLYER_LCD_B4, 1),
	LCM_GPIO_CFG(FLYER_LCD_B5, 1),
	LCM_GPIO_CFG(FLYER_LCD_B6, 1),
	LCM_GPIO_CFG(FLYER_LCD_B7, 1),
	LCM_GPIO_CFG(FLYER_LCD_R3, 1),
	LCM_GPIO_CFG(FLYER_LCD_R4, 1),
	LCM_GPIO_CFG(FLYER_LCD_R5, 1),
	LCM_GPIO_CFG(FLYER_LCD_R6, 1),
	LCM_GPIO_CFG(FLYER_LCD_R7, 1),
};

static uint32_t display_off_gpio_table[] = {
	LCM_GPIO_CFG(FLYER_LCD_PCLK, 0),
	LCM_GPIO_CFG(FLYER_LCD_DE, 0),
	LCM_GPIO_CFG(FLYER_LCD_VSYNC, 0),
	LCM_GPIO_CFG(FLYER_LCD_HSYNC, 0),
	LCM_GPIO_CFG(FLYER_LCD_G2, 0),
	LCM_GPIO_CFG(FLYER_LCD_G3, 0),
	LCM_GPIO_CFG(FLYER_LCD_G4, 0),
	LCM_GPIO_CFG(FLYER_LCD_G5, 0),
	LCM_GPIO_CFG(FLYER_LCD_G6, 0),
	LCM_GPIO_CFG(FLYER_LCD_G7, 0),
	LCM_GPIO_CFG(FLYER_LCD_B3, 0),
	LCM_GPIO_CFG(FLYER_LCD_B4, 0),
	LCM_GPIO_CFG(FLYER_LCD_B5, 0),
	LCM_GPIO_CFG(FLYER_LCD_B6, 0),
	LCM_GPIO_CFG(FLYER_LCD_B7, 0),
	LCM_GPIO_CFG(FLYER_LCD_R3, 0),
	LCM_GPIO_CFG(FLYER_LCD_R4, 0),
	LCM_GPIO_CFG(FLYER_LCD_R5, 0),
	LCM_GPIO_CFG(FLYER_LCD_R6, 0),
	LCM_GPIO_CFG(FLYER_LCD_R7, 0),
};
#endif

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("s5k4e1gx", 0x20 >> 1),
	},
	{
		I2C_BOARD_INFO("s5k6aafx", 0x78 >> 1),
	},
	{
		I2C_BOARD_INFO("s5k6aafx", 0x5A >> 1),/*for cob,*/
	},
};

static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
PCOM_GPIO_CFG(FLYER_CAM_RST,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM_PWD,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_RST,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_PWD,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),

	PCOM_GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */

};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
PCOM_GPIO_CFG(FLYER_CAM_RST,   0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM_PWD,   0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_RST,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_PWD,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),

	PCOM_GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* MCLK */

};

/* for XC board */
static uint32_t camera_off_gpio_table_XC[] = {
	/* parallel CAMERA interfaces */
PCOM_GPIO_CFG(FLYER_CAM_RST_XC,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM_PWD,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_RST,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_PWD_XC,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),

	PCOM_GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */

};

/* for XC board */
static uint32_t camera_on_gpio_table_XC[] = {
	/* parallel CAMERA interfaces */
PCOM_GPIO_CFG(FLYER_CAM_RST_XC,   0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM_PWD,   0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_RST,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_PWD_XC,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),

	PCOM_GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* MCLK */

};

/* for XD board : change FLYER_CAM2_PWD*/
static uint32_t camera_off_gpio_table_XD[] = {
	/* parallel CAMERA interfaces */
PCOM_GPIO_CFG(FLYER_CAM_RST_XC,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM_PWD,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_RST,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_PWD_XD,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),

	PCOM_GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */

};

/* for XD board  : change FLYER_CAM2_PWD*/
static uint32_t camera_on_gpio_table_XD[] = {
	/* parallel CAMERA interfaces */
PCOM_GPIO_CFG(FLYER_CAM_RST_XC,   0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM_PWD,   0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_RST,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
PCOM_GPIO_CFG(FLYER_CAM2_PWD_XD,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),

	PCOM_GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* MCLK */

};

static int flyer_sensor_power_enable(char *power, unsigned volt)
{
	struct vreg *vreg_gp;
	int rc;

	if (power == NULL)
		return EIO;

	vreg_gp = vreg_get(NULL, power);
	if (IS_ERR(vreg_gp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
			__func__, power, PTR_ERR(vreg_gp));
		return EIO;
	}

	rc = vreg_set_level(vreg_gp, volt);
	if (rc) {
		pr_err("%s: vreg wlan set %s level failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}

	rc = vreg_enable(vreg_gp);
	if (rc) {
		pr_err("%s: vreg enable %s failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}
	return rc;
}

static int flyer_sensor_power_disable(char *power)
{
	struct vreg *vreg_gp;
	int rc;
	vreg_gp = vreg_get(NULL, power);
	if (IS_ERR(vreg_gp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
			__func__, power, PTR_ERR(vreg_gp));
		return EIO;
	}

	rc = vreg_disable(vreg_gp);
	if (rc) {
		pr_err("%s: vreg disable %s failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}
	return rc;
}

static int flyer_sensor_vreg_on(void)
{
	int rc;
	pr_info("%s camera vreg on\n", __func__);

	/*camera power down*/
#if 0
	gpio_set_value(FLYER_CAM_PWD, 1);
	udelay(200);
#endif

	/*camera VCM power*/
	rc = flyer_sensor_power_enable("gp9", 2850);

	udelay(150);

	/*camera analog power*/
	/* This delay is just temporary work-around,*/
	/*and will remove when HW power team fix */
	/*the power up two stage problem with pmic */

	if (system_rev >= 3) /* for XD board */
	rc = flyer_sensor_power_enable("gp6", 2850);
	else
	rc = flyer_sensor_power_enable("gp4", 2850);

	udelay(150);

	/*camera digital power*/
	rc = flyer_sensor_power_enable("wlan", 1800);

	udelay(150);

	/*camera IO power*/
	rc = flyer_sensor_power_enable("gp2", 1800);


	return rc;
}



static int flyer_sensor_vreg_off(void)
{
	int rc;
	/*camera analog power*/
	if (system_rev >= 3) /* for XD board */
	rc = flyer_sensor_power_disable("gp6");
	else
	rc = flyer_sensor_power_disable("gp4");

	/*camera digital power*/
	rc = flyer_sensor_power_disable("wlan");
	/*camera IO power*/
	rc = flyer_sensor_power_disable("gp2");
	/*camera VCM power*/
	rc = flyer_sensor_power_disable("gp9");
	return rc;
}


static void config_flyer_camera_on_gpios(void)
{
	if (system_rev >= 3) /* for XD board */
	config_gpio_table(camera_on_gpio_table_XD,
		ARRAY_SIZE(camera_on_gpio_table_XD));
	else if (system_rev == 2) /* for XC board */
	config_gpio_table(camera_on_gpio_table_XC,
		ARRAY_SIZE(camera_on_gpio_table_XC));
	else
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_flyer_camera_off_gpios(void)
{
	if (system_rev >= 3) /* for XD board */
	config_gpio_table(camera_off_gpio_table_XD,
		ARRAY_SIZE(camera_off_gpio_table_XD));
	else if (system_rev == 2) /* for XC board */
	config_gpio_table(camera_off_gpio_table_XC,
		ARRAY_SIZE(camera_off_gpio_table_XC));
	else
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_flyer_camera_on_gpios,
	.camera_gpio_off = config_flyer_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400
};

#if 0 /* flyer has no flashlight module */
static int flashlight_control(int mode)
{
	return 0; /* flyer has no flashlight */
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};
#endif

static void flyer_s5k4e1gx_clk_switch(void){
	int rc = 0;
	pr_info("Flyer: Doing clk switch (Flyer)(s5k4e1gx)\n");
	rc = gpio_request(gCAM_GPIO_SEL, "s5k4e1gx");
	if (rc < 0)
		pr_err("GPIO (%d) request fail\n", gCAM_GPIO_SEL);
	else
		gpio_direction_output(gCAM_GPIO_SEL, 0);
	gpio_free(gCAM_GPIO_SEL);

	return;
}

static void flyer_s5k6aafx_clk_switch(void){
	int rc = 0;
	pr_info("Flyer: Doing clk switch (Flyer)(s5k6aafx)\n");
	rc = gpio_request(gCAM_GPIO_SEL, "s5k6aafx");
	if (rc < 0)
		pr_err("GPIO (%d) request fail\n", gCAM_GPIO_SEL);
	else
		gpio_direction_output(gCAM_GPIO_SEL, 1);
	gpio_free(gCAM_GPIO_SEL);

	return;
}

/* S5K4E1GX */
static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_data = {
	.sensor_name    = "s5k4e1gx",
	.sensor_reset   = FLYER_CAM_RST,
	.vcm_pwd     = FLYER_CAM_PWD,
	.camera_power_on = flyer_sensor_vreg_on,
	.camera_power_off = flyer_sensor_vreg_off,
	.camera_clk_switch = flyer_s5k4e1gx_clk_switch,
	.pdata          = &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_NONE,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	/* .flash_cfg	= &msm_camera_sensor_flash_cfg, */
	.cam_select_pin = FLYER_CLK_SWITCH,
	.sensor_lc_disable = true, /* disable sensor lens correction */
};

static struct platform_device msm_camera_sensor_s5k4e1gx = {
	.name      = "msm_camera_s5k4e1gx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k4e1gx_data,
	},
};

/* S5K6AAFX */
static struct msm_camera_sensor_info msm_camera_sensor_s5k6aafx_data = {
	.sensor_name	= "s5k6aafx",
	.sensor_reset	= FLYER_CAM2_RST,
	.sensor_pwd		= FLYER_CAM2_PWD,
	.camera_power_on = flyer_sensor_vreg_on,
	.camera_power_off = flyer_sensor_vreg_off,
	.camera_clk_switch	= flyer_s5k6aafx_clk_switch,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_NONE,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.power_down_disable = true, /* true: disable pwd down function */
	.full_size_preview = false, /* true: use full size preview */
	.cam_select_pin = FLYER_CLK_SWITCH,
};

static struct platform_device msm_camera_sensor_s5k6aafx = {
	.name	   = "msm_camera_s5k6aafx",
	.dev	    = {
		.platform_data = &msm_camera_sensor_s5k6aafx_data,
	},
};

#ifdef CONFIG_MSM_HDMI_MHL
//static struct msm_lcdc_panel_ops flyer_dtv_ops = {};

//720p
static struct msm_lcdc_timing flyer_dtv_default_timing = {
        .clk_rate               = 74250000,
        .hsync_pulse_width      = 40,
        .hsync_back_porch       = 219,
        .hsync_front_porch      = 111,
        .hsync_skew             = 0,
        .vsync_pulse_width      = 5,
        .vsync_back_porch       = 20,
        .vsync_front_porch      = 5,
        .vsync_act_low          = 0,
        .hsync_act_low          = 0,
        .den_act_low            = 0,
};
/* 480p */
#if 0
static struct msm_lcdc_timing flyer_dtv_default_timing = {
                .clk_rate               = 27027000,
                .hsync_pulse_width      = 62,
                .hsync_back_porch       = 59,
                .hsync_front_porch      = 17,
                .hsync_skew             = 0,
                .vsync_pulse_width      = 6,
                .vsync_back_porch       = 30,
                .vsync_front_porch      = 9,
#if  1
                .vsync_act_low          = 1,
                .hsync_act_low          = 1,
#else
                .vsync_act_low          = 0,
                .hsync_act_low          = 0,
#endif

                .den_act_low            = 0,
};
#endif

static struct resource resources_msm_fb1[] = {
	{
		.start = MSM_FB1_BASE,
		.end = MSM_FB1_BASE + MSM_FB1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct msm_fb_data flyer_dtv_fb_data = {
	.xres		= 1280,
	.yres		= 720,
	.width		= 153,
	.height		= 90,
	.output_format	= 0,
	.overrides = MSM_FB_PM_DISABLE,
};

static struct msm_lcdc_platform_data flyer_dtv_platform_data = {
	.fb_id		= 1,
	.fb_data	= &flyer_dtv_fb_data,
	.fb_resource	= &resources_msm_fb1[0],
};

static struct platform_device flyer_dtv_device = {
	.name	= "msm_mdp_dtv",
	.id	= -1,
	.dev	= {
		.platform_data = &flyer_dtv_platform_data,
	},
};

static uint32_t dtv_on_gpio_table[] = {
        LCM_GPIO_CFG(FLYER_DTV_PCLK, 1),
        LCM_GPIO_CFG(FLYER_DTV_DE, 1),
        LCM_GPIO_CFG(FLYER_DTV_VSYNC, 1),
        LCM_GPIO_CFG(FLYER_DTV_HSYNC, 1),

        LCM_GPIO_CFG_14MA(FLYER_DTV_R0, 1),
        LCM_GPIO_CFG_14MA(FLYER_DTV_R1, 1),
        LCM_GPIO_CFG_14MA(FLYER_DTV_R2, 1),
        LCM_GPIO_CFG_14MA(FLYER_DTV_R3, 1),
        LCM_GPIO_CFG_14MA(FLYER_DTV_R4, 1),
        LCM_GPIO_CFG_14MA(FLYER_DTV_R5, 1),
        LCM_GPIO_CFG_14MA(FLYER_DTV_R6, 1),
        LCM_GPIO_CFG_14MA(FLYER_DTV_R7, 1),

        LCM_GPIO_CFG_8MA(FLYER_DTV_G0, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_G1, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_G2, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_G3, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_G4, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_G5, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_G6, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_G7, 1),

        LCM_GPIO_CFG_8MA(FLYER_DTV_B0, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_B1, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_B2, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_B3, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_B4, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_B5, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_B6, 1),
        LCM_GPIO_CFG_8MA(FLYER_DTV_B7, 1),
};


static uint32_t dtv_off_gpio_table[] = {
        LCM_GPIO_CFG(FLYER_DTV_PCLK, 0),
        LCM_GPIO_CFG(FLYER_DTV_DE, 0),
        LCM_GPIO_CFG(FLYER_DTV_VSYNC, 0),
        LCM_GPIO_CFG(FLYER_DTV_HSYNC, 0),

        LCM_GPIO_CFG(FLYER_DTV_R0, 0),
        LCM_GPIO_CFG(FLYER_DTV_R1, 0),
        LCM_GPIO_CFG(FLYER_DTV_R2, 0),
        LCM_GPIO_CFG(FLYER_DTV_R3, 0),
        LCM_GPIO_CFG(FLYER_DTV_R4, 0),
        LCM_GPIO_CFG(FLYER_DTV_R5, 0),
        LCM_GPIO_CFG(FLYER_DTV_R6, 0),
        LCM_GPIO_CFG(FLYER_DTV_R7, 0),

        LCM_GPIO_CFG(FLYER_DTV_G0, 0),
        LCM_GPIO_CFG(FLYER_DTV_G1, 0),
        LCM_GPIO_CFG(FLYER_DTV_G2, 0),
        LCM_GPIO_CFG(FLYER_DTV_G3, 0),
        LCM_GPIO_CFG(FLYER_DTV_G4, 0),
        LCM_GPIO_CFG(FLYER_DTV_G5, 0),
        LCM_GPIO_CFG(FLYER_DTV_G6, 0),
        LCM_GPIO_CFG(FLYER_DTV_G7, 0),

        LCM_GPIO_CFG(FLYER_DTV_B0, 0),
        LCM_GPIO_CFG(FLYER_DTV_B1, 0),
        LCM_GPIO_CFG(FLYER_DTV_B2, 0),
        LCM_GPIO_CFG(FLYER_DTV_B3, 0),
        LCM_GPIO_CFG(FLYER_DTV_B4, 0),
        LCM_GPIO_CFG(FLYER_DTV_B5, 0),
        LCM_GPIO_CFG(FLYER_DTV_B6, 0),
        LCM_GPIO_CFG(FLYER_DTV_B7, 0),
};

static struct platform_device hdmi_device = {
        .name = "hdmi_msm",
        .id = 1,
};

static struct vreg *vreg_mhl_1v2;
static struct vreg *vreg_mhl_1v8;

void flyer_mhl_power(int on)
{
	if (on) {
		config_gpio_table(dtv_on_gpio_table, ARRAY_SIZE(dtv_on_gpio_table));
		vreg_enable(vreg_mhl_1v8);
		vreg_enable(vreg_mhl_1v2);
		gpio_set_value(FLYER_MHL_RSTz, 1);
	} else {
		config_gpio_table(dtv_off_gpio_table,
			ARRAY_SIZE(dtv_off_gpio_table));
		gpio_set_value(FLYER_MHL_RSTz, 0);
		vreg_disable(vreg_mhl_1v2);
	}
}

static struct mhl_platform_data mhl_sii9232_device_data = {
	.i2c_addr_tpi = FLYER_MHL_I2C_TPI,
	.i2c_addr_cbus = FLYER_MHL_I2C_CBUS,
	.gpio_intr = FLYER_MHL_INT,
	.gpio_reset = FLYER_MHL_RSTz,
	.power_switch = flyer_mhl_power,
};

static struct i2c_board_info i2c_mhl_device[] = {
    {
            I2C_BOARD_INFO(MHL_SII9232_I2C_NAME, FLYER_MHL_I2C_TPI >> 1),
            .platform_data = &mhl_sii9232_device_data,
            .irq = MSM_GPIO_TO_INT(FLYER_MHL_INT),
    },
};


int __init flyer_init_mhl(void)
{
	unsigned ret, id;
	pr_debug("%s\n", __func__);

        vreg_mhl_1v2 = vreg_get(0, "gp5");
        vreg_set_level(vreg_mhl_1v2, 1200);

	//L20
	vreg_mhl_1v8 = vreg_get(0, "gp13");
	vreg_set_level(vreg_mhl_1v8, 1850);

	id = PCOM_GPIO_CFG(FLYER_MHL_3V3_EN, 0, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	id = PCOM_GPIO_CFG(FLYER_MHL_RSTz, 0, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	id = PCOM_GPIO_CFG(FLYER_MHL_INT, 0, GPIO_INPUT, GPIO_PULL_UP,
		GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	id = PCOM_GPIO_CFG(FLYER_GPIO_USB_ID_PIN_XC, 0, GPIO_INPUT, GPIO_NO_PULL,
		GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	id = PCOM_GPIO_CFG(FLYER_MHL_SW, 0, GPIO_OUTPUT, GPIO_NO_PULL,
		GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	gpio_set_value(FLYER_MHL_3V3_EN, 1);
	vreg_enable(vreg_mhl_1v8);
	vreg_enable(vreg_mhl_1v2);

	flyer_dtv_platform_data.timing = &flyer_dtv_default_timing;

	ret = platform_device_register(&flyer_dtv_device);
	if (ret != 0) {
		pr_err("flyer_init_mhl FAILED: platform_device_register dtv ret=%d\n",
		ret);
		return ret;
	}

	ret = platform_device_register(&hdmi_device);
	if (ret != 0) {
		pr_err("flyer_init_mhl FAILED: platform_device_register hdmi ret=%d\n",
                   ret);
		return ret;
	}
	return 0;
}

#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif


static struct platform_device flyer_rfkill = {
	.name = "flyer_rfkill",
	.id = -1,
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name           = PMEM_KERNEL_EBI1_DATA_NAME,
	.start          = PMEM_KERNEL_EBI1_BASE,
	.size           = PMEM_KERNEL_EBI1_SIZE,
	.cached         = 0,
};

static struct platform_device android_pmem_kernel_ebi1_devices = {
	.name           = "android_pmem",
	.id = 2,
	.dev = {.platform_data = &android_pmem_kernel_ebi1_pdata },
};

#if defined(CONFIG_SERIAL_MSM_HS) && defined(CONFIG_SERIAL_MSM_HS_PURE_ANDROID)
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = -1,
	.inject_rx_on_wakeup = 0,
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
};

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = FLYER_GPIO_BT_CHIP_WAKE,
	.gpio_host_wake = FLYER_GPIO_BT_HOST_WAKE,
	.request_clock_off_locked = msm_hs_request_clock_off_locked,
	.request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};
#elif defined(CONFIG_SERIAL_MSM_HS)
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(FLYER_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = FLYER_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = FLYER_GPIO_BT_HOST_WAKE,
};
#endif

/* for bcm */
static char bdaddress[20];
extern unsigned char *get_bt_bd_ram(void);

static void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);
	sprintf(bdaddress, "%02x:%02x:%02x:%02x:%02x:%02x",
		cTemp[0], cTemp[1], cTemp[2], cTemp[3], cTemp[4], cTemp[5]);
	printk(KERN_INFO "YoYo--BD_ADDRESS=%s\n", bdaddress);
}

module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");

static char bt_chip_id[10] = "bcm4329";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

struct platform_device htc_drm = {
	.name = "htcdrm",
	.id = 0,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_uart2,
	&msm_device_smd,
	&flyer_rfkill,
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
	&bcm_bt_lpm_device,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi6,
	&msm_device_ssbi7,
#endif
	&qsdnew_device_spi,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&qup_device_i2c,
	&htc_battery_pdev,
	&ds2746_battery_pdev,
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_device_adspdec,
#endif
	&android_pmem_kernel_ebi1_devices,
	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aux_pcm_device,
#endif
	&msm_camera_sensor_s5k4e1gx, /* Main CAM */
	&msm_camera_sensor_s5k6aafx, /* 2nd CAM	*/
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&gpio_leds,
	&htc_drm,
};

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock = 100000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_8MA,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static void qup_i2c_gpio_config(int adap_id, int config_type)
{
	unsigned id;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type) {
		id = PCOM_GPIO_CFG(16, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(17, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	} else {
		id = PCOM_GPIO_CFG(16, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(17, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 384000,
	.pclk = "camif_pad_pclk",
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
}

static struct msm_pmem_setting pmem_setting = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
//	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
//	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
//	.pmem_gpu1_start = MSM_PMEM_GPU1_BASE,
//	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
	.kgsl_start = MSM_GPU_MEM_BASE,
	.kgsl_size = MSM_GPU_MEM_SIZE,
};

static struct msm_acpu_clock_platform_data flyer_clock_data = {
	.acpu_switch_time_us = 50,
	.vdd_switch_time_us = 62,
	.wait_for_irq_khz	= 0,
};

static unsigned flyer_perf_acpu_table[] = {
	245000000,
	768000000,
	1024000000,
};

static struct perflock_platform_data flyer_perflock_data = {
	.perf_acpu_table = flyer_perf_acpu_table,
	.table_size = ARRAY_SIZE(flyer_perf_acpu_table),
};

#ifdef CONFIG_MSM_SSBI
static int flyer_pmic_init(struct device *dev)
{
	struct pm8058_chip *pm_chip = NULL;

	flyer_pm8058_gpios_init(pm_chip);

	return 0;
}
static struct pm8058_platform_data flyer_pm8058_pdata = {
	.irq_base	= PM8058_FIRST_IRQ,
	.gpio_base	= FIRST_BOARD_GPIO,
	.init		= flyer_pmic_init,
	.num_subdevs = 3,
	.sub_devices_htc = {
		{	.name = "pm8058-gpio",
		},
		{	.name = "pm8058-mpp",
		},
		{	.name = "pm8058-pwm",
		},
	},
};

static struct msm_ssbi_platform_data flyer_ssbi_pmic_pdata = {
	.slave		= {
		.name		= "pm8058-core",
		.irq		= MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data	= &flyer_pm8058_pdata,
	},
	.rspinlock_name	= "D:PMIC_SSBI",
};

static int __init flyer_ssbi_pmic_init(void)
{
	int ret;
	u32 id;

	pr_info("%s()\n", __func__);
	id = PCOM_GPIO_CFG(flyer_get_PMIC_GPIO_INT(), 1, GPIO_INPUT,
			   GPIO_NO_PULL, GPIO_2MA);
	ret = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	if (ret)
		pr_err("%s: gpio %d cfg failed\n", __func__,
		       flyer_get_PMIC_GPIO_INT());

	ret = gpiochip_reserve(flyer_pm8058_pdata.gpio_base,
			       PM8058_GPIOS);
	WARN(ret, "can't reserve pm8058 gpios. badness will ensue...\n");

	flyer_ssbi_pmic_pdata.slave.irq = MSM_GPIO_TO_INT(flyer_get_PMIC_GPIO_INT());

	msm_device_ssbi_pmic.dev.platform_data = &flyer_ssbi_pmic_pdata;
	return platform_device_register(&msm_device_ssbi_pmic);
}
#endif

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_APPS_SLEEP].supported = 1,
	[MSM_PM_SLEEP_MODE_APPS_SLEEP].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_APPS_SLEEP].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_APPS_SLEEP].latency = 8594,
	[MSM_PM_SLEEP_MODE_APPS_SLEEP].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].latency = 500,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].residency = 6000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

static ssize_t flyer_virtual_keys_pen_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_RIGHTCTRL)":550:1050:90:70"
		":" __stringify(EV_KEY) ":" __stringify(KEY_RIGHTCTRL)":-35:1000:90:70"
		"\n");
}

static struct kobj_attribute flyer_virtual_key_pen_attr = {
	.attr = {
		.name = "virtualkeys.Ntrig-Pen-touchscreen",
		.mode = S_IRUGO,
	},
	.show = flyer_virtual_keys_pen_show,
};

static struct attribute *flyer_properties_attrs[] = {
	&flyer_virtual_key_pen_attr.attr,
	NULL
};

static struct attribute_group flyer_properties_attr_group = {
	.attrs = flyer_properties_attrs,
};

static void flyer_reset(void)
{
	gpio_set_value(FLYER_GPIO_PS_HOLD, 0);
}

static void __init flyer_init(void)
{
	int ret = 0;
	struct kobject *properties_kobj;
	printk(KERN_INFO "%s: revision = %d\n", __func__, system_rev);
	printk(KERN_INFO "%s: microp version = %s\n", __func__, microp_ver);

	/* Must set msm_hw_reset_hook before first proc comm */
	msm_hw_reset_hook = flyer_reset;

	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n", __func__);

	msm_clock_init();

	/* for bcm */
	bt_export_bd_address();

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart2)
		msm_serial_debug_init(MSM_UART2_PHYS, INT_UART2,
		&msm_device_uart2.dev, 23, MSM_GPIO_TO_INT(FLYER_GPIO_UART2_RX));
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	#ifndef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
	msm_device_uart_dm1.name = "msm_serial_hs_bcm";	/* for bcm */
	#endif
	msm_add_serial_devices(3);
#else
	msm_add_serial_devices(0);
#endif

	msm_pm_set_platform_data(msm_pm_data);
	msm_device_i2c_init();
	qup_device_i2c_init();
	msm7x30_init_marimba();
#ifdef CONFIG_MSM7KV2_AUDIO
	msm_snddev_init();
	aux_pcm_gpio_init();
#endif
	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));
	msm_spm_init(&msm_spm_data, 1);
	msm_acpu_clock_init(&flyer_clock_data);
	perflock_init(&flyer_perflock_data);

#ifdef CONFIG_MICROP_COMMON
	flyer_microp_init();
#endif

	qup_device_i2c_init();
	msm_add_mem_devices(&pmem_setting);

	platform_add_devices(devices, ARRAY_SIZE(devices));

#ifdef CONFIG_USB_ANDROID
	flyer_add_usb_devices();
#endif

	if (board_emmc_boot()) {
#if defined(CONFIG_MSM_RMT_STORAGE_SERVER)
		rmt_storage_add_ramfs();
#endif
		create_proc_read_entry("emmc", 0, NULL, emmc_partition_read_proc, NULL);
	} else
		platform_device_register(&msm_device_nand);

#ifdef CONFIG_MSM_SSBI
	flyer_ssbi_pmic_init();
#else
	pm8058_boardinfo->irq = MSM_GPIO_TO_INT(flyer_get_PMIC_GPIO_INT());
#endif

	ret = flyer_init_mmc(system_rev);
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC\n", __func__);

	msm_qsd_spi_init();
	spi_register_board_info(msm_spi_board_info, ARRAY_SIZE(msm_spi_board_info));
	if (system_rev >= 2) {
		flyer_ts_ntrig_data[0].spi_enable = FLYER_GPIO_SPI_ENABLE_XC;
	}

	if (system_rev >= 3) {/* for Camera XD board */
		pr_info("[camera] XD GPIO pin changed\n");
		gCAM_GPIO_SEL = FLYER_CLK_SWITCH_XC;
		gCAM_RST_GPIO = FLYER_CAM_RST_XC;
		gCAM2_PWN_GPIO = FLYER_CAM2_PWD_XD;
	} else if (system_rev == 2) {/* for Camera XC board */
		pr_info("[camera] XC GPIO pin changed\n");
		gCAM_GPIO_SEL = FLYER_CLK_SWITCH_XC;
		gCAM_RST_GPIO = FLYER_CAM_RST_XC;
		gCAM2_PWN_GPIO = FLYER_CAM2_PWD_XC;
	}
	msm_camera_sensor_s5k4e1gx_data.sensor_reset = gCAM_RST_GPIO;
	msm_camera_sensor_s5k6aafx_data.sensor_pwd = gCAM2_PWN_GPIO;
	msm_camera_sensor_s5k4e1gx_data.cam_select_pin = gCAM_GPIO_SEL;
	msm_camera_sensor_s5k6aafx_data.cam_select_pin = gCAM_GPIO_SEL;

	i2c_register_board_info(0, i2c_devices,	ARRAY_SIZE(i2c_devices));

	if (system_rev <= 1) {
		i2c_register_board_info(0, i2c_compass_devices_XA_XB,
			ARRAY_SIZE(i2c_compass_devices_XA_XB));
	} else if (system_rev == 2) {
		i2c_register_board_info(0, i2c_compass_devices_XC,
			ARRAY_SIZE(i2c_compass_devices_XC));
	} else if (system_rev == 3) {
		i2c_register_board_info(0, i2c_compass_devices_XD,
			ARRAY_SIZE(i2c_compass_devices_XD));
	} else {
		i2c_register_board_info(0, i2c_compass_devices_VER_A,
			ARRAY_SIZE(i2c_compass_devices_VER_A));
	}

	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));

#ifdef CONFIG_SENSORS_CAPSENSE
	if (system_rev >= 2) {
		pm8058_gpio_cfg(FLYER_NC_PMIC14, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 0, PM_GPIO_PULL_UP_31P5, PM_GPIO_VIN_S3, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_NORMAL, 0);
		mdelay(1);
		if (gpio_get_value(PM8058_GPIO_PM_TO_SYS(FLYER_NC_PMIC14)) == 0) {
			i2c_register_board_info(0, i2c_capsense_devices_XC,
				ARRAY_SIZE(i2c_capsense_devices_XC));
		}
		pm8058_gpio_cfg(FLYER_NC_PMIC14, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 0, PM_GPIO_PULL_DN, PM_GPIO_VIN_S3, PM_GPIO_STRENGTH_HIGH, PM_GPIO_FUNC_NORMAL, 0);
	}
#endif
#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				&flyer_properties_attr_group);
	flyer_audio_init();
	flyer_init_keypad();
	flyer_wifi_init();
	flyer_init_panel();

#ifdef CONFIG_MSM_HDMI_MHL
	if (system_rev >= 2) {
		i2c_register_board_info(0, i2c_mhl_device,
			 ARRAY_SIZE(i2c_mhl_device));
		flyer_init_mhl();
	}
#endif

	if (system_rev == 2) {/* for Led XC board */
		printk(KERN_INFO "device=%d change gpio_led_config to XC\n",system_rev);
		gpio_leds_data.leds = gpio_led_config_XC;
		gpio_leds_data.num_leds= ARRAY_SIZE(gpio_led_config_XC);
	}
	else if (system_rev >= 3) {/* for Led XD board */
		if(gpio_get_value(PM8058_GPIO_PM_TO_SYS(FLYER_PEN_LED3))){
			printk(KERN_INFO "device=%d change gpio_led_config to XD\n",system_rev);
			gpio_leds_data.leds = gpio_led_config_XD;
			gpio_leds_data.num_leds= ARRAY_SIZE(gpio_led_config_XD);
		}
		else{
			printk(KERN_INFO "device=%d with XD I/O board change gpio_led_config to XD2\n",system_rev);
			gpio_leds_data.leds = gpio_led_config_XD2;
			gpio_leds_data.num_leds= ARRAY_SIZE(gpio_led_config_XD2);

			microp_leds_data.num_leds	= ARRAY_SIZE(up_led_config_XD2);
			microp_leds_data.led_config	= up_led_config_XD2;
		}
	}

	/* for Vibrator XC board */
	if (system_rev >= 2)
		msm_init_pmic_vibrator(3000);
}

static void __init flyer_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	printk(KERN_INFO "%s\n", __func__);

	parse_tag_memsize((const struct tag *)tags);
	engineerid = parse_tag_engineerid(tags);

	mi->nr_banks = 4;
	mi->bank[0].start = MSM_LINUX_BASE1;
	mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE1);
	mi->bank[0].size = MSM_LINUX_SIZE1;
	mi->bank[1].start = MSM_LINUX_BASE2;
	mi->bank[1].node = PHYS_TO_NID(MSM_LINUX_BASE2);
	mi->bank[1].size = MSM_LINUX_SIZE2;
	mi->bank[2].start = MSM_LINUX_BASE3;
	mi->bank[2].node = PHYS_TO_NID(MSM_LINUX_BASE3);
	mi->bank[2].size = MSM_LINUX_SIZE3;
	mi->bank[3].start = MSM_LINUX_BASE4;
	mi->bank[3].node = PHYS_TO_NID(MSM_LINUX_BASE4);
	mi->bank[3].size = MSM_LINUX_SIZE4;
}

static void __init flyer_map_io(void)
{
	printk(KERN_INFO "%s\n", __func__);

	msm_map_common_io();
}

extern struct sys_timer msm_timer;

MACHINE_START(FLYER, "flyer")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x14400100,
	.fixup		= flyer_fixup,
	.map_io		= flyer_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= flyer_init,
	.timer		= &msm_timer,
MACHINE_END
