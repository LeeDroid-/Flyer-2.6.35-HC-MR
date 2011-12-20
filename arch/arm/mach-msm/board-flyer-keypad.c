/* linux/arch/arm/mach-msm/board-flyer-keypad.c
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

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/gpio.h>

#include "board-flyer.h"
#include "proc_comm.h"
#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_flyer."
module_param_named(keycaps, keycaps, charp, 0);

static struct gpio_event_direct_entry flyer_keypad_input_map_xc[] = {
	{
		.gpio = FLYER_GPIO_KEYPAD_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_VOL_UP_XC),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_VOL_DN_XC),
		.code = KEY_VOLUMEDOWN,
	},
};

static struct gpio_event_direct_entry flyer_keypad_input_map[] = {
	{
		.gpio = FLYER_GPIO_KEYPAD_POWER_KEY,
		.code = KEY_POWER,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_VOL_UP),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(FLYER_VOL_DN),
		.code = KEY_VOLUMEDOWN,
	},
};

static void flyer_setup_input_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		PCOM_GPIO_CFG(FLYER_GPIO_KEYPAD_POWER_KEY, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_input_info flyer_keypad_input_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = flyer_keypad_input_map,
	.keymap_size = ARRAY_SIZE(flyer_keypad_input_map),
	.setup_input_gpio = flyer_setup_input_gpio,
};

static struct gpio_event_info *flyer_keypad_info[] = {
	&flyer_keypad_input_info.info,
};

static struct gpio_event_platform_data flyer_keypad_data = {
	.names = {
		"flyer-keypad",
		NULL,
	},
	.info = flyer_keypad_info,
	.info_count = ARRAY_SIZE(flyer_keypad_info),
};

static struct platform_device flyer_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &flyer_keypad_data,
	},
};

int __init flyer_init_keypad(void)
{
	printk(KERN_DEBUG "%s\n", __func__);

	flyer_keypad_data.name = "flyer-keypad-v0";
	printk("direct key:flyer-keypad-v0\n");

	if (system_rev >= 2) {
		flyer_keypad_input_info.keymap = flyer_keypad_input_map_xc;
		flyer_keypad_input_info.keymap_size =
				ARRAY_SIZE(flyer_keypad_input_map_xc);
	}
	return platform_device_register(&flyer_keypad_input_device);
}
