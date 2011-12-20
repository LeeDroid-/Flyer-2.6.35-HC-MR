/* linux/arch/arm/mach-msm/board-flyer.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_FLYER_H
#define __ARCH_ARM_MACH_MSM_BOARD_FLYER_H

#include <mach/board.h>

#define FLYER_PROJECT_NAME	"flyer"

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define MSM_LINUX_BASE1		0x14400000
#define MSM_LINUX_SIZE1		0x0BC00000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x0FB00000
#define MSM_LINUX_BASE3		0x40000000
#define MSM_LINUX_SIZE3		0x10000000
#define MSM_LINUX_BASE4		0x60000000
#define MSM_LINUX_SIZE4		0x09900000
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_FB1_BASE		0x2FB00000
#define MSM_FB1_SIZE		0x00500000

#define MSM_GPU_MEM_BASE	0x69900000
#define MSM_GPU_MEM_SIZE	0x01000000

#define MSM_PMEM_ADSP_BASE  	0x6A900000
#define MSM_PMEM_ADSP_SIZE      0x03200000
#define PMEM_KERNEL_EBI1_BASE   0x6DB00000
#define PMEM_KERNEL_EBI1_SIZE   0x00D00000

#define MSM_PMEM_CAMERA_BASE	0x6E800000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE	0x6E800000
#define MSM_PMEM_MDP_SIZE	0x01000000

#define MSM_FB_BASE		0x6F800000
#define MSM_FB_SIZE		0x00800000

/* GPIO definition */

/* Direct Keys */
#define FLYER_GPIO_KEYPAD_POWER_KEY  (46)

/* Battery */
#define FLYER_GPIO_MCHG_EN_N         (162)
#define FLYER_GPIO_MCHG_EN_N_XC      (149)
#define FLYER_GPIO_ISET		     (127)
#define FLYER_GPIO_ISET_XC           (33)
#define FLYER_GPIO_ADP_9V            (178)

/* Wifi */
#define FLYER_GPIO_WIFI_IRQ          (147)
#define FLYER_GPIO_WIFI_EN           (39)

#define FLYER_GPIO_UART2_RX          (51)
#define FLYER_GPIO_UART2_TX          (52)

/* Sensors */
#define FLYER_GPIO_COMPASS_INT_XA_XB (42)
#define FLYER_GPIO_COMPASS_INT_XC    (143)

#define FLYER_COMPASS_LAYOUTS_XA_XB	{ \
	{ { 0,  1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
	{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
	{ { 1,  0, 0}, { 0, -1,  0}, {0, 0, -1} }, \
	{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}

#define FLYER_COMPASS_LAYOUTS_XC	{ \
	{ { 0,  1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
	{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
	{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
	{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}

#define FLYER_COMPASS_LAYOUTS_VER_A	{ \
	{ { 0,  1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
	{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
	{ { 1,  0, 0}, { 0, -1,  0}, {0, 0, -1} }, \
	{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}

/* Microp */
#define FLYER_GPIO_UP_RESET_N        (43)
#define FLYER_GPIO_UP_INT_N          (142)

/* TP */
#define FLYER_GPIO_TP_ATT_N          (40)
#define FLYER_GPIO_TP_3V3_ENABLE	(55)
#define FLYER_GPIO_SPI_ENABLE		(177)
#define FLYER_GPIO_SPI_ENABLE_XC	(148)

/* LED */
#define FLYER_2ND_CAM_LED_XD	(36)

/* LCD */
#define FLYER_LCD_PCLK			(90)
#define FLYER_LCD_DE                    (91)
#define FLYER_LCD_VSYNC                 (92)
#define FLYER_LCD_HSYNC                 (93)

#define FLYER_LCD_G0                    (18)
#define FLYER_LCD_G1                    (19)
#define FLYER_LCD_G2                    (94)
#define FLYER_LCD_G3                    (95)
#define FLYER_LCD_G4                    (96)
#define FLYER_LCD_G5                    (97)
#define FLYER_LCD_G6                    (98)
#define FLYER_LCD_G7                    (99)

#define FLYER_LCD_B0                    (20)
#define FLYER_LCD_B1                    (21)
#define FLYER_LCD_B2                    (22)
#define FLYER_LCD_B3                    (100)
#define FLYER_LCD_B4                    (101)
#define FLYER_LCD_B5                    (102)
#define FLYER_LCD_B6                    (103)
#define FLYER_LCD_B7                    (104)

#define FLYER_LCD_R0                    (23)
#define FLYER_LCD_R1                    (24)
#define FLYER_LCD_R2                    (25)
#define FLYER_LCD_R3                    (105)
#define FLYER_LCD_R4                    (106)
#define FLYER_LCD_R5                    (107)
#define FLYER_LCD_R6                    (108)
#define FLYER_LCD_R7                    (109)
#define FLYER_LCD_RSTz                  (126)
#define FLYER_LCM_3V3_EN                (129)
#define FLYER_LCM_3V3_EN_XC             (181)
#define FLYER_LVDS_ON                   (144)
#define FLYER_LVDS_ON_XC                (80)

/* DTV */
#define FLYER_DTV_PCLK			(124)
#define FLYER_DTV_DE                    (125)
#define FLYER_DTV_VSYNC                 (126)
#define FLYER_DTV_HSYNC                 (127)

#define FLYER_DTV_G0                    (128)
#define FLYER_DTV_G1                    (129)
#define FLYER_DTV_G2                    (130)
#define FLYER_DTV_G3                    (131)
#define FLYER_DTV_G4                    (132)
#define FLYER_DTV_G5                    (160)
#define FLYER_DTV_G6                    (161)
#define FLYER_DTV_G7                    (162)

#define FLYER_DTV_B0                    (163)
#define FLYER_DTV_B1                    (164)
#define FLYER_DTV_B2                    (165)
#define FLYER_DTV_B3                    (166)
#define FLYER_DTV_B4                    (167)
#define FLYER_DTV_B5                    (168)
#define FLYER_DTV_B6                    (169)
#define FLYER_DTV_B7                    (170)

#define FLYER_DTV_R0                    (171)
#define FLYER_DTV_R1                    (172)
#define FLYER_DTV_R2                    (173)
#define FLYER_DTV_R3                    (174)
#define FLYER_DTV_R4                    (175)
#define FLYER_DTV_R5                    (176)
#define FLYER_DTV_R6                    (177)
#define FLYER_DTV_R7                    (178)


/* Audio */
#define FLYER_AUD_CODEC_RST          (34)
#define FLYER_AUD_MIC_BIAS           (56)
#define FLYER_AUD_A1026_INT           (120)
#define FLYER_AUD_MICPATH_SEL        (121)
#define FLYER_AUD_MICPATH_SEL_XC     PMGPIO(6)
#define FLYER_AUD_A1026_RESET        (122)
#define FLYER_AUD_A1026_WAKEUP       (123)
#define FLYER_WCA_MCLK_XC	     (120)
#define FLYER_WCA_DATA_SD0_XC        (121)
#define FLYER_WCA_DATA_SD1_XC        (122)
#define FLYER_WCA_DATA_SD2_XC        (123)
#define FLYER_WCF_I2S_WS_XC          (144)
#define FLYER_WCF_I2S_CLK_XC         (145)
#define FLYER_WCF_I2S_DATA_XC        (146)

/* BT */
#define FLYER_GPIO_BT_SHUTDOWN_N     (38)
#define FLYER_GPIO_BT_RESET_N        (41)
#define FLYER_GPIO_BT_HOST_WAKE      (44)
#define FLYER_GPIO_BT_CHIP_WAKE      (50)
#define FLYER_GPIO_BT_UART1_RTS      (134)
#define FLYER_GPIO_BT_UART1_CTS      (135)
#define FLYER_GPIO_BT_UART1_RX       (136)
#define FLYER_GPIO_BT_UART1_TX       (137)
#define FLYER_GPIO_BT_PCM_OUT        (138)
#define FLYER_GPIO_BT_PCM_IN         (139)
#define FLYER_GPIO_BT_PCM_SYNC       (140)
#define FLYER_GPIO_BT_PCM_CLK        (141)

/* USB */
#define FLYER_GPIO_USB_ID2_PIN       (86)
#define FLYER_GPIO_USB_ID_PIN        (145)
#define FLYER_GPIO_USB_ID_PIN_XC     (42)
#define FLYER_GPIO_USB_MHL_SEL_XC    (56)
#define FLYER_GPIO_MHL_RESET_XC      (89)
#define FLYER_GPIO_MHL_POWER_XD      (143)

/* Camera */

#define FLYER_CAM_PWD			 	 (37)
#define FLYER_CAM_RST_XC             (151)
#define FLYER_CAM_RST                (124)
#define FLYER_CLK_SWITCH_XC			 (150)
#define FLYER_CLK_SWITCH			 (128) /* camera select pin */
#define FLYER_CAM2_PWD_XC            (79)
#define FLYER_CAM2_PWD_XD			 (35)
#define FLYER_CAM2_PWD               (180)
#define FLYER_CAM2_RST               (31)

/* MHL */
#define FLYER_MHL_RSTz                (157)
#define FLYER_MHL_INT                 (180)
#define FLYER_MHL_3V3_EN              (49)
#define FLYER_MHL_SW                  (56)
#define FLYER_MHL_I2C_TPI             (0x72)
#define FLYER_MHL_I2C_CBUS            (0xC8)


#define FLYER_SPI_CLK                (45)
#define FLYER_SPI_DO                 (47)
#define FLYER_SPI_DI                 (48)
#define FLYER_SPI_CS2                (87)

/* PMIC */
#define PMIC_GPIO_INT                (27)
#define FLYER_GPIO_PS_HOLD           (29)

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define FLYER_AUD_LO_EN_XD     PMGPIO(4)
#define FLYER_CSA_INTz_XC      PMGPIO(5)
#define FLYER_PEN_LED1         PMGPIO(10)
#define FLYER_PEN_LED2         PMGPIO(11)
#define FLYER_PEN_LED3         PMGPIO(12)
#define FLYER_PEN_LED4         PMGPIO(13)
#define FLYER_PEN_LED5         PMGPIO(14)
#define FLYER_NC_PMIC13         PMGPIO(13)
#define FLYER_NC_PMIC14         PMGPIO(14)
#define FLYER_PEN_LED6         PMGPIO(15)
#define FLYER_UART_EN	         PMGPIO(15)
#define FLYER_AUD_SPKL_EN      PMGPIO(16)
#define FLYER_AUD_SPKR_EN      PMGPIO(18)
#define FLYER_AUD_HP_EN        PMGPIO(19)
#define FLYER_GPIO_COMPASS_INT_XD      PMGPIO(20)
#define FLYER_TP_RSTz          PMGPIO(21)
#define FLYER_LED_3V3_EN       PMGPIO(22)
#define FLYER_SDMC_CD_N        PMGPIO(23)
#define FLYER_VOL_UP           PMGPIO(24)
#define FLYER_VOL_DN           PMGPIO(25)
#define FLYER_VOL_DN_XC        PMGPIO(24)
#define FLYER_VOL_UP_XC        PMGPIO(25)
#define FLYER_AUD_HP_DETz      PMGPIO(26)
#define FLYER_ADP_9V           PMGPIO(31)
#define FLYER_9V_AC_DETECT           PMGPIO(32)
#define FLYER_GSENSOR_INTz     PMGPIO(35)
#define FLYER_SLEEP_CLK2       PMGPIO(39)
#define FLYER_TP_ATT_PMIC	   PMGPIO(1)
#define FLYER_TP_ATT		   PMGPIO(2)
#define FLYER_2ND_CAM_LED	PMGPIO(3)
#define FLYER_H2W_CABLE_IN1    PMGPIO(9)
#define FLYER_H2W_CABLE_IN2    PMGPIO(33)
#define FLYER_H2W_IO1_CLK      PMGPIO(17)
#define FLYER_H2W_IO2_DAT      PMGPIO(37)

#ifdef CONFIG_MICROP_COMMON
void __init flyer_microp_init(void);
#endif
int flyer_init_mmc(unsigned int sys_rev);
void __init flyer_audio_init(void);
int __init flyer_init_keypad(void);
int __init flyer_wifi_init(void);
int __init flyer_init_panel(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_FLYER_H */
