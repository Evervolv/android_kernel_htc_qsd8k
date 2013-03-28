/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LEDS_MAX8957_LPG_H__
#define __LEDS_MAX8957_LPG_H__

#include <linux/leds.h>

#define MAXIM_LEDS_LPG_DEBUG	1

/*===========================================================================

Definitions for Maxim MAX8957/MAX8967 LEDs

============================================================================*/
#define MAX8957_REG_LED0BRT         0xAA
#define MAX8957_REG_LED1BRT         0xAB
#define MAX8957_REG_LED2BRT         0xAC
#define MAX8957_REG_LED3BRT         0xAD
#define MAX8957_REG_LED4BRT         0xAE
#define MAX8957_REG_LED5BRT         0xAF

#define MAX8957_LEDBRT_MASK         0xFF
#define MAX8957_LEDBRT_SHIFT        0

#define MAX8957_REG_LED0CNTL1       0xB0
#define MAX8957_REG_LED1CNTL1       0xB1
#define MAX8957_REG_LED2CNTL1       0xB2
#define MAX8957_REG_LED3CNTL1       0xB3
#define MAX8957_REG_LED4CNTL1       0xB4
#define MAX8957_REG_LED5CNTL1       0xB5

#define MAX8957_MODE_MASK           0x03
#define MAX8957_MODE_SHIFT          0
#define MAX8957_BLINKON_MASK        0x0C
#define MAX8957_BLINKON_SHIFT       2
#define MAX8957_BLINKPER_MASK       0x30
#define MAX8957_BLINKPER_SHIFT      4
#define MAX8957_BLINKDELAY_MASK     0xC0
#define MAX8957_BLINKDELAY_SHIFT    6

#define MAX8957_REG_LED0CNTL2       0xB6
#define MAX8957_REG_LED1CNTL2       0xB7
#define MAX8957_REG_LED2CNTL2       0xB8
#define MAX8957_REG_LED3CNTL2       0xB9
#define MAX8957_REG_LED4CNTL2       0xBA
#define MAX8957_REG_LED5CNTL2       0xBB

#define MAX8957_MINHLDTIME_MASK     0x1C
#define MAX8957_MINHLDTIME_SHIFT    2
#define MAX8957_MAXHLDTIME_MASK     0xE0
#define MAX8957_MAXHLDTIME_SHIFT    5

#define MAX8957_REG_LED0CNTL3       0xBC
#define MAX8957_REG_LED1CNTL3       0xBD
#define MAX8957_REG_LED2CNTL3       0xBE
#define MAX8957_REG_LED3CNTL3       0xBF
#define MAX8957_REG_LED4CNTL3       0xC0
#define MAX8957_REG_LED5CNTL3       0xC1

#define MAX8957_RAMPRATE_MASK       0x1F
#define MAX8957_RAMPRATE_SHIFT      0

#define MAX8957_REG_LEDEN           0xC2

#define MAX8957_LED0EN_MASK         0x01
#define MAX8957_LED0EN_SHIFT        0
#define MAX8957_LED1EN_MASK         0x02
#define MAX8957_LED1EN_SHIFT        1
#define MAX8957_LED2EN_MASK         0x04
#define MAX8957_LED2EN_SHIFT        2
#define MAX8957_LED3EN_MASK         0x08
#define MAX8957_LED3EN_SHIFT        3
#define MAX8957_LED4EN_MASK         0x10
#define MAX8957_LED4EN_SHIFT        4
#define MAX8957_LED5EN_MASK         0x20
#define MAX8957_LED5EN_SHIFT        5
#define MAX8957_WDEN_MASK           0x40
#define MAX8957_WDEN_SHIFT          6
#define MAX8957_LEDINTM_MASK        0x80
#define MAX8957_LEDINTM_SHIFT       7

#define MAX8957_LED_DISABLE         0
#define MAX8957_LED_ENABLE          1

#define MAX8957_REG_LEDGRP          0xC3

#define MAX8957_LEDGRP1_MASK        0x07
#define MAX8957_LEDGRP1_SHIFT       0
#define MAX8957_LEDGRP2_MASK        0x38
#define MAX8957_LEDGRP2_SHIFT       3
#define MAX8957_WDTIME_MASK         0xC0
#define MAX8957_WDTIME_SHIFT        6

#define MAX8957_REG_LEDSTTS         0xC4
#define MAX8957_LED0SHORT_MASK      0x01
#define MAX8957_LED0SHORT_SHIFT     0
#define MAX8957_LED1SHORT_MASK      0x02
#define MAX8957_LED1SHORT_SHIFT     1
#define MAX8957_LED2SHORT_MASK      0x04
#define MAX8957_LED2SHORT_SHIFT     2
#define MAX8957_LED3SHORT_MASK      0x08
#define MAX8957_LED3SHORT_SHIFT     3
#define MAX8957_LED4SHORT_MASK      0x10
#define MAX8957_LED4SHORT_SHIFT     4
#define MAX8957_LED5SHORT_MASK      0x20
#define MAX8957_LED5SHORT_SHIFT     5

/*
#define MAX8957_REG_LEDINT          0xC6
#define MAX8957_LEDINT_MASK         0x01
#define MAX8957_LEDINT_SHIFT        0
*/

#define MAX8957_REG_LED0PATTERN0    0xC7
#define MAX8957_REG_LED0PATTERN1    0xC8
#define MAX8957_REG_LED0PATTERN2    0xC9
#define MAX8957_REG_LED0PATTERN3    0xCA
#define MAX8957_REG_LED0PATTERN4    0xCB
#define MAX8957_REG_LED0PATTERN5    0xCC

#define MAX8957_REG_LED1PATTERN0    0xCD
#define MAX8957_REG_LED1PATTERN1    0xCE
#define MAX8957_REG_LED1PATTERN2    0xCF
#define MAX8957_REG_LED1PATTERN3    0xD0
#define MAX8957_REG_LED1PATTERN4    0xD1
#define MAX8957_REG_LED1PATTERN5    0xD2

#define MAX8957_REG_LED2PATTERN0    0xD3
#define MAX8957_REG_LED2PATTERN1    0xD4
#define MAX8957_REG_LED2PATTERN2    0xD5
#define MAX8957_REG_LED2PATTERN3    0xD6
#define MAX8957_REG_LED2PATTERN4    0xD7
#define MAX8957_REG_LED2PATTERN5    0xD8

#define MAX8957_REG_LED3PATTERN0    0xD9
#define MAX8957_REG_LED3PATTERN1    0xDA
#define MAX8957_REG_LED3PATTERN2    0xDB
#define MAX8957_REG_LED3PATTERN3    0xDC
#define MAX8957_REG_LED3PATTERN4    0xDD
#define MAX8957_REG_LED3PATTERN5    0xDE

#define MAX8957_REG_LED4PATTERN0    0xDF
#define MAX8957_REG_LED4PATTERN1    0xE0
#define MAX8957_REG_LED4PATTERN2    0xE1
#define MAX8957_REG_LED4PATTERN3    0xE2
#define MAX8957_REG_LED4PATTERN4    0xE3
#define MAX8957_REG_LED4PATTERN5    0xE4

#define MAX8957_REG_LED5PATTERN0    0xE5
#define MAX8957_REG_LED5PATTERN1    0xE6
#define MAX8957_REG_LED5PATTERN2    0xE7
#define MAX8957_REG_LED5PATTERN3    0xE8
#define MAX8957_REG_LED5PATTERN4    0xE9
#define MAX8957_REG_LED5PATTERN5    0xEA

#define MAX8957_POINT1_MASK         0x07
#define MAX8957_POINT1_SHIFT        0
#define MAX8957_POINT2_MASK         0x38
#define MAX8957_POINT2_SHIFT        3
#define MAX8957_POINT3_B1_B0_MASK   0xC0
#define MAX8957_POINT3_B1_B0_SHIFT  6
#define MAX8957_POINT3_B2_MASK      0x01
#define MAX8957_POINT3_B2_SHIFT     0
#define MAX8957_POINT4_MASK         0x0E
#define MAX8957_POINT4_SHIFT        1
#define MAX8957_POINT5_MASK         0x70
#define MAX8957_POINT5_SHIFT        4
#define MAX8957_POINT6_B0_MASK      0x80
#define MAX8957_POINT6_B0_SHIFT     7
#define MAX8957_POINT6_B2_B1_MASK   0x03
#define MAX8957_POINT6_B2_B1_SHIFT  0
#define MAX8957_POINT7_MASK         0x1C
#define MAX8957_POINT7_SHIFT        2
#define MAX8957_POINT8_MASK         0xE0
#define MAX8957_POINT8_SHIFT        5
#define MAX8957_POINT9_MASK         0x07
#define MAX8957_POINT9_SHIFT        0
#define MAX8957_POINT10_MASK        0x38
#define MAX8957_POINT10_SHIFT       3
#define MAX8957_POINT11_B1_B0_MASK  0xC0
#define MAX8957_POINT11_B1_B0_SHIFT 6
#define MAX8957_POINT11_B2_MASK     0x01
#define MAX8957_POINT11_B2_SHIFT    0
#define MAX8957_POINT12_MASK        0x0E
#define MAX8957_POINT12_SHIFT       1
#define MAX8957_POINT13_MASK        0x70
#define MAX8957_POINT13_SHIFT       4
#define MAX8957_POINT14_B0_MASK     0x80
#define MAX8957_POINT14_B0_SHIFT    7
#define MAX8957_POINT14_B2_B1_MASK  0x03
#define MAX8957_POINT14_B2_B1_SHIFT 0
#define MAX8957_POINT15_MASK        0x1C
#define MAX8957_POINT15_SHIFT       2
#define MAX8957_POINT16_MASK        0xE0
#define MAX8957_POINT16_SHIFT       5

#define MAX8957_D2_UPDOWN_MASK      0x04
#define MAX8957_D2_DOWN_VAL         0x00
#define MAX8957_D2_UP_VAL           0x00
#define MAX8957_D1D0_RAMPRATE_MASK  0x03

#if 1 /*MAXIM_PMIC_8957_PORTING - LED Blink*/
#define BLINK_TO_ON(x) ((x <= 64) ? 0 :		\
			(x <= 125) ? 1 :		\
			(x <= 250) ? 2 :		\
			(x <= 500) ? 3 : -EINVAL)

#define BLINK_TO_PERIOD(x)	((x <= 1000) ? 1 :		\
				(x <= 2000) ? 2 :			\
				(x <= 5000) ? 3 : -EINVAL)
#else /*MAXIM_PMIC_8957_PORTING - LED Blink*/
#define BLINK_TO_ON(x) ((x == 64) ? 0 :		\
			(x == 125) ? 1 :		\
			(x == 250) ? 2 :		\
			(x == 500) ? 3 : -EINVAL)

#define BLINK_TO_PERIOD(x)	((x == 1000) ? 1 :		\
				(x == 2000) ? 2 :			\
				(x == 5000) ? 3 : -EINVAL)
#endif /*MAXIM_PMIC_8957_PORTING - LED Blink*/

#define MAX8957_MAX_LEDS		6

enum max8957_lpg_leds {
	MAX8957_ID_LED0 = 0,
	MAX8957_ID_LED1,
	MAX8957_ID_LED2,
	MAX8957_ID_LED3,
	MAX8957_ID_LED4,
	MAX8957_ID_LED5,
	MAX8957_ID_LEDMAX,
};

enum max8957_pattern_ramprate {
	RAMPRATE_JUMP,
	RAMPRATE_2MS,
	RAMPRATE_4MS,
	RAMPRATE_8MS,
};

enum max8957_lpg_mode {
	BLINK_MODE,
	PATTERN_MODE_NO_REPEAT,
	PATTERN_MODE_REPEAT,
};

enum max8957_lpg_ledgrp1 {
	GRP1_INDEPENDENT,
	LED0_LED1_SYNC,
	LED0_LED2_SYNC,
	LED1_LED2_SYNC,
	LED0_LED1_LED2_SYNC,
};

enum max8957_lpg_ledgrp2 {
	GRP2_INDEPENDENT,
	LED3_LED4_SYNC,
	LED3_LED5_SYNC,
	LED4_LED5_SYNC,
	LED3_LED4_LED5_SYNC,
};

enum max8957_lpg_ramprate {
	LPG_RAMPRATE_JUMP,
	LPG_RAMPRATE_1MS,
	LPG_RAMPRATE_2MS,
	LPG_RAMPRATE_3MS,
	LPG_RAMPRATE_4MS,
	LPG_RAMPRATE_5MS,
	LPG_RAMPRATE_6MS,
	LPG_RAMPRATE_7MS,
	LPG_RAMPRATE_8MS,
	LPG_RAMPRATE_9MS,
	LPG_RAMPRATE_10MS,
	LPG_RAMPRATE_11MS,
	LPG_RAMPRATE_12MS,
	LPG_RAMPRATE_13MS,
	LPG_RAMPRATE_14MS,
	LPG_RAMPRATE_15MS,
	LPG_RAMPRATE_16MS,
	LPG_RAMPRATE_17MS,
	LPG_RAMPRATE_18MS,
	LPG_RAMPRATE_19MS,
	LPG_RAMPRATE_20MS,
	LPG_RAMPRATE_21MS,
	LPG_RAMPRATE_22MS,
	LPG_RAMPRATE_23MS,
	LPG_RAMPRATE_24MS,
	LPG_RAMPRATE_25MS,
	LPG_RAMPRATE_26MS,
	LPG_RAMPRATE_27MS,
	LPG_RAMPRATE_28MS,
	LPG_RAMPRATE_29MS,
	LPG_RAMPRATE_30MS,
	LPG_RAMPRATE_31MS,
};

enum max8957_lpg_holdtime {
    LPG_HOLD_0MS,
    LPG_HOLD_8MS,
    LPG_HOLD_16MS,
    LPG_HOLD_32MS,
    LPG_HOLD_64MS,
    LPG_HOLD_128MS,
    LPG_HOLD_256MS,
    LPG_HOLD_512MS,
};

enum max8957_lpg_point_direction {
    LPG_POINT_DIR_DOWN,
    LPG_POINT_DIR_UP,
};

enum max8957_lpg_point_ramprate {
    LPG_POINT_RAMP_JUMP,
    LPG_POINT_RAMP_2MS,
    LPG_POINT_RAMP_4MS,
    LPG_POINT_RAMP_8MS,
};

struct max8957_lpg_pattern {
	u64 p1_ramprate:2;
	u64 p1_direction:1;
	u64 p2_ramprate:2;
	u64 p2_direction:1;
	u64 p3_ramprate:2;
	u64 p3_direction:1;
	u64 p4_ramprate:2;
	u64 p4_direction:1;
	u64 p5_ramprate:2;
	u64 p5_direction:1;
	u64 p6_ramprate:2;
	u64 p6_direction:1;
	u64 p7_ramprate:2;
	u64 p7_direction:1;
	u64 p8_ramprate:2;
	u64 p8_direction:1;
	u64 p9_ramprate:2;
	u64 p9_direction:1;
	u64 p10_ramprate:2;
	u64 p10_direction:1;
	u64 p11_ramprate:2;
	u64 p11_direction:1;
	u64 p12_ramprate:2;
	u64 p12_direction:1;
	u64 p13_ramprate:2;
	u64 p13_direction:1;
	u64 p14_ramprate:2;
	u64 p14_direction:1;
	u64 p15_ramprate:2;
	u64 p15_direction:1;
	u64 p16_ramprate:2;
	u64 p16_direction:1;
	u64 reserved:16;
};

struct max8957_lpg {
	const char	*name;
	int		id;
	int		ledgrp;
	int		ramprate;
	int		holdtime;
};

struct max8957_lpg_platform_data {
	int			num_leds;
	struct max8957_lpg	*leds;
};
void button_backlight_flash(int brightness_key);

int max8957_lpg_set_pattern(struct led_classdev *led_cdev,
			struct max8957_lpg_pattern *lpg_pattern,
			u8 max_hold_time,
			u8 min_hole_time,
			u8 max_brightness);
#endif /* __LEDS_MAX8957_LPG_H__ */
