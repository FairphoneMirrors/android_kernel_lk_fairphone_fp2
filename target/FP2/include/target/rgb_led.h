/*
 * Copyright 2018 Fairphone B.V.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _TARGET_FP2_RGB_LED_H_
#define _TARGET_FP2_RGB_LED_H_

#include <pm8x41.h>

// Slave-ID
#define SLAVE_ID                  0x10000

// RGB module registers
#define RGB_DRIVER_BASE_ADDR      0xD000
#define RGB_DRIVER_LED_SRC_SEL    0x45
#define RGB_DRIVER_EN_CTL         0x46
#define RGB_LED_VALUE_RED         0x80
#define RGB_LED_VALUE_GREEN       0x40
#define RGB_LED_VALUE_BLUE        0x20

// LPG module registers
#define LPG_DRIVER_BASE_ADDR      0xB000
#define LPG_DRIVER_LED_RED        0x700
#define LPG_DRIVER_LED_GREEN      0x600
#define LPG_DRIVER_LED_BLUE       0x500
#define LPG_PATTERN_CONFIG        0x40
#define LPG_PWM_SIZE_CLK          0x41
#define LPG_PWM_FREQ_PREDIV       0x42
#define LPG_PWM_TYPE_CONFIG       0x43
#define LPG_VALUE_LSB             0x44
#define LPG_VALUE_MSB             0x45
#define LPG_ENABLE_CONTROL        0x46

typedef enum rgb_led_return_code
{
    RGB_LED_SUCCESS,
    RGB_LED_GENERIC_ERROR,
    RGB_LED_INVALID_PARAMETER
} rgb_led_return_code;

typedef enum rgb_led_brightness
{
    RGB_LED_BRIGHTNESS_LOW = 0x80,
    RGB_LED_BRIGHTNESS_MID = 0x7F,
    RGB_LED_BRIGHTNESS_HIG = 0xFF
} rgb_led_brightness;

rgb_led_return_code led_init(void);
rgb_led_return_code led_enable(uint8_t led, rgb_led_brightness brightness);
rgb_led_return_code led_blink_enable(uint8_t led, uint8_t pwm_freq, uint8_t duty_cycle);
rgb_led_return_code led_disable(uint8_t led);

rgb_led_return_code led_deinit(void);

#endif  //_TARGET_FP2_RGB_LED_H_
