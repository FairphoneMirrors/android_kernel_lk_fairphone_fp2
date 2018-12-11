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

#include <target/rgb_led.h>

rgb_led_return_code led_init()
{
    /* Enable RGB module, Select Vph_pwr as power source */
    pm8x41_reg_write( SLAVE_ID + RGB_DRIVER_BASE_ADDR + RGB_DRIVER_LED_SRC_SEL, 0x01);

    return RGB_LED_SUCCESS;
}

rgb_led_return_code led_enable(uint8_t led, rgb_led_brightness brightness)
{
    uint32_t led_lpg_value = 0;
    switch(led)
    {
        case RGB_LED_VALUE_RED:
            led_lpg_value = LPG_DRIVER_LED_RED;
            break;
        case RGB_LED_VALUE_GREEN:
            led_lpg_value = LPG_DRIVER_LED_GREEN;
            break;
        case RGB_LED_VALUE_BLUE:
            led_lpg_value = LPG_DRIVER_LED_BLUE;
            break;
        default:
            return RGB_LED_INVALID_PARAMETER;
    }
    /* Building LED specific base address */
    const uint32_t led_lpg_base_address = SLAVE_ID + LPG_DRIVER_BASE_ADDR + led_lpg_value;
    /* Enable selected LED, preserving the previous value */
    uint8_t val = pm8x41_reg_read( SLAVE_ID + RGB_DRIVER_BASE_ADDR + RGB_DRIVER_EN_CTL );
    val |= led;
    pm8x41_reg_write( SLAVE_ID + RGB_DRIVER_BASE_ADDR + RGB_DRIVER_EN_CTL, val);
    /*
    *  Enable PWM at requested duty cycle
    *  For a always-on behaviour no pattern is needed.
    *  PWM is set to 7-bit mode in order to be able to use both PWM channels (4 mA + 8 mA).
    *  PWM frequency is set to 390 Hz (Pre-divide=5, Exponent=7).
    *  The type config is not used here but is mandatory to set the register.
    *  PWM value acts here as brightness parameter. 0xFF is the full-scale value.
    *  PWM value MSB is useless here.
    *  PWM is enabled by writing 0xE4 in the enable register.
    */
    pm8x41_reg_write( led_lpg_base_address + LPG_PATTERN_CONFIG, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_PWM_SIZE_CLK, 0x13);
    pm8x41_reg_write( led_lpg_base_address + LPG_PWM_FREQ_PREDIV, 0x47);
    pm8x41_reg_write( led_lpg_base_address + LPG_PWM_TYPE_CONFIG, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_VALUE_LSB, brightness);
    pm8x41_reg_write( led_lpg_base_address + LPG_VALUE_MSB, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_ENABLE_CONTROL, 0xE4);

    return RGB_LED_SUCCESS;
}

rgb_led_return_code led_blink_enable(uint8_t led, uint8_t pwm_freq, uint8_t duty_cycle)
{
    if(duty_cycle > 0x3F)
    {
        return RGB_LED_INVALID_PARAMETER;
    }

    uint32_t led_lpg_value = 0;
    switch(led)
    {
        case RGB_LED_VALUE_RED:
            led_lpg_value = LPG_DRIVER_LED_RED;
            break;
        case RGB_LED_VALUE_GREEN:
            led_lpg_value = LPG_DRIVER_LED_GREEN;
            break;
        case RGB_LED_VALUE_BLUE:
            led_lpg_value = LPG_DRIVER_LED_BLUE;
            break;
        default:
            return RGB_LED_INVALID_PARAMETER;
    }
    /* Building LED specific base address */
    const uint32_t led_lpg_base_address = SLAVE_ID + LPG_DRIVER_BASE_ADDR + led_lpg_value;
    /* Enable selected LED, preserving the previous value */
    uint8_t val = pm8x41_reg_read( SLAVE_ID + RGB_DRIVER_BASE_ADDR + RGB_DRIVER_EN_CTL );
    val |= led;
    pm8x41_reg_write( SLAVE_ID + RGB_DRIVER_BASE_ADDR + RGB_DRIVER_EN_CTL, val);
    /*
    *  Enable PWM at requested duty cycle
    *  For a simple blinking behaviour no pattern is needed.
    *  PWM is set to 6-bit mode. This results in a clock of 1 KHz.
    *  PWM frequency is set as desidered. Refer to docs to properly set this parameter.
    *  The type config is not used here but is mandatory to set the register.
    *  PWM value acts here as duty cycle value. 0x3F is the full-scale value.
    *  PWM value MSB is useless here.
    *  PWM is enabled by writing 0xE4 in the enable register.
    */
    pm8x41_reg_write( led_lpg_base_address + LPG_PATTERN_CONFIG, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_PWM_SIZE_CLK, 0x01);
    pm8x41_reg_write( led_lpg_base_address + LPG_PWM_FREQ_PREDIV, pwm_freq);
    pm8x41_reg_write( led_lpg_base_address + LPG_PWM_TYPE_CONFIG, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_VALUE_LSB, duty_cycle);
    pm8x41_reg_write( led_lpg_base_address + LPG_VALUE_MSB, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_ENABLE_CONTROL, 0xE4);

    return RGB_LED_SUCCESS;
}

rgb_led_return_code led_disable(uint8_t led)
{
    uint32_t led_lpg_value = 0;
    switch(led)
    {
        case RGB_LED_VALUE_RED:
            led_lpg_value = LPG_DRIVER_LED_RED;
            break;
        case RGB_LED_VALUE_GREEN:
            led_lpg_value = LPG_DRIVER_LED_GREEN;
            break;
        case RGB_LED_VALUE_BLUE:
            led_lpg_value = LPG_DRIVER_LED_BLUE;
            break;
        default:
            return RGB_LED_INVALID_PARAMETER;
    }
    /* Building LED specific base address */
    const uint32_t led_lpg_base_address = SLAVE_ID + LPG_DRIVER_BASE_ADDR + led_lpg_value;
    /* Disable selected LED, preserving the previous value */
    uint8_t val = pm8x41_reg_read( SLAVE_ID + RGB_DRIVER_BASE_ADDR + RGB_DRIVER_EN_CTL );
    val &= ~(led);
    pm8x41_reg_write( SLAVE_ID + RGB_DRIVER_BASE_ADDR + RGB_DRIVER_EN_CTL, val);

    /* Clear LPG registers */
    pm8x41_reg_write( led_lpg_base_address + LPG_PATTERN_CONFIG, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_PWM_SIZE_CLK, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_PWM_FREQ_PREDIV, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_PWM_TYPE_CONFIG, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_VALUE_LSB, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_VALUE_MSB, 0x00);
    pm8x41_reg_write( led_lpg_base_address + LPG_ENABLE_CONTROL, 0x00);

    return RGB_LED_SUCCESS;
}

rgb_led_return_code led_deinit()
{
    rgb_led_return_code rc;
    rc = led_disable(RGB_LED_VALUE_RED);
    if(rc != RGB_LED_SUCCESS)
    {
        return rc;
    }
    rc = led_disable(RGB_LED_VALUE_GREEN);
    if(rc != RGB_LED_SUCCESS)
    {
        return rc;
    }
    rc = led_disable(RGB_LED_VALUE_BLUE);
    if(rc != RGB_LED_SUCCESS)
    {
        return rc;
    }

    /* Power off the RGB module by setting no power source */
    pm8x41_reg_write( SLAVE_ID + RGB_DRIVER_BASE_ADDR + RGB_DRIVER_LED_SRC_SEL, 0x00);

    return RGB_LED_SUCCESS;
}
