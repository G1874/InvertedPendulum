/*************************************************************************

Title:    as5600.h - Driver for AMS AS5600 12-Bit Programmable Contactless Potentiometer

Original author:   Nicholas Morrow <nickhudspeth@gmail.com> github: nicholasmorrow

Modified by: Grzegorz Niedziółka

Modifications included: repurposing the drivers to work with STM32L1xx hardware,
adding low level functions for better code readability, changing the error handling,
general changes meant to make the drivers work better with the rest of the project
(github: https://github.com/G1874/InvertedPendulum)

File:     as5600.h
Software: STM32L1xx_HAL_Driver
Hardware: STM32L1xx
License:  The MIT License (MIT)


LICENSE:
    Copyright (C) 2018 Pathogen Systems, Inc. dba Crystal Diagnostics

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*************************************************************************/

#ifndef AS5600_H_
#define AS5600_H_

/* HAL library for stm32l1 */
#include "stm32l1xx_hal.h"

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/

/* Slave address is shifted left for proper transmission */
#define AS5600_I2C_ADDRESS (0x36 << 1)

/* AS5600 CONFIGURATION REGISTERS */
#define AS5600_REGISTER_ZMCO 0x00
#define AS5600_REGISTER_ZPOS_HIGH 0x01
#define AS5600_REGISTER_ZPOS_LOW 0x02
#define AS5600_REGISTER_MPOS_HIGH 0x03
#define AS5600_REGISTER_MPOS_LOW 0x04
#define AS5600_REGISTER_MANG_HIGH 0x05
#define AS5600_REGISTER_MANG_LOW 0x06
#define AS5600_REGISTER_CONF_HIGH 0x07
#define AS5600_REGISTER_CONF_LOW 0x08

/* AS5600 OUTPUT REGISTERS */
#define AS5600_REGISTER_RAW_ANGLE_HIGH 0x0C
#define AS5600_REGISTER_RAW_ANGLE_LOW 0x0D
#define AS5600_REGISTER_ANGLE_HIGH 0x0E
#define AS5600_REGISTER_ANGLE_LOW 0x0F

/* AS5600 STATUS REGISTERS */
#define AS5600_REGISTER_STATUS 0x0B
#define AS5600_REGISTER_AGC 0x1A
#define AS5600_REGISTER_MAGNITUDE_HIGH 0x1B
#define AS5600_REGISTER_MAGNITUDE_LOW 0x1C
#define AS5600_REGISTER_BURN 0xFF

/* AS5600 CONFIGUTRATION REGISTERS */
#define AS5600_POWER_MODE_NOM 1
#define AS5600_POWER_MODE_LPM1 2
#define AS5600_POWER_MODE_LPM2 3
#define AS5600_POWER_MODE_LPM3 4
#define AS5600_POWER_MODE_DEFAULT AS5600_POWER_MODE_NOM

#define AS5600_HYSTERESIS_OFF 1
#define AS5600_HYSTERESIS_1LSB 2
#define AS5600_HYSTERESIS_2LSB 3
#define AS5600_HYSTERESIS_3LSB 4
#define AS5600_HYSTERESIS_DEFAULT AS5600_HYSTERESIS_OFF

/* Ratiometric analog output ranging from GND-VCC */
#define AS5600_OUTPUT_STAGE_FULL 1
/* Ratiometric analog output ranging from 10% to 90% of VCC */
#define AS5600_OUTPUT_STAGE_REDUCED 2
/* Digital PWM output */
#define AS5600_OUTPUT_STAGE_PWM 3
#define AS5600_OUTPUT_STAGE_DEFAULT AS5600_OUTPUT_STAGE_FULL

#define AS5600_PWM_FREQUENCY_115HZ 1
#define AS5600_PWM_FREQUENCY_230HZ 2
#define AS5600_PWM_FREQUENCY_460HZ 3
#define AS5600_PWM_FREQUENCY_920HZ 4
#define AS5600_PWM_FREQUENCY_DEFAULT AS5600_PWM_FREQUENCY_115HZ

#define AS5600_SLOW_FILTER_16X 1
#define AS5600_SLOW_FILTER_8X 2
#define AS5600_SLOW_FILTER_4X 3
#define AS5600_SLOW_FILTER_2X 4
#define AS5600_SLOW_FILTER_DEFAULT AS5600_SLOW_FILTER_16X

#define AS5600_FAST_FILTER_SLOW_ONLY 1
#define AS5600_FAST_FILTER_6LSB 2
#define AS5600_FAST_FILTER_7LSB 3
#define AS5600_FAST_FILTER_9LSB 4
#define AS5600_FAST_FILTER_18LSB 5
#define AS5600_FAST_FILTER_21LSB 6
#define AS5600_FAST_FILTER_24LSB 7
#define AS5600_FAST_FILTER_10LSB 8
#define AS5600_FAST_FILTER_DEFAULT AS5600_FAST_FILTER_SLOW_ONLY

#define AS5600_WATCHDOG_OFF 1
#define AS5600_WATCHDOG_ON 2
#define AS5600_WATCHDOG_DEFAULT AS5600_WATCHDOG_ON

/* AS5600 STATUS DEFINITIONS */
/* Error bit indicates B-field is too strong */
#define AS5600_AGC_MIN_GAIN_OVERFLOW (uint8_t)(1UL << 3)
/* Error bit indicates B-field is too weak */
#define AS5600_AGC_MAX_GAIN_OVERFLOW (uint8_t)(1UL << 4)
/* Status bit indicates B-field is detected */
#define AS5600_MAGNET_DETECTED (uint8_t)(1UL << 5)

/* Clockwise direction */
#define AS5600_DIR_CW 1
/* Counter clockwise direction */
#define AS5600_DIR_CCW 2

/* 12 bit mask - 0000 1111 1111 1111 */
#define AS5600_12_BIT_MASK (uint16_t)4095
/* Magnet status mask - 0011 1000 */
#define AS5600_MAGSTAT_MASK (uint8_t)56
/* Configuration register mask - 0011 1111 1111 1111 */
#define AS5600_CONFREG_MASK (uint16_t)16383

typedef struct
{
    I2C_HandleTypeDef* i2cHandle;

    GPIO_TypeDef* DirPort;
    uint16_t DirPin;

    uint8_t PositiveRotationDirection;
    uint8_t LowPowerMode;
    uint8_t Hysteresis;
    uint8_t OutputMode;
    uint8_t PWMFrequency;
    uint8_t SlowFilter;
    uint8_t FastFilterThreshold;
    uint8_t WatchdogTimer;

    volatile uint8_t confRegister[2];

} AS5600_TypeDef;

/***********************    FUNCTION PROTOTYPES    ***********************/

/* INITIALIZATION */
AS5600_TypeDef* AS5600_New(void);
HAL_StatusTypeDef AS5600_Init(AS5600_TypeDef* const dev, I2C_HandleTypeDef* const i2cHandle);

/* LOW-LEVEL FUNCTIONS */
HAL_StatusTypeDef AS5600_ReadRegister(AS5600_TypeDef* const dev, uint8_t reg, uint8_t* const data);
HAL_StatusTypeDef AS5600_ReadRegisters(AS5600_TypeDef* const dev, uint8_t reg, uint8_t* const data, uint8_t length);

HAL_StatusTypeDef AS5600_WriteRegister(AS5600_TypeDef* const dev, uint8_t reg, uint8_t* const data);
HAL_StatusTypeDef AS5600_WriteRegisters(AS5600_TypeDef* const dev, uint8_t reg, uint8_t* const data, uint8_t length);

/* HIGH-LEVEL FUNCTIONS */
HAL_StatusTypeDef AS5600_SetStartPosition(AS5600_TypeDef* const dev, const uint16_t pos);
HAL_StatusTypeDef AS5600_SetStopPosition(AS5600_TypeDef* const dev, const uint16_t pos);
HAL_StatusTypeDef AS5600_SetMaxAngle(AS5600_TypeDef* const dev, const uint16_t angle);

HAL_StatusTypeDef AS5600_SetPositiveRotationDirection(AS5600_TypeDef* const dev, const uint8_t dir);
HAL_StatusTypeDef AS5600_SetLowPowerMode(AS5600_TypeDef* const dev, const uint8_t mode);
HAL_StatusTypeDef AS5600_SetHysteresis(AS5600_TypeDef* const dev, const uint8_t hysteresis);
HAL_StatusTypeDef AS5600_SetOutputMode(AS5600_TypeDef* const dev, const uint8_t mode);
HAL_StatusTypeDef AS5600_SetPwmFrequency(AS5600_TypeDef* const dev, const uint8_t freq);
HAL_StatusTypeDef AS5600_SetSlowFilter(AS5600_TypeDef* const dev, const uint8_t mode);
HAL_StatusTypeDef AS5600_SetFastFilterThreshold(AS5600_TypeDef* const dev, const uint8_t threshold);
HAL_StatusTypeDef AS5600_SetWatchdogTimer(AS5600_TypeDef* const dev, const uint8_t mode);

HAL_StatusTypeDef AS5600_GetRawAngle(AS5600_TypeDef* const dev, uint16_t* const angle);
HAL_StatusTypeDef AS5600_GetAngle(AS5600_TypeDef* const dev, uint16_t* const angle);
HAL_StatusTypeDef AS5600_GetMagnetStatus(AS5600_TypeDef* const dev, uint8_t* const stat);
HAL_StatusTypeDef AS5600_GetAGCSetting(AS5600_TypeDef* const dev, uint8_t* const agc);
HAL_StatusTypeDef AS5600_GetCORDICMagnitude(AS5600_TypeDef* const dev, uint16_t* const mag);
HAL_StatusTypeDef AS5600_GetConfRegister(AS5600_TypeDef* const dev, uint16_t* const conf);

#endif /* AS5600_H_ */
