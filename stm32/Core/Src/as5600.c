/*************************************************************************

Title:    as5600.c - Driver for AMS AS5600 12-Bit Programmable Contactless Potentiometer

Original author:   Nicholas Morrow <nickhudspeth@gmail.com> github: nicholasmorrow

Modified by: Grzegorz Niedziółka

Modifications included: repurposing the drivers to work with STM32L1xx hardware,
adding low level functions for better code readability, changing the error handling,
general changes meant to make the drivers work better with the rest of the project
(github: https://github.com/G1874/InvertedPendulum)

File:     as5600.c
Software: STM32L1xx_HAL_Driver
Hardware: STM32L1xx
License:  The MIT License (MIT)


LICENSE:
    Copyright (C) 2018 Nicholas Morrow

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

#include "as5600.h"

/*******************    FUNCTION IMPLEMENTATIONS    ********************/

/* INITIALIZATION */
AS5600_TypeDef* AS5600_New(void)
{
    AS5600_TypeDef* dev = (AS5600_TypeDef*)calloc(1, sizeof(AS5600_TypeDef));
    return dev;
}

uint8_t AS5600_Init(AS5600_TypeDef* const dev, I2C_HandleTypeDef* const i2cHandle)
{
    uint8_t errNum = 0;
    uint8_t pwm = 0;
    uint8_t mag_status = 0;

    /* Set default configuration for uninitialized values */
    if (!(dev->i2cHandle)) {
    	dev->i2cHandle = i2cHandle;
    }
    if (!(dev->PositiveRotationDirection)) {
        dev->PositiveRotationDirection = AS5600_DIR_CW;
    }
    if (!(dev->LowPowerMode)) {
        dev->LowPowerMode = AS5600_POWER_MODE_DEFAULT;
    }
    if (!(dev->Hysteresis)) {
        dev->Hysteresis = AS5600_HYSTERESIS_DEFAULT;
    }
    if (!(dev->OutputMode)) {
        dev->OutputMode = AS5600_OUTPUT_STAGE_DEFAULT;
    }
    if (!(dev->PWMFrequency)) {
        dev->PWMFrequency = AS5600_PWM_FREQUENCY_DEFAULT;
    }
    if (!(dev->SlowFilter)) {
        dev->SlowFilter = AS5600_SLOW_FILTER_DEFAULT;
    }
    if (!(dev->FastFilterThreshold)) {
        dev->FastFilterThreshold = AS5600_FAST_FILTER_DEFAULT;
    }
    if (!(dev->WatchdogTimer)) {
        dev->WatchdogTimer = AS5600_WATCHDOG_DEFAULT;
    }

    /* Write configuration settings */
    switch (dev->LowPowerMode)
    {
        case AS5600_POWER_MODE_NOM:
            dev->confRegister[1] &= ~((1UL << 1) | (1UL << 0));
            break;
        case AS5600_POWER_MODE_LPM1:
            dev->confRegister[1] |= (1UL << 0);
            dev->confRegister[1] &= ~(1UL << 1);
            break;
        case AS5600_POWER_MODE_LPM2:
            dev->confRegister[1] |= (1UL << 1);
            dev->confRegister[1] &= (1UL << 0);
            break;
        case AS5600_POWER_MODE_LPM3:
            dev->confRegister[1] |= ((1UL << 1) | (1UL << 0));
            break;
        default:
            /* Invalid low power mode specified */
            errNum = 1;
            return errNum;
    }
    switch (dev->Hysteresis)
    {
        case AS5600_HYSTERESIS_OFF:
            dev->confRegister[1] &= ~((1UL << 3) | (1UL << 2));
            break;
        case AS5600_HYSTERESIS_1LSB:
            dev->confRegister[1] |= (1UL << 2);
            dev->confRegister[1] &= ~(1UL << 3);
            break;
        case AS5600_HYSTERESIS_2LSB:
            dev->confRegister[1] &= ~(1UL << 2);
            dev->confRegister[1] |= (1UL << 3);
            break;
        case AS5600_HYSTERESIS_3LSB:
            dev->confRegister[1] |= ((1UL << 3) | (1UL << 2));
            break;
        default:
            /* Invalid hysteresis mode specified */
        	errNum = 2;
        	return errNum;
    }
    switch (dev->OutputMode)
    {
        case AS5600_OUTPUT_STAGE_FULL:
            dev->confRegister[1] &= ~((1UL << 5) | (1UL << 4));
            break;
        case AS5600_OUTPUT_STAGE_REDUCED:
            dev->confRegister[1] |= (1UL << 4);
            dev->confRegister[1] &= ~(1UL << 5);
            break;
        case AS5600_OUTPUT_STAGE_PWM:
            dev->confRegister[1] &= ~(1UL << 4);
            dev->confRegister[1] |= (1UL << 5);
            pwm = 1;
            break;
        default:
            /* Invalid output mode specified */
        	errNum = 3;
        	return errNum;
    }
    if (pwm) {
        switch (dev->PWMFrequency)
        {
            case AS5600_PWM_FREQUENCY_115HZ:
                dev->confRegister[1] &= ~((1UL << 7) | (1UL << 6));
                break;
            case AS5600_PWM_FREQUENCY_230HZ:
                dev->confRegister[1] |= (1UL << 6);
                dev->confRegister[1] &= ~(1UL << 7);
                break;
            case AS5600_PWM_FREQUENCY_460HZ:
                dev->confRegister[1] &= ~(1UL << 6);
                dev->confRegister[1] |= (1UL << 7);
                break;
            case AS5600_PWM_FREQUENCY_920HZ:
                dev->confRegister[1] |= ((1UL << 7) | (1UL << 6));
                break;
            default:
                /* Invalid PWM frequency specified */
            	errNum = 4;
            	return errNum;
        }
    }
    switch (dev->SlowFilter)
    {
        case AS5600_SLOW_FILTER_16X:
            dev->confRegister[0] &= ~((1UL << 1) | (1UL << 0));
            break;
        case AS5600_SLOW_FILTER_8X:
            dev->confRegister[0] |= (1UL << 0);
            dev->confRegister[0] &= ~(1UL << 1);
            break;
        case AS5600_SLOW_FILTER_4X:
            dev->confRegister[0] &= ~(1UL << 0);
            dev->confRegister[0] |= (1UL << 1);
            break;
        case AS5600_SLOW_FILTER_2X:
            dev->confRegister[0] |= ((1UL << 1) | (1UL << 0));
            break;
        default:
            /* Invalid slow filter mode specified */
        	errNum = 5;
        	return errNum;
    }
    switch (dev->FastFilterThreshold)
    {
        case AS5600_FAST_FILTER_SLOW_ONLY:
            dev->confRegister[0] &= ~((1UL << 4) | (1UL << 3) | (1UL << 2));
            break;
        case AS5600_FAST_FILTER_6LSB:
            dev->confRegister[0] &= ~((1UL << 4) | (1UL << 3));
            dev->confRegister[0] |= (1UL << 2);
            break;
        case AS5600_FAST_FILTER_7LSB:
            dev->confRegister[0] &= ~((1UL << 4) | (1UL << 2));
            dev->confRegister[0] |= (1UL << 3);
            break;
        case AS5600_FAST_FILTER_9LSB:
            dev->confRegister[0] &= ~(1UL << 4);
            dev->confRegister[0] |= ((1UL << 3) | (1UL << 2));
            break;
        case AS5600_FAST_FILTER_18LSB:
            dev->confRegister[0] &= ~((1UL << 3) | (1UL << 2));
            dev->confRegister[0] |= (1UL << 4);
            break;
        case AS5600_FAST_FILTER_21LSB:
            dev->confRegister[0] &= ~(1UL << 3);
            dev->confRegister[0] |= ((1UL << 4) | (1UL << 2));
            break;
        case AS5600_FAST_FILTER_24LSB:
            dev->confRegister[0] &= ~(1UL << 2);
            dev->confRegister[0] |= ((1UL << 4) | (1UL << 3));
            break;
        case AS5600_FAST_FILTER_10LSB:
            dev->confRegister[0] |= ((1UL << 4) | (1UL << 3) | (1UL << 2));
            break;
        default:
            /* Invalid fast filter mode specified */
        	errNum = 6;
        	return errNum;;
    }
    switch (dev->WatchdogTimer)
    {
        case AS5600_WATCHDOG_OFF:
            dev->confRegister[0] &= ~(1UL << 5);
            break;
        case AS5600_WATCHDOG_ON:
            dev->confRegister[0] |= (1UL << 5);
            break;
        default:
            /* Invalid watchdog state specified */
        	errNum = 7;
        	return errNum;
    }
    if (AS5600_WriteRegisters(dev, AS5600_REGISTER_CONF_HIGH, (uint8_t*)dev->confRegister, 2) != HAL_OK) {
    	/* I2C write register error */
    	errNum = 8;
    	return errNum;
    }

    /* Check magnet status */
    if(AS5600_GetMagnetStatus(dev, &mag_status) != HAL_OK) {
    	/* I2C magnet status read error */
    	errNum = 9;
    	return errNum;
    }
    if ((mag_status & AS5600_AGC_MIN_GAIN_OVERFLOW)) {
        /* B-field is too strong */
    	errNum = 11;
    }
    if ((mag_status & AS5600_AGC_MAX_GAIN_OVERFLOW)) {
        /* B-field is too weak */
    	errNum = 12;
    }
    if (!(mag_status & AS5600_MAGNET_DETECTED)) {
        /* Magnet not detected */
        errNum = 10;
    }

    return errNum;
}

/* LOW-LEVEL FUNCTIONS */
HAL_StatusTypeDef AS5600_ReadRegister(AS5600_TypeDef* dev, uint8_t reg, uint8_t* data)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, AS5600_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AS5600_ReadRegisters(AS5600_TypeDef* dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, AS5600_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AS5600_WriteRegister(AS5600_TypeDef* dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(dev->i2cHandle, AS5600_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AS5600_WriteRegisters(AS5600_TypeDef* dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Write(dev->i2cHandle, AS5600_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/* HIGH-LEVEL FUNCTIONS	*/
HAL_StatusTypeDef AS5600_SetStartPosition(AS5600_TypeDef* const dev, uint16_t const pos)
{
    uint8_t data[2] = {0};

    data[0] = (uint8_t)((pos & AS5600_12_BIT_MASK) >> 8);
    data[1] = (uint8_t)pos;

    return AS5600_WriteRegisters(dev, AS5600_REGISTER_ZPOS_HIGH, data, 2);
}

HAL_StatusTypeDef AS5600_SetStopPosition(AS5600_TypeDef* const dev, uint16_t const pos)
{
    uint8_t data[2] = {0};
    
    data[0] = (uint8_t)((pos & AS5600_12_BIT_MASK) >> 8); 
    data[1] = (uint8_t)pos;
    
    return AS5600_WriteRegisters(dev, AS5600_REGISTER_MPOS_HIGH, data, 2);
}

HAL_StatusTypeDef AS5600_SetMaxAngle(AS5600_TypeDef* const dev, uint16_t const angle)
{
    uint8_t data[2] = {0};

    data[0] = (uint8_t)((angle & AS5600_12_BIT_MASK) >> 8);
    data[1] = (uint8_t)angle;

    return AS5600_WriteRegisters(dev, AS5600_REGISTER_MANG_HIGH, data, 2);
}

HAL_StatusTypeDef AS5600_SetPositiveRotationDirection(AS5600_TypeDef* const dev, uint8_t const dir)
{
    HAL_StatusTypeDef status = HAL_OK;

    switch (dir)
    {
    	case AS5600_DIR_CW:
    		HAL_GPIO_WritePin(dev->DirPort, dev->DirPin, GPIO_PIN_RESET);
    		break;
    	case AS5600_DIR_CCW:
    		HAL_GPIO_WritePin(dev->DirPort, dev->DirPin, GPIO_PIN_SET);
    		break;
    	default:
    		/* Invalid rotation direction specified. */
    		status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef AS5600_SetLowPowerMode(AS5600_TypeDef* const dev, uint8_t const mode)
{
    HAL_StatusTypeDef status;

    switch (mode)
    {
        case AS5600_POWER_MODE_NOM:
            dev->confRegister[1] &= ~((1UL << 1) | (1UL << 0));
            break;
        case AS5600_POWER_MODE_LPM1:
            dev->confRegister[1] |= (1UL << 0);
            dev->confRegister[1] &= ~(1UL << 1);
            break;
        case AS5600_POWER_MODE_LPM2:
            dev->confRegister[1] |= (1UL << 1);
            dev->confRegister[1] &= (1UL << 0);
            break;
        case AS5600_POWER_MODE_LPM3:
            dev->confRegister[1] |= ((1UL << 1) | (1UL << 0));
            break;
        default:
            /* Invalid low power mode specified */
            status = HAL_ERROR;
            return status;
    }

    return AS5600_WriteRegisters(dev, AS5600_REGISTER_CONF_HIGH, (uint8_t*)dev->confRegister, 2);
}

HAL_StatusTypeDef AS5600_SetHysteresis(AS5600_TypeDef* const dev, uint8_t const hysteresis)
{
    HAL_StatusTypeDef status;

    switch (hysteresis)
    {
        case AS5600_HYSTERESIS_OFF:
            dev->confRegister[1] &= ~((1UL << 3) | (1UL << 2));
            break;
        case AS5600_HYSTERESIS_1LSB:
            dev->confRegister[1] |= (1UL << 2);
            dev->confRegister[1] &= ~(1UL << 3);
            break;
        case AS5600_HYSTERESIS_2LSB:
            dev->confRegister[1] &= ~(1UL << 2);
            dev->confRegister[1] |= (1UL << 3);
            break;
        case AS5600_HYSTERESIS_3LSB:
            dev->confRegister[1] |= ((1UL << 3) | (1UL << 2));
            break;
        default:
            /* Invalid hysteresis mode specified */
            status = HAL_ERROR;
            return status;
    }

    return AS5600_WriteRegisters(dev, AS5600_REGISTER_CONF_HIGH, (uint8_t*)dev->confRegister, 2);
}

HAL_StatusTypeDef AS5600_SetOutputMode(AS5600_TypeDef* const dev, uint8_t const mode)
{
    HAL_StatusTypeDef status;

    switch (mode)
    {
        case AS5600_OUTPUT_STAGE_FULL:
            dev->confRegister[1] &= ~((1UL << 5) | (1UL << 4));
            break;
        case AS5600_OUTPUT_STAGE_REDUCED:
            dev->confRegister[1] |= (1UL << 4);
            dev->confRegister[1] &= ~(1UL << 5);
            break;
        case AS5600_OUTPUT_STAGE_PWM:
            dev->confRegister[1] &= (1UL << 4);
            dev->confRegister[1] |= (1UL << 5);
            break;
        default:
            /* Invalid output mode specified */
            status = HAL_ERROR;
            return status;
    }

    return AS5600_WriteRegisters(dev, AS5600_REGISTER_CONF_HIGH, (uint8_t*)dev->confRegister, 2);
}

HAL_StatusTypeDef AS5600_SetPwmFrequency(AS5600_TypeDef* const dev, uint8_t const freq)
{
	HAL_StatusTypeDef status;

	switch (freq)
	{
		case AS5600_PWM_FREQUENCY_115HZ:
			dev->confRegister[1] &= ~((1UL << 7) | (1UL << 6));
			break;
		case AS5600_PWM_FREQUENCY_230HZ:
			dev->confRegister[1] |= (1UL << 6);
			dev->confRegister[1] &= ~(1UL << 7);
			break;
		case AS5600_PWM_FREQUENCY_460HZ:
			dev->confRegister[1] &= ~(1UL << 6);
			dev->confRegister[1] |= (1UL << 7);
			break;
		case AS5600_PWM_FREQUENCY_920HZ:
			dev->confRegister[1] |= ((1UL << 7) | (1UL << 6));
			break;
		default:
			/* Invalid PWM frequency specified. */
			status = HAL_ERROR;
			return status;
	}

	return AS5600_WriteRegisters(dev, AS5600_REGISTER_CONF_HIGH, (uint8_t*)dev->confRegister, 2);
}

HAL_StatusTypeDef AS5600_SetSlowFilter(AS5600_TypeDef* const dev, uint8_t const mode)
{
    HAL_StatusTypeDef status;

    switch (mode)
    {
        case AS5600_SLOW_FILTER_16X:
            dev->confRegister[0] &= ~((1UL << 1) | (1UL << 0));
            break;
        case AS5600_SLOW_FILTER_8X:
            dev->confRegister[0] |= (1UL << 0);
            dev->confRegister[0] &= ~(1UL << 1);
            break;
        case AS5600_SLOW_FILTER_4X:
            dev->confRegister[0] &= ~(1UL << 0);
            dev->confRegister[0] |= (1UL << 1);
            break;
        case AS5600_SLOW_FILTER_2X:
            dev->confRegister[0] |= ((1UL << 1) | (1UL << 0));
            break;
        default:
            /* Invalid slow filter mode specified */
            status = HAL_ERROR;
            return status;
    }

    return AS5600_WriteRegisters(dev, AS5600_REGISTER_CONF_HIGH, (uint8_t*)dev->confRegister, 2);
}

HAL_StatusTypeDef AS5600_SetFastFilterThreshold(AS5600_TypeDef* const dev, uint8_t const threshold)
{
    HAL_StatusTypeDef status;

    switch (threshold)
    {
        case AS5600_FAST_FILTER_SLOW_ONLY:
            dev->confRegister[0] &= ~((1UL << 4) | (1UL << 3) | (1UL << 2));
            break;
        case AS5600_FAST_FILTER_6LSB:
            dev->confRegister[0] &= ~((1UL << 4) | (1UL << 3));
            dev->confRegister[0] |= (1UL << 2);
            break;
        case AS5600_FAST_FILTER_7LSB:
            dev->confRegister[0] &= ~((1UL << 4) | (1UL << 2));
            dev->confRegister[0] |= (1UL << 3);
            break;
        case AS5600_FAST_FILTER_9LSB:
            dev->confRegister[0] &= ~(1UL << 4);
            dev->confRegister[0] |= ((1UL << 3) | (1UL << 2));
            break;
        case AS5600_FAST_FILTER_18LSB:
            dev->confRegister[0] &= ~((1UL << 3) | (1UL << 2));
            dev->confRegister[0] |= (1UL << 4);
            break;
        case AS5600_FAST_FILTER_21LSB:
            dev->confRegister[0] &= ~(1UL << 3);
            dev->confRegister[0] |= ((1UL << 4) | (1UL << 2));
            break;
        case AS5600_FAST_FILTER_24LSB:
            dev->confRegister[0] &= ~(1UL << 2);
            dev->confRegister[0] |= ((1UL << 4) | (1UL << 3));
            break;
        case AS5600_FAST_FILTER_10LSB:
            dev->confRegister[0] |= ((1UL << 4) | (1UL << 3) | (1UL << 2));
            break;
        default:
            /* Invalid slow filter mode specified */
            status = HAL_ERROR;
            return status;
    }

    return AS5600_WriteRegisters(dev, AS5600_REGISTER_CONF_HIGH, (uint8_t*)dev->confRegister, 2);
}

HAL_StatusTypeDef AS5600_SetWatchdogTimer(AS5600_TypeDef* const dev, uint8_t const mode)
{
    HAL_StatusTypeDef status;

    switch (mode)
    {
        case AS5600_WATCHDOG_OFF:
            dev->confRegister[0] &= ~(1UL << 5);
            break;
        case AS5600_WATCHDOG_ON:
            dev->confRegister[0] |= (1UL << 5);
            break;
        default:
            /* Invalid watchdog state specified */
            status = HAL_ERROR;
            return status;
    }

    return AS5600_WriteRegisters(dev, AS5600_REGISTER_CONF_HIGH, (uint8_t*)dev->confRegister, 2);
}

HAL_StatusTypeDef AS5600_GetRawAngle(AS5600_TypeDef* const dev, uint16_t* const angle)
{
    HAL_StatusTypeDef status;
    uint8_t data[2] = {0};

    status = AS5600_ReadRegisters(dev, AS5600_REGISTER_RAW_ANGLE_HIGH, data, 2);
    *angle = ((data[0] << 8) | data[1]) & AS5600_12_BIT_MASK;

    return status;
}

HAL_StatusTypeDef AS5600_GetAngle(AS5600_TypeDef* const dev, uint16_t* const angle)
{
    HAL_StatusTypeDef status;
    uint8_t data[2] = {0};

    status = AS5600_ReadRegisters(dev, AS5600_REGISTER_ANGLE_HIGH, data, 2);
    *angle = ((data[0] << 8) | data[1]) & AS5600_12_BIT_MASK;

    return status;
}

HAL_StatusTypeDef AS5600_GetMagnetStatus(AS5600_TypeDef* const dev, uint8_t* const mag_stat)
{
	HAL_StatusTypeDef status;

    status = AS5600_ReadRegister(dev, AS5600_REGISTER_STATUS, mag_stat);
    *mag_stat &= AS5600_MAGSTAT_MASK;

    return status;
}

HAL_StatusTypeDef AS5600_GetAGCSetting(AS5600_TypeDef* const dev, uint8_t* const agc)
{
    return AS5600_ReadRegister(dev, AS5600_REGISTER_AGC, agc);
}

HAL_StatusTypeDef AS5600_GetCORDICMagnitude(AS5600_TypeDef* const dev, uint16_t* const mag)
{
    HAL_StatusTypeDef status;
    uint8_t data[2] = {0};

    status = AS5600_ReadRegisters(dev, AS5600_REGISTER_MAGNITUDE_HIGH, data, 2);
    *mag = ((data[0] << 8) | data[1]) & AS5600_12_BIT_MASK;

    return status;
}

HAL_StatusTypeDef AS5600_GetConfRegister(AS5600_TypeDef* const dev, uint16_t* const conf)
{
	HAL_StatusTypeDef status;
	uint8_t data[2] = {0};

	status = AS5600_ReadRegisters(dev, AS5600_REGISTER_CONF_HIGH, data, 2);
	*conf = ((data[0] << 8) | data[1]) & AS5600_CONFREG_MASK;

	return status;
}
