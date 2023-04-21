/*************************************************************************

Author:   Grzegorz Niedziółka
File:     inv_pen.c
Software: STM32L1xx_HAL_Driver
Hardware: STM32L1xx
License:  The MIT License (MIT)

DESCRIPTION:
Library for control of an inverted pendulum on a card (github: https://github.com/G1874/InvertedPendulum)

LICENSE:
	Copyright (c) 2023 Grzegorz Niedziółka

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

*************************************************************************/

#include "inv_pen.h"

/*******************  CONSTANTS & GLOBAL VARIABLES  ********************/

const int32_t LQR_K_MATRIX[4] = {-4697,-4809,-97364,-24450};	//{-2697,-2809,-97364,-24450},{-6723,-20395,-98249,-32525}

/*******************    FUNCTION IMPLEMENTATIONS    ********************/

HAL_StatusTypeDef PcSendErrorMessage(UART_HandleTypeDef* uart, uint8_t errNum, uint8_t dataMarker)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t data8b[2];
	data8b[0] = dataMarker;
	data8b[1] = errNum;

	status = HAL_UART_Transmit(uart, data8b, 2, HAL_MAX_DELAY);

	return status;
}

HAL_StatusTypeDef PcSend8bitData(UART_HandleTypeDef* uart , uint8_t data, uint8_t dataMarker)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t data8b[3];
	data8b[0] = dataMarker;
	data8b[1] = (uint8_t)(data);

	status = HAL_UART_Transmit(uart, data8b, 3, HAL_MAX_DELAY);

	return status;
}

HAL_StatusTypeDef PcSend16bitData(UART_HandleTypeDef* uart , uint16_t data, uint8_t dataMarker)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t data8b[3];
	data8b[0] = dataMarker;
	data8b[1] = (uint8_t)(data >> 8);
	data8b[2] = (uint8_t)(data);

	status = HAL_UART_Transmit(uart, data8b, 3, HAL_MAX_DELAY);

	return status;
}

HAL_StatusTypeDef PcSend16bitData2(UART_HandleTypeDef* uart, uint16_t data1, uint16_t data2, uint8_t dataMarker)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t data8b[5];
	data8b[0] = dataMarker;
	data8b[1] = (uint8_t)(data1 >> 8);
	data8b[2] = (uint8_t)(data1);
	data8b[3] = (uint8_t)(data2 >> 8);
	data8b[4] = (uint8_t)(data2);

	status = HAL_UART_Transmit(uart, data8b, 5, HAL_MAX_DELAY);

	return status;
}

HAL_StatusTypeDef PcSend32bitData(UART_HandleTypeDef* uart , uint32_t data, uint8_t dataMarker)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t data8b[5];
	data8b[0] = dataMarker;
	data8b[1] = (uint8_t)(data >> 24);
	data8b[2] = (uint8_t)(data >> 16);
	data8b[3] = (uint8_t)(data >> 8);
	data8b[4] = (uint8_t)(data);

	status = HAL_UART_Transmit(uart, data8b, 5, HAL_MAX_DELAY);

	return status;
}

HAL_StatusTypeDef PcSend32bitSignedData(UART_HandleTypeDef* uart , int32_t data, uint8_t dataMarker)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t data8b[6];
	uint8_t sign = 0;

	if(data < 0){
		sign = 1;
		data = -data;
	}

	data8b[0] = dataMarker;
	data8b[1] =	sign;
	data8b[2] = (uint8_t)(data >> 24);
	data8b[3] = (uint8_t)(data >> 16);
	data8b[4] = (uint8_t)(data >> 8);
	data8b[5] = (uint8_t)(data);

	status = HAL_UART_Transmit(uart, data8b, 6, HAL_MAX_DELAY);

	return status;
}

uint8_t OverflowProc(TIM_HandleTypeDef* timHandle, TIM_HandleTypeDef* synchTimHandle, TIM_HandleTypeDef* trigTimHandle, GPIO_TypeDef* TrigPort,
						int16_t TrigPin, uint8_t* direction, uint16_t* period, uint8_t* flag1, uint8_t* flag2, uint8_t* flag3, GPIO_TypeDef* DirPort,
						int16_t DirPin, TIM_HandleTypeDef* pwmTimHandle, uint32_t pwmTimChannel, uint8_t mode)
{
	if(timHandle->Instance == trigTimHandle->Instance) {
		HAL_TIM_Base_Stop(timHandle);
		HAL_GPIO_WritePin(TrigPort, TrigPin, GPIO_PIN_RESET);
	}

	if(timHandle->Instance == synchTimHandle->Instance) {
		if(!*flag3) {

			__HAL_TIM_SET_AUTORELOAD(pwmTimHandle, *period);
			HAL_GPIO_WritePin(DirPort, DirPin, *direction);

			if(*flag2) {
				if(mode == 0) {
					HAL_TIM_PWM_Stop(pwmTimHandle, pwmTimChannel);
				} else if(mode == 3) {
					HAL_TIM_PWM_Stop(pwmTimHandle, pwmTimChannel);
				} else if(mode == 1 || mode == 2) {
					HAL_TIM_PWM_Start(pwmTimHandle, pwmTimChannel);
				}
				*flag2 = 0;
			}

			*flag3 = 1;
			*flag1 = 1;

		} else {
			return ERR_CTRL_TIMEOUT;
		}
	}

	return 0;
}

void RxCallbackProc(UART_HandleTypeDef* huart, AS5600_TypeDef* dev, uint8_t* dataIn, uint8_t* direction, int32_t* speed, uint8_t* mode, uint8_t* flag)
{
	if(huart->Instance == USART2) {
		/* Manual change of stepper motor speed and direction */
		if(dataIn[0] == 82) {
			*flag = 1;
			*speed = 100 * (int32_t)dataIn[1];
			*mode = 2;
		} else if(dataIn[0] == 76) {
			*flag = 1;
			*speed = -100 * (int32_t)dataIn[1];
			*mode = 2;
		}

		uint8_t as5600_data;
		uint16_t as5600_angle;

		switch(dataIn[0]) {
			/* Mode change */
			case MAN_CTRL:		// Manual control
				*mode = 2;
				PcSendErrorMessage(huart, COM_MAN_CTRL, COMMUNICATE);
				break;
			case AUTO_CTRL:		// Automatic control
				*speed = 0;
				*mode = 1;
				*flag = 1;
				PcSendErrorMessage(huart, COM_AUTO_CTRL, COMMUNICATE);
				break;
			case PAUSE_MTR:		// Pause motor
				*mode = 0;
				*flag = 1;
				break;
			case STOP_MTR:		// Stop motor
				*mode = 3;
				*flag = 1;
				PcSendErrorMessage(huart, COM_STOP_MTR, COMMUNICATE);
				break;
			/* AS5600 magnetic encoder state report */
			case SEND_MAG_STAT:
				if(AS5600_GetMagnetStatus(dev, &as5600_data) == HAL_OK) {
					PcSendErrorMessage(huart, COM_MAG_STAT, COMMUNICATE);
					PcSend8bitData(huart, as5600_data, AS5600_CONFIG_DATA);
				} else {
					PcSendErrorMessage(huart, ERR_AS5600_READ, ERROR_DATA);
				}
				break;
			case SEND_AGC_REG:
				if(AS5600_GetAGCSetting(dev, &as5600_data) == HAL_OK) {
					PcSendErrorMessage(huart, COM_AGC_REG, COMMUNICATE);
					PcSend8bitData(huart, as5600_data, AS5600_CONFIG_DATA);
				} else {
					PcSendErrorMessage(huart, ERR_AS5600_READ, ERROR_DATA);
				}
				break;
			/* Set AS5600 magnetic encoder zero position */
			case SET_ZERO_POSITION:
				AS5600_GetRawAngle(dev, &as5600_angle);
				if(AS5600_SetStartPosition(dev, as5600_angle) == HAL_OK) {
					PcSendErrorMessage(huart, COM_ZERO_POSITION, COMMUNICATE);
				} else {
					PcSendErrorMessage(huart, ERR_AS5600_WRITE, ERROR_DATA);
				}
				break;
		}

		HAL_UART_Receive_IT(huart, dataIn, 2);
	}
}

uint8_t InvPen_Init(TIM_HandleTypeDef* RtimHandle, uint32_t RtimChannel, TIM_HandleTypeDef* FtimHandle, uint32_t FtimChannel,
						TIM_HandleTypeDef* synchTimHandle, uint8_t* dataIn, TIM_HandleTypeDef* pwmTimHandle,
						uint32_t pwmTimChannel, UART_HandleTypeDef* huart)
{
	__HAL_TIM_SET_COUNTER(RtimHandle, 0);
	__HAL_TIM_SET_COUNTER(FtimHandle, 0);
	__HAL_TIM_SET_COUNTER(synchTimHandle, 0);
	__HAL_TIM_SET_COUNTER(pwmTimHandle, 0);
	__HAL_TIM_SET_COMPARE(pwmTimHandle, pwmTimChannel, 20);

	if(HAL_TIM_IC_Start_IT(RtimHandle, RtimChannel) != HAL_OK) {
		return ERR_R_ICTIM_INIT;
	}
	if(HAL_TIM_IC_Start_IT(FtimHandle, FtimChannel) != HAL_OK) {
		return ERR_F_ICTIM_INIT;
	}
	if(HAL_TIM_Base_Start_IT(synchTimHandle) != HAL_OK) {
		return ERR_SYNCH_TIM_INIT;
	}
	HAL_UART_Receive_IT(huart, dataIn, 2);
	return 0;
}

uint8_t HCSR04_StartMeasurment(TIM_HandleTypeDef* RtimHandle, TIM_HandleTypeDef* FtimHandle, TIM_HandleTypeDef* trigTimHandle,
								GPIO_TypeDef* TrigPort, int16_t TrigPin)
{
	HAL_GPIO_WritePin(TrigPort, TrigPin, GPIO_PIN_SET);

	if(HAL_TIM_Base_Start_IT(trigTimHandle) != HAL_OK) {
		return ERR_STAR_MEASURMENT;
	}

	__HAL_TIM_SET_COUNTER(RtimHandle, 0);
	__HAL_TIM_SET_COUNTER(FtimHandle, 0);

	return 0;
}

void HCSR04_CaptureProc(TIM_HandleTypeDef* timHandle, TIM_HandleTypeDef* RtimHandle, uint32_t RtimChannel, TIM_HandleTypeDef* FtimHandle,
							uint32_t FtimChannel, uint16_t* RechoTime, uint16_t* FechoTime, uint8_t* flag)
{
	if(timHandle->Instance == FtimHandle->Instance) {
		*RechoTime = HAL_TIM_ReadCapturedValue(RtimHandle, RtimChannel);
		*FechoTime = HAL_TIM_ReadCapturedValue(FtimHandle, FtimChannel);
		*flag = 1;
	}
}

void HCSR04_GetDistance(uint16_t* RechoTime, uint16_t* FechoTime, int32_t* distance)
{
	*distance = ((int32_t)(*FechoTime - *RechoTime) / 58 - DISTANCE_NORMALIZATION) * (SCALE_FACTOR / 100);
}

void AngleRescaling(uint16_t raw_angle, int32_t* angle_deg, int32_t* angle)
{
	int64_t a;
	a = (360 * (int64_t)raw_angle * SCALE_FACTOR / (AS5600_RESOLUTION - 1)) - 180 * SCALE_FACTOR;
	*angle_deg = (int32_t)(-a);
	*angle = (int32_t)(-a * 31415 / 180 / SCALE_FACTOR);
}

void ControlAlg(int32_t distance, int32_t* p_distance, int32_t angle, int32_t* p_angle, int32_t* speed, uint8_t mode)
{
	if(mode == 1) {
		/* LQR control algorithm */

		int64_t u = 0;
		int64_t x[4];

		x[0] = (int64_t)distance;							// distance in meters times SCALE_FACTOR
		x[1] = (int64_t)*speed;                     		// previous linear velocity in meters per second times SCALE_FACTOR
		x[2] = (int64_t)angle;               				// angle in radians times SCALE_FACTOR
		x[3] = (int64_t)(angle - *p_angle) * 1000 / TP;		// angular velocity in radians per second times SCALE_FACTOR

		/****
		u = -K*(x-r)
		                          [x1]   [r1]
		                          [x2]   [r2]
		u = [-k1 -k2 -k3 -k4] * ( [x3] - [r3] )
		                          [x4]   [r4]

		u = -k1(x1-r1) -k2(x2-r2) -k3(x3-r3) -k4(x4-r4)
		****/

		for(int i=0;i<4;i++) {
			u += (int64_t)(-LQR_K_MATRIX[i]) * x[i] / SCALE_FACTOR;
		}

		/* new linear velocity in meters per second times SCALE_FACTOR */
		*speed = *speed + (u * ACC_TO_SPD * TP / 1000 / SCALE_FACTOR);

		if(*speed > MAX_SPEED) {
			*speed = MAX_SPEED;
		} else if(*speed < -MAX_SPEED) {
			*speed = -MAX_SPEED;
		}

		*p_angle = angle;
		*p_distance = distance;

	} else {
		*p_angle = angle;
		*p_distance = distance;
	}
}

void StepperNewPWM(int32_t speed, int32_t distance, uint8_t* direction, uint16_t* period, uint8_t* flag)
{
	int64_t p;
	if(speed != 0){
		p = 2 * 31415 * TORQUE_RADIUS * 100 / (int64_t)speed / STEPS_PER_REVOLUTION;
	} else {
	    p = MAX_TIMER_PERIOD + 1;
	}

	if(p > 0) {
		*direction = 1;
	} else if(p < 0) {
		*direction = 0;
		p = -p;
	 } else if(p == 0) {
		 if(speed > 0) {
			 *direction = 1;
	     } else if(speed < 0) {
	    	 *direction = 0;
	     }
	 }

	 if(p > MAX_TIMER_PERIOD) {
		 *period = 0;
	 } else if(p < MIN_TIMER_PERIOD){
		 *period = MIN_TIMER_PERIOD;
	 } else {
		 *period = (uint16_t)(p) - 1;
	 }

	 if(*direction == 1 && distance >= 1800) {
		 *period = 0;
	 } else if(*direction == 0 && distance <= -1800) {
		 *period = 0;
	 }

	 *flag = 0;
}
