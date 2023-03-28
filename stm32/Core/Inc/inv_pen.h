/*************************************************************************

Author:   Grzegorz Niedziółka
File:     inv_pen.h
Software: STM32L1xx_HAL_Driver
Hardware: STM32L1xx
License:  The MIT License (MIT)

DESCRIPTION:
Library for control of an inverted pendulum on a card (github: https://github.com/G1874/InvertedPendulum)

USAGE:
https://github.com/G1874/InvertedPendulum#readme

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

#ifndef INV_PEN_H_
#define INV_PEN_H_

#include "stm32l1xx_hal.h"

/***********************	CONSTANTS & MACROS	   ***********************/

extern const int32_t LQR_K_MATRIX[4];

#define SCALE_FACTOR 10000

#define DISTANCE_NORMALIZATION 20UL
#define ANGLE_NORMALIZATION 20UL

/* Sample time in ms */
#define TP 20

#define MAX_SPEED 10000

/***********************    FUNCTION PROTOTYPES    ***********************/

/* FUNCTIONS FOR COMMUNICATION WITH PC SIDE PYTHON GUI */
HAL_StatusTypeDef PcSendErrorMessage(UART_HandleTypeDef* uart, uint8_t errNum, uint8_t dataMarker);
HAL_StatusTypeDef PcSend16bitData(UART_HandleTypeDef* uart, uint16_t* data, uint8_t dataMarker);
HAL_StatusTypeDef PcSend16bitData2(UART_HandleTypeDef* uart, uint16_t* data1, uint16_t* data2, uint8_t dataMarker);
HAL_StatusTypeDef PcSend32bitData(UART_HandleTypeDef* uart ,uint32_t* data, uint8_t dataMarker);

/* FUNCTION FOR STARTING INPUT CAPTURE AND SYNCHRONIZATION TIMERS */
uint8_t InvPen_Init(TIM_HandleTypeDef* RtimHandle, uint32_t RtimChannel, TIM_HandleTypeDef* FtimHandle, uint32_t FtimChannel,
						TIM_HandleTypeDef* synchTimHandle, uint8_t* dataIn, TIM_HandleTypeDef* pwmTimHandle,
						uint32_t pwmTimChannel, UART_HandleTypeDef* huart);

/* FUNCTION STARTING HSCR04 DISTANCE MEASURMENT */
uint8_t HCSR04_StartMeasurment(TIM_HandleTypeDef* RtimHandle, TIM_HandleTypeDef* FtimHandle, TIM_HandleTypeDef* trigTimHandle,
								GPIO_TypeDef* TrigPort, int16_t TrigPin);

/* TIMER INTERUPT PROCEDURES */
void HCSR04_CaptureProc(TIM_HandleTypeDef* timHandle, TIM_HandleTypeDef* RtimHandle, uint32_t RtimChannel, TIM_HandleTypeDef* FtimHandle,
							uint32_t FtimChannel, uint16_t* RechoTime, uint16_t* FechoTime, uint8_t* flag);
uint8_t OverflowProc(TIM_HandleTypeDef* timHandle, TIM_HandleTypeDef* synchTimHandle, TIM_HandleTypeDef* trigTimHandle, GPIO_TypeDef* TrigPort,
						int16_t TrigPin, uint8_t* direction, uint16_t* freq, uint8_t* flag1, uint8_t* flag2, uint8_t* flag3, GPIO_TypeDef* DirPort,
						int16_t DirPin, TIM_HandleTypeDef* pwmTimHandle, uint32_t pwmTimChannel, uint8_t mode);

/* UART RX INTERUPT PROCEDURE */
void RxCallbackProc(UART_HandleTypeDef* huart, uint8_t* dataIn, uint8_t* direction, int32_t* speed, uint8_t* mode, uint8_t* flag);

/* FUNCTIONS FOR COMPUTING NECESSARY PARAMETERS */
void HCSR04_GetDistance(uint16_t* RechoTime, uint16_t* FechoTime, uint16_t* distance);
void ControlAlg(uint16_t distance, uint16_t angle, int32_t* p_angle, int32_t* speed, uint8_t mode);
void StepperNewPWM(uint8_t speed, uint8_t* direction, uint16_t* freq, uint8_t* flag);

#endif /* INV_PEN_H_ */
