/*
 * encoder.h
 *
 *  Created on: Nov 20, 2017
 *      Author: Leon
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdbool.h>
#include "stm32f0xx_hal.h"
#include "tim.h"
#include "gpio.h"

typedef struct{
	uint16_t ENCODER_A;
	GPIO_TypeDef * ENCODER_A_Port;
	uint16_t ENCODER_B;
	GPIO_TypeDef * ENCODER_B_Port;
	TIM_HandleTypeDef* MeasurementTimer;
	int a_cnt;
	int b_cnt;
	bool B_high;
	bool A_high;
	int8_t direction;
	int period;//us
	float speed;
	float COUNTS_PER_ROTATION;
	float CLK_FREQUENCY;
	float GEAR_RATIO;
}encoder;	//holds values calculated from the encoders


void encoder_Init(encoder* enc);
void encoder_Input(uint16_t channel, encoder* enc);
float encoder_CalculateSpeed(encoder* enc);
float encoder_GetLatestSpeed(encoder* enc);

#endif /* ENCODER_H_ */
