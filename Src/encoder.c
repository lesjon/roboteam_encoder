/*
 * encoder.c
 *
 *  Created on: Nov 20, 2017
 *      Author: Leon
 */
#include "encoder.h"

void encoder_Init(encoder_HandleTypeDef* enc){
	__HAL_TIM_ENABLE(enc->MeasurementTimer);
}
void encoder_Input(uint16_t channel, encoder_HandleTypeDef* enc){
	if(channel == enc->CHANNEL[0]){
		enc->high[0] = HAL_GPIO_ReadPin(enc->CHANNEL_Port[0], enc->CHANNEL[0]);
		if(enc->high[0]){
			enc->period = __HAL_TIM_GET_COUNTER(enc->MeasurementTimer);
			__HAL_TIM_SET_COUNTER(enc->MeasurementTimer, 0);
		}
		enc->direction = enc->high[1] != enc->high[0] ? 1 : -1;
		enc->cnt[0] += enc->direction;
	}else if(channel == enc->CHANNEL[1]){
		enc->high[1] = HAL_GPIO_ReadPin(enc->CHANNEL_Port[1], enc->CHANNEL[1]);

		enc->direction = enc->high[0] != enc->high[1] ? -1 : 1;
		enc->cnt[1] += enc->direction;
	}
}

float encoder_CalculateSpeed(encoder_HandleTypeDef* enc){
	float rot_speed = 0;
	float rot_period = (enc->period > __HAL_TIM_GET_COUNTER(enc->MeasurementTimer) ? enc->period : __HAL_TIM_GET_COUNTER(enc->MeasurementTimer))* (enc->COUNTS_PER_ROTATION);
	rot_speed = 1/rot_period;
	rot_speed = rot_speed * ((enc->CLK_FREQUENCY/(enc->MeasurementTimer->Init.Prescaler+1))/enc->GEAR_RATIO) * enc->direction;
	enc->speed = rot_speed;
	return rot_speed;
}

float encoder_GetLatestSpeed(encoder_HandleTypeDef* enc){
	return enc->speed;
}
