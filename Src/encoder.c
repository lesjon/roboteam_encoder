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
	if(channel == enc->CHANNEL_A){
		enc->A_high = HAL_GPIO_ReadPin(enc->CHANNEL_A_Port, enc->CHANNEL_A);
		if(enc->A_high){
			enc->period = __HAL_TIM_GET_COUNTER(enc->MeasurementTimer);
			__HAL_TIM_SET_COUNTER(enc->MeasurementTimer, 0);
			if(enc->B_high){
				enc->a_cnt--;
				enc->direction = -1;
			}else{
				enc->a_cnt++;
				enc->direction = 1;
			}
		}else{
			if(enc->B_high){
				enc->a_cnt++;
				enc->direction = 1;
			}else{
				enc->a_cnt--;
				enc->direction = -1;
			}
		}
	}else if(channel == enc->CHANNEL_B){
		enc->B_high = HAL_GPIO_ReadPin(enc->CHANNEL_B_Port, enc->CHANNEL_B);
		if(enc->B_high){
			if(enc->A_high){
				enc->b_cnt++;
				enc->direction = 1;
			}else{
				enc->b_cnt--;
				enc->direction = -1;
			}
		}else{
			if(enc->A_high){
				enc->b_cnt--;
				enc->direction = -1;
			}else{
				enc->b_cnt++;
				enc->direction = 1;
			}
		}
	}
}

float encoder_CalculateSpeed(encoder_HandleTypeDef* enc){
	float rot_speed = 0;
	if(enc->period > __HAL_TIM_GET_COUNTER(enc->MeasurementTimer)){// if counter is higher, the motor slowed down a lot and did not yet receive a new period
		float rot_period = enc->period * enc->COUNTS_PER_ROTATION;
		rot_speed = 1/rot_period;
		rot_speed = rot_speed * ((enc->CLK_FREQUENCY/(enc->MeasurementTimer->Init.Prescaler+1))/enc->GEAR_RATIO) * enc->direction;
	}else{
		float rot_period = __HAL_TIM_GET_COUNTER(enc->MeasurementTimer) * (enc->COUNTS_PER_ROTATION);
		rot_speed = 1/rot_period;
		rot_speed = rot_speed * ((enc->CLK_FREQUENCY/(enc->MeasurementTimer->Init.Prescaler+1))/enc->GEAR_RATIO) * enc->direction;
	}
	enc->speed = rot_speed;
	return rot_speed;
}

float encoder_GetLatestSpeed(encoder_HandleTypeDef* enc){
	return enc->speed;
}
