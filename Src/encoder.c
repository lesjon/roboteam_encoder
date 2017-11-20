/*
 * encoder.c
 *
 *  Created on: Nov 20, 2017
 *      Author: Leon
 */
#include "encoder.h"

void encoder_Init(encoder* enc){
	__HAL_TIM_ENABLE(enc->MeasurementTimer);
}
void encoder_Input(uint16_t channel, encoder* enc){
	if(channel == enc->ENCODER_A){
		enc->A_high = HAL_GPIO_ReadPin(enc->ENCODER_A_Port, enc->ENCODER_A);
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
	}else if(channel == enc->ENCODER_B){
		enc->B_high = HAL_GPIO_ReadPin(enc->ENCODER_B_Port, enc->ENCODER_B);
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

float encoder_CalculateSpeed(encoder* enc){
	float rot_speed = 0;
	if(enc->period > __HAL_TIM_GET_COUNTER(enc->MeasurementTimer)){// if still zero, no encoder values were received
		float rot_period = enc->period * enc->COUNTS_PER_ROTATION;
		rot_speed = 1/rot_period;
		rot_speed = rot_speed * ((enc->CLK_FREQUENCY/(enc->MeasurementTimer->Init.Prescaler+1))/enc->GEAR_RATIO) * enc->direction;
		//encoder->period = 0;// So that if no new encoder values are caught we now speed = zero
	}else{
		float rot_period = __HAL_TIM_GET_COUNTER(enc->MeasurementTimer) * (enc->COUNTS_PER_ROTATION);
		rot_speed = 1/rot_period;
		rot_speed = rot_speed * ((enc->CLK_FREQUENCY/(enc->MeasurementTimer->Init.Prescaler+1))/enc->GEAR_RATIO) * enc->direction;
	}
	enc->speed = rot_speed;
	return rot_speed;
}

float encoder_GetLatestSpeed(encoder* enc){
	return enc->speed;
}
