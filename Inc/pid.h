/*
 * pid.h
 *
 *  Created on: Nov 16, 2017
 *      Author: Leon
 */

#ifndef PID_H_
#define PID_H_

#include "stm32f0xx_hal.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct{
	int a_cnt;
	int b_cnt;
	bool B_high;
	bool A_high;
	int8_t direction;
	int period;//us
}encoder;
typedef struct{
	float P;
	float I;
	float D;
}PID;
typedef struct{
	float Kp;
	float Ki;
	float Kd;
}PID_Terms;
typedef struct{
	PID_Terms K_terms;
	PID pid;
	float speed;
	float v_ref;
	float timestep;
	float COUNTS_PER_ROTATION;
	float CLK_FREQUENCY;
	float GEAR_RATIO;
	TIM_HandleTypeDef* actuator;
	TIM_HandleTypeDef* MeasurementTimer;
	TIM_HandleTypeDef* CallbackTimer;
}PID_controller;

void pid_SetOutput(int pwm);
PID_controller pid_GetControllerValue();
encoder pid_GetEncoderValues();
PID pid_GetCurrentPIDValues();
float pid_GetLatestSpeed();
int16_t pid_GetCurrentOutput();
void pid_SetReference(float ref);
void pid_HandleCommand(char * input);
void pid_EncoderInput(uint8_t channel);//0 = a, 0 = b;
float pid_CalculateSpeed(encoder *encoder);
void pid_Init(PID_controller PID_controller);
void pid_Control();
#endif /* PID_H_ */
