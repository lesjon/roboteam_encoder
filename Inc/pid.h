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
}encoder;//holds values calculated from the encoders
typedef struct{
	float P;
	float I;
	float D;
}PID;// holds the current PID values
typedef struct{
	float Kp;
	float Ki;
	float Kd;
}PID_Terms;// holds the constants
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
// directly set the current output, if the pid control loop is running, this will not have much effect
void pid_SetOutput(int pwm);
// return all pid controller parameters
PID_controller pid_GetControllerValue();
// Returns a struct with the encoder values
encoder pid_GetEncoderValues();
/* return current P, I and D values
 *
 */
PID pid_GetCurrentPIDValues();
// returns the latest read speed
float pid_GetLatestSpeed();
// Returns the current output to the actuator
int16_t pid_GetCurrentOutput();
// Set the reference value
void pid_SetReference(float ref);
// Handle the interrupts of the encoder lines
void pid_EncoderInput(uint8_t channel);//0 = a, 0 = b;
// calculate the current speed according to the encoder values
float pid_CalculateSpeed(encoder *encoder);
// Initialize the pid controller
void pid_Init(PID_controller PID_controller);
// controls the output, to be called on a regular schedule
void pid_Control();
#endif /* PID_H_ */
