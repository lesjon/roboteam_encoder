#include "pid.h"
#include "stm32f0xx_hal.h"
#include "tim.h"
#include "PuttyInterface.h"

#define CLK_FREQUENCY 48000000

float timestep;
PID_controller global_PID;
int16_t current_pwm = 0x0;
char small_buf;

void pid_SetOutput(int pwm){
	current_pwm = pwm;
	__HAL_TIM_SET_COMPARE(global_PID.actuator, TIM_CHANNEL_1, current_pwm);
}
PID_controller pid_GetControllerValue(){
	return global_PID;
}
int16_t pid_GetCurrentOutput(){
	return current_pwm;
}
void pid_SetReference(float ref){
	global_PID.v_ref = ref;
}

PID pid_GetCurrentPIDValues(){
	return global_PID.pid;
}
void pid_Init(PID_controller PID_controller){
	global_PID = PID_controller;
	HAL_TIM_PWM_Start(global_PID.actuator,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(global_PID.CallbackTimer);
	timestep = ((float)global_PID.CallbackTimer->Init.Period)/((float)CLK_FREQUENCY/((float)(global_PID.CallbackTimer->Init.Prescaler + 1)));
	uprintf("Init pid:\n\rtimestep = [%f]", timestep);
}
void pid_Control(float current_speed){
	static float prev_e = 0;
	float e = global_PID.v_ref - current_speed;
	global_PID.pid.P = global_PID.K_terms.Kp*e;
	global_PID.pid.I += global_PID.K_terms.Ki*e*timestep;
	global_PID.pid.I = (global_PID.pid.I > 0xfff) ? 0xfff : (global_PID.pid.I < 0) ? 0 : global_PID.pid.I;
	global_PID.pid.D = (global_PID.K_terms.Kd*(e-prev_e))/timestep;
	prev_e = e;

	current_pwm = (int)(global_PID.pid.P + global_PID.pid.I + global_PID.pid.D);
	current_pwm = (current_pwm > 0xfff) ? 0xfff : (current_pwm < 0) ? 0 : current_pwm;

	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, current_pwm);
}
