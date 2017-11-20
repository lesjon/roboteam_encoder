#include "pid.h"
#include "stm32f0xx_hal.h"
#include "tim.h"



void pid_SetOutput(int pwm, PID_controller_HandleTypeDef* pc){
	global_PID.current_pwm = pwm;
	__HAL_TIM_SET_COMPARE(global_PID.actuator, TIM_CHANNEL_1, global_PID.current_pwm);
}
PID_controller_HandleTypeDef pid_GetControllerValue(PID_controller_HandleTypeDef* pc){
	return global_PID;
}
int16_t pid_GetCurrentOutput(PID_controller_HandleTypeDef* pc){
	return global_PID.current_pwm;
}
void pid_SetReference(float ref, PID_controller_HandleTypeDef* pc){
	global_PID.v_ref = ref;
}

PID pid_GetCurrentPIDValues(PID_controller_HandleTypeDef* pc){
	return pc->pid;
}
void pid_Init(PID_controller_HandleTypeDef* PID_controller){
	HAL_TIM_PWM_Start(PID_controller->actuator,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(PID_controller->CallbackTimer);
	PID_controller->timestep = ((float)PID_controller->CallbackTimer->Init.Period)/(PID_controller->CLK_FREQUENCY/((float)(PID_controller->CallbackTimer->Init.Prescaler + 1)));
}
void pid_Control(float current_speed, PID_controller_HandleTypeDef* pc){
	static float prev_e = 0;
	float e = global_PID.v_ref - current_speed;
	pc->pid.P = pc->K_terms.Kp*e;
	pc->pid.I += pc->K_terms.Ki*e*pc->timestep;
	pc->pid.I = (pc->pid.I > 0xfff) ? 0xfff : (pc->pid.I < 0) ? 0 : pc->pid.I;
	pc->pid.D = (pc->K_terms.Kd*(e-prev_e))/pc->timestep;
	prev_e = e;

	pc->current_pwm = (int)(pc->pid.P + pc->pid.I + pc->pid.D);
	pc->current_pwm = (pc->current_pwm > 0xfff) ? 0xfff : (pc->current_pwm < 0) ? 0 : pc->current_pwm;

	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pc->current_pwm);
}
