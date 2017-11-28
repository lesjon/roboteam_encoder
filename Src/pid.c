#include "pid.h"
#include "stm32f0xx_hal.h"
#include "tim.h"
#include "PuttyInterface.h"

static int32_t ClipInt(int32_t input, int32_t min, int32_t max){
	return (input > max) ? max : (input < min) ? min : input;
}

void pid_SetOutput(int pwm, PID_controller_HandleTypeDef* pc){
	pc->current_pwm = ClipInt(pwm, 0, pc->actuator->Init.Period);
	__HAL_TIM_SET_COMPARE(pc->actuator, TIM_CHANNEL_1, pc->current_pwm);
}
int16_t pid_GetCurrentOutput(PID_controller_HandleTypeDef* pc){
	return pc->current_pwm;
}
void pid_SetReference(float ref, PID_controller_HandleTypeDef* pc){
	pc->ref = ref;
}

PID pid_GetCurrentPIDValues(PID_controller_HandleTypeDef* pc){
	return pc->pid;
}
void pid_Init(PID_controller_HandleTypeDef* PID_controller, uint32_t n_motors){
	HAL_TIM_PWM_Start(PID_controller->actuator,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(PID_controller->CallbackTimer);
	PID_controller->timestep = (((float)n_motors*(float)PID_controller->CallbackTimer->Init.Period))/(PID_controller->CLK_FREQUENCY/((float)(PID_controller->CallbackTimer->Init.Prescaler + 1)));
}
void pid_Control(float sensor_output, PID_controller_HandleTypeDef* pc){
	static float prev_e = 0;
	float err = pc->ref - sensor_output;
	pc->pid.P = pc->K_terms.Kp*err;
	pc->pid.I += pc->K_terms.Ki*err*pc->timestep;
	pc->pid.D = (pc->K_terms.Kd*(err-prev_e))/pc->timestep;
	prev_e = err;

	pc->pid.P = ClipInt(pc->pid.P, -0xffff, 0xffff);
	pc->pid.I = ClipInt(pc->pid.I, -0xffff, 0xffff);
	pc->current_pwm = (int16_t)(pc->pid.P + pc->pid.I + pc->pid.D);
	pid_SetOutput(pc->current_pwm, pc);
}
