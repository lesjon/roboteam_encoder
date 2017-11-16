#include "pid.h"
#include "stm32f0xx_hal.h"
#include "tim.h"

float timestep;
PID_controller global_PID;
int16_t current_pwm = 0x0;
char small_buf;
encoder enc = {0,0,0,0};

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
encoder pid_GetEncoderValues(){
	return enc;
}
PID pid_GetCurrentPIDValues(){
	return global_PID.pid;
}
float pid_GetLatestSpeed(){
	return global_PID.speed;
}
void pid_EncoderInput(uint8_t channel){
	if(channel == 0){
		enc.A_high = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
		if(enc.A_high){
			enc.period = __HAL_TIM_GET_COUNTER(global_PID.MeasurementTimer);
			__HAL_TIM_SET_COUNTER(global_PID.MeasurementTimer, 0);
			if(enc.B_high){
				enc.a_cnt--;
				enc.direction = -1;
			}else{
				enc.a_cnt++;
				enc.direction = 1;
			}
		}else{
			if(enc.B_high){
				enc.a_cnt++;
				enc.direction = 1;
			}else{
				enc.a_cnt--;
				enc.direction = -1;
			}
		}
	}else if(channel == 1){
		enc.B_high = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);
		if(enc.B_high){
			if(enc.A_high){
				enc.b_cnt++;
				enc.direction = 1;
			}else{
				enc.b_cnt--;
				enc.direction = -1;
			}
		}else{
			if(enc.A_high){
				enc.b_cnt--;
				enc.direction = -1;
			}else{
				enc.b_cnt++;
				enc.direction = 1;
			}
		}
	}
}
//#define COUNTS_PER_ROTATION 12
//#define CLK_FREQUENCY		48000000
//#define GEAR_RATIO			10
//#define Kp					1000
//#define Ki					100
//#define Kd					.1

float pid_CalculateSpeed(encoder *encoder){
	float rot_speed = 0;
	if(encoder->period > __HAL_TIM_GET_COUNTER(global_PID.MeasurementTimer)){// if still zero, no encoder values were received
		float rot_period = encoder->period * global_PID.COUNTS_PER_ROTATION;
		rot_speed = 1/rot_period;
		rot_speed = rot_speed * ((global_PID.CLK_FREQUENCY/(global_PID.MeasurementTimer->Init.Prescaler+1))/global_PID.GEAR_RATIO) * enc.direction;
		//encoder->period = 0;// So that if no new encoder values are caught we now speed = zero
	}else{
		float rot_period = __HAL_TIM_GET_COUNTER(global_PID.MeasurementTimer) * (global_PID.COUNTS_PER_ROTATION);
		rot_speed = 1/rot_period;
		rot_speed = rot_speed * ((global_PID.CLK_FREQUENCY/(global_PID.MeasurementTimer->Init.Prescaler+1))/global_PID.GEAR_RATIO) * enc.direction;
	}
	global_PID.speed = rot_speed;
	return rot_speed;
}

void pid_Init(PID_controller PID_controller){
	global_PID = PID_controller;
	HAL_TIM_PWM_Start(global_PID.actuator,TIM_CHANNEL_1);
	__HAL_TIM_ENABLE(global_PID.MeasurementTimer);
	HAL_TIM_Base_Start_IT(global_PID.CallbackTimer);
	timestep = ((float)global_PID.CallbackTimer->Init.Period)/((float)global_PID.CLK_FREQUENCY/((float)(global_PID.CallbackTimer->Init.Prescaler + 1)));
}
void pid_Control(){
	static float prev_error = 0;
	float error = global_PID.v_ref - pid_CalculateSpeed(&enc);
	global_PID.pid.P = global_PID.K_terms.Kp*error;
	global_PID.pid.I += global_PID.K_terms.Ki*error*timestep;
	global_PID.pid.I = (global_PID.pid.I > 0xfff) ? 0xfff : (global_PID.pid.I < 0) ? 0 : global_PID.pid.I;
	global_PID.pid.D = (global_PID.K_terms.Kd*(error-prev_error))/timestep;
	prev_error = error;

	current_pwm = (int)(global_PID.pid.P + global_PID.pid.I + global_PID.pid.D);
	current_pwm = (current_pwm > 0xfff) ? 0xfff : (current_pwm < 0) ? 0 : current_pwm;

	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, current_pwm);
}
