/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include "PuttyInterface.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct{
	int a_cnt;
	int b_cnt;
	int8_t direction;
	int period;//us
}encoder;
typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float timestep;
	float COUNTS_PER_ROTATION;
	float CLK_FREQUENCY;
	float GEAR_RATIO;
	TIM_HandleTypeDef* actuator;
	TIM_HandleTypeDef* MeasurementTimer;
	TIM_HandleTypeDef* CallbackTimer;
}PID_controller;

bool huart2_Rx_flag = false;
bool print_time = false;
bool print_encoder = false;
bool print_speed = false;
bool print_dac = false;
bool print_pid = false;
float P = 0;
float I = 0;
float D = 0;
float timestep;
uint32_t prev_tick = 0;
uint8_t rec_buf[8];
int64_t current_pwm = 0x0;
float v_ref = 0;
char small_buf;
encoder enc = {0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HandleCommand(char * input);
void EncoderInput(uint8_t channel);//0 = a, 0 = b;
float CalculateSpeed(encoder *encoder);
void Pid_Init(PID_controller PID_controller);
void Pid(float v_ref);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();

  /* USER CODE BEGIN 2 */
  PID_controller pid = {
  .CallbackTimer = &htim6,
  .MeasurementTimer = &htim2,
  .actuator = &htim14,
  };
  Pid_Init(pid);

	char * startmessage = "---------------------\n\r";
	uprintf(startmessage);
	HAL_UART_Receive_IT(&huart1, rec_buf, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(HAL_GetTick() > prev_tick + 100){
			prev_tick = HAL_GetTick();

			if(print_dac){
				uprintf("v_ref = [%f], current_pwm = [%d]\n\r", v_ref, current_pwm);
			}
			if(print_time){
				uprintf("htim2 CNT = [%ld];\n\r", __HAL_TIM_GET_COUNTER(&htim2));
			}
			if(print_encoder){
				uprintf("encoder readings are = [%d , %d, %d, %i];\n\r", enc.a_cnt, enc.b_cnt, enc.period, enc.direction);
			}
			if(print_speed){
				uprintf("encoder speed = [%f];\n\r", CalculateSpeed(&enc));
			}
			if(print_pid){
				uprintf("PID = [%f, %f, %f]\n\r", P, I, D);
			}
		}
		if(huart2_Rx_flag){
			huart2_Rx_flag = false;
			HandlePcInput(&small_buf, 1, HandleCommand);
			HAL_UART_Receive_IT(&huart1, rec_buf, 1);
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HandleCommand(char * input){
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	if(!strcmp(input, "time")){
		print_time = !print_time;
	}else if(!strcmp(input, "encoder")){
		print_encoder = !print_encoder;
	}else if(!strcmp(input, "speed")){
		print_speed = !print_speed;
	}else if(!strcmp(input, "pid")){
		print_pid = !print_pid;
	}else if(!memcmp(input, "dac", 3)){
		char * ptr;
	    current_pwm = strtol(input+4, &ptr, 10);

		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, current_pwm);
	}else if(!memcmp(input, "ref", 3)){
	    v_ref = atof(input+4);
	}else if(!strcmp(input, "printdac")){
		print_dac = !print_dac;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	if(huart->Instance == huart1.Instance){
		huart2_Rx_flag = true;
		small_buf = *(huart->pRxBuffPtr-1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  switch(GPIO_Pin){
  case 0x0040:
	  EncoderInput(0);
	  break;
  case 0x0080:
	  EncoderInput(1);
	  break;
  }

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    Pid(v_ref);
}

void EncoderInput(uint8_t channel){
	if(channel){
		if(HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin)){
			enc.a_cnt++;
			enc.period = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin)){
				enc.direction = 1;
			}else{
				enc.direction = -1;
			}
		}
	}else{
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin)){
			enc.b_cnt++;
		}
	}
}
#define COUNTS_PER_ROTATION 12
#define CLK_FREQUENCY		48000000
#define GEAR_RATIO			10
#define Kp					1000
#define Ki					100
#define Kd					0.1

float CalculateSpeed(encoder *encoder){
	float rot_speed = 0;
	if(encoder->period > __HAL_TIM_GET_COUNTER(&htim2)){// if still zero, no encoder values were received
		float rot_period = encoder->period * COUNTS_PER_ROTATION;
		rot_speed = 1/rot_period;
		rot_speed = rot_speed * ((CLK_FREQUENCY/(htim2.Init.Prescaler+1))/GEAR_RATIO) * enc.direction;
		//encoder->period = 0;// So that if no new encoder values are caught we now speed = zero
	}else{
		float rot_period = __HAL_TIM_GET_COUNTER(&htim2) * COUNTS_PER_ROTATION;
		rot_speed = 1/rot_period;
		rot_speed = rot_speed * ((CLK_FREQUENCY/(htim2.Init.Prescaler+1))/GEAR_RATIO) * enc.direction;
	}
	return rot_speed;
}

void Pid_Init(PID_controller PID_controller){
	HAL_TIM_PWM_Start(PID_controller.actuator,TIM_CHANNEL_1);
	__HAL_TIM_ENABLE(PID_controller.MeasurementTimer);
	HAL_TIM_Base_Start_IT(PID_controller.CallbackTimer);
	timestep = ((float)htim6.Init.Period)/((float)CLK_FREQUENCY/((float)(htim6.Init.Prescaler + 1)));
}
void Pid(float v_ref){
	static float prev_error = 0;
	float error = v_ref - CalculateSpeed(&enc);
	P = Kp*error;
	I += Ki*error*timestep;
	I = (I > 0xfff) ? 0xfff : (I < 0) ? 0 : I;
	D = (Kd*(error-prev_error))/timestep;
	prev_error = error;

	current_pwm = (int)(P + I + D);
	current_pwm = (current_pwm > 0xfff) ? 0xfff : (current_pwm < 0) ? 0 : current_pwm;

	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, current_pwm);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
