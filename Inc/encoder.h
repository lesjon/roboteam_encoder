/*
 * encoder.h
 *
 *  Created on: Nov 20, 2017
 *      Author: Leon
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdbool.h>
#include "stm32f0xx_hal.h"
#include "tim.h"
#include "gpio.h"

/* holds values for one encoder
 * holds both constants and output
 */
typedef struct{
	uint16_t CHANNEL[2];// pin number of channel A
	GPIO_TypeDef * CHANNEL_Port[2]; // GPIO Port of channel A
	TIM_HandleTypeDef* MeasurementTimer;// This is the handle of the timer that will be used to measure the time between interrupts
	int cnt[2];//counter for the rising edges of channel A, counts up or down depending on direction
	bool high[2];
	int8_t direction;// current direction, 1 is forward voltage, -1 is reverse voltage 0 is not yet known
	int period;// period in us for the latest count
	float speed;// speed calculated by encoder_CalculateSpeed()
	float COUNTS_PER_ROTATION;// Encoder stat
	float CLK_FREQUENCY;// frequency of the MeasurementTimer clock
	float GEAR_RATIO;// Gear Ratio of the motor
}encoder_HandleTypeDef;

// Initialize the encoder pointed to by enc
void encoder_Init(encoder_HandleTypeDef* enc);
/* handle input from the encoder lines, should be called within the external interrupt call
 * and needs to be called for all encoders which could be affected
 */
void encoder_Input(uint16_t channel, encoder_HandleTypeDef* enc);
/* Calculate the speed of encoder enc, this function also stores the speed in teh object
 *
 */
float encoder_CalculateSpeed(encoder_HandleTypeDef* enc);
/* Returns the last calculated speed from this encoder
 *
 */
float encoder_GetLatestSpeed(encoder_HandleTypeDef* enc);

#endif /* ENCODER_H_ */
