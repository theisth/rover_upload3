/*
 * DC_MOTORS.h
 *
 *  Created on: Apr 30, 2021
 *      Author: yusuf
 */

#ifndef INC_DC_MOTORS_H_
#define INC_DC_MOTORS_H_
#include "stdio.h"
#include "main.h"

//void Forward_Drive(GPIO_TypeDef* port, uint16_t forwardPin,uint16_t backwardPin);
//void Backward_Drive(GPIO_TypeDef* port, uint16_t forwardPin,uint16_t backwardPin);
//void Reset_Drive(GPIO_TypeDef* port, uint16_t forwardPin,uint16_t backwardPin);
//int map(int x, int in_min, int in_max, int out_min, int out_max);
void DC_Control(int16_t pwm_linear,int16_t pwm_angular,TIM_HandleTypeDef *tim);
int map(int x, int in_min, int in_max, int out_min, int out_max);

#endif /* INC_DC_MOTORS_H_ */
