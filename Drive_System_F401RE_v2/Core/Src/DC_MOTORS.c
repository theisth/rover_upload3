/*
 * DC_MOTORS.c
 *
 *  Created on: Apr 30, 2021
 *      Author: yusuf
 *      New commitment added by Cemal on 22.07.2022
 */

#include "DC_MOTORS.h"
#include "main.h"
#include "stdlib.h"

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
}

void DC_Control(int16_t pwm_linear,int16_t pwm_angular,TIM_HandleTypeDef *tim)
{
  int16_t speed_left = 0;
  int16_t speed_right = 0;

  //	0,2,4,6 IN1 for forward
  //	1,3,5,7 IN2 for backward

  //	Pin_B 0,1 ||------|| Pin_B 4,5
  //	   	         |  |
  //		         |  |
  //	Pin_B 2,3 ||------|| Pin_B 6,7


	speed_left = (pwm_linear - pwm_angular) + 1525;
	speed_right = (pwm_linear + pwm_angular) + 1525;

	if(speed_left < 1525)
		speed_left = map(speed_left,1125,1525,1300,1450);
	else if(speed_left > 1525)
		speed_left = map(speed_left,1525,1925,1600,1750);

	if(speed_right < 1525)
		speed_right = map(speed_right,1125,1525,1300,1450);
	else if(speed_right > 1525)
		speed_right = map(speed_right,1525,1925,1600,1750);

//	speed_left = map(speed_left,1125,1925,1300,1750);
//	speed_right = map(speed_right,1125,1925,1300,1750);


  	if(speed_left > 2000)
  		speed_left = 2000;
  	if(speed_right > 2000)
  		speed_right = 2000;

	if(speed_left < 1000)
  		speed_left = 1000;
  	if(speed_right < 1000)
  		speed_right = 1000;

    tim->Instance->CCR1 = speed_right;
    tim->Instance->CCR2 = speed_right;
    tim->Instance->CCR3 = speed_left;
    tim->Instance->CCR4 = speed_left;

}
