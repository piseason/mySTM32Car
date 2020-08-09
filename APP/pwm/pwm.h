#ifndef _pwm_H
#define _pwm_H

#include "system.h"
	
#define PWM_PIN_TIM1 		GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11

#define PWM_PIN_TIM3_A 		GPIO_Pin_6 | GPIO_Pin_7
#define PWM_PIN_TIM3_B		GPIO_Pin_0 | GPIO_Pin_1

#define msSpeed 			500		//100ms

extern volatile u16 cntL;
extern volatile u16 cntR;


void TIM1_PWM_Init(u16 arr, u16 psc);	//USART1占了PA9&10
void TIM3_PWM_Init(u16 arr, u16 psc);
void TIM2_CH12_PWM_Init(u16 arr,u16 psc);
void TIM2_CH34_Input_Init(u16 arr,u16 psc);
void TIM4_CH12_PWM_Init(u16 arr,u16 psc);
void startSpeedCount(void);
void TIM4_counter_Init(u16 arr,u16 psc);

#endif
