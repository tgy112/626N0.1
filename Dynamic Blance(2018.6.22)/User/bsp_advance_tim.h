#ifndef __GENERAL_TIM_H
#define	__GENERAL_TIM_H

#include "stm32f4xx.h"

/* ͨ�ö�ʱ�� */
#define GENERAL_TIM           		    TIM2
#define GENERAL_TIM_CLK       		    RCC_APB1Periph_TIM2

/* �߼����ƶ�ʱ��PWM���벶�� */
/* PWM���벶������ */
#define ADVANCE_ICPWM_PIN             GPIO_Pin_6            
#define ADVANCE_ICPWM_GPIO_PORT       GPIOI                      
#define ADVANCE_ICPWM_GPIO_CLK        RCC_AHB1Periph_GPIOI
#define ADVANCE_ICPWM_PINSOURCE				GPIO_PinSource6
#define ADVANCE_ICPWM_AF							GPIO_AF_TIM8
#define ADVANCE_IC1PWM_CHANNEL        TIM_Channel_1
#define ADVANCE_IC2PWM_CHANNEL        TIM_Channel_2

/* �߼����ƶ�ʱ�� */
#define ADVANCE_TIM           		    TIM8
#define ADVANCE_TIM_CLK       		    RCC_APB2Periph_TIM8

/* ����/�Ƚ��ж� */
#define ADVANCE_TIM_IRQn					    TIM8_CC_IRQn
#define ADVANCE_TIM_IRQHandler        TIM8_CC_IRQHandler


void TIMx_Configuration(void);

#endif /* __GENERAL_TIM_H */

