#ifndef __EXTI_H
#define	__EXTI_H

#include "stm32f4xx.h"
#define LENGTH_COUNTER 3
//���Ŷ���
/*******************************************************/
//PH11�������ź�����
#define KEY1_INT_GPIO_PORT                GPIOH
#define KEY1_INT_GPIO_CLK                 RCC_AHB1Periph_GPIOH
#define KEY1_INT_GPIO_PIN                 GPIO_Pin_11
#define KEY1_INT_EXTI_PORTSOURCE          EXTI_PortSourceGPIOH
#define KEY1_INT_EXTI_PINSOURCE           EXTI_PinSource11
#define KEY1_INT_EXTI_LINE                EXTI_Line11
#define KEY1_INT_EXTI_IRQ                 EXTI15_10_IRQn
#define KEY1_IRQHandler                   EXTI15_10_IRQHandler

//PA6��32��Ƶ�ź�����
#define KEY2_INT_GPIO_PORT                GPIOA
#define KEY2_INT_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define KEY2_INT_GPIO_PIN                 GPIO_Pin_6
#define KEY2_INT_EXTI_PORTSOURCE          EXTI_PortSourceGPIOA
#define KEY2_INT_EXTI_PINSOURCE           EXTI_PinSource6
#define KEY2_INT_EXTI_LINE                EXTI_Line6
#define KEY2_INT_EXTI_IRQ                 EXTI9_5_IRQn
#define KEY2_IRQHandler                   EXTI9_5_IRQHandler

/*******************************************************/

void EXTI_Key_Config(void);
void Get_Vibration_phase_left(void);
void Get_Vibration_phase_right(void);
#endif /* __EXTI_H */
