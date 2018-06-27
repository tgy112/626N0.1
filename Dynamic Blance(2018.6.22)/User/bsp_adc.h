#ifndef __BSP_ADC_H
#define	__BSP_ADC_H
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"

// ADC GPIO 宏定义
#define RHEOSTAT_ADC_GPIO_PORT    GPIOC
#define RHEOSTAT_ADC_GPIO_PIN     GPIO_Pin_2
#define RHEOSTAT_ADC_GPIO_CLK     RCC_AHB1Periph_GPIOC

// ADC 序号宏定义
#define RHEOSTAT_ADC              ADC1
#define RHEOSTAT_ADC_CLK          RCC_APB2Periph_ADC1
#define RHEOSTAT_ADC_CHANNEL      ADC_Channel_12


// ADC 中断宏定义
#define Rheostat_ADC_IRQ            ADC_IRQn
#define Rheostat_ADC_INT_FUNCTION   ADC_IRQHandler

uint16_t Get_Adc_Average(uint16_t ch,uint16_t times);
void Rheostat_Init(void);
uint16_t Get_Adc(uint8_t ch);
#endif /* __BSP_ADC_H */



