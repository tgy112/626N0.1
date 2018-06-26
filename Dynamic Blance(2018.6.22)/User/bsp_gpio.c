/**
  ******************************************************************************
  * @file    bsp_gpio.c
  * @author  zhangmi
  * @version V1.0
  * @date    2018-6-19
  * @brief   振动信号放大倍数选择AB=00：放大4倍，AB=10：放大2倍,AB=01：放大1倍,AB=00：缩小2倍.
	* 
	* @date    2018-6-20
  * @brief   沟槽信号分频，CD4522BPW---P0，P1，P2，P3；
	*P0P1P2P3=0000~1111，代表N分频，可选16个沟槽信号
  ******************************************************************************
  */ 

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "bsp_gpio.h"

void LED_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI|RCC_AHB1Periph_GPIOD|
	RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOH,ENABLE);
	
	//PD5，连接到CD4052B的放大选择A引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_5);
	
	//PI3，连接到CD4052B的放大选择B引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOI, &GPIO_InitStructure);
	GPIO_SetBits(GPIOI, GPIO_Pin_3);
	
		//PA8，连接到CD4522BPW的P0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	
		//PG2，连接到CD4522BPW的P1;PG3，连接到CD4522BPW的P2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOG, GPIO_Pin_2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOG, GPIO_InitStructure.GPIO_Pin);

  	//PH9，连接到CD4522BPW的P3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOH, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOH, GPIO_Pin_9);
	
}

