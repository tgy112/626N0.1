/**
  ******************************************************************************
  * @file    bsp_gpio.c
  * @author  zhangmi
  * @version V1.0
  * @date    2018-6-19
  * @brief   ���źŷŴ���ѡ��AB=00���Ŵ�4����AB=10���Ŵ�2��,AB=01���Ŵ�1��,AB=00����С2��.
	* 
	* @date    2018-6-20
  * @brief   �����źŷ�Ƶ��CD4522BPW---P0��P1��P2��P3��
	*P0P1P2P3=0000~1111������N��Ƶ����ѡ16�������ź�
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
	
	//PD5�����ӵ�CD4052B�ķŴ�ѡ��A����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_5);
	
	//PI3�����ӵ�CD4052B�ķŴ�ѡ��B����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOI, &GPIO_InitStructure);
	GPIO_SetBits(GPIOI, GPIO_Pin_3);
	
		//PA8�����ӵ�CD4522BPW��P0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	
		//PG2�����ӵ�CD4522BPW��P1;PG3�����ӵ�CD4522BPW��P2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOG, GPIO_Pin_2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOG, GPIO_InitStructure.GPIO_Pin);

  	//PH9�����ӵ�CD4522BPW��P3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOH, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOH, GPIO_Pin_9);
	
}

