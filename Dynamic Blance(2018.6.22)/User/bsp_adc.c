/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @author  tanguoyan/zhangmi
  * @version V1.0
  * @date    2018-5-15
  * @brief   ad采样
  ******************************************************************************
  */ 

#include "stm32f4xx_conf.h"
#include "bsp_adc.h"

static void Rheostat_ADC_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	
	// 使能 GPIO 时钟
	RCC_AHB1PeriphClockCmd(RHEOSTAT_ADC_GPIO_CLK|RCC_AHB1Periph_GPIOA, ENABLE);
		
	// 配置PA5，PC2，分别为左右通道
	GPIO_InitStructure.GPIO_Pin = RHEOSTAT_ADC_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	    
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; //不上拉不下拉
	GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStructure);	
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	    
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; //不上拉不下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);		
}

static void Rheostat_ADC_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
  // 开启ADC时钟
	RCC_APB2PeriphClockCmd(RHEOSTAT_ADC_CLK , ENABLE);

  // -------------------ADC Common 结构体 参数 初始化------------------------
	// 独立ADC模式
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  // 时钟为fpclk x分频	
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
  // 禁止DMA直接访问模式	
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  // 采样时间间隔	
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  
  ADC_CommonInit(&ADC_CommonInitStructure);
	
  // -------------------ADC Init 结构体 参数 初始化--------------------------
	ADC_StructInit(&ADC_InitStructure);
  // ADC 分辨率
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  // 禁止扫描模式，多通道采集才需要	
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
  // 连续转换	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
  //禁止外部边沿触发
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  //数据右对齐	
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  //转换通道 1个
  ADC_InitStructure.ADC_NbrOfConversion = 1;                                    
  ADC_Init(RHEOSTAT_ADC, &ADC_InitStructure);

  // 使能ADC
  ADC_Cmd(RHEOSTAT_ADC, ENABLE);  
  //开始adc转换，软件触发
  ADC_SoftwareStartConv(RHEOSTAT_ADC);
}

uint16_t Get_Adc(uint8_t ch)//设置指定ADC的规则通道，一个序列，采样时间
{
  ADC_RegularChannelConfig(RHEOSTAT_ADC,ch,1,ADC_SampleTime_480Cycles);//ADC1，ADC通道，采样时间为28周期，（1.5+28）/12M的转换时间
	ADC_SoftwareStartConv(ADC1);//使能指定的ADC1的软件转换启动功能
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//等待转换结束
	return ADC_GetConversionValue(ADC1);//返回最近一次ADC1规则的转换结果
}

void Rheostat_Init(void)
{
	Rheostat_ADC_GPIO_Config();
	Rheostat_ADC_Mode_Config();
}

//void Get_Adc_Vual(void)
//{
//	   float adcx1,adcx2,temp1,temp2;
//	
//    adcx1=(float)Get_Adc_Average(ADC_Channel_1,10);//ADC的值
//		temp1=adcx1*(3.3/4096);
//		adcx1=temp1;//ADC电压值
//		
//		adcx2=(float)Get_Adc_Average(ADC_Channel_12,10);
//		temp2=adcx2*(3.3/4096);
//		adcx2=temp2;
//		printf(" %f %f\r\n",adcx1,adcx2);
//	
//}
/*********************************************END OF FILE**********************/
