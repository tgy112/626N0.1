/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @author  tanguoyan/zhangmi
  * @version V1.0
  * @date    2018-5-15
  * @brief   ad����
  ******************************************************************************
  */ 

#include "stm32f4xx_conf.h"
#include "bsp_adc.h"

static void Rheostat_ADC_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	
	// ʹ�� GPIO ʱ��
	RCC_AHB1PeriphClockCmd(RHEOSTAT_ADC_GPIO_CLK|RCC_AHB1Periph_GPIOA, ENABLE);
		
	// ����PA5��PC2���ֱ�Ϊ����ͨ��
	GPIO_InitStructure.GPIO_Pin = RHEOSTAT_ADC_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	    
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; //������������
	GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStructure);	
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	    
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; //������������
	GPIO_Init(GPIOA, &GPIO_InitStructure);		
}

static void Rheostat_ADC_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
  // ����ADCʱ��
	RCC_APB2PeriphClockCmd(RHEOSTAT_ADC_CLK , ENABLE);

  // -------------------ADC Common �ṹ�� ���� ��ʼ��------------------------
	// ����ADCģʽ
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  // ʱ��Ϊfpclk x��Ƶ	
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
  // ��ֹDMAֱ�ӷ���ģʽ	
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  // ����ʱ����	
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  
  ADC_CommonInit(&ADC_CommonInitStructure);
	
  // -------------------ADC Init �ṹ�� ���� ��ʼ��--------------------------
	ADC_StructInit(&ADC_InitStructure);
  // ADC �ֱ���
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  // ��ֹɨ��ģʽ����ͨ���ɼ�����Ҫ	
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
  // ����ת��	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
  //��ֹ�ⲿ���ش���
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  //�����Ҷ���	
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  //ת��ͨ�� 1��
  ADC_InitStructure.ADC_NbrOfConversion = 1;                                    
  ADC_Init(RHEOSTAT_ADC, &ADC_InitStructure);

  // ʹ��ADC
  ADC_Cmd(RHEOSTAT_ADC, ENABLE);  
  //��ʼadcת�����������
  ADC_SoftwareStartConv(RHEOSTAT_ADC);
}

uint16_t Get_Adc(uint8_t ch)//����ָ��ADC�Ĺ���ͨ����һ�����У�����ʱ��
{
  ADC_RegularChannelConfig(RHEOSTAT_ADC,ch,1,ADC_SampleTime_480Cycles);//ADC1��ADCͨ��������ʱ��Ϊ28���ڣ���1.5+28��/12M��ת��ʱ��
	ADC_SoftwareStartConv(ADC1);//ʹ��ָ����ADC1�����ת����������
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//�ȴ�ת������
	return ADC_GetConversionValue(ADC1);//�������һ��ADC1�����ת�����
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
//    adcx1=(float)Get_Adc_Average(ADC_Channel_1,10);//ADC��ֵ
//		temp1=adcx1*(3.3/4096);
//		adcx1=temp1;//ADC��ѹֵ
//		
//		adcx2=(float)Get_Adc_Average(ADC_Channel_12,10);
//		temp2=adcx2*(3.3/4096);
//		adcx2=temp2;
//		printf(" %f %f\r\n",adcx1,adcx2);
//	
//}
/*********************************************END OF FILE**********************/
