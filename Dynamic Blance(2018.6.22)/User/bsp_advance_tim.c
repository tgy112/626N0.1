/**
  ******************************************************************************
  * @file    bsp_advance_tim.c
  * @author  tanguoyan/zhangmi
  * @version V1.0
  * @date    2018-5-13
  * @brief   ���ת�ٲ���
  ******************************************************************************
  */ 

#include "bsp_advance_tim.h"
#include "bsp_debug_usart.h"
#include "stm32f4xx.h"  
#include "stm32f4xx_conf.h"

static void TIMx_GPIO_Config(void) 
{
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

//	RCC_AHB1PeriphClockCmd (GENERAL_OCPWM_GPIO_CLK, ENABLE); 
	RCC_AHB1PeriphClockCmd (ADVANCE_ICPWM_GPIO_CLK, ENABLE); 	

	/* ��ʱ���������� */
	GPIO_PinAFConfig(ADVANCE_ICPWM_GPIO_PORT,ADVANCE_ICPWM_PINSOURCE,ADVANCE_ICPWM_AF); 
	
	/* �߼����ƶ�ʱ��PWM���벶������ */													   	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_InitStructure.GPIO_Pin = ADVANCE_ICPWM_PIN;	
	GPIO_Init(ADVANCE_ICPWM_GPIO_PORT, &GPIO_InitStructure);
}

 // �߼����ƶ�ʱ�� TIM8,�ж����ȼ�����
static void TIMx_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    // �����ж���Ϊ2
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = ADVANCE_TIM_IRQn; 	
		// ������ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
	  // ���������ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//PWM���벶������
static void TIM_PWMINPUT_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	// ����TIMx_CLK
  RCC_APB2PeriphClockCmd(ADVANCE_TIM_CLK, ENABLE); 

  TIM_TimeBaseStructure.TIM_Period = 0xFFFF-1; 	
	// �߼����ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK=180MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=100KHz
  TIM_TimeBaseStructure.TIM_Prescaler = 1800-1;	
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  // ������ʽ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	// ��ʼ����ʱ��TIMx, x[1,8]
	TIM_TimeBaseInit(ADVANCE_TIM, &TIM_TimeBaseStructure);
	
	/* IC2���������ش��� TI1FP1 */
  TIM_ICInitStructure.TIM_Channel = ADVANCE_IC2PWM_CHANNEL;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_PWMIConfig(ADVANCE_TIM, &TIM_ICInitStructure);
	
	/* ѡ��ʱ�����봥��: TI1FP1 */
  TIM_SelectInputTrigger(ADVANCE_TIM, TIM_TS_TI2FP2);		

  /* ѡ���ģʽ: ��λģʽ */
  TIM_SelectSlaveMode(ADVANCE_TIM, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(ADVANCE_TIM,TIM_MasterSlaveMode_Enable);

  /* ʹ�ܸ߼����ƶ�ʱ�� */
  TIM_Cmd(ADVANCE_TIM, ENABLE);

  /* ʹ�ܲ���/�Ƚ�2�ж����� */
  TIM_ITConfig(ADVANCE_TIM, TIM_IT_CC2, ENABLE);
}

/**
  * @brief  ��ʼ���߼����ƶ�ʱ����ʱ��1ms����һ���ж�
  * @param  ��
  * @retval ��
  */
void TIMx_Configuration(void)
{
	TIMx_GPIO_Config();
	
	TIMx_NVIC_Configuration();	
  	
	TIM_PWMINPUT_Config();
}

/*********************************************END OF FILE**********************/

__IO uint16_t IC2Value = 0;
__IO float Frequency = 0;
	
float RPM=0,flag_RPM;

//��ʱ���жϷ�����
void  ADVANCE_TIM_IRQHandler (void)
{
  /* �����ʱ������/�Ƚ�1�ж� */
  TIM_ClearITPendingBit(ADVANCE_TIM, TIM_IT_CC2);

  /* ��ȡ���벶��ֵ */
  IC2Value = TIM_GetCapture2(ADVANCE_TIM);	
//  printf("IC1Value = %d  IC2Value = %d ",IC1Value,IC2Value);
	
  if (IC2Value != 0)
  {
    /* Ƶ�ʼ��� */
    Frequency = 180000000/1800/(float)IC2Value;
		RPM = 60 * Frequency;
		printf("%f\r\n",RPM);
  }
  else
  {
    Frequency = 0;
  }
}

void Get_RPM(void)
{     
	   // delay_ms(100);
			flag_RPM=flag_RPM+1;
			if(flag_RPM>1)
			{
			  RPM=0;
			}
}

/*********************************************END OF FILE**********************/


