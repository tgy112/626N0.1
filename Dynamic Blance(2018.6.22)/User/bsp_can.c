/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  zhangmi
  * @version V1.0
  * @date    2018-5-18
  * @brief   canͨ��
  ******************************************************************************
  */ 

#include "bsp_can.h"
#include "stm32f4xx_conf.h"
/*
 * ��������CAN_GPIO_Config
 * ����  ��CAN��GPIO ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;   	

  /* ʹ��GPIOʱ��*/
  RCC_AHB1PeriphClockCmd(CAN_TX_GPIO_CLK|CAN_RX_GPIO_CLK, ENABLE);
	
	  /* ����Դ*/
  GPIO_PinAFConfig(CAN_TX_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
  GPIO_PinAFConfig(CAN_RX_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT); 

	  /* ���� CAN TX ���� */
  GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN_TX_GPIO_PORT, &GPIO_InitStructure);
	
	  /* ���� CAN RX ���� */
  GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStructure);
}

/*
 * ��������CAN_NVIC_Config
 * ����  ��CAN��NVIC ����,��1���ȼ��飬0��0���ȼ�
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*�ж�����*/
		NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;	   //CAN RX�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * ��������CAN_Mode_Config
 * ����  ��CAN��ģʽ ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Mode_Config(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CANͨ�Ų�������**********************************/
	/* Enable CAN clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);
	/*CAN�Ĵ�����ʼ��*/
	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  ʹ���Զ����߹��� 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ������ ���ʱ�±��ĻḲ��ԭ�б���  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  ����FIFO���ȼ� ȡ���ڱ��ı�ʾ�� 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //��������ģʽ
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		   //BTR-SJW ����ͬ����Ծ���� 2��ʱ�䵥Ԫ
	 
	/* ss=1 bs1=5 bs2=3 λʱ�����Ϊ(1+5+3) �����ʼ�Ϊʱ������tq*(1+3+5)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;		   //BTR-TS1 ʱ���1 ռ����5��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;		   //BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ	
	
	/* CAN Baudrate = 1 MBps (1MBps��Ϊstm32��CAN�������) (CAN ʱ��Ƶ��Ϊ APB 1 = 45 MHz) */     
	CAN_InitStructure.CAN_Prescaler =5;		  //BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 45/(1+5+3)/5=1 Mbps
	CAN_Init(CANx, &CAN_InitStructure);
}

/*
 * ��������CAN_Filter_Config
 * ����  ��CAN��ɸѡ�� ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CANɸѡ����ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber=14;						//ɸѡ����0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//����������ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//ɸѡ��λ��Ϊ����32λ��
	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0�� */

	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;//((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;//(((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//ɸѡ����������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ��ɸѡ��
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CANͨ���ж�ʹ��*/
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}

/*
 * ��������CAN_Config
 * ����  ����������CAN�Ĺ���
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void CAN_Config(void)
{
  CAN_GPIO_Config();
  CAN_NVIC_Config();
  CAN_Mode_Config();
  CAN_Filter_Config();   
}

/**
  * @brief  ��ʼ�� Rx Message���ݽṹ��
  * @param  RxMessage: ָ��Ҫ��ʼ�������ݽṹ��
  * @retval None
  */
void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t ubCounter = 0;

	/*�ѽ��սṹ������*/
  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    RxMessage->Data[ubCounter] = 0x00;
  }
}

/*
 * ��������CAN_SetMsg
 * ����  ��CANͨ�ű�����������,����һ����������Ϊ0-7�����ݰ�
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	 
void CAN_SetMsg(CanTxMsg *TxMessage)
{	  
	uint8_t ubCounter = 0;

  //TxMessage.StdId=0x00;						 
  TxMessage->ExtId=0x1314;					 //ʹ�õ���չID
  TxMessage->IDE=CAN_ID_EXT;					 //��չģʽ
  TxMessage->RTR=CAN_RTR_DATA;				 //���͵�������
  TxMessage->DLC=8;							 //���ݳ���Ϊ8�ֽ�
	
	/*����Ҫ���͵�����0-7*/
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    TxMessage->Data[ubCounter] = ubCounter;
  }
}


/**************************END OF FILE************************************/










