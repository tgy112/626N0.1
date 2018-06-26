/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  zhangmi
  * @version V1.0
  * @date    2018-6-12
  * @brief   485����
  ******************************************************************************
  */ 
#include "bsp_485.h"
#include <stdarg.h>
#include "stm32f4xx_conf.h"

static void Delay(__IO uint32_t nCount); 

// ����USART�����ж�
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 * ��������_485_Config
 * ����  ��USART GPIO ����,����ģʽ����
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void _485_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOI, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 
	
	  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_USART6);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_USART6);

	/* USART GPIO config  PC6����������*/
   /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6  ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(_485_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* USART GPIO config  PC7����������*/
  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(_485_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
  /* 485�շ����ƹܽ� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7  ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOI, &GPIO_InitStructure);
	  
	/* USART mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART6, &USART_InitStructure); 
  USART_Cmd(USART6, ENABLE);
	
	NVIC_Configuration();
	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	
	GPIO_ResetBits(GPIOI,GPIO_Pin_7); //Ĭ�Ͻ������ģʽ
}

/***************** ����һ���ַ�  **********************/
//ʹ�õ��ֽ����ݷ���ǰҪʹ�ܷ������ţ����ͺ�Ҫʹ�ܽ������š�
void _485_SendByte(  uint8_t ch )
{
	/* ����һ���ֽ����ݵ�USART1 */
	USART_SendData(_485_USART,ch);
		
	/* �ȴ�������� */
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);	
	
}
/*****************  ����ָ�����ȵ��ַ��� **********************/
void _485_SendStr_length( uint8_t *str,uint32_t strlen )
{
	unsigned int k=0;

	_485_TX_EN()	;//	ʹ�ܷ�������	
    do 
    {
        _485_SendByte( *(str + k) );
        k++;
    } while(k < strlen);
		
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	_485_RX_EN()	;//	ʹ�ܽ�������
}


/*****************  �����ַ��� **********************/
void _485_SendString(  uint8_t *str)
{
	unsigned int k=0;
	
	_485_TX_EN()	;//	ʹ�ܷ�������
	
    do 
    {
        _485_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	_485_RX_EN()	;//	ʹ�ܽ�������
}

//�жϻ��洮������
#define UART_BUFF_SIZE      1024
volatile    uint16_t uart_p = 0;
uint8_t     uart_buff[UART_BUFF_SIZE];

void bsp_485_IRQHandler(void)
{
    if(uart_p<UART_BUFF_SIZE)
    {
        if(USART_GetITStatus(_485_USART, USART_IT_RXNE) != RESET)
        {
            uart_buff[uart_p] = USART_ReceiveData(_485_USART);
            uart_p++;
						
						USART_ClearITPendingBit(_485_USART, USART_IT_RXNE);
        }
    }
		else
		{
			USART_ClearITPendingBit(_485_USART, USART_IT_RXNE);
//			clean_rebuff();       
		}
}

//��ȡ���յ������ݺͳ���
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
    return (char *)&uart_buff;
}

//��ջ�����
void clean_rebuff(void) 
{

    uint16_t i=UART_BUFF_SIZE+1;
    uart_p = 0;
	while(i)
		uart_buff[--i]=0;

}

static void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}

