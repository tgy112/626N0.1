/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  tanguoyan/zhangmi
  * @version V1.0
  * @date    2018-5-14
  * @brief   �ⲿ�жϻ�ȡ���źţ����˲������ֵ��λ
  ******************************************************************************
  */ 

#include "stm32f4xx_conf.h"
#include "bsp_exti.h"
#include "math.h"
#include <stdio.h>
#include "bsp_adc.h"
//#include <stdint.h>

extern uint16_t RPM;//ת��

float Vibration_A[LENGTH_COUNTER] = {0};//����3������ͨ�������ķ�ֵ����λ
float Phase_a[LENGTH_COUNTER] = {0};
float Vibration_B[LENGTH_COUNTER] = {0};
float Phase_b[LENGTH_COUNTER] = {0};

uint16_t num_shuzu = 0; 
uint16_t flag_T_Signal=0;     //���ת�ٴ���2000��־λ
uint16_t flag_Start=1;        //��ʼ��־λ
uint16_t flag_Calculate = 0;  //������ɱ�־λ
uint16_t flag_print_OK = 0;   //��ӡ��־λ

float X[8][32],Y[8][32],X_TEMP[8][32],Y_TEMP[8][32];//���ݴ洢���飬8�����ڣ�ÿ������32������
float M[33],N[33];                                  //�˲����32������
float angle=11.25*3.141592653/180;                  //�����˲�����ĽǶȣ�������

/**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* ����NVICΪ���ȼ���2 */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* �����ж�Դ��PH11 */
  NVIC_InitStructure.NVIC_IRQChannel = KEY1_INT_EXTI_IRQ;
  /* ������ռ���ȼ���1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* ���������ȼ���2 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  /* ʹ���ж�ͨ�� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	  /* �����ж�Դ��PA6 */
  NVIC_InitStructure.NVIC_IRQChannel = KEY2_INT_EXTI_IRQ;
  NVIC_Init(&NVIC_InitStructure);
}

//�ж�����
void EXTI_Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
  
	/*��������GPIO�ڵ�ʱ��*/
	RCC_AHB1PeriphClockCmd(KEY1_INT_GPIO_CLK|KEY2_INT_GPIO_CLK ,ENABLE);
  
  /* ʹ�� SYSCFG ʱ�� ��ʹ��GPIO�ⲿ�ж�ʱ����ʹ��SYSCFGʱ��*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* ���� NVIC */
  NVIC_Configuration();
  
	/* ѡ�񰴼�1������ */ 
  GPIO_InitStructure.GPIO_Pin = KEY1_INT_GPIO_PIN;
  /* ��������Ϊ����ģʽ */ 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	    		
  /* �������Ų�����Ҳ������ */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  /* ʹ������Ľṹ���ʼ������ */
  GPIO_Init(KEY1_INT_GPIO_PORT, &GPIO_InitStructure); 

	/* ���� EXTI �ж�Դ ��key1���� */
  SYSCFG_EXTILineConfig(KEY1_INT_EXTI_PORTSOURCE, KEY1_INT_EXTI_PINSOURCE);
	
  /* ѡ�� EXTI �ж�Դ */
  EXTI_InitStructure.EXTI_Line = KEY1_INT_EXTI_LINE;
  /* �ж�ģʽ */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  /* �����ش��� */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  /* ʹ���ж�/�¼��� */
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* ѡ�񰴼�2������ */ 
  GPIO_InitStructure.GPIO_Pin = KEY2_INT_GPIO_PIN;  
  /* ����������������ͬ */
  GPIO_Init(KEY2_INT_GPIO_PORT, &GPIO_InitStructure);      

	/* ���� EXTI �ж�Դ ��key2 ���� */
  SYSCFG_EXTILineConfig(KEY2_INT_EXTI_PORTSOURCE,KEY2_INT_EXTI_PINSOURCE);

  /* ѡ�� EXTI �ж�Դ */
  EXTI_InitStructure.EXTI_Line = KEY2_INT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  /* �½��ش��� */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

//�жϵĿ�����ر�
void EXT(uint8_t en, uint8_t w)		//en:1������;0�ر�;
{
    EXTI->PR=1<<w;  //���11��ΪLINE11�ϵ��жϱ�־λ
    if(en)EXTI->IMR|=1<<w;//������line11�ϵ��ж�
    else EXTI->IMR&=~(1<<w);//����line11�ϵ��ж�   
}

void KEY1_IRQHandler(void)//PH11�������ж�
{
  //ȷ���Ƿ������EXTI Line�ж�
	if(EXTI_GetITStatus(KEY1_INT_EXTI_LINE) != RESET) 
	{
				if(RPM>200)//��ת�ٴﵽ2000ʱ����ʼ����
				{
						flag_T_Signal=1;
//						EXT(1,6);//�򿪲����ź��ж�
//						EXT(0,11);//�ر������ж�
				}
				else
					flag_T_Signal=0;//�ٶ�С��2000

					//����жϱ�־λ
			  	EXTI_ClearITPendingBit(KEY1_INT_EXTI_LINE);     
	}  
}

void KEY2_IRQHandler(void)//PA6
{
  //ȷ���Ƿ������EXTI Line�ж�
	if(EXTI_GetITStatus(KEY2_INT_EXTI_LINE) != RESET) 
		{
			 uint16_t i = 0,j = 0;
			
			 if (flag_T_Signal==1 && flag_Start == 1) //��һ���ٶȴﵽ2000���յ�һ����ʼ�ź� ��ʼ�ɼ�
				 { 
						 i = num_shuzu / 32;//ȡ��
						 j = num_shuzu % 32;//ȡ��
						 
						 X[i][j]=Get_Adc(ADC_Channel_5)*(3.3/4096);
						 Y[i][j]=Get_Adc(ADC_Channel_12)*(3.3/4096);
						 
						 num_shuzu = num_shuzu + 1;
						 
						 if(num_shuzu > 256)//���������
							 { 
									// flag_T_Signal=0;
									// flag_Start = 0;//ֹͣ����
									 flag_Calculate = 1;//��Ǽ�����ɵ�������ĺ�����֪ͨ���ͣ��
									 flag_print_OK = 1; //�ɼ��������ˣ���ʼ��ӡ
									 num_shuzu = 0;
									 flag_T_Signal = 0;
//									 EXT(0,6);
//									 EXT(1,11);
												 
									for(i=0;i<=31;i++)//����ת�棬�������ڴ������ݹ����жϵ�����������
										{
												for(j=0;j<=7;j++)
												{ 
													X_TEMP[j][i] = X[j][i]; 
												}
										}
									for(i=0;i<=31;i++)//����ת�棬�������ڴ������ݹ����жϵ�����������
										{
												for(j=0;j<=7;j++)
												{ 
													Y_TEMP[j][i] = Y[j][i]; 
												}
										}
						  }
			 }
				
				//����жϱ�־λ
				EXTI_ClearITPendingBit(KEY2_INT_EXTI_LINE);     
		}  
}

//�õ���У��ƽ����������λ
uint8_t max_i=0;
float x_sum=0,mix_m,max_m;
float A,a,X_a,Y_a,a_temp,a_test;
void Get_Vibration_phase_left(void)
  {
		uint8_t i,j;
		for(j=0;j<=31;j++)//��ÿ��AD��ѹƽ��ֵ
			{
				x_sum = 0;
					for(i=0;i<=7;i++)
						{ 
							x_sum += X_TEMP[i][j]; 
						}
					M[j] = (x_sum / 8);
					x_sum = 0;
					//printf("%f \r\n",M[j]);
			}
			
//		printf("1 \r\n");
			
		//������˲�(ë��ʦ�ķ���)
		X_a=M[0]*cos(0)+M[1]*(cos(angle))+M[2]*(cos(2*angle))+M[3]*(cos(3*angle))+M[4]*(cos(4*angle))+M[5]*(cos(5*angle))+M[6]*(cos(6*angle))+M[7]*(cos(7*angle))
		+M[8]*(cos(8*angle))+M[9]*(cos(9*angle))+M[10]*(cos(10*angle))+M[11]*(cos(11*angle))+M[12]*(cos(12*angle))+M[13]*(cos(13*angle))+M[14]*(cos(14*angle))+M[15]*(cos(15*angle))
		+M[16]*(cos(16*angle))+M[17]*(cos(17*angle))+M[18]*(cos(18*angle))+M[19]*(cos(19*angle))+M[20]*(cos(20*angle))+M[21]*(cos(21*angle))+M[22]*(cos(22*angle))+M[23]*(cos(23*angle))
		+M[24]*(cos(24*angle))+M[25]*(cos(25*angle))+M[26]*(cos(26*angle))+M[27]*(cos(27*angle))+M[28]*(cos(28*angle))+M[29]*(cos(29*angle))+M[30]*(cos(30*angle))+M[31]*(cos(31*angle));
		
		Y_a=M[0]*sin(0)+M[1]*(sin(angle))+M[2]*(sin(2*angle))+M[3]*(sin(3*angle))+M[4]*(sin(4*angle))+M[5]*(sin(5*angle))+M[6]*(sin(6*angle))+M[7]*(sin(7*angle))
		+M[8]*(sin(8*angle))+M[9]*(sin(9*angle))+M[10]*(sin(10*angle))+M[11]*(sin(11*angle))+M[12]*(sin(12*angle))+M[13]*(sin(13*angle))+M[14]*(sin(14*angle))+M[15]*(sin(15*angle))
		+M[16]*(sin(16*angle))+M[17]*(sin(17*angle))+M[18]*(sin(18*angle))+M[19]*(sin(19*angle))+M[20]*(sin(20*angle))+M[21]*(sin(21*angle))+M[22]*(sin(22*angle))+M[23]*(sin(23*angle))
		+M[24]*(sin(24*angle))+M[25]*(sin(25*angle))+M[26]*(sin(26*angle))+M[27]*(sin(27*angle))+M[28]*(sin(28*angle))+M[29]*(sin(29*angle))+M[30]*(sin(30*angle))+M[31]*(sin(31*angle));
		
		max_m = M[0];
		for(i=0;i<=31;i++)//������η�ֵ�������
			{
					if(max_m<M[i]){max_m=M[i];max_i = i;}
					else;
			}
		
		A=sqrt(X_a*X_a+Y_a*Y_a)*78.92;//��У��ƽ����񶯷�ֵ
//	Get_three_times(Vibration_A,LENGTH_COUNTER,A);//�õ�ԭʼ�񶯡���У��ƽ����ء���У��ƽ����صķ�ֵA0��A1��A2����Vibration_A[0]��Vibration_A[1]��Vibration_A[2]
		a_temp=atan(Y_a/X_a)/3.14159*180;//��У��ƽ�����λ��
		
		if(X_a>0)//��Ų���
			{
				if(Y_a<0)a = a_temp + 360;
				else		 a = a_temp;
			}
		else a = a_temp + 180;
		
		a-=80;//������ת����������ë��ʦ�豸�Աȣ��������ʵ����Ҫ�޸�
		if(a<0)a_test=a+360;
		else a_test = a;
			
//	printf("%f   %f   \r\n",A,a);
//	Get_three_times(Phase_a,LENGTH_COUNTER,a);//�õ�ԭʼ�񶯡���У��ƽ����ء���У��ƽ����ص���λa0��a1��a2����Phase_a[0]��Phase_a[1]��Phase_a[2]
//  printf("%f   %f   %f   %f   %f   %f\r\n",Vibration_A[0],Vibration_A[1],Vibration_A[2],Phase_a[0],Phase_a[1],Phase_a[2]);
  }

//�õ���У��ƽ����������λ
float y_sum=0,max_n;
float B,b,X_b,Y_b,b_temp,b_test;
void Get_Vibration_phase_right(void)
  {
		uint8_t i,j;
		for (j=0;j<=31;j++)//�������ź�ÿ�е�ѹƽ��ֵ
		  {
        for(i=0;i<=7;i++)
					{  
						 y_sum=y_sum+(Y[i][j]);
					}
				N[j] = (y_sum / 8 );
				y_sum = 0;
				//printf("%f \r\n",N[j]);
			}	
    printf("2 \r\n");
			
		//����˲�
		X_b=N[0]*cos(0)+N[1]*(cos(angle))+N[2]*(cos(2*angle))+N[3]*(cos(3*angle))+N[4]*(cos(4*angle))+N[5]*(cos(5*angle))+N[6]*(cos(6*angle))+N[7]*(cos(7*angle))
		+N[8]*(cos(8*angle))+N[9]*(cos(9*angle))+N[10]*(cos(10*angle))+N[11]*(cos(11*angle))+N[12]*(cos(12*angle))+N[13]*(cos(13*angle))+N[14]*(cos(14*angle))+N[15]*(cos(15*angle))
		+N[16]*(cos(16*angle))+N[17]*(cos(17*angle))+N[18]*(cos(18*angle))+N[19]*(cos(19*angle))+N[20]*(cos(20*angle))+N[21]*(cos(21*angle))+N[22]*(cos(22*angle))+N[23]*(cos(23*angle))
		+N[24]*(cos(24*angle))+N[25]*(cos(25*angle))+N[26]*(cos(26*angle))+N[27]*(cos(27*angle))+N[28]*(cos(28*angle))+N[29]*(cos(29*angle))+N[30]*(cos(30*angle))+N[30]*(cos(31*angle));
		
		Y_b=N[0]*sin(0)+N[1]*(sin(angle))+N[2]*(sin(2*angle))+N[3]*(sin(3*angle))+N[4]*(sin(4*angle))+N[5]*(sin(5*angle))+N[6]*(sin(6*angle))+N[7]*(sin(7*angle))
		+N[8]*(sin(8*angle))+N[9]*(sin(9*angle))+N[10]*(sin(10*angle))+N[11]*(sin(11*angle))+N[12]*(sin(12*angle))+N[13]*(sin(13*angle))+N[14]*(sin(14*angle))+N[15]*(sin(15*angle))
		+N[16]*(sin(16*angle))+N[17]*(sin(17*angle))+N[18]*(sin(18*angle))+N[19]*(sin(19*angle))+N[20]*(sin(20*angle))+N[21]*(sin(21*angle))+N[22]*(sin(22*angle))+N[23]*(sin(23*angle))
		+N[24]*(sin(24*angle))+N[25]*(sin(25*angle))+N[26]*(sin(26*angle))+N[27]*(sin(27*angle))+N[28]*(sin(28*angle))+N[29]*(sin(29*angle))+N[30]*(sin(30*angle))+N[31]*(sin(31*angle));
					
			max_n = N[0];
		for(i=0;i<=31;i++)//������η�ֵ�������
			{
					if(max_n<N[i]){max_n=N[i];max_i = i;}
					else;
			}
		
		B=sqrt(X_b*X_b+Y_b*Y_b)*78.92;//��У��ƽ�����񶯷�ֵ
			
		b_temp=atan(Y_b/X_b)/3.14159*180;//��У��ƽ�����λ
		
		//��λ����
//		if((max_i<12 && (b_temp<=90 && b_temp>=0)) || (max_i>=24 && (b_temp>=-90 && b_temp<=0)))//��-90��90�ȵ�����ת����Ϊ360����
//			b=b_temp-180+270;
//		else 			
//			b=b_temp+270;
		
			if(X_b>0)//��Ų���
			{
				if(Y_b<0)b = b_temp + 360;
				else		 b = b_temp;
			}
		else b = b_temp + 180;
			
		b+=190;//������ת����������ë��ʦ�豸�Աȣ��������ʵ����Ҫ�޸�
		if(b>360)b_test=b-360;
		else b_test = b;
		
		//Get_three_times(Vibration_B,LENGTH_COUNTER,B);//�õ�ԭʼ�񶯡���У��ƽ����ء���У��ƽ����صķ�ֵB0��B1��B2����Vibration_B[0]��Vibration_B[1]��Vibration_B[2]

		//Get_three_times(Phase_b,LENGTH_COUNTER,b);//�õ�ԭʼ�񶯡���У��ƽ����ء���У��ƽ����ص���λb0��b1��b2����Phase_b[0]��Phase_b[1]��Phase_b[2]
		//printf("%f %f %f %f %f %f\r\n",Vibration_B[0],Vibration_B[1],Vibration_B[2],Phase_b[0],Phase_b[1],Phase_b[2]);
  }	
	 
	//�������ε��񶯷�ֵ����λ��ֵ
	void Get_three_times(float Vibration_phase[],int n,float num)
	{
	  int m = 0;
		for(m = n-2;m >= 0;m--)
			{
				Vibration_phase[m+1] = Vibration_phase[m];
			}
	  	Vibration_phase[0] = num;
	}

	
	
	
/*********************************************END OF FILE**********************/



	
	