/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  tanguoyan/zhangmi
  * @version V1.0
  * @date    2018-5-14
  * @brief   外部中断获取振动信号，并滤波计算幅值相位
  ******************************************************************************
  */ 

#include "stm32f4xx_conf.h"
#include "bsp_exti.h"
#include "math.h"
#include <stdio.h>
#include "bsp_adc.h"
//#include <stdint.h>

extern uint16_t RPM;//转速

float Vibration_A[LENGTH_COUNTER] = {0};//保存3次左右通道测量的幅值和相位
float Phase_a[LENGTH_COUNTER] = {0};
float Vibration_B[LENGTH_COUNTER] = {0};
float Phase_b[LENGTH_COUNTER] = {0};

uint16_t num_shuzu = 0; 
uint16_t flag_T_Signal=0;     //电机转速大于2000标志位
uint16_t flag_Start=1;        //开始标志位
uint16_t flag_Calculate = 0;  //计算完成标志位
uint16_t flag_print_OK = 0;   //打印标志位

float X[8][32],Y[8][32],X_TEMP[8][32],Y_TEMP[8][32];//数据存储数组，8个周期，每个周期32个数据
float M[33],N[33];                                  //滤波后的32个数据
float angle=11.25*3.141592653/180;                  //用于滤波计算的角度，弧度制

/**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 配置NVIC为优先级组2 */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置中断源：PH11 */
  NVIC_InitStructure.NVIC_IRQChannel = KEY1_INT_EXTI_IRQ;
  /* 配置抢占优先级：1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* 配置子优先级：2 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  /* 使能中断通道 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	  /* 配置中断源：PA6 */
  NVIC_InitStructure.NVIC_IRQChannel = KEY2_INT_EXTI_IRQ;
  NVIC_Init(&NVIC_InitStructure);
}

//中断配置
void EXTI_Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
  
	/*开启按键GPIO口的时钟*/
	RCC_AHB1PeriphClockCmd(KEY1_INT_GPIO_CLK|KEY2_INT_GPIO_CLK ,ENABLE);
  
  /* 使能 SYSCFG 时钟 ，使用GPIO外部中断时必须使能SYSCFG时钟*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* 配置 NVIC */
  NVIC_Configuration();
  
	/* 选择按键1的引脚 */ 
  GPIO_InitStructure.GPIO_Pin = KEY1_INT_GPIO_PIN;
  /* 设置引脚为输入模式 */ 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	    		
  /* 设置引脚不上拉也不下拉 */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  /* 使用上面的结构体初始化按键 */
  GPIO_Init(KEY1_INT_GPIO_PORT, &GPIO_InitStructure); 

	/* 连接 EXTI 中断源 到key1引脚 */
  SYSCFG_EXTILineConfig(KEY1_INT_EXTI_PORTSOURCE, KEY1_INT_EXTI_PINSOURCE);
	
  /* 选择 EXTI 中断源 */
  EXTI_InitStructure.EXTI_Line = KEY1_INT_EXTI_LINE;
  /* 中断模式 */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  /* 上升沿触发 */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  /* 使能中断/事件线 */
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* 选择按键2的引脚 */ 
  GPIO_InitStructure.GPIO_Pin = KEY2_INT_GPIO_PIN;  
  /* 其他配置与上面相同 */
  GPIO_Init(KEY2_INT_GPIO_PORT, &GPIO_InitStructure);      

	/* 连接 EXTI 中断源 到key2 引脚 */
  SYSCFG_EXTILineConfig(KEY2_INT_EXTI_PORTSOURCE,KEY2_INT_EXTI_PINSOURCE);

  /* 选择 EXTI 中断源 */
  EXTI_InitStructure.EXTI_Line = KEY2_INT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  /* 下降沿触发 */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

//中断的开启与关闭
void EXT(uint8_t en, uint8_t w)		//en:1，开启;0关闭;
{
    EXTI->PR=1<<w;  //清除11则为LINE11上的中断标志位
    if(en)EXTI->IMR|=1<<w;//不屏蔽line11上的中断
    else EXTI->IMR&=~(1<<w);//屏蔽line11上的中断   
}

void KEY1_IRQHandler(void)//PH11，周期中断
{
  //确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(KEY1_INT_EXTI_LINE) != RESET) 
	{
				if(RPM>200)//当转速达到2000时，开始采样
				{
						flag_T_Signal=1;
//						EXT(1,6);//打开采样信号中断
//						EXT(0,11);//关闭周期中断
				}
				else
					flag_T_Signal=0;//速度小于2000

					//清除中断标志位
			  	EXTI_ClearITPendingBit(KEY1_INT_EXTI_LINE);     
	}  
}

void KEY2_IRQHandler(void)//PA6
{
  //确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(KEY2_INT_EXTI_LINE) != RESET) 
		{
			 uint16_t i = 0,j = 0;
			
			 if (flag_T_Signal==1 && flag_Start == 1) //下一次速度达到2000且收到一个开始信号 开始采集
				 { 
						 i = num_shuzu / 32;//取商
						 j = num_shuzu % 32;//取余
						 
						 X[i][j]=Get_Adc(ADC_Channel_5)*(3.3/4096);
						 Y[i][j]=Get_Adc(ADC_Channel_12)*(3.3/4096);
						 
						 num_shuzu = num_shuzu + 1;
						 
						 if(num_shuzu > 256)//采样点结束
							 { 
									// flag_T_Signal=0;
									// flag_Start = 0;//停止采样
									 flag_Calculate = 1;//标记计算完成调用下面的函数，通知电机停机
									 flag_print_OK = 1; //采集完数据了，开始打印
									 num_shuzu = 0;
									 flag_T_Signal = 0;
//									 EXT(0,6);
//									 EXT(1,11);
												 
									for(i=0;i<=31;i++)//数组转存，避免正在处理数据过程中断到来更改数据
										{
												for(j=0;j<=7;j++)
												{ 
													X_TEMP[j][i] = X[j][i]; 
												}
										}
									for(i=0;i<=31;i++)//数组转存，避免正在处理数据过程中断到来更改数据
										{
												for(j=0;j<=7;j++)
												{ 
													Y_TEMP[j][i] = Y[j][i]; 
												}
										}
						  }
			 }
				
				//清除中断标志位
				EXTI_ClearITPendingBit(KEY2_INT_EXTI_LINE);     
		}  
}

//得到左校正平面的振幅与相位
uint8_t max_i=0;
float x_sum=0,mix_m,max_m;
float A,a,X_a,Y_a,a_temp,a_test;
void Get_Vibration_phase_left(void)
  {
		uint8_t i,j;
		for(j=0;j<=31;j++)//求每列AD电压平均值
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
			
		//互相关滤波(毛老师的法子)
		X_a=M[0]*cos(0)+M[1]*(cos(angle))+M[2]*(cos(2*angle))+M[3]*(cos(3*angle))+M[4]*(cos(4*angle))+M[5]*(cos(5*angle))+M[6]*(cos(6*angle))+M[7]*(cos(7*angle))
		+M[8]*(cos(8*angle))+M[9]*(cos(9*angle))+M[10]*(cos(10*angle))+M[11]*(cos(11*angle))+M[12]*(cos(12*angle))+M[13]*(cos(13*angle))+M[14]*(cos(14*angle))+M[15]*(cos(15*angle))
		+M[16]*(cos(16*angle))+M[17]*(cos(17*angle))+M[18]*(cos(18*angle))+M[19]*(cos(19*angle))+M[20]*(cos(20*angle))+M[21]*(cos(21*angle))+M[22]*(cos(22*angle))+M[23]*(cos(23*angle))
		+M[24]*(cos(24*angle))+M[25]*(cos(25*angle))+M[26]*(cos(26*angle))+M[27]*(cos(27*angle))+M[28]*(cos(28*angle))+M[29]*(cos(29*angle))+M[30]*(cos(30*angle))+M[31]*(cos(31*angle));
		
		Y_a=M[0]*sin(0)+M[1]*(sin(angle))+M[2]*(sin(2*angle))+M[3]*(sin(3*angle))+M[4]*(sin(4*angle))+M[5]*(sin(5*angle))+M[6]*(sin(6*angle))+M[7]*(sin(7*angle))
		+M[8]*(sin(8*angle))+M[9]*(sin(9*angle))+M[10]*(sin(10*angle))+M[11]*(sin(11*angle))+M[12]*(sin(12*angle))+M[13]*(sin(13*angle))+M[14]*(sin(14*angle))+M[15]*(sin(15*angle))
		+M[16]*(sin(16*angle))+M[17]*(sin(17*angle))+M[18]*(sin(18*angle))+M[19]*(sin(19*angle))+M[20]*(sin(20*angle))+M[21]*(sin(21*angle))+M[22]*(sin(22*angle))+M[23]*(sin(23*angle))
		+M[24]*(sin(24*angle))+M[25]*(sin(25*angle))+M[26]*(sin(26*angle))+M[27]*(sin(27*angle))+M[28]*(sin(28*angle))+M[29]*(sin(29*angle))+M[30]*(sin(30*angle))+M[31]*(sin(31*angle));
		
		max_m = M[0];
		for(i=0;i<=31;i++)//求出波形峰值移相点数
			{
					if(max_m<M[i]){max_m=M[i];max_i = i;}
					else;
			}
		
		A=sqrt(X_a*X_a+Y_a*Y_a)*78.92;//左校正平面的振动幅值
//	Get_three_times(Vibration_A,LENGTH_COUNTER,A);//得到原始振动、左校正平面加重、右校正平面加重的幅值A0、A1、A2，即Vibration_A[0]、Vibration_A[1]、Vibration_A[2]
		a_temp=atan(Y_a/X_a)/3.14159*180;//左校正平面的相位角
		
		if(X_a>0)//相脚补偿
			{
				if(Y_a<0)a = a_temp + 360;
				else		 a = a_temp;
			}
		else a = a_temp + 180;
		
		a-=80;//简单数据转换，方便与毛老师设备对比，后面根据实际需要修改
		if(a<0)a_test=a+360;
		else a_test = a;
			
//	printf("%f   %f   \r\n",A,a);
//	Get_three_times(Phase_a,LENGTH_COUNTER,a);//得到原始振动、左校正平面加重、右校正平面加重的相位a0、a1、a2，即Phase_a[0]、Phase_a[1]、Phase_a[2]
//  printf("%f   %f   %f   %f   %f   %f\r\n",Vibration_A[0],Vibration_A[1],Vibration_A[2],Phase_a[0],Phase_a[1],Phase_a[2]);
  }

//得到右校正平面的振幅与相位
float y_sum=0,max_n;
float B,b,X_b,Y_b,b_temp,b_test;
void Get_Vibration_phase_right(void)
  {
		uint8_t i,j;
		for (j=0;j<=31;j++)//求右振动信号每列电压平均值
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
			
		//相关滤波
		X_b=N[0]*cos(0)+N[1]*(cos(angle))+N[2]*(cos(2*angle))+N[3]*(cos(3*angle))+N[4]*(cos(4*angle))+N[5]*(cos(5*angle))+N[6]*(cos(6*angle))+N[7]*(cos(7*angle))
		+N[8]*(cos(8*angle))+N[9]*(cos(9*angle))+N[10]*(cos(10*angle))+N[11]*(cos(11*angle))+N[12]*(cos(12*angle))+N[13]*(cos(13*angle))+N[14]*(cos(14*angle))+N[15]*(cos(15*angle))
		+N[16]*(cos(16*angle))+N[17]*(cos(17*angle))+N[18]*(cos(18*angle))+N[19]*(cos(19*angle))+N[20]*(cos(20*angle))+N[21]*(cos(21*angle))+N[22]*(cos(22*angle))+N[23]*(cos(23*angle))
		+N[24]*(cos(24*angle))+N[25]*(cos(25*angle))+N[26]*(cos(26*angle))+N[27]*(cos(27*angle))+N[28]*(cos(28*angle))+N[29]*(cos(29*angle))+N[30]*(cos(30*angle))+N[30]*(cos(31*angle));
		
		Y_b=N[0]*sin(0)+N[1]*(sin(angle))+N[2]*(sin(2*angle))+N[3]*(sin(3*angle))+N[4]*(sin(4*angle))+N[5]*(sin(5*angle))+N[6]*(sin(6*angle))+N[7]*(sin(7*angle))
		+N[8]*(sin(8*angle))+N[9]*(sin(9*angle))+N[10]*(sin(10*angle))+N[11]*(sin(11*angle))+N[12]*(sin(12*angle))+N[13]*(sin(13*angle))+N[14]*(sin(14*angle))+N[15]*(sin(15*angle))
		+N[16]*(sin(16*angle))+N[17]*(sin(17*angle))+N[18]*(sin(18*angle))+N[19]*(sin(19*angle))+N[20]*(sin(20*angle))+N[21]*(sin(21*angle))+N[22]*(sin(22*angle))+N[23]*(sin(23*angle))
		+N[24]*(sin(24*angle))+N[25]*(sin(25*angle))+N[26]*(sin(26*angle))+N[27]*(sin(27*angle))+N[28]*(sin(28*angle))+N[29]*(sin(29*angle))+N[30]*(sin(30*angle))+N[31]*(sin(31*angle));
					
			max_n = N[0];
		for(i=0;i<=31;i++)//求出波形峰值移相点数
			{
					if(max_n<N[i]){max_n=N[i];max_i = i;}
					else;
			}
		
		B=sqrt(X_b*X_b+Y_b*Y_b)*78.92;//右校正平面在振动幅值
			
		b_temp=atan(Y_b/X_b)/3.14159*180;//右校正平面的相位
		
		//相位补偿
//		if((max_i<12 && (b_temp<=90 && b_temp>=0)) || (max_i>=24 && (b_temp>=-90 && b_temp<=0)))//将-90到90度的数据转换成为360数据
//			b=b_temp-180+270;
//		else 			
//			b=b_temp+270;
		
			if(X_b>0)//相脚补偿
			{
				if(Y_b<0)b = b_temp + 360;
				else		 b = b_temp;
			}
		else b = b_temp + 180;
			
		b+=190;//简单数据转换，方便与毛老师设备对比，后面根据实际需要修改
		if(b>360)b_test=b-360;
		else b_test = b;
		
		//Get_three_times(Vibration_B,LENGTH_COUNTER,B);//得到原始振动、左校正平面加重、右校正平面加重的幅值B0、B1、B2，即Vibration_B[0]、Vibration_B[1]、Vibration_B[2]

		//Get_three_times(Phase_b,LENGTH_COUNTER,b);//得到原始振动、左校正平面加重、右校正平面加重的相位b0、b1、b2，即Phase_b[0]、Phase_b[1]、Phase_b[2]
		//printf("%f %f %f %f %f %f\r\n",Vibration_B[0],Vibration_B[1],Vibration_B[2],Phase_b[0],Phase_b[1],Phase_b[2]);
  }	
	 
	//保存三次的振动幅值和相位的值
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



	
	