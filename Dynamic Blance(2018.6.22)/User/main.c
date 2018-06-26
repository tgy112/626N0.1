#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "bsp_advance_tim.h"
#include "bsp_exti.h"
#include "bsp_adc.h"
#include "math.h"
#include "bsp_debug_usart.h"
#include "bsp_can.h"
#include "bsp_key.h" 
#include "bsp_485.h"
#include "bsp_gpio.h"

//定义矩阵的行与列的个数
#define ROW 100
#define COL 100
#define pi 	3.14159

struct polar_coordinate//创建矢量结构体，极坐标的大小和相位
{
	float size;
	float angle;
};
struct rectangular_coordinate//创建矢量结构体，直角坐标
{
	float x;
	float y;
};
/*****************************************************
***********
*********   数组计算相关函数(以下)
***********
*****************************************************/
void show_matrix(double [][COL], int, int);
int guass_elimination(double *[ROW], int, int);
void exchange_row(double *[ROW], int, int);
void show_solution(double *[ROW], int, int);
void count_solution(double *[ROW],struct polar_coordinate*,struct polar_coordinate*);
/*****************************************************
***********
*********   数组计算相关函数得到大小和相位
***********
*****************************************************/
//相位补偿，将atan函数的值域扩为360度
float myatan(float,float);		
//测试使用函数
void cal_m_test(struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,	
						struct polar_coordinate,struct polar_coordinate,struct polar_coordinate*,struct polar_coordinate*);
//求永久配重函数(去掉试重P)
void cal_m(struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,			
						struct polar_coordinate,struct polar_coordinate,struct polar_coordinate*,struct polar_coordinate*);
//声明求影响系数函数							
void cal_k(struct polar_coordinate ca11,struct polar_coordinate ca10,struct polar_coordinate cp,struct polar_coordinate *ck);
//外部变量标志位
extern uint16_t flag_T_Signal;
extern uint16_t flag_Calculate;
extern uint16_t flag_print_OK;
extern float adcx1,adcx2,temp1,temp2;
extern float X[16][32];
//外部变量不赋值
extern float Vibration_A[LENGTH_COUNTER],Phase_a[LENGTH_COUNTER];
extern float Vibration_B[LENGTH_COUNTER],Phase_b[LENGTH_COUNTER];

float adcx;
float temp;
int x,y;
//结构体变量，配种，振动，影响系数
struct polar_coordinate 			P_M1,P_M2,P_P1,P_P2,P_A10,P_A20,P_A11,P_A12,P_A21,P_A22,P_K11,P_K21,P_K12,P_K22;
struct rectangular_coordinate R_M1,R_M2,R_P1,R_P2,R_A10,R_A20,R_A11,R_A12,R_A21,R_A22,R_K11,R_K21,R_K12,R_K22;
//中断线
extern void EXT(uint8_t en,uint8_t w);
extern uint16_t RPM;//转速

//can代码
__IO uint32_t flag = 0;		 //用于标志是否接收到数据，在中断函数中赋值
CanTxMsg TxMessage;			     //发送缓冲区
CanRxMsg RxMessage;				 //接收缓冲区
static void can_delay(__IO uint32_t nCount)
{
	for(; nCount != 0; nCount--);
} 

//LED灯，作为调试使用
void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOH, &GPIO_InitStructure);
	GPIO_SetBits(GPIOH, GPIO_InitStructure.GPIO_Pin);
}

int main(void)
{		
		char *pbuf;
   	uint16_t len;
	
		TIMx_Configuration();//初始化输入捕获
		Debug_USART_Config();//串口初始化
		Rheostat_Init();//adc采集初始化
		EXTI_Key_Config();//周期以及倍频信号外部中断初始化
	  CAN_Config();//can初始化
	  Key_GPIO_Config();//初始化按键
	  _485_Config();
    LED_GPIO_Config();
   
	  EXT(0,11);
	  EXT(0,6);
	
	//485测试
	/*********************************************BEGIN OF FILE**********************/
//  while(1)
//  {
//		/*按一次按键发送一次数据*/
//		if(	Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
//		{
//			uint16_t i;
//			
//			_485_TX_EN();
//			
//			for(i=0;i<=0xff;i++)
//			{
//			_485_SendByte(i);	 //发送数据
//			}
//			
//			/*加短暂延时，保证485发送数据完毕*/
//			
//			_485_RX_EN();
//			
//			printf("\r\n发送数据成功！\r\n"); //使用调试串口打印调试信息到终端

//		}
//		else
//		{		
//			pbuf = get_rebuff(&len);
//			if(len>=256)
//			{
//				printf("\r\n接收到长度为%d的数据\r\n",len);	
//				_485_DEBUG_ARRAY((uint8_t*)pbuf,len);
//				clean_rebuff();
//			}
//		}
//  }

///*********************************************END OF FILE**********************/
//	
//	//CAN代码测试
//  while(1)
//	{
//		/*按一次按键发送一次数据*/
//		if(	Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
//		{
//			/*设置要发送的报文*/
//			CAN_SetMsg(&TxMessage);
//			/*把报文存储到发送邮箱，发送*/
//			CAN_Transmit(CANx, &TxMessage);
//			
//			can_delay(10000);//等待发送完毕，可使用CAN_TransmitStatus查看状态
//			
//			printf("\r\n已使用CAN发送数据包！\r\n"); 			
//			printf("\r\n发送的报文内容为：\r\n");
//			printf("\r\n 扩展ID号ExtId：0x%x \r\n",TxMessage.ExtId);
//			CAN_DEBUG_ARRAY(TxMessage.Data,8); 
//		}
//		if(flag==1)
//		{	
//			printf("\r\nCAN接收到数据：\r\n");	

//			CAN_DEBUG_ARRAY(RxMessage.Data,8); 
//			
//			flag=0;
//		}
//	}
	 
	//求影响系数以及永久配重
	 while(1)
	  {
					if(flag_Calculate == 1)	//当滤波完成后，得到左右校正平面幅相值
				{
					Get_Vibration_phase_left();
					Get_Vibration_phase_right();
					flag_Calculate = 0;
				}
				printf("%f\r\n",RPM);
				cal_k(P_A10,P_A11,P_P1,&P_K11);//求取影响系数k，需要分别求取，注意求K时候需要的振动量
				cal_k(P_A20,P_A21,P_P1,&P_K21);
				cal_k(P_A10,P_A12,P_P2,&P_K12);
				cal_k(P_A20,P_A22,P_P2,&P_K22);
				
				cal_m(P_A10,P_A20,P_K11,P_K12,P_K21,P_K22,&P_M1,&P_M2);//求取永久配重		     
  	}

}

//a10为原始震动，a11为加试重后的震动，pp为增加的试重，*pk为求得的影响系数。
void cal_k(struct polar_coordinate pa10,struct polar_coordinate pa11,struct polar_coordinate pp,struct polar_coordinate *pk)
{
		float ax,ay;

		pk->size  = sqrt(pa11.size*pa11.size+pa10.size*pa10.size-2*pa11.size*pa10.size*cos(pa11.angle-pa10.angle))/pp.size;
		ax = pa11.size*cos(pa11.angle)-pa10.size*cos(pa10.angle);
		ay = pa11.size*sin(pa11.angle)-pa10.size*sin(pa10.angle);
		
	//相位补偿，将atan函数的值域扩为360度
		if(ax>0)
			{
				if(ay<0) pk->angle = atan(ay/ax)-pp.angle + 3.14159*2;
				else		 pk->angle = atan(ay/ax)-pp.angle;
			}
		else pk->angle = atan(ay/ax)-pp.angle + 3.14159;	
			
//		pk->angle = pk->angle/pi*180;
}

//计算永久配重
void cal_m(struct polar_coordinate pa10,struct polar_coordinate pa20,struct polar_coordinate pk11,struct polar_coordinate pk12,
						struct polar_coordinate pk21,struct polar_coordinate pk22,struct polar_coordinate *pm1,struct polar_coordinate *pm2)
{
		double Receptacle[4][5]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//想要计算的矩阵存放于此，并在row和col说明矩阵的维数
		double *Ab_pointer[ROW];
    int row=4, col=4, i, result;	
							
			//赋值数组
			Receptacle[0][0]=pk11.size*cos(pk11.angle);Receptacle[0][1]=-pk11.size*sin(pk11.angle);Receptacle[0][2]=pk12.size*cos(pk12.angle);Receptacle[0][3]=-pk12.size*sin(pk12.angle);Receptacle[0][4]=-pa10.size*cos(pa10.angle);
			Receptacle[1][0]=pk21.size*cos(pk21.angle);Receptacle[1][1]=-pk21.size*sin(pk21.angle);Receptacle[1][2]=pk22.size*cos(pk22.angle);Receptacle[1][3]=-pk22.size*sin(pk22.angle);Receptacle[1][4]=-pa20.size*cos(pa20.angle);
			Receptacle[2][0]=pk11.size*sin(pk11.angle);Receptacle[2][1]= pk11.size*cos(pk11.angle);Receptacle[2][2]=pk12.size*sin(pk12.angle);Receptacle[2][3]= pk12.size*cos(pk12.angle);Receptacle[2][4]=-pa10.size*sin(pa10.angle);
			Receptacle[3][0]=pk21.size*sin(pk21.angle);Receptacle[3][1]= pk21.size*cos(pk21.angle);Receptacle[3][2]=pk22.size*sin(pk22.angle);Receptacle[3][3]= pk22.size*cos(pk22.angle);Receptacle[3][4]=-pa20.size*sin(pa20.angle);
						
			for(i=0;i<ROW;i++)
            Ab_pointer[i] = Receptacle[i];
			
			result = guass_elimination(Ab_pointer, row, col+1);
        if(result==1){
           // printf("无数个解!\n");
//            show_solution(Ab_pointer, row, col+1);
        }
        else if(result==0){
//            printf("只有一个解!\n");
//            show_solution(Ab_pointer, row, col+1);
					count_solution(Ab_pointer,pm1,pm2);
        }
//        else
//            printf("无解!\n");
}

void show_matrix(double matrix[ROW][COL], int row, int col)
{
    int i, j;

    for(i=0;i<row;i++){
        for(j=0;j<col;j++)
          //  printf("%-8.3f", matrix[i][j]);
        putchar('\n');
    }
    return;
}

int guass_elimination(double *matrix[ROW], int row, int col)
{
    int result, i, j, k;
    double coe;

    for(i=0;i<row-1;i++){
        exchange_row(matrix, i, row);
        if(fabs(*(matrix[i]+i))<0.00001)
            continue;
        for(j=i+1;j<row;j++){
            coe = *(matrix[j]+i) / *(matrix[i]+i);
            for(k=i;k<col;k++)
                *(matrix[j]+k) -= coe * *(matrix[i]+k);
        }
    }

    if(col-1>row)
        result = 1;
    else if(col-1==row){
        if(fabs(*(matrix[row-1]+row-1))>0.00001)
            result = 0;
        else{
            if(fabs(*(matrix[row-1]+row))>0.00001)
                result = -1;
            else
                result = 1;
        }
    }
    else{
        result = 0;
        for(i=0;i<row;i++)
            if(fabs(*(matrix[i]+col-2))<0.00001&&fabs(*(matrix[i]+col-1))>0.00001){
                result = -1;
                break;
            }
    }


    return result;
}

void exchange_row(double *matrix[ROW], int flag, int row)
{
    int i;
    double *temp;

    for(i=flag+1;i<row;i++)
        if(fabs(*(matrix[flag]+flag))<fabs(*(matrix[i]+flag))){
            temp = matrix[flag];
            matrix[flag] = matrix[i];
            matrix[i] = temp;
        }

    return;
}

void show_solution(double *matrix[ROW], int row, int col)
{
    int i, j;

    for(i=0;i<row;i++){
        for(j=0;j<col;j++)
           // printf("%-8.3f", *(matrix[i]+j));
        putchar('\n');
    }

    return;
}

void count_solution(double *matrix[ROW],struct polar_coordinate *pm1,struct polar_coordinate *pm2)
{
    float x1,x2,x3,x4;
	//计算上三角矩阵
    x4=matrix[3][4]/matrix[3][3];
		x3=(matrix[2][4]-x4*matrix[2][3])/matrix[2][2];
		x2=(matrix[1][4]-matrix[1][3]*x4-matrix[1][2]*x3)/matrix[1][1];
		x1=(matrix[0][4]-x4*matrix[0][3]-x3*matrix[0][2]-x2*matrix[0][1])/matrix[0][0];
	//得到配重大小和相位
	pm1->angle = myatan(x1,x2);
	pm1->size	 = x1/(cos(pm1->angle));
	pm2->angle = myatan(x3,x4);
	pm2->size	 = x3/(cos(pm2->angle));
//P_M1.angle = pm1->angle/pi*180;//转换成度数
//P_M2.angle = pm2->angle/pi*180;
}

//相位补偿，将atan函数的值域扩为360度
float myatan(float x,float y)
{
	if(x>0)
		{
			if(y<0) return (atan(y/x) + 3.14159*2);
			else		return (atan(y/x));
		}
	else return (atan(y/x) + 3.14159);
}


/*********************************************END OF FILE**********************/

