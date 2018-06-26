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

//�������������еĸ���
#define ROW 100
#define COL 100
#define pi 	3.14159

struct polar_coordinate//����ʸ���ṹ�壬������Ĵ�С����λ
{
	float size;
	float angle;
};
struct rectangular_coordinate//����ʸ���ṹ�壬ֱ������
{
	float x;
	float y;
};
/*****************************************************
***********
*********   ���������غ���(����)
***********
*****************************************************/
void show_matrix(double [][COL], int, int);
int guass_elimination(double *[ROW], int, int);
void exchange_row(double *[ROW], int, int);
void show_solution(double *[ROW], int, int);
void count_solution(double *[ROW],struct polar_coordinate*,struct polar_coordinate*);
/*****************************************************
***********
*********   ���������غ����õ���С����λ
***********
*****************************************************/
//��λ��������atan������ֵ����Ϊ360��
float myatan(float,float);		
//����ʹ�ú���
void cal_m_test(struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,	
						struct polar_coordinate,struct polar_coordinate,struct polar_coordinate*,struct polar_coordinate*);
//���������غ���(ȥ������P)
void cal_m(struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,			
						struct polar_coordinate,struct polar_coordinate,struct polar_coordinate*,struct polar_coordinate*);
//������Ӱ��ϵ������							
void cal_k(struct polar_coordinate ca11,struct polar_coordinate ca10,struct polar_coordinate cp,struct polar_coordinate *ck);
//�ⲿ������־λ
extern uint16_t flag_T_Signal;
extern uint16_t flag_Calculate;
extern uint16_t flag_print_OK;
extern float adcx1,adcx2,temp1,temp2;
extern float X[16][32];
//�ⲿ��������ֵ
extern float Vibration_A[LENGTH_COUNTER],Phase_a[LENGTH_COUNTER];
extern float Vibration_B[LENGTH_COUNTER],Phase_b[LENGTH_COUNTER];

float adcx;
float temp;
int x,y;
//�ṹ����������֣��񶯣�Ӱ��ϵ��
struct polar_coordinate 			P_M1,P_M2,P_P1,P_P2,P_A10,P_A20,P_A11,P_A12,P_A21,P_A22,P_K11,P_K21,P_K12,P_K22;
struct rectangular_coordinate R_M1,R_M2,R_P1,R_P2,R_A10,R_A20,R_A11,R_A12,R_A21,R_A22,R_K11,R_K21,R_K12,R_K22;
//�ж���
extern void EXT(uint8_t en,uint8_t w);
extern uint16_t RPM;//ת��

//can����
__IO uint32_t flag = 0;		 //���ڱ�־�Ƿ���յ����ݣ����жϺ����и�ֵ
CanTxMsg TxMessage;			     //���ͻ�����
CanRxMsg RxMessage;				 //���ջ�����
static void can_delay(__IO uint32_t nCount)
{
	for(; nCount != 0; nCount--);
} 

//LED�ƣ���Ϊ����ʹ��
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
	
		TIMx_Configuration();//��ʼ�����벶��
		Debug_USART_Config();//���ڳ�ʼ��
		Rheostat_Init();//adc�ɼ���ʼ��
		EXTI_Key_Config();//�����Լ���Ƶ�ź��ⲿ�жϳ�ʼ��
	  CAN_Config();//can��ʼ��
	  Key_GPIO_Config();//��ʼ������
	  _485_Config();
    LED_GPIO_Config();
   
	  EXT(0,11);
	  EXT(0,6);
	
	//485����
	/*********************************************BEGIN OF FILE**********************/
//  while(1)
//  {
//		/*��һ�ΰ�������һ������*/
//		if(	Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
//		{
//			uint16_t i;
//			
//			_485_TX_EN();
//			
//			for(i=0;i<=0xff;i++)
//			{
//			_485_SendByte(i);	 //��������
//			}
//			
//			/*�Ӷ�����ʱ����֤485�����������*/
//			
//			_485_RX_EN();
//			
//			printf("\r\n�������ݳɹ���\r\n"); //ʹ�õ��Դ��ڴ�ӡ������Ϣ���ն�

//		}
//		else
//		{		
//			pbuf = get_rebuff(&len);
//			if(len>=256)
//			{
//				printf("\r\n���յ�����Ϊ%d������\r\n",len);	
//				_485_DEBUG_ARRAY((uint8_t*)pbuf,len);
//				clean_rebuff();
//			}
//		}
//  }

///*********************************************END OF FILE**********************/
//	
//	//CAN�������
//  while(1)
//	{
//		/*��һ�ΰ�������һ������*/
//		if(	Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
//		{
//			/*����Ҫ���͵ı���*/
//			CAN_SetMsg(&TxMessage);
//			/*�ѱ��Ĵ洢���������䣬����*/
//			CAN_Transmit(CANx, &TxMessage);
//			
//			can_delay(10000);//�ȴ�������ϣ���ʹ��CAN_TransmitStatus�鿴״̬
//			
//			printf("\r\n��ʹ��CAN�������ݰ���\r\n"); 			
//			printf("\r\n���͵ı�������Ϊ��\r\n");
//			printf("\r\n ��չID��ExtId��0x%x \r\n",TxMessage.ExtId);
//			CAN_DEBUG_ARRAY(TxMessage.Data,8); 
//		}
//		if(flag==1)
//		{	
//			printf("\r\nCAN���յ����ݣ�\r\n");	

//			CAN_DEBUG_ARRAY(RxMessage.Data,8); 
//			
//			flag=0;
//		}
//	}
	 
	//��Ӱ��ϵ���Լ���������
	 while(1)
	  {
					if(flag_Calculate == 1)	//���˲���ɺ󣬵õ�����У��ƽ�����ֵ
				{
					Get_Vibration_phase_left();
					Get_Vibration_phase_right();
					flag_Calculate = 0;
				}
				printf("%f\r\n",RPM);
				cal_k(P_A10,P_A11,P_P1,&P_K11);//��ȡӰ��ϵ��k����Ҫ�ֱ���ȡ��ע����Kʱ����Ҫ������
				cal_k(P_A20,P_A21,P_P1,&P_K21);
				cal_k(P_A10,P_A12,P_P2,&P_K12);
				cal_k(P_A20,P_A22,P_P2,&P_K22);
				
				cal_m(P_A10,P_A20,P_K11,P_K12,P_K21,P_K22,&P_M1,&P_M2);//��ȡ��������		     
  	}

}

//a10Ϊԭʼ�𶯣�a11Ϊ�����غ���𶯣�ppΪ���ӵ����أ�*pkΪ��õ�Ӱ��ϵ����
void cal_k(struct polar_coordinate pa10,struct polar_coordinate pa11,struct polar_coordinate pp,struct polar_coordinate *pk)
{
		float ax,ay;

		pk->size  = sqrt(pa11.size*pa11.size+pa10.size*pa10.size-2*pa11.size*pa10.size*cos(pa11.angle-pa10.angle))/pp.size;
		ax = pa11.size*cos(pa11.angle)-pa10.size*cos(pa10.angle);
		ay = pa11.size*sin(pa11.angle)-pa10.size*sin(pa10.angle);
		
	//��λ��������atan������ֵ����Ϊ360��
		if(ax>0)
			{
				if(ay<0) pk->angle = atan(ay/ax)-pp.angle + 3.14159*2;
				else		 pk->angle = atan(ay/ax)-pp.angle;
			}
		else pk->angle = atan(ay/ax)-pp.angle + 3.14159;	
			
//		pk->angle = pk->angle/pi*180;
}

//������������
void cal_m(struct polar_coordinate pa10,struct polar_coordinate pa20,struct polar_coordinate pk11,struct polar_coordinate pk12,
						struct polar_coordinate pk21,struct polar_coordinate pk22,struct polar_coordinate *pm1,struct polar_coordinate *pm2)
{
		double Receptacle[4][5]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//��Ҫ����ľ������ڴˣ�����row��col˵�������ά��
		double *Ab_pointer[ROW];
    int row=4, col=4, i, result;	
							
			//��ֵ����
			Receptacle[0][0]=pk11.size*cos(pk11.angle);Receptacle[0][1]=-pk11.size*sin(pk11.angle);Receptacle[0][2]=pk12.size*cos(pk12.angle);Receptacle[0][3]=-pk12.size*sin(pk12.angle);Receptacle[0][4]=-pa10.size*cos(pa10.angle);
			Receptacle[1][0]=pk21.size*cos(pk21.angle);Receptacle[1][1]=-pk21.size*sin(pk21.angle);Receptacle[1][2]=pk22.size*cos(pk22.angle);Receptacle[1][3]=-pk22.size*sin(pk22.angle);Receptacle[1][4]=-pa20.size*cos(pa20.angle);
			Receptacle[2][0]=pk11.size*sin(pk11.angle);Receptacle[2][1]= pk11.size*cos(pk11.angle);Receptacle[2][2]=pk12.size*sin(pk12.angle);Receptacle[2][3]= pk12.size*cos(pk12.angle);Receptacle[2][4]=-pa10.size*sin(pa10.angle);
			Receptacle[3][0]=pk21.size*sin(pk21.angle);Receptacle[3][1]= pk21.size*cos(pk21.angle);Receptacle[3][2]=pk22.size*sin(pk22.angle);Receptacle[3][3]= pk22.size*cos(pk22.angle);Receptacle[3][4]=-pa20.size*sin(pa20.angle);
						
			for(i=0;i<ROW;i++)
            Ab_pointer[i] = Receptacle[i];
			
			result = guass_elimination(Ab_pointer, row, col+1);
        if(result==1){
           // printf("��������!\n");
//            show_solution(Ab_pointer, row, col+1);
        }
        else if(result==0){
//            printf("ֻ��һ����!\n");
//            show_solution(Ab_pointer, row, col+1);
					count_solution(Ab_pointer,pm1,pm2);
        }
//        else
//            printf("�޽�!\n");
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
	//���������Ǿ���
    x4=matrix[3][4]/matrix[3][3];
		x3=(matrix[2][4]-x4*matrix[2][3])/matrix[2][2];
		x2=(matrix[1][4]-matrix[1][3]*x4-matrix[1][2]*x3)/matrix[1][1];
		x1=(matrix[0][4]-x4*matrix[0][3]-x3*matrix[0][2]-x2*matrix[0][1])/matrix[0][0];
	//�õ����ش�С����λ
	pm1->angle = myatan(x1,x2);
	pm1->size	 = x1/(cos(pm1->angle));
	pm2->angle = myatan(x3,x4);
	pm2->size	 = x3/(cos(pm2->angle));
//P_M1.angle = pm1->angle/pi*180;//ת���ɶ���
//P_M2.angle = pm2->angle/pi*180;
}

//��λ��������atan������ֵ����Ϊ360��
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

