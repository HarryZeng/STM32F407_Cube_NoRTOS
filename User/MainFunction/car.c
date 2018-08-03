
#include "car.h"
#include "usart.h"
#include "contact.h"
#include "odometry.h"
#include "tim.h"

#define CommunicateSerialPort   huart3
/***********************************************  说明  *****************************************************************
*
*   1.串口接收
*    （1）内容：小车左右轮速度,单位:mm/s（所有数据都为float型，float型占4字节）
*    （2）格式：10字节 [右轮速度4字节][左轮速度4字节][结束符"\r\n"2字节]
*
*   2.串口发送
*    （1）内容：里程计（x,y坐标、线速度、角速度和方向角，单位依次为：mm,mm,mm/s,rad/s,rad，所有数据都为float型，float型占4字节）
*    （2）格式：21字节 [x坐标4字节][y坐标4字节][方向角4字节][线速度4字节][角速度4字节][结束符"\n"1字节]
*
************************************************************************************************************************/
/***********************************************  输出  *****************************************************************/

char odometry_data[21]={0};   //发送给串口的里程计数据数组

float odometry_right=0,odometry_left=0;//串口得到的左右轮速度

/***********************************************  输入  *****************************************************************/

extern float position_x,position_y,oriention,velocity_linear,velocity_angular;         //计算得到的里程计数值

extern uint8_t USART_RX_BUF[USART_REC_LEN];     //串口接收缓冲,最大USART_REC_LEN个字节.
extern uint16_t USART_RX_STA;                   //串口接收状态标记	

extern float Milemeter_L_Motor,Milemeter_R_Motor;     //dt时间内的左右轮速度,用于里程计计算

/***********************************************  变量  *****************************************************************/

uint8_t main_sta=0; //用作处理主函数各种if，去掉多余的flag（1打印里程计）（2调用计算里程计数据函数）（3串口接收成功）（4串口接收失败）

union recieveData  //接收到的数据
{
	float d;    //左右轮速度
	unsigned char data[4];
}leftdata,rightdata;       //接收的左右轮数据

union odometry  //里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     //要发布的里程计数据，分别为：X，Y方向移动的距离，当前角度，线速度，角速度

/****************************************************************************************************************/	

extern int32_t hSpeed_Buffer1[],hSpeed_Buffer2[];//左右轮速度缓存数组
extern uint8_t main_sta;//主函数步骤执行标志位

//extern u8 bSpeed_Buffer_Index;
uint8_t bSpeed_Buffer_Index = 0;//缓存左右轮编码数到数组变量

//extern float Milemeter_L_Motor,Milemeter_R_Motor;      //累计电机一次运行的里程 cm		
float  Milemeter_L_Motor=0,Milemeter_R_Motor=0;//dt时间内的左右轮速度,用于里程计计算

uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART_RX_STA=0;   //接收状态标记	
uint8_t serial_rec=0x31;   //接收串口数据变量

extern float pulse;//电机A PID调节后的PWM值缓存
extern float pulse1;//电机B PID调节后的PWM值缓存


/*Printf重映射*/
int fputc(int ch, FILE *f)
{
   CommunicateSerialPort.Instance ->DR=(uint8_t)ch;
   while((CommunicateSerialPort.Instance->SR&0X40)==0);
   return ch;
}
int GetKey(void) 
{ 
   while (!(CommunicateSerialPort.Instance->SR & 0x20));
   return ((int)(CommunicateSerialPort.Instance->DR & 0x1FF));
}


/*串口接收回调函数*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if( huart == &CommunicateSerialPort) 
	{ 
		if((USART_RX_STA&0x8000)==0)//接收未完成
        {
            if(USART_RX_STA&0x4000)//接收到了0x0d
            {
                if(serial_rec==0x0a)
                {
                    if((USART_RX_STA&0x3f)==8)
                    {							
                        USART_RX_STA|=0x8000;	//接收完成了 
                        main_sta|=0x04;
                        main_sta&=0xF7;
											HAL_GPIO_TogglePin(U1_RX_LED_GPIO_Port,U1_RX_LED_Pin);
                    }
                    else
                    {
                        main_sta|=0x08;
                        main_sta&=0xFB;
                        USART_RX_STA=0;//接收错误,重新开始
                    }
                }
                else 
                {
                    main_sta|=0x08;
                    USART_RX_STA=0;//接收错误,重新开始
                }
            }
            else //还没收到0X0D
            {	
                if(serial_rec==0x0d)
								{
									USART_RX_STA|=0x4000;
									HAL_GPIO_TogglePin(U1_RX_LED_GPIO_Port,U1_RX_LED_Pin);
								}
                else
                {
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=serial_rec ;
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))
                    {
                        main_sta|=0x08;
                        USART_RX_STA=0;//接收数据错误,重新开始接收
                    }							
                }		 
            }
        } 
	} 
	HAL_UART_Receive_IT(huart, (uint8_t *)&serial_rec, 1) ; /*开启接收终端，接收下一个数据*/
} 

/*定时器回调函数*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if (htim->Instance == htim6.Instance)
		{
				if (hSpeedMeas_Timebase_500us !=0)//电机编码数采集时间间隔未到
				{
						hSpeedMeas_Timebase_500us--;//开始倒数	
				}
				else    //电机编码数采集时间间隔到了
				{
						int32_t wtemp2,wtemp1;
						
						hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//恢复电机编码数采集时间间隔
						
						/************************ 1 ***************************/
						
						wtemp2 = ENC_Calc_Rot_Speed2(); //A 获取的编码数
						wtemp1 = ENC_Calc_Rot_Speed1(); //B 获取的编码数
						
	//            //如果为停止指令，即左右轮速度为零，则清除速度存储器防止前后速度差太大造成小车冲转
	//            if((wtemp2 == 0) && (wtemp1 == 0))
	//            {
	//                pulse=pulse1=0;
	//            }
						 
						/************************ 2 ***************************/
						
						//储存编码数（脉冲数），用于里程计计算
						Milemeter_L_Motor= (float)wtemp1; //储存脉冲数
						Milemeter_R_Motor= (float)wtemp2;
						
						main_sta|=0x02;//执行计算里程计数据步骤

						/************************ 3 ***************************/
						
						//开始缓存左右轮编码数到数组
						hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2;
						hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
						bSpeed_Buffer_Index++;//数组移位
						
						//缓存左右轮编码数到数组结束判断
						if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
						{
								bSpeed_Buffer_Index=0;//缓存左右轮编码数到数组变量清零
						}
						
						/************************ 4 ***************************/
						
						ENC_Calc_Average_Speed();//计算三次电机的平均编码数
						Gain2(); //电机A转速PID调节控制 右
						Gain1(); //电机B转速PID调节控制 左
				}
		}
		
		if (htim->Instance == htim7.Instance)
		{
				HAL_GPIO_TogglePin(RUN_LED_GPIO_Port,RUN_LED_Pin);
		}
}

/**/

void CarMainFunction(void)
{
	uint8_t t=0;
	uint8_t i=0,j=0,m=0;
	uint8_t SendZero=0x00;
	uint8_t Send_n='\n';
		/*执行发送里程计数据步骤*/
		if(main_sta&0x01)
		{
            //里程计数据获取
			x_data.odoemtry_float=position_x;//单位mm
			y_data.odoemtry_float=position_y;//单位mm
			theta_data.odoemtry_float=oriention;//单位rad
			vel_linear.odoemtry_float=velocity_linear;//单位mm/s
			vel_angular.odoemtry_float=velocity_angular;//单位rad/s
            
            //将所有里程计数据存到要发送的数组
			for(j=0;j<4;j++)
			{
				odometry_data[j]=x_data.odometry_char[j];
				odometry_data[j+4]=y_data.odometry_char[j];
				odometry_data[j+8]=theta_data.odometry_char[j];
				odometry_data[j+12]=vel_linear.odometry_char[j];
				odometry_data[j+16]=vel_angular.odometry_char[j];
			}
            
			odometry_data[20]='\n';//添加结束符
            
				//发送数据要串口
				if(HAL_UART_Transmit_IT(&CommunicateSerialPort,(uint8_t*)odometry_data,21)!=HAL_OK)
					Error_Handler(); //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题				
            
			main_sta&=0xFE;//执行计算里程计数据步骤
		}
		
		
		/*执行计算里程计数据步骤*/
		if(main_sta&0x02)
		{
			odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程计
        
			main_sta&=0xFD;//执行发送里程计数据步骤
		} 
		
		/*当发送指令没有正确接收时*/
		if(main_sta&0x08)
		{
					  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题
            for(m=0;m<3;m++)
            {
							if(HAL_UART_Transmit_IT(&CommunicateSerialPort,&SendZero,1)!=HAL_OK)
								Error_Handler(); //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题
            }		
						
						if(HAL_UART_Transmit_IT(&CommunicateSerialPort,&Send_n,1)!=HAL_OK)
								Error_Handler();
            main_sta&=0xF7;
		}
		
		/*串口3接收函数*/
		if(USART_RX_STA&0x8000)
		{			
            //接收左右轮速度
            for(t=0;t<4;t++)
            {
                rightdata.data[t]=USART_RX_BUF[t];
                leftdata.data[t]=USART_RX_BUF[t+4];
            }
            
            //储存左右轮速度
            odometry_right=rightdata.d;//单位mm/s
            odometry_left=leftdata.d;//单位mm/s
            
			USART_RX_STA=0;//清楚接收标志位
		}
       
    car_control(rightdata.d,leftdata.d);	 //将接收到的左右轮速度赋给小车	
}


