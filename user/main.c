#include "stm32f10x.h"
#include "pid.h"
#include "grey.h"
#include "motor.h"
#include "mpu6050.h"
#include "Buzzer.h"
#include "Key.h"
#include "oled.h"
#include "Delay.h"
#include <math.h>
#include "control.h"
#include "timer.h"

int pid_out;
int find_err;
int PID_angle(int target,int yaw);
float target_angle=0;
int left=0,right=0,aleft=0,aright=0;
float Yaw0_360=0;
int ack=0;
int all_state=0;
int steps;	
int yaw_int;
int wait_enable,stop_cnt=0;
int main(void)
{	
	//初始化
	Track_Init();
	Beep_LED_Init();
	Motor_Init();
	Key_Init();	
	OLED_Init();
	MPU6050_Init();				
	Beep_Off();
	LED_On();
	TIM2_Init();
	Motor_Set_PWM(0, 0); 
	steps=1;		
	while(1)
{     
       if(Key1_Scan()==1)
	   {	
            while(1)
			{			
			if(mpu_flag)
            {
            mpu_flag = 0;
            MPU6050_DMP_Get_Yaw(&Yaw0_360);
			   if(wait_enable)
				{
				stop_cnt++;
				}
            }
			
			all_state=Get_Infrared_State();
			
			if(oled_flag)
			{
			oled_flag=0;	
			OLED_ShowSignedNum(3,1,Yaw0_360,3);			
			OLED_ShowBinNum(1,1,all_state,8);
			OLED_ShowSignedNum(2,1,pid_out,2);
			OLED_ShowNum(4,1,steps,1);
				
			}
			switch (steps)
			{				
				case 1:  //A-B
				{
					if(all_state==0)
					{	
						LED_Off();																																													   
						yaw_int = (int)(Yaw0_360 + 0.5f);
                        							
						// 调用角度环
					    pid_out = PID_angle(359, yaw_int);                         
						
						// 电机速度分配
						aleft = 35 - pid_out;
						aright = 35 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Motor_Set_PWM(aleft,aright);						    		
					}
					else 
					{
						
						LED_On();
						Beep_On();
						Motor_Set_PWM(0, 0);       // 第一步：先让电机完全停车
					    wait_enable = 1;

					if(stop_cnt >= 50)
					{
						Beep_Off();
						LED_Off();
						wait_enable = 0;
						stop_cnt = 0;
						steps = 2;
						find_err = 0.0f;
					}       																	       
					}				
				}break;
						    			     				
		
				case 2:  //B-C
				{   				
					if(all_state!=0)
					{	
						
						find_err=Track_err();		
						pid_out=PID_out(find_err,0); //第一次循迹
						Final_Speed(pid_out,30); 
					}												
					else
					{	
                    wait_enable = 1;
					LED_On();
					Beep_On();
					Motor_Set_PWM(0,0);
					if(stop_cnt >= 50)
					{
						Beep_Off();
						LED_Off();
						wait_enable = 0;
						stop_cnt = 0;												
					    steps=3;				
					}       																				
					}			
				}break;	
							
				
				case 3:  //C-D
				{	
                  if(all_state==0)
					{	
						
						yaw_int = (int)(Yaw0_360 + 0.5f);
                        							
						// 调用角度环
					    int pid_out = PID_angle(165, yaw_int);                         
				
						// 电机速度分配
						aleft = 35 - pid_out;
						aright = 35 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Motor_Set_PWM(aleft,aright);
					}
                  else 
					{
						
						LED_On();
						Beep_On();
					wait_enable = 1;
					Motor_Set_PWM(0, 0); 
					find_err = 0.0f;
					if(stop_cnt >= 50)
					{
						Beep_Off();
						LED_Off();
						wait_enable = 0;
						stop_cnt = 0;
						steps = 4;
						
					}  												      								
					}				
				}break;						
	
				case 4:   //D-A
					{   										
					if(all_state!=0)
					{	
						Beep_Off();
						LED_Off();
						find_err=Track_err();		
						pid_out=PID_out(find_err,0); //第二次循迹
						Final_Speed(pid_out,30); 
					}												
					else
					{	
						wait_enable=1;
						
						LED_On();
						Beep_On();
						Motor_Set_PWM(0,0);	
						if(stop_cnt >= 50)
					{
						Beep_Off();
						LED_Off();
						wait_enable = 0;
						stop_cnt = 0;												
					    steps=999;				
					}  
					}			              
					
				}break;				
												
				default:
					{   
					Beep_Off();
		            LED_Off();																					
					Motor_Set_PWM(0,0);								
				
				     }break;	
				 }
			 }
		 }			
			
										
	if(Key2_Scan()==1)
	{		
		while (1)
		{	
						
			if(mpu_flag)
            {
            mpu_flag = 0;
            MPU6050_DMP_Get_Yaw(&Yaw0_360);
				if(wait_enable)
				{
				stop_cnt++;
				}
            }		
			all_state=Get_Infrared_State();			
			if(oled_flag)
			{
			oled_flag=0;	
			OLED_ShowSignedNum(3,1,Yaw0_360,3);			
			OLED_ShowBinNum(1,1,all_state,8);
			OLED_ShowSignedNum(2,1,pid_out,2);
			OLED_ShowNum(4,1,steps,1);
				
			}
			switch (steps)
			{
				
				case 1: //A到C
				{
					if(all_state==0)
					{	
						
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用角度环
						int pid_out = PID_angle(320, yaw_int);   //第一次斜线 

						// 电机速度分配
						int aleft = 35 - pid_out;
						int aright = 35 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Motor_Set_PWM(aleft, aright);					
					}
					else 
					{
						LED_On();
						Beep_On();
						Motor_Set_PWM(0, 20);       // 第一步：先让电机完全停车
				        wait_enable = 1;

					if(stop_cnt >= 50)
					{
						Beep_Off();
		                LED_Off();	
						wait_enable = 0;
						stop_cnt = 0;
						steps = 2;
						find_err = 0.0f;
					}       							
					}				
				}break;
			
				case 2:  //C-B
				{   					
					if(all_state!=0)
					{							
												    
						find_err=Track_err();		
						pid_out=PID_out(find_err ,0); //第一次循迹
						Final_Speed(pid_out,27); 
					}		
										
					else
					{
						LED_On();
						Beep_On();
						Motor_Set_PWM(0, 0);       // 第一步：先让电机完全停车
				        wait_enable = 1;

					if(stop_cnt >= 50)
					{
						Beep_Off();
		                LED_Off();	
						wait_enable = 0;
						stop_cnt = 0;
						steps = 3;
						
					}       					
				}
				}break;			
				
				case 3:  //B-D
					{   
					if(all_state==0)
					{
																				
						int yaw_int = (int)(Yaw0_360 + 0.5f);

						// 调用无死区的pid_angle
						int pid_out = PID_angle(237, yaw_int);

						// 电机速度分配
						int aleft =35 - pid_out;
						int aright = 35 + pid_out;

						// 速度限幅与执行
						Limit(&aleft, &aright);
						Motor_Set_PWM(aleft, aright);						
					}

					else
					{
						LED_On();
						Beep_On();
						Motor_Set_PWM(20, 0);       // 第一步：先让电机完全停车
						wait_enable = 1;

					    if(stop_cnt >= 50)
					    {
						Beep_Off();
		                LED_Off();	
						wait_enable = 0;
						stop_cnt = 0;
						steps = 4;
						find_err = 0.0f;
					    } 
					}						
					}	break;				
	
				case 4:{   		//D-A								
					if(all_state!=0)
					{
										
						find_err=Track_err();		
						pid_out=PID_out(find_err ,0); //第二次循迹
						Final_Speed(pid_out,30); 
					}
					else
					{
					LED_On();
					Beep_On();
					Motor_Set_PWM(0, 0);       // 第一步：先让电机完全停车
				    wait_enable = 1;

					if(stop_cnt >= 50)
					{
					Beep_Off();
		            LED_Off();	
					wait_enable = 0;
					stop_cnt = 0;
					steps = 999;
										
					}
					}
				}
				break;
				default:
				{   
				Beep_Off();
		           LED_Off();																					
				Motor_Set_PWM(0,0);								

				}
							
		}
	}
}				
}
}		
		
		



		
		
//		while(MPU6050_DMP_Get_Yaw(&Yaw0_360)!=0){ack=999;};
//		ack=1;	
//		OLED_ShowNum(1,1,12,2);
//		Motor_Set_PWM(30, 30); 	
        //陀螺仪测试
//		while(MPU6050_DMP_Get_Yaw(&Yaw0_360)!=0){ack=999;};
//		
//		OLED_ShowSignedNum(2,1,Yaw0_360,4);	}}
		//传感器测试
//		all_state=Get_Infrared_State();	
//        OLED_ShowBinNum(2,1,all_state,8);}}
//		//角度PID测试
//		while(MPU6050_DMP_Get_Yaw(&Yaw0_360)!=0){ack=999;};
//		ack=1;
// 		int yaw_int = (int)(Yaw0_360 + 0.5f);				
//		int pid_out = PID_angle(0, yaw_int); 
//		OLED_ShowSignedNum(4,1,pid_out,2);
//		
//		//速度PID测试
															    
//		find_err=Track_err();		
//		pid_out=PID_out(find_err ,0); //第一次循迹
//		Final_Speed(pid_out,25); 
//	    OLED_ShowSignedNum(3,1,find_err,1);
//		OLED_ShowSignedNum(4,1,pid_out,1);
		
//		Motor_Set_PWM(30,30);
//	}}

	
			  
			
