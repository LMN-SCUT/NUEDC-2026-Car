#include "motor.h"
#include "stm32f10x.h"                  // Device header

#define LEFT_IN1_PIN  GPIO_Pin_14
#define LEFT_IN2_PIN  GPIO_Pin_15
#define LEFT_PORT     GPIOB

#define RIGHT_IN3_PIN GPIO_Pin_4
#define RIGHT_IN4_PIN GPIO_Pin_5
#define RIGHT_PORT    GPIOB

#define LEFT_PWM_PIN  GPIO_Pin_6
#define LEFT_PWM_PORT GPIOA

#define RIGHT_PWM_PIN GPIO_Pin_1
#define RIGHT_PWM_PORT GPIOB

void Motor_Init(void) {
	
	
    GPIO_InitTypeDef GPIO_InitStructure;
    
    
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 配置左电机方向引脚
    GPIO_InitStructure.GPIO_Pin = LEFT_IN1_PIN | LEFT_IN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置右电机方向引脚
    GPIO_InitStructure.GPIO_Pin = RIGHT_IN3_PIN | RIGHT_IN4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    

    // 配置PWM引脚（完全保留原有配置）
    GPIO_InitStructure.GPIO_Pin = LEFT_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = RIGHT_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // TIM3定时器配置（完全保留原有配置）
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_Period = 100-1;
    TIM_TimeBaseStructure.TIM_Prescaler = 36-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // PWM输出配置（完全保留原有配置）
	TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    
    // 使能TIM3和PWM输出
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    
    Motor_Set_PWM(0, 0);
}
//int myabs(int a)//取绝对值函数
//{
//	int temp;
//	if(a>=0)  temp=a;
//	else      temp=-a;
//	return temp;

//}

void Limit(int *motor_left,int *motor_right)
{
	if (*motor_left>50)  *motor_left=50;
	if (*motor_left<-50)  *motor_left=-50;
	
    if (*motor_right>50)  *motor_right=50;
	if (*motor_right<-50)  *motor_right=-50;

}
	

void Motor_Set_PWM(int motor_l, int motor_r) {
    // 左电机方向
    if (motor_l >= 0) 
		{
//        GPIO_ResetBits(LEFT_PORT, LEFT_IN1_PIN);
//        GPIO_SetBits(LEFT_PORT,LEFT_IN2_PIN );
		GPIO_SetBits(LEFT_PORT, LEFT_IN1_PIN);
        GPIO_ResetBits(LEFT_PORT, LEFT_IN2_PIN);
        TIM_SetCompare1(TIM3, motor_l);
	    } 
	else 
		{
//        GPIO_SetBits(LEFT_PORT, LEFT_IN1_PIN);
//        GPIO_ResetBits(LEFT_PORT, LEFT_IN2_PIN);
		GPIO_ResetBits(LEFT_PORT, LEFT_IN1_PIN);
        GPIO_SetBits(LEFT_PORT,LEFT_IN2_PIN );
        TIM_SetCompare1(TIM3, -motor_l);
        }
    
    // 右电机方向
    if (motor_r >= 0) 
		{
        GPIO_SetBits(RIGHT_PORT, RIGHT_IN3_PIN);
        GPIO_ResetBits(RIGHT_PORT, RIGHT_IN4_PIN);
		TIM_SetCompare4(TIM3, motor_r);
        } 
		else 
		{
		 GPIO_ResetBits(RIGHT_PORT, RIGHT_IN3_PIN);
         GPIO_SetBits(RIGHT_PORT, RIGHT_IN4_PIN);	
       
       TIM_SetCompare4(TIM3, -motor_r);
       }
   

}
