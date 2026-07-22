#include "Buzzer.h"
#include "stm32f10x.h"                  // Device header



// 初始化函数
void Beep_LED_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化时也确保关闭
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
}

void Beep_On(void) 
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_8); 
}
void Beep_Off(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
}
void LED_On(void) 
{ 
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
}
void LED_Off(void)
{ 
	GPIO_ResetBits(GPIOB, GPIO_Pin_9); 
}
