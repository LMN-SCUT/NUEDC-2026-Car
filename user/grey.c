#include "grey.h"
#include "stm32f10x.h"                  // Device header


 uint8_t TRACK1;
 uint8_t TRACK2;
 uint8_t TRACK3;
 uint8_t TRACK4;
 uint8_t TRACK5;
 uint8_t TRACK6;
 uint8_t TRACK7;
 uint8_t TRACK8;

void Track_Init(void)
{
	//A口的引脚配置
	GPIO_InitTypeDef GPIO_InitStructure;//GPIO结构体定义
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,ENABLE);//打开端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;//配置传感器读取引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//配置为输入上拉模式
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化端口
	
	//B口的引脚配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB ,ENABLE);      //打开端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  //配置传感器读取引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//配置为输入上拉模式
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化端口
}

uint8_t Get_Infrared_State(void)
{
	TRACK1= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0 )<<7 ;
	TRACK2= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1 )<<6 ;
	TRACK3= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2 )<<5 ;
	TRACK4= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3 )<<4 ;
	TRACK5= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4 )<<3 ;
	TRACK6= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5 )<<2 ;
	TRACK7= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7 )<<1 ;
	TRACK8= GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0 )<<0 ;	

	//0扫到黑线，1没扫到
	uint8_t state = 0;
	
	state=(u8)(TRACK1|TRACK2|TRACK3|TRACK4|TRACK5|TRACK6|TRACK7|TRACK8);//拼接成八位数据，最高位为为传感器的的左1，最低为为传感器的右1
	state = ~state;
	return state;
	
	
}





