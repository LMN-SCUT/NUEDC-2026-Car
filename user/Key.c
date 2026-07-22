#include "key.h"

// 按键1：PB10，按键2：PB11，上拉输入
void Key_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint8_t Key1_Scan(void) {
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0) {
        for (volatile uint32_t i = 0; i < 10000; i++);
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0) {
            while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0);
            return 1;
        }
    }
    return 0;
}

uint8_t Key2_Scan(void) {
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0) {
        for (volatile uint32_t i = 0; i < 10000; i++);
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0) {
            while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0);
            return 1;
        }
    }
    return 0;
}
