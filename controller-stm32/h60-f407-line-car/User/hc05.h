#ifndef HC05_H
#define HC05_H

#include <stdint.h>

/**
 * @brief 初始化USART3蓝牙串口。
 *
 * 接线：PB10(USART3_TX)→HC-05 RXD，PB11(USART3_RX)←HC-05 TXD，
 * 波特率由HC05_BAUD_RATE配置，格式8数据位、无校验、1停止位。
 */
void HC05_Init(void);

/** @brief 阻塞发送一个字节。 */
void HC05_SendByte(uint8_t data);

/** @brief 发送以'\0'结尾的字符串。 */
void HC05_SendString(const char *text);

/** @brief 以十进制发送32位有符号整数，不依赖printf。 */
void HC05_SendInt32(int32_t value);

/** @brief 以两位十六进制发送一个字节。 */
void HC05_SendHex8(uint8_t value);

/** @brief 查询是否收到一个字节，非阻塞。 */
uint8_t HC05_ByteAvailable(void);

/**
 * @brief 非阻塞读取一个字节。
 * @param data 输出接收到的数据。
 * @return 有数据并成功读取时返回1，否则返回0。
 */
uint8_t HC05_ReadByte(uint8_t *data);

#endif
