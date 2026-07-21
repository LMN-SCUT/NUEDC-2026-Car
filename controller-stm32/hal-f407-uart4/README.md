# STM32F407 HAL 接入说明

旧 `DCCC` 工程虽然已经使用 HAL、UART4 和接收中断，但混有整车、云台和大量无关外设，不作为新的最小通信工程直接迁入。

## 已核对的开发板信息

根据 OpenCTR H60 V3.6 原理图与开发手册确认：

- MCU：STM32F407VET6，LQFP100。
- 外部高速晶振 HSE：8 MHz。
- SWD：PA13=SWDIO，PA14=SWCLK。
- UART4_TX：PC10，对应 EXP/J5 第 5 脚。
- UART4_RX：PC11，对应 EXP/J5 第 7 脚。
- EXP/J5 可就近使用 GND；UART 信号是 3.3 V 逻辑，禁止把 5 V 接到 TX/RX。
- K230 与 STM32 必须共地。K230 供电方式和允许电流需按 K230 板卡资料另行确认。

## 建议创建的最小工程

使用 STM32CubeMX 新建工程：

1. MCU 选择 `STM32F407VETx`，封装 LQFP100。
2. Debug 选择 Serial Wire。
3. RCC 的 HSE 选择 Crystal/Ceramic Resonator，输入频率 8 MHz；系统时钟配置为 168 MHz。
4. UART4 选择 Asynchronous：PC10=TX、PC11=RX。
5. 设置 115200、8 data bits、None parity、1 stop bit、TX/RX、No flow control。
6. 在 NVIC 中启用 UART4 global interrupt。
7. 工具链选择 MDK-ARM V5，生成工程。

生成后加入：

- `shared/protocol/c/vision_link.h`
- `shared/protocol/c/vision_link.c`
- `controller-stm32/hal-f407-uart4/App/vision_link_hal.h`
- `controller-stm32/hal-f407-uart4/App/vision_link_hal.c`

并把两个目录加入 Keil Include Paths。

## main.c 接入

```c
#include "vision_link_hal.h"

static vl_hal_link_t g_vision_link;

/* MX_UART4_Init() 之后 */
vl_hal_init(&g_vision_link, &huart4);
if (vl_hal_start_rx(&g_vision_link) != HAL_OK) {
    Error_Handler();
}

/* while (1) 中 */
vl_hal_process(&g_vision_link, HAL_GetTick());

vl_observation_t obs;
if (vl_hal_take_observation(&g_vision_link, &obs)) {
    if ((obs.flags & VL_FLAG_TARGET_VALID) != 0u) {
        /* 使用 obs.error_x / obs.error_y；不要在这里做阻塞串口输出。 */
    } else {
        /* 立即停用旧视觉误差。 */
    }
}
```

在 CubeMX 保留区内加入回调：

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    vl_hal_uart_rx_cplt_isr(&g_vision_link, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    vl_hal_uart_error_isr(&g_vision_link, huart);
}
```

## 设计说明

- 中断只收一个字节并写入环形缓冲区，不在中断中解析业务或控制电机。
- 主循环调用 `vl_hal_process()` 完成 CRC 和帧解析。
- 使用 `vl_hal_observation_is_fresh(..., 150)` 判断观测是否仍可用于控制。
- 环形缓冲区溢出、CRC 错误和格式错误均有独立计数，便于联调定位。

CubeMX 工程保存到 `CubeMX/F407_Vision_Link.ioc`。旧工程只用于核对板级配置，不应整包复制。

## 首轮实机联调指示灯

OpenCTR H60 板载两颗用户 LED 分别连接 PB0、PB1，均为低电平点亮。当前最小工程加入以下可见诊断：

- 每次复位后两灯同时短亮约 150 ms，表示新固件已正常启动。
- PB0 在最近 150 ms 内收到有效 `VISION_OBSERVATION` 时常亮；观测超时或目标无效时熄灭。
- 双向测试固件中，F407 每 2 秒发送一次 `SET_MODE` 命令，模式在 0/1 间切换；PB1 每收到一个与待处理命令序号匹配且状态为 OK 的 ACK，短亮约 150 ms。100 ms 未收到 ACK 时使用相同序号重试，最多重试 2 次。

首轮接线：K230 GPIO32/UART3_TXD → F407 PC11/UART4_RX，K230 GPIO33/UART3_RXD ← F407 PC10/UART4_TX，双方 GND 必须连接。两块板分别正常供电，只连接 3.3 V TTL 信号与地，禁止把 5 V 接入 TX/RX。
