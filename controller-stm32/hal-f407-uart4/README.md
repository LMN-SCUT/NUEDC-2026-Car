# STM32F407 HAL 接入说明

旧 `DCCC` 工程虽然已经使用 HAL、UART4 和接收中断，但混有整车、云台和大量无关外设，不作为新的最小通信工程直接迁入。

## 建议创建的最小工程

使用 STM32CubeMX 新建工程：

1. MCU 选择 `STM32F407VETx`，封装 LQFP100。
2. Debug 选择 Serial Wire。
3. 按实际开发板配置时钟；旧板工程使用外部 HSE 和 168 MHz 系统时钟。
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

当前电脑未找到 STM32CubeMX，因此这里只准备了可复用协议与 HAL 接入层，尚未生成新的 `.ioc` 和完整 Keil 工程。旧工程可用于核对 PC10/PC11 和 UART4 配置，但不应整包复制。
