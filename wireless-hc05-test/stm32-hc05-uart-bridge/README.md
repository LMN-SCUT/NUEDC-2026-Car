# STM32F407 HC-05 串口桥

本工程将 HC-05 串口与电脑 USB-TTL 串口双向透明转发：

```text
TI 主控 -> HC-05 A ))) ((( HC-05 B -> STM32 UART4 -> USART1 -> USB-TTL -> 电脑
```

它不解析、修改或重新打包数据。TI 发出的 `$TEL,...*XX` 原文会出现在电脑串口
助手中。电脑串口助手发送的数据也会反向转发到 TI，供以后增加调试命令。

如果已有 USB-TTL，实际上可以跳过 STM32，直接将 `HC-05 B TXD` 接
`USB-TTL RXD`。使用本桥接工程的价值是保留双向缓冲、活动灯和 Watch 计数。

## 接线

HC-05 B 到 STM32F407：

| HC-05 B | STM32F407 | 本次是否必需 |
| --- | --- | --- |
| TXD | PC11（UART4 RX） | 必需 |
| RXD | PC10（UART4 TX） | 反向命令才需要 |
| GND | GND | 必需 |

STM32F407 到 USB-TTL：

| STM32F407 | USB-TTL | 本次是否必需 |
| --- | --- | --- |
| PA9（USART1 TX） | RXD | 必需 |
| PA10（USART1 RX） | TXD | 反向命令才需要 |
| GND | GND | 必需 |

两组 UART 都是 `115200 8N1`、无流控，信号按 3.3 V 逻辑处理。所有设备必须
共地，不能把 USB-TTL 的 5 V 信号接入 STM32 或 HC-05 串口脚。

## Keil 烧录

打开：

```text
Projects/MDK-ARM/hc05_uart_bridge.uvprojx
```

执行 Build，然后用 ST-Link 下载。工程已经在本机完整编译通过：`0 Error(s),
0 Warning(s)`。

为避免在仓库中重复保存 STM32 HAL/CMSIS，Keil 工程复用仓库中已有的
`controller-stm32/hal-f407-uart4/CubeMX/F407_Vision_Link/Drivers`。因此应在
完整仓库目录结构下打开工程。

## 电脑串口助手

1. 将 USB-TTL 插入电脑，在设备管理器查看 COM 号。
2. 串口助手选择该 COM 口。
3. 设置波特率 `115200`、数据位 `8`、停止位 `1`、无校验、无流控。
4. 接收显示选择“文本/ASCII”，不要选择 HEX 显示。
5. 打开串口后，应每 100 ms 看到一行 `$TEL` 数据。

串口助手不需要理解校验，也不需要自动添加换行；TI 发送端已经在每帧末尾发送
`CRLF`。

## 指示灯与 Watch

- PB0：约 500 ms 翻转一次，表示桥接固件仍在运行。
- PB1：收到或发出串口数据时短暂点亮。

Keil Watch 可观察：

- `g_hc05_to_pc_bytes`：蓝牙到电脑的已转发字节数，应持续增长。
- `g_pc_to_hc05_bytes`：电脑到蓝牙的已转发字节数。
- `g_hc05_rx_overflow`、`g_pc_rx_overflow`：环形缓冲溢出，正常应为 0。
- `g_hc05_uart_errors`、`g_pc_uart_errors`：UART 错误，正常应为 0。

首轮测试只打开电脑串口助手，不要主动发送字符。确认 `$TEL` 连续显示且错误、
溢出计数保持为 0 后，再测试反向通道。
