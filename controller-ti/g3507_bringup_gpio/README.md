# MSPM0G3507 最小系统板基础验证

本工程用于验证队伍现有的 **MSP-LITO-G3507 最小系统板**，并逐步扩展为 MSPM0G3507 与 K230 的视觉通信工程。

> 注意：目标板不是 TI 官方 `LP-MSPM0G3507 LaunchPad`。不得直接照搬 LaunchPad 的板载 LED、按键和跳线映射；所有引脚均以 MSP-LITO-G3507 板上丝印、原理图和实机验证为准。

## 当前已验证

- 芯片：MSPM0G3507，LQFP-64
- 调试/下载：外接 XDS110，SWD
- 板载用户灯：`D1`，连接 `PB14`
- 板载电源灯：`D2`，上电常亮，不能作为 GPIO 测试灯
- 板载按键：`S1 = NRST`，`S2 = PB21`
- `PB26` 只是排针引出，并未连接板载 LED
- PB14 闪烁程序已完成编译、下载和实机验证

## 当前 GPIO 配置

| 名称 | 引脚 | 用途 | 状态 |
| --- | --- | --- | --- |
| `GPIO_LED_USER_LED_PIN` | PB14 | 板载 D1 用户灯 | 已验证 |
| PA19 | SWDIO | 外接调试器数据 | 调试时保留 |
| PA20 | SWCLK | 外接调试器时钟 | 调试时保留 |

## K230 串口

使用以下独立串口，不占用板载 LED、按键或 SWD：

| 信号 | MSPM0G3507 引脚 | 方向 |
| --- | --- | --- |
| UART1 TX | PA8 | MSPM0G3507 → K230 |
| UART1 RX | PA9 | K230 → MSPM0G3507 |
| GND | GND | 两块板共地 |

串口参数为 `115200 / 8 data bits / no parity / 1 stop bit`。当前联调配置已改为 UART1 的 PA8/PA9，以同时绕开原 UART0 外设及 PA10/PA11 引脚组。PA10/PA11 外部短接回环测试曾通过，仅作为历史诊断结论保留。

接线时必须交叉连接 TX/RX，并确保两块板逻辑电平兼容：

```text
MSPM0 PA8 (TX)   ->  K230 IO33 (RX)
MSPM0 PA9 (RX)   <-  K230 IO32 (TX)
MSPM0 GND        --- K230 GND
```

### PA10/PA11 联调异常记录（2026-07-22）

- `UART0 / PA10(TX) / PA11(RX)` 的外部短接回环测试通过，暂不能据此判定 MCU 引脚或最小系统板损坏。
- 使用 XDS110 完整排线调试时，K230 与 PA10/PA11 跨板通信失败；换为独立的 `UART1 / PA8(TX) / PA9(RX)` 后，双向协议立即通过。
- 本板原理图将 PA10/PA11 同时连接到 J3 的 XDS110 backchannel UART：`PA10_Debug_TX`、`PA11_Debug_RX`。因此完整连接 XDS110 时，调试器回传串口与 K230 可能同时占用 UART0，尤其可能在 PA11 接收线上形成两个发送端竞争。
- 当前工程固定使用 PA8/PA9。除非断开 J3 的 UART TX/RX 并单独复测，否则不要把 PA10/PA11 用作 K230 通信口。
- 仍保留“接线错误或接触不良”的可能性；以上冲突属于根据原理图和现象作出的高概率判断，并非已通过示波器确认的最终故障结论。

## K230 协议联调

运行 K230 的 `vision-k230/examples/uart_protocol_smoke_standalone.py`。MSPM0 收到合法帧后发送 `SET_MODE(1)`；仅当收到成功 ACK，并在后续观测帧中确认 `mode=1` 后，D1 才会按 K230 心跳持续慢闪。

因此 D1 慢闪同时证明：K230→MSPM0 接收、CRC 与流式解析、MSPM0→K230 命令发送、K230 ACK 返回以及命令实际生效均正常。可在 CCS Watch 中观察：

- `g_validFrameCount`
- `g_observationCount`
- `g_heartbeatCount`
- `g_badFrameCount`
- `g_uartRxByteCount`
- `g_ackReceived`
- `g_modeConfirmed`
- `g_commandSendCount`
- `g_latestObservation`

协议实现不在 TI 工程中复制。`vision_link_shared.c` 直接构建仓库唯一的 `shared/protocol/c/vision_link.c`，保证 STM32 与 MSPM0 使用相同协议核心。

接收端已从联调时的轮询诊断模式切换为 UART RX 中断与 256 字节环形缓冲。运行时保护与 STM32 端保持一致：半帧停顿超过 20 ms 时重置解析器；有效观测超过 150 ms 未更新时令 `g_observationFresh=false`；任意合法帧超过 500 ms 未更新时令 `g_linkAlive=false`，同时清除旧 ACK、模式确认和观测状态。新增的 CCS Watch 诊断量包括：

- `g_observationFresh`
- `g_linkAlive`
- `g_uartRxOverflowCount`
- `g_parserTimeoutCount`

### 实机异常流验收（2026-07-22）

K230 测试脚本依次注入随机噪声、坏 CRC、65 字节非法长度头、截断帧，并在截断帧后停顿 30 ms。MSPM0G3507 未出现解析器卡死，随后恢复接收合法观测和心跳。测试结束时 CCS Watch 记录：

- `g_validFrameCount = 618`，并可继续增长
- `g_uartRxByteCount = 14539`，并可继续增长
- `g_parserTimeoutCount = 1`，符合 20 ms 半帧超时预期
- `g_uartRxOverflowCount = 0`
- `g_linkAlive = 1`、`g_observationFresh = 1`
- `g_ackReceived = 1`、`g_modeConfirmed = 1`

同轮测试已确认拔掉 K230 TX 后观测先失效、链路随后失联；重新接线无需复位即可恢复。K230 对重复命令只重发缓存 ACK，不重复执行命令。异常注入开关 `FAULT_INJECTION_ENABLED` 默认保持关闭，需要复测时临时改为 `True`。

## 已知经验

- 调试器上的红/绿灯属于 XDS110，不是目标 MCU 可控 LED。
- CCS 的 `Flash Project` 只负责烧录；进入 Debug 后还需要按 Continue 才会运行。
- 修改 `.syscfg` 后应重新构建，确认生成文件与实际目标板一致。
- MSP-LITO 的 PA10/PA11 兼作 XDS110 backchannel UART；完整连接调试器时优先避开，视觉通信使用 UART1 的 PA8/PA9。
- 后续若更换板型，必须先记录准确板名、原理图和板载资源映射，再配置外设。
