# HC-05 无线链路技术储备

本目录保存 HC-05 的 AT 配置、双模块链路验证以及串口桥接备用方案，不接入
K230、视觉代码、电机或整车控制。

## 当前推荐方案

整车调试优先使用一块 HC-05 直接连接电脑：

```text
TI MSPM0G3507 --UART1--> HC-05 Slave ))) 电脑内置蓝牙
                                      -> Windows 虚拟 COM -> 串口助手
```

- 从机地址：`98D3:02:9687D2`
- 角色：Slave
- PIN：`1234`
- 透明串口：`115200 8N1`
- 2026-07-28 已在 Windows 上配对成功，并通过虚拟 COM 连续收到 TI 遥测。

发送固件位于 `../controller-ti/g3507_hc05_telemetry/`。双 HC-05 与 STM32
PING/PONG 链路保留作为协议、距离和自动重连验证平台，不作为当前整车默认结构。

## 目标和结构

```text
LP-MSPM0G3507 官方 LaunchPad          STM32F407 OpenCTR H60
PA8 (TX) -> HC-05 A RXD       ~~~      HC-05 B RXD <- PC10 (TX)
PA9 (RX) <- HC-05 A TXD       ~~~      HC-05 B TXD -> PC11 (RX)
GND       --- HC-05 A GND              HC-05 B GND --- GND
```

每端每 500 ms 发一个 `PING` 帧。收到对方 `PING` 会立即返回 `PONG`。帧格式固定为：

```text
A5 5A TYPE NODE_ID SEQ_LO SEQ_HI CRC8
```

CRC-8 覆盖 `TYPE` 到 `SEQ_HI`，多余噪声、断帧和错误帧不会被计为有效通信。

## 目录说明

- `shared/`：两端共用的固定帧解析和 AT 指令状态机。
- `mspm0-hc05-node/`：MSPM0G3507 最小系统板固件和两份 SysConfig。
- `stm32-hc05-node/`：F407 的 HAL 固件、CubeMX 配置参考。
- `stm32-hc05-uart-bridge/`：HC-05 到 USB-TTL 的双向串口桥备用工程。
- `test_record.md`：实际查询结果和距离测试记录。

## 0. 先确认实物，暂时不要接线

对两块模块分别拍照或记下底板丝印。至少确认这些针脚：`VCC`、`GND`、`TXD`、`RXD`、`KEY` 或 `EN`、`STATE`。`KEY/EN` 和 `STATE` 不是所有底板都会引出。

安全规则：

1. 当前两块 HC-05 背面明确标注 `Power: 3.6V--6V`，VCC 可接 LaunchPad 的 5 V；串口电平仍按背面标注使用 3.3 V。
2. 不论 VCC 用 3.3 V 还是 5 V，MCU 的 `TX/RX` 都只接 3.3 V 逻辑。禁止把 5 V 接到 PA8、PA9、PC10、PC11。
3. 每块 HC-05 必须与其本地 MCU 共地。两个 MCU 之间不需要额外拉一根地线，因为它们之间的数据走蓝牙。
4. 首轮仅 USB/下载器供电，不接电机电池、降压模块或整车供电。

## 1. 单模块上电检查

先只给模块供电，不接 TX/RX。

- 快速闪烁通常表示未配对、等待连接。
- 慢闪或常亮的具体含义取决于底板固件，先记录，不据此判断好坏。
- 发热、无 LED 且供电电压正确，立即断电，不继续接主控。

将结果写进 `test_record.md`。

## 2. AT 查询：先读参数，再改参数

对 A、B 轮流操作；一次只接一块模块。完整 AT 模式的常规过程是：**断电 -> KEY/EN 拉高到 3.3 V -> 再上电**。不能确认 KEY/EN 接法时，不要猜测，先停下并拍模块正反面。

### 使用 MSPM0 查询（推荐）

1. 使用官方 `LP-MSPM0G3507`。首轮测试直接拔下 J14 跳线帽，将模块 TXD 用母头杜邦线直接接到 J14 裸露的 `PA9` 针；不要接 J14 的 `SW1` 或 `PB23`。模块 RXD 接 BoosterPack 排针的 PA8。
2. 在 CCS 选择 `File -> Import Project(s)`，导入 `mspm0-hc05-node`。工程名应显示为 `hc05_mspm0_at_query`。
3. 当前活动文件 `hc05.syscfg` 已固定为 `38400 / 8N1 / PA8 TX / PA9 RX`，并明确关闭 UART 内部回环；`HC05_TEST_MODE_AT` 也已设为 `1u`，不需要再次手工配置。内部回环若误开，MSPM0 会把自己发送的命令当作接收数据，形成假回复。
4. `profiles/` 中保存了 AT 和透明传输两份 SysConfig 文本模板；首轮不要替换当前 `hc05.syscfg`。
5. 接线：模块 RXD 接 BoosterPack 排针上标出的 `PA8`；模块 TXD 直接接拔掉跳线帽后 J14 上标出的 `PA9` 针；`GND --- GND`。J14 的 `SW1` 和 `PB23` 均悬空。
6. 编译、烧录并运行。CCS Watch 添加：
   - `g_at_session.step`
   - `g_at_session.success_count`
   - `g_at_session.timeout_count`
   - `g_at_session.complete`
   - `g_at_session.current_command`
   - `g_at_session.last_response`
   - `g_at_session.responses[0]`：`AT`
   - `g_at_session.responses[1]`：`AT+VERSION?`
   - `g_at_session.responses[2]`：`AT+ADDR?`
   - `g_at_session.responses[3]`：`AT+UART?`
   - `g_at_session.responses[4]`：`AT+ROLE?`
7. 预期：5 条查询均有回复，`success_count = 5`、`timeout_count = 0`。将 `VERSION`、`ADDR`、`UART`、`ROLE` 抄到记录表。

模块没有回复时，依序检查：KEY 是否在上电前已拉高、SysConfig 是否是 38400、TX/RX 是否交叉、GND 是否共地、模块 VCC 是否正确。不要直接把波特率挨个乱试。

### 配置配对角色

确认两块均能回复标准 `AT+...` 指令后：

1. 选定 A 为 Master、B 为 Slave。
2. 对 B：把 `HC05_AT_SCRIPT` 改为 `HC05_AT_CONFIG_SLAVE`，运行一次，确认每步回 `OK`。
3. 对 A：先把 B 的 `ADDR` 中冒号替换为逗号，例如 `+ADDR:1234:56:ABCDEF` 写为 `1234,56,ABCDEF`；填入 `HC05_PEER_ADDRESS`。把脚本改为 `HC05_AT_CONFIG_MASTER`，运行一次。
4. 两边成功后都断电；移除 KEY/EN 的高电平；重新上电。配置脚本已把透明传输 UART 统一为 `115200 / 8N1`。

若 `AT+BIND=`、`AT+CMODE=` 的返回不是 `OK`，不要继续透明传输。不同克隆固件可能使用不同 AT 语法，应保留 `VERSION` 和原始回复后再处理。

## 3. 透明传输固件

### MSPM0 A 端

1. 完成两块模块配置后，用 `profiles/hc05_transparent.syscfg.txt` 的内容替换活动文件 `hc05.syscfg`。
2. 将 `HC05_TEST_MODE_AT` 改回 `0u`，重新编译、烧录。
3. 保持 J14 跳线帽拔下。接线：排针 `PA8 (TX) -> HC-05 A RXD`，J14 的 `PA9 (RX) <- HC-05 A TXD`，GND 共地；J14 的 `SW1` 和 `PB23` 不接。
4. 本板 D1：无有效无线帧时慢闪；收到有效无线帧后稳定亮。CCS Watch 观察 `g_tx_ping_count`、`g_rx_valid_count`、`g_rx_pong_count`、`g_rx_bad_count`、`g_link_timeout_count`。

### STM32 B 端

1. 在 STM32CubeMX 打开 `hc05_f407.ioc` 并生成 MDK-ARM 工程。确认 `UART4 = 115200/8N1`、`PC10=TX`、`PC11=RX`、UART4 中断已开启。
2. 将本目录 `Core/Inc` 与 `Core/Src` 中的 HC-05 文件保留到生成工程；不要用现有视觉工程替换它们。生成的 `gpio.c`、`usart.c`、`stm32f4xx_it.c` 保持 CubeMX 版本。
3. 确认工程编译了 `hc05_node.c`、`hc05_protocol_shared.c`、`hc05_at_shared.c` 和本目录的 `main.c`。
4. 接线：`PC10 (TX) -> HC-05 B RXD`，`PC11 (RX) <- HC-05 B TXD`，GND 共地。
5. 烧录运行。PB0/PB1 均为低电平点亮，无 ST-Link 时使用以下灯语诊断：
   - 复位后两灯同时短亮，表示新固件已经启动。
   - PB0 在尚未收到有效帧时以 1 秒周期慢闪，收到有效帧并保持链路后常亮。
   - PB1 每发送一个 PING 短闪约 60 ms；只要 UART 收到任意字节就会亮约 250 ms。
   - 因此“PB0 慢闪、PB1 规律短闪”表示程序在发送但完全收不到；“PB0 慢闪、PB1 长亮或频繁亮”表示收到字节但协议校验失败；“PB0 常亮、PB1 频繁活动”表示双向协议正常。
   Keil Watch 可额外观察与 MSPM0 同名的 `g_tx_*`、`g_rx_*`、`g_link_alive`，以及原始串口接收计数 `g_rx_byte_count`。

## 4. 验收顺序

1. 两端上电后等待 5-15 秒，确认两个模块的 LED 从未连接状态变为已连接状态。
2. 两块 MCU 的 `g_tx_ping_count`、`g_rx_valid_count`、`g_rx_pong_count` 应持续增长；`g_rx_bad_count` 和 `g_rx_overflow_count` 应保持 0。
3. 保持 1 m 距离运行 10 分钟。其间任一端不应出现 `g_link_timeout_count` 增长。
4. 重复 5 m、10 m；每项至少运行 2 分钟。
5. 拔掉任一模块的 VCC：另一端应在约 1.5 秒后熄灭链路灯并使 `g_link_timeout_count` 加一。恢复供电后，模块重新配对且有效帧恢复增长，不需要重新烧录。

首次验收通过前，不测试电机干扰。之后再以相同 1 m 测试为基线，逐步接入电机供电和电机运行，记录新增丢帧或重连情况。

## 常见故障

| 现象 | 优先检查 |
| --- | --- |
| AT 全部超时 | KEY/EN 上电时序、38400 SysConfig、TX/RX 是否交叉、共地 |
| 只有一端计数增长 | 模块尚未配对，或另一端透明 UART 仍是 38400 |
| 有效帧和坏帧都快速增长 | 串口波特率不一致或 TX/RX 误接 |
| 烧录后仍在 AT 模式 | `HC05_TEST_MODE_AT` 未改回 `0u` |
| 已配对但一加电机就掉线 | 模块供电压降、地线回流或电机噪声；先恢复到独立 USB 供电复测 |
