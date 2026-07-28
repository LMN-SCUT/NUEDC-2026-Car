# HC-05 测试记录

日期：2026-07-27  
测试主控：官方 LP-MSPM0G3507 LaunchPad

## 模块信息

| 项目 | HC-05 A（当前改为 Slave） | HC-05 B（当前改为 Master） |
| --- | --- | --- |
| 底板丝印 / 针脚顺序 | ZS-040；EN/VCC/GND/TXD/RXD/STATE | ZS-040；EN/VCC/GND/TXD/RXD/STATE |
| 丝印电气规格 | Power 3.6–6 V；LEVEL 3.3 V | Power 3.6–6 V；LEVEL 3.3 V |
| VCC 实际供电 | LaunchPad 5 V | LaunchPad 5 V |
| AT 模式现象 | EN 上电前拉高；红灯约每 4 秒闪烁 | EN 上电前拉高；红灯约每 4 秒闪烁 |
| `AT` | `OK` | `OK` |
| `AT+VERSION?` | `+VERSION:2.0-20100601` | `+VERSION:2.0-20100601` |
| `AT+ADDR?` | `+ADDR:98D3:02:9687D2` | `+ADDR:98D3:02:969F36` |
| `AT+UART?` | `+UART:38400,0,0` | `+UART:38400,0,0` |
| `AT+ROLE?` | `+ROLE:0`（当前为 Slave） | `+ROLE:0`（当前为 Slave） |
| AT 查询统计 | 5 成功，0 超时 | 5 成功，0 超时 |
| 最终配置结果 | 已复核：Master、定向绑定 B、115200 | 已复核：Slave、115200 |

接线：PA8（TX）接模块 RXD；拔掉 J14 跳线帽后，模块 TXD 接
J14 的 PA9 针；SW1 和 PB23 不接；两端共地。

## 调试记录

- 首次查询出现“命令原样返回”的假成功。
- 根因是 SysConfig 误开启 UART 内部回环，生成代码调用了
  `DL_UART_Main_enableLoopbackMode()`。
- 已在当前 AT 配置和透明传输配置模板中明确设置
  `enableInternalLoopback = false`。
- 关闭内部回环后，模块 A 的五项 AT 查询全部正常。
- 模块 B 的五项 AT 查询同样全部正常；两块模块固件版本一致。
- 模块 B 执行 `AT`、`AT+ROLE=0`、`AT+UART=115200,0,0`，
  结果为 3 成功、0 超时，三条回复均为 `OK`。
- 再次查询模块 B，确认地址未变，`+UART:115200,0,0`、
  `+ROLE:0`，结果为 5 成功、0 超时。
- 模块 A 执行 `AT`、`AT+ROLE=1`、`AT+CMODE=0`、
  `AT+BIND=98D3,02,969F36`、`AT+UART=115200,0,0`，
  结果为 5 成功、0 超时，五条回复均为 `OK`。
- 再次查询模块 A，确认 `+ROLE:1`、`+CMODE:0`、
  `+BIND:98D3:02:969F36`、`+UART:115200,0,0`，
  结果为 5 成功、0 超时。
- 两块模块退出 AT 模式后持续快闪，未自动建立连接。原版 HC-05
  手册表明，首次连接还需要 Master 执行 `AT+INIT`、
  `AT+PAIR=<B 地址>,<超时>` 和 `AT+LINK=<B 地址>`；
  已增加专用首次配对脚本。
- 首次直接执行 `PAIR/LINK` 时均返回 `ERROR:(16)`。由于
  `AT+INIT` 已成功，按手册补入 `AT+INQM=0,9,5` 和
  `AT+INQ`，先确认 Master 搜索到 B，再执行配对与连接。
- 模块 B 在 EN 悬空、仅接 5 V/GND 时可被手机正常发现，并出现
  PIN 输入提示，确认 B 的普通可发现状态和供电正常。问题收敛到
  A 的搜索侧；配对脚本进一步补入 `RMAAD`、通用 IAC、
  `CLASS=0` 和 `PSWD=1234`。
- A 接受上述参数，但 `INQ`、`PAIR`、`LINK` 仍全部返回
  `ERROR:(16)`；其中 `INQ` 已经失败，后两项失败是连带结果。
  下一轮交换角色，先把 A 配为 Slave 并验证其可发现性，再尝试
  由 B 充当 Master。
- A 已成功改为 Slave、PIN 1234、透明串口 115200；在 EN 悬空、
  仅接 5 V/GND 时同样可被手机正常发现，确认 A 的从机广播正常。
  下一轮由 B 作为 Master，绑定 A 地址 `98D3:02:9687D2`。
- B 改为 Master 后同样在 `INQ`、`PAIR`、`LINK` 返回
  `ERROR:(16)`，而前置参数均设置成功。两块互换角色后现象一致，
  排除单块射频故障，下一轮改用 `CMODE=1` 普通模式自动连接，
  绕开手动 `INQ/PAIR` 路径。
- B 已配置为 `ROLE=1`、`CMODE=1`、PIN 1234、透明串口 115200；
  A 保持 Slave、PIN 1234、透明串口 115200。两块退出 AT 模式后
  先快闪搜索，随后变为同步双闪，确认自动连接成功。
- MSPM0 连接 Master B，Slave A 的 TXD/RXD 直接短接进行无线
  硬件回环。观测到 `tx_ping=24`、`rx_ping=24`、
  `tx_pong=24`、`rx_pong=24`、`rx_valid=48`，
  `rx_bad=0`、`link_timeout=0`、`link_alive=1`；各有效计数
  持续增长，透明传输与帧协议回环验收通过。
- 双主控首次联调时 MSPM0 仅有 `tx_ping` 增长，所有接收计数为
  0。为便于无 ST-Link 诊断，STM32 固件加入 PB0/PB1 三态灯语：
  PB0 慢闪表示程序运行但没有有效帧、常亮表示链路有效；PB1
  发送 PING 时短闪，收到任意 UART 字节时延长点亮。诊断版已用
  Keil ARMCC 编译，结果为 0 错误、0 警告。
- 诊断版烧录后 PB0/PB1 始终常亮，MSPM0 仍只有 `tx_ping`
  增长。检查发现 CubeMX 生成的 `stm32f4xx_it.c` 漏掉
  `SysTick_Handler()`，CPU 在 HAL 初始化后的首个 SysTick
  中断进入启动文件的弱默认死循环，主循环从未运行。已补回
  `HAL_IncTick()` 系统时基处理函数。
- 补回 SysTick 后完成 MSPM0 ↔ HC-05 B ↔ HC-05 A ↔ STM32F407
  双主控实测。MSPM0 端观测到 `tx_ping=20`、`tx_pong=19`、
  `rx_valid=38`、`rx_ping=19`、`rx_pong=19`，
  `rx_bad=0`、`rx_overflow=0`、`link_timeout=0`、
  `link_alive=1`。`rx_valid` 与两类有效帧之和完全一致，证明
  双向透明传输、PING/PONG 应答和 CRC 帧解析全部通过。
- 完成断线与自动重连实测。恢复连接后 MSPM0 端为
  `tx_ping=397`、`tx_pong=361`、`rx_valid=725`、
  `rx_ping=361`、`rx_pong=364`、`rx_bad=0`、
  `rx_overflow=0`、`link_timeout=13`、`link_alive=1`。
  `rx_valid=rx_ping+rx_pong`，且无需重新烧录或复位即可恢复。
  超时累计值包含多次主动断线及暂停调试造成的链路超时事件。

## 透明传输验收

| 项目 | 结果 | 备注 |
| --- | --- | --- |
| TI + 双 HC-05 短距离硬件回环 | 通过 | 24 PING + 24 PONG，0 坏帧，0 超时 |
| MSPM0 + STM32 双主控短距离通信 | 通过 | 19 PING + 19 PONG，0 坏帧，0 溢出，0 超时 |
| 1 m，10 分钟无持续断链 | 待测 | |
| 5 m 测试 | 待测 | |
| 10 m 测试 | 待测 | |
| 单端断电后故障指示 | 通过 | `link_alive` 清零并累计超时 |
| 恢复供电后自动重连 | 通过 | 无需复位或重烧，计数自动恢复 |
| MSPM0 `g_rx_bad_count` | 通过 | 断线重连后仍为 0 |
| STM32 `g_rx_bad_count` | 待测 | |
| 电机干扰测试（后续） | 待测 | |

## 单模块直连电脑验收

日期：2026-07-28

最终选用地址为 `98D3:02:9687D2` 的 Slave 模块，参数为 PIN `1234`、透明串口
`115200 8N1`。另一块 Master 模块保持断电，避免抢先自动连接。

Windows 内置蓝牙成功发现并配对 `HC-05`，系统创建两个标准蓝牙串口，其中带有
设备地址 `98D3029687D2` 的传出端口为 `COM11`。串口助手以 `115200 8N1`、
无流控、ASCII 模式打开后，HC-05 建立连接并连续收到：

```text
$TEL,9941,ms=1036460,LF=0/0/0,LR=0/0/0,RF=0/0/0,RR=0/0/0*4D
```

后续序号 `9942`、`9943`、`9944`、`9945`、`9946` 连续增长，约每 100 ms
一帧，无乱码和断行。该次只在空开发板上验证串口与蓝牙链路，未接入整车编码器；
四轮旋转、长时间运行、电机干扰和距离测试留待车辆空闲后完成。

| 项目 | 结果 | 备注 |
| --- | --- | --- |
| Windows 搜索与 PIN 1234 配对 | 通过 | 设备名 `HC-05` |
| Bluetooth SPP 虚拟 COM | 通过 | 本机为 `COM11`，其他电脑可能不同 |
| TI 遥测连续接收 | 通过 | 100 ms ASCII 帧，序号连续 |
| 单 HC-05 作为整车默认调试链路 | 采用 | 双模块方案转为备用 |
