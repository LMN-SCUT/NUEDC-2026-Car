# MSPM0G3507 HC-05 小车遥测

这是一个独立 CCS 工程。它读取四路 AB 相编码器，每 100 ms 通过 HC-05
发送一行可读数据，同时将完全相同的数据镜像到板载 XDS110 调试串口。
程序不会驱动电机，也不会修改已经验证通过的 HC-05 PING/PONG 工程。

## 当前推荐数据链路

```text
TI 小车主控 --UART1--> HC-05 Slave ))) 电脑内置蓝牙
                                      -> Windows 虚拟 COM -> 串口助手

TI 同时通过 XDS110 COM 口输出一份相同数据，供有线对照。
```

当前实测模块地址为 `98D3:02:9687D2`，Slave、PIN `1234`、透明串口
`115200 8N1`。电脑内置蓝牙已成功配对并通过 SPP 虚拟 COM 接收遥测。
另一块 Master 模块在该方案中保持断电。USB-TTL、双 HC-05 和 STM32 串口桥均
保留为备用诊断路径。

## 接线

TI 小车主控一侧的 HC-05：

| MSPM0G3507 | HC-05 | 是否必需 |
| --- | --- | --- |
| PA8（UART1 TX） | RXD | 必需 |
| PA9（UART1 RX） | TXD | 暂时可不接，预留给以后下发命令 |
| GND | GND | 必需 |

UART 信号按 3.3 V 逻辑处理。HC-05 的 VCC 应以模块底板丝印为准，绝对不要把
5 V 接到 MCU 串口脚。官方 LaunchPad 上，只有需要使用 PA9 接收时才拔掉占用
PA9 的 J14 跳帽；本次单向遥测只接 PA8 和 GND 即可。

备用方案可将 HC-05 直接接到 USB-TTL：

| HC-05 B | USB-TTL |
| --- | --- |
| TXD | RXD |
| RXD | TXD（本次只接收时可以不接） |
| GND | GND |

串口参数统一为 `115200 8N1`，无流控。

### Windows 内置蓝牙

1. 保持另一块 Master 模块断电。
2. 在 Windows 蓝牙设置中添加 `HC-05`，PIN 输入 `1234`。
3. 在蓝牙 COM 端口中找到带 HC-05 地址的“传出”端口。
4. 串口助手以 `115200 8N1`、无流控、ASCII 模式打开该端口。

本机实测为 `COM11`，COM 号由 Windows 动态分配，换电脑后必须重新查看。

XDS110 有线镜像使用：

- PA10 = UART0 TX
- PA11 = UART0 RX
- Windows 中的 `XDS110 Class Application/User UART`
- `115200 8N1`

## 编码器引脚

| 车轮 | A 相 | B 相 |
| --- | --- | --- |
| 左前 LF | PB15 | PB16 |
| 左后 LR | PB10 | PB11 |
| 右前 RF | PA28 | PA31 |
| 右后 RR | PA29 | PA30 |

8 个输入均为内部上拉、双边沿 GPIO 中断。之前实测每个车轮约为
`1064 counts/rev`。

## CCS 编译烧录

1. 在 CCS 选择 `File > Import Projects from File System`。
2. 选择整个 `g3507_hc05_telemetry` 文件夹。
3. 编译 Debug 配置，再点击 `Flash Project`。
4. 给 Slave HC-05 上电，在 Windows 中打开它的蓝牙虚拟 COM 口。
5. 先运行 `open_serial.ps1` 检查 XDS110 有线镜像。
6. 接收端使用 USB-TTL 时，运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\open_wireless_serial.ps1 -Port COM8
```

把 `COM8` 换成设备管理器里 USB-TTL 的实际端口。不要同时用两个串口软件打开
同一个 COM 口。

## 输出格式

```text
$TEL,42,ms=4200,LF=3/1080/0,LR=2/1068/0,RF=-1/-1062/0,RR=0/1064/0*7E
```

- `42`：每发送一行加一的序号
- `ms`：上电后的毫秒数
- 每个车轮依次为 `最近100ms增量/上电累计值/非法跳变数`
- `*XX`：从 `$` 后第一个字符到 `*` 前一个字符的逐字节 XOR，十六进制大写

在 CCS Watch 中，`g_telemetry_sequence`、`g_hc05_lines_sent` 和
`g_debug_lines_sent` 应持续同步增长。首次测试应架空车轮：只转一个轮子时只允许
对应字段变化，反向旋转时 `delta` 符号应反转，`invalid` 正常应保持为 0。

## 首轮验收

1. 不接 HC-05，确认 XDS110 每 100 ms 收到一行完整数据。
2. 通过电脑内置蓝牙连接单块 HC-05，确认无线端看到与 XDS110 相同的序号和内容。
3. 分别旋转 LF、LR、RF、RR，确认轮序没有接反。
4. 关闭并重新打开蓝牙 COM，确认恢复连接后序号继续增长。
5. 连续运行 10 分钟，检查行中没有乱码、断行或序号异常跳变。
