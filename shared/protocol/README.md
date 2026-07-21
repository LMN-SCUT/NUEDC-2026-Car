# K230—主控统一通信协议 v1（本地评审稿）

本文定义 K230 视觉模块与候选主控 STM32、MSPM0G3507 共用的 UART 协议。协议与具体赛题、视觉算法、底盘和舵机解耦；换主控时只替换串口驱动层，不修改帧格式和业务字段。

当前状态：本地评审稿，尚未推送 GitHub。Python 与纯 C 参考实现已通过自动测试，C 核心与 STM32 HAL 适配层已通过 Keil ARMCC 编译检查。2026-07-22 已完成 K230 ↔ STM32F407 双向实机验证：K230 UART3（GPIO32/33）以 115200 8N1 发送观测帧和心跳，F407 UART4（PC10/PC11）能够持续接收并通过 CRC 解析；F407 周期发送 `SET_MODE`，K230 能正确解析、切换模式并返回 ACK。拔掉串口线后主控能够停止使用旧数据，重新接线后无需复位即可自动恢复通信。STM32F407 链路可作为当前稳定基线；MSPM0G3507 端仍待实现和实机验证。

## 1. 设计结论

- 保留旧协议中有效的二进制帧、双字节帧头、长度、序号、小端序和流式解析思路。
- 增加协议版本字段，避免以后扩展时两端默默使用不同格式。
- 将旧版累加和升级为 CRC-16/CCITT-FALSE，提高发现多字节错误的能力。
- 删除 F407、云台、PWM、PID 等设备专用语义；K230 只输出视觉观测，控制决策留在主控。
- 不采用厂商例程的 `$长度,功能号,x,y,w,h#` 文本协议。文本协议便于观察，但开销大、无可靠校验、字段扩展和异常恢复较弱。
- 禁止直接把 C 结构体内存发上串口。各端必须逐字段按小端序序列化和反序列化，避免结构体对齐、类型宽度和编译器差异。

## 2. 物理层

| 项目 | 约定 |
|---|---|
| 接口 | 3.3 V TTL UART，K230 与主控必须共地 |
| 接线 | K230 TX → 主控 RX；K230 RX ← 主控 TX |
| 串口参数 | 115200 baud，8 数据位，无校验，1 停止位，无流控 |
| 最大负载 | 64 字节 |
| 多字节顺序 | 小端序；有符号整数使用二进制补码 |

具体 UART 编号、GPIO 和复用配置属于板级配置，不写死在本协议中。后续分别记录 K230、STM32 和 MSPM0G3507 的实际引脚。

## 3. 通用帧格式

| 偏移 | 字段 | 长度 | 说明 |
|---:|---|---:|---|
| 0 | SOF1 | 1 | 固定 `0xAA` |
| 1 | SOF2 | 1 | 固定 `0x55` |
| 2 | Version | 1 | 当前固定 `0x01` |
| 3 | Type | 1 | 消息类型 |
| 4 | Seq | 1 | 发送端独立计数，0～255 循环 |
| 5 | Length | 1 | Payload 字节数，0～64 |
| 6 | Payload | Length | 消息负载 |
| 6+Length | CRC16_L | 1 | CRC 低字节 |
| 7+Length | CRC16_H | 1 | CRC 高字节 |

总帧长为 `8 + Length`。CRC 覆盖 `Version、Type、Seq、Length、Payload`，不包含两个帧头字节。

CRC 参数固定为：

- 名称：CRC-16/CCITT-FALSE
- Poly：`0x1021`
- Init：`0xFFFF`
- RefIn / RefOut：false
- XorOut：`0x0000`
- 输出顺序：低字节在前，高字节在后

## 4. 消息类型

| Type | 方向 | 名称 | 是否需要 ACK |
|---:|---|---|---|
| `0x01` | K230 → 主控 | VISION_OBSERVATION | 否 |
| `0x02` | K230 → 主控 | HEARTBEAT | 否 |
| `0x10` | 主控 → K230 | COMMAND | 是 |
| `0x90` | K230 → 主控 | ACK | 不适用 |

实时观测和心跳不确认，避免 ACK 堵塞链路；只有低频命令需要 ACK。

### 4.1 VISION_OBSERVATION，Type `0x01`

Payload 固定 16 字节：

| 偏移 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 0 | timestamp_ms | uint32 | ms | K230 启动后的毫秒计数，允许回绕 |
| 4 | center_x | int16 | px | 目标中心横坐标，原点位于画面左上 |
| 6 | center_y | int16 | px | 目标中心纵坐标 |
| 8 | error_x | int16 | px | `center_x - reference_x`，向右为正 |
| 10 | error_y | int16 | px | `center_y - reference_y`，向下为正 |
| 12 | confidence | uint8 | 0～100 | 视觉置信度 |
| 13 | flags | uint8 | 位标志 | 见下表 |
| 14 | target_id | uint8 | — | 当前目标编号；无目标时为 `0xFF` |
| 15 | mode | uint8 | — | 当前算法模式；`0` 为默认模式 |

`flags`：

| 位 | 名称 | 含义 |
|---:|---|---|
| 0 | TARGET_VALID | 1 表示本帧目标有效 |
| 1 | RESULT_STABLE | 1 表示目标已连续稳定若干帧 |
| 2 | VALUE_SATURATED | 1 表示坐标或误差发生限幅 |
| 3 | PROCESSING_DEGRADED | 1 表示算法降级运行 |
| 4～7 | RESERVED | 发送端必须置 0 |

无有效目标时必须同时满足：`TARGET_VALID=0`、`confidence=0`、`center_x=center_y=error_x=error_y=0`、`target_id=0xFF`。主控不得继续使用上一帧误差。

### 4.2 HEARTBEAT，Type `0x02`

Payload 固定 8 字节：

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | timestamp_ms | uint32 | K230 启动后的毫秒计数 |
| 4 | status_bits | uint16 | K230 状态位 |
| 6 | fps_x10 | uint16 | 视觉帧率乘 10，例如 298 表示 29.8 FPS |

`status_bits`：bit0=摄像头就绪，bit1=算法就绪，bit2=模型就绪，bit3=内存告警，bit4=算法降级；其余位保留并置 0。

### 4.3 COMMAND，Type `0x10`

Payload 固定 8 字节：

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | command_id | uint8 | 命令编号 |
| 1 | mode | uint8 | 目标模式或子命令 |
| 2 | arg0 | int16 | 参数 0 |
| 4 | arg1 | int16 | 参数 1 |
| 6 | arg2 | int16 | 参数 2 |

首版命令：

| command_id | 名称 | 参数 |
|---:|---|---|
| `0x01` | SET_MODE | `mode` 为算法模式，其他参数暂为 0 |
| `0x02` | START_STREAM | 所有参数为 0 |
| `0x03` | STOP_STREAM | 所有参数为 0；心跳仍需发送 |
| `0x04` | SET_REFERENCE | `arg0=reference_x`，`arg1=reference_y`，`arg2=0` |

未知命令不得执行，应返回 `UNSUPPORTED`。重复收到相同 `Type + Seq` 的命令时，不得重复产生副作用，应返回上一次 ACK。

### 4.4 ACK，Type `0x90`

Payload 固定 4 字节：

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | request_type | uint8 | 被确认消息的 Type，首版为 `0x10` |
| 1 | request_seq | uint8 | 被确认命令的 Seq |
| 2 | status | uint8 | 执行结果 |
| 3 | detail | uint8 | 可选细分错误码，无细分时为 0 |

`status`：0=OK，1=UNSUPPORTED，2=BAD_PAYLOAD，3=BAD_STATE，4=BUSY，5=INTERNAL_ERROR。

## 5. 发送频率与失效保护

- VISION_OBSERVATION：推荐 30 Hz，允许 20～50 Hz。
- HEARTBEAT：2 Hz；停止观测流时仍持续发送。
- COMMAND：低频按需发送；100 ms 未收到 ACK 可重发，最多重试 2 次。
- 主控 150 ms 未收到新的有效观测，立即把视觉量判为无效并停止使用旧误差。
- 主控 500 ms 未收到 K230 的任何合法帧，判定通信失联并进入题目对应的安全状态。
- CRC 错误、长度越界、版本不支持和字段越界的帧一律丢弃，不更新控制量。
- Seq 只用于统计丢帧和关联 ACK；实时观测不重传。

## 6. 接收状态机

接收端按字节处理：

1. 等待 `0xAA`。
2. 等待 `0x55`；若再次收到 `0xAA`，继续视为可能的新帧头。
3. 读取 Version、Type、Seq、Length。
4. 若 Version 不支持或 Length>64，立即丢弃并重新寻找帧头。
5. 按 Length 接收 Payload，再读取两个 CRC 字节。
6. CRC 正确后才把完整帧交给业务层。
7. 一帧接收过程中若连续 20 ms 没有新字节，丢弃半帧并复位状态机。

STM32/MSPM0G3507 侧建议使用 UART 中断或 DMA 加环形缓冲区；协议解析不得阻塞控制循环。

## 7. 标准测试向量

观测数据：Version=1、Type=`0x01`、Seq=5、timestamp=1000、center=(320,240)、error=(10,-5)、confidence=90、flags=`TARGET_VALID`、target_id=0、mode=0。

完整 24 字节帧：

```text
AA 55 01 01 05 10 E8 03 00 00 40 01 F0 00
0A 00 FB FF 5A 01 00 00 BF BE
```

CRC 计算结果为 `0xBEBF`，在线路中低字节 `BF` 先发送。三端实现必须能生成并解析完全相同的字节序列。

## 8. 联调验收

1. K230、STM32、MSPM0G3507 分别通过标准测试向量。
2. 连续通信 10 分钟，无解析器卡死；统计接收帧、CRC 错误和 Seq 跳变。
3. 人工插入错 CRC、错长度、截断帧和随机噪声，后续合法帧仍可恢复解析。
4. 拔掉 K230 TX 后，主控在 150 ms 内停用旧视觉误差，500 ms 内报告通信失联。
5. 恢复接线后无需复位，两秒内自动恢复正常数据。
6. 命令重复发送不会被重复执行，ACK 能正确关联原命令。

## 9. 参考实现

- K230/PC Python：`python/vision_link.py`
- STM32/TI 共用纯 C 核心：`c/vision_link.h`、`c/vision_link.c`
- STM32F407 HAL UART4 适配层：`../../controller-stm32/hal-f407-uart4/App/`
- 自动测试：`../../tests/protocol-v1/`

Python 与 C 实现已经对标准测试向量、字段解码和坏 CRC 拒收进行交叉核验。协议核心不依赖 HAL，后续 TI 工程直接复用 `c/`，只重写串口收发适配层。

## 10. 历史协议关系

`legacy/K230_F407_通信协议_v1.md` 仅用于复现上一届链路。旧协议中的 F407 反馈、云台 PWM、PID 和零点校准字段不进入本协议。旧 Python/F407 实现也曾依赖紧凑结构体和简单累加校验，本届实现不得直接照搬。

正式发布前仍需补齐：MSPM0G3507 的最终 UART 引脚表、适配层和实机联调结果。

### K230 启动脚本注意事项

- CanMV K230 上电后会自动执行存储介质根目录中的 `main.py`。
- 旧比赛遗留的 `main.py` 可能抢占摄像头、UART 或其他外设，使 IDE 中当前脚本出现黑屏、初始化失败或表现为运行了旧逻辑。
- 联调前应检查 SD 卡和板载存储中的自动启动脚本；不需要自启动时，将旧 `main.py` 改名或移出根目录。不要仅凭 IDE 当前打开的文件判断板上实际运行内容。
- 本轮实测删除旧 `main.py` 后，单文件通信测试脚本运行正常；拔线后链路停止动作，重新接线后能够自动恢复。
