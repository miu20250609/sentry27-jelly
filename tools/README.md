# 裁判系统模拟工具

用于当前仅接两块 DM Tool 的底盘通信联调。脚本只打开每次显式指定的 UART 端口，不打开用于 CAN 接收的另一端口，不向 NUC/NX 发送控制指令，也不向裁判系统发送复活、兑换等自主决策指令。

## 接线与烧录前提

- 使用 `jelly-sentry/MDK-ARM/standard_robot_c.uvprojx`，保持 `CHASSIS` 启用、`CHASSIS_COMM_ONLY=1`，编译烧录底盘。
- 连接 UART 的 DM Tool：TX 接底盘 USART1 RX（PB7），GND 共地，115200、8N1，C 板独立供电；USART1 是模拟裁判输入，不是 VOFA 输出。
- 连接 CAN 的 DM Tool：CAN_H/CAN_L 接底盘 CAN2 对应端口。达妙 USB 转 CAN 通用上位机选择本次确认连接 CAN 的 DM Tool 对应端口，CAN 总线应匹配 C 板的 1 Mbit/s，使用正常模式接收，关注标准数据帧 0x501。电脑到 CAN 适配器的串口波特率按对应软件要求设置（本次使用的达妙助手为 921600），这不是 CAN 总线速率，不与模拟裁判 UART 的 115200 混淆。
- 先启动 CAN 接收，再运行 UART 脚本；关闭占用本次 UART 端口的 SSCOM/VOFA。两块 DM Tool 分别连接，各软件不抢占同一个 COM 口。
- CAN 总线两端按实际接线配置终端电阻。通信联调模式不运行电机、IMU、遥控、超电、0x601 周期发送和电机 VOFA；只接两块 DM Tool 即可进行本次验证。

## 运行

在 PowerShell 中进入工程：

```powershell
cd F:\RoboMaster27\sentry27\jelly-sentry
python tools/referee_simulator.py --self-test
python tools/referee_simulator.py --dry-run
```

以上两个命令不打开串口。若发送时提示缺少依赖：

```powershell
python -m pip install pyserial
```

### 每次运行前重新确认端口

DM Tool 重新插拔、换 USB 接口或换电脑后，COM 编号可能变化，不能沿用上一次的编号。设备名称也不能区分哪块接 UART、哪块接 CAN；有两块同型号时，可逐个插拔识别，结合实际线缆标记 UART/CAN，再重新枚举。

```powershell
python tools/referee_simulator.py --list-ports
$uartPort = (Read-Host "输入本次已确认连接底盘 USART1 的串口名").Trim()
```

在达妙上位机另行选择本次确认的 CAN 适配器端口并开始接收。`$uartPort` 仅供本轮命令复用；每次真实发送前都应重新核对，尤其在重新插拔之后。脚本每次发送前会重新枚举，拒绝打开已不存在的端口，但不能自动识别接线角色；必须由操作者确认。

正常模式通过 `--port` 显式选择端口，以 115200、8N1 发送 30 秒，Ctrl+C 可提前停止：

```powershell
python tools/referee_simulator.py --port $uartPort
```

只发一组（10 帧，便于查看 CAN 分包），或持续发送并记录每条原始帧：

```powershell
python tools/referee_simulator.py --port $uartPort --once --verbose
python tools/referee_simulator.py --port $uartPort --duration 0 --log "$env:TEMP\referee_tx.jsonl"
```

日志会覆盖同名文件；日志是主机生成并提交串口的数据，不是电控接收确认。机器人默认模拟红方哨兵 ID 7，模拟蓝方用 `--robot-id 107`。脚本没有默认发送端口，也没有固定编号的 CAN 端口禁用规则；缺少 `--port` 时实际发送会报错，`--list-ports`、`--dry-run`、`--self-test` 不要求发送端口。

## 模拟的数据

按 2027 V1.0.0 的字段顺序、小端和长度构造报文，使用官方 CRC8/CRC16 算法。测试值不代表官方性能上限、真实比赛结果或完整裁判业务逻辑；UNIX/NTP 时间字段置 0，不伪造同步。

- 0x0001：RMUL 3V3、比赛中，阶段剩余时间递减，1 Hz。
- 0x0003：24 字节血量信息，包含有符号伤害差和控制度，3 Hz。
- 0x0101：8 字节场地事件，示例为补给区及 R 高地相关状态，1 Hz。
- 0x0201：17 字节性能数据，含弹速上限与三路电源输出标志，10 Hz。
- 0x0202：14 字节缓冲/热量，17 mm 热量随时间变化，10 Hz。
- 0x0204：7 字节增益数据，3 Hz。
- 0x0206：7 字节命中事件，带序号与有符号坐标；模拟每秒一次命中，不代表官方为固定 1 Hz 周期消息。
- 0x0208：8 字节弹量与金币，10 Hz。
- 0x0209：6 字节 RFID，三个 RMUL 控制区位轮换，3 Hz。脚本可以模拟赛内状态，不能据此证明真实 RFID 硬件已工作。
- 0x020D：14 字节哨兵状态，普通姿态每三秒在 1/2/3 之间切换，含时长字段，1 Hz。只是下发状态，不执行切换动作。

默认总计约 43 帧/秒。调度落后时跳过错过的发送周期，不堆积追发；拆帧模式的额外等待可能降低实际发送频率。具体测试值集中在 `payloads()`，新增模拟命令还需维护 `MESSAGES` 中的长度和频率。

## 异常与拆帧测试

```powershell
# 帧头与后续内容分两次写入，中间等待 5 ms
python tools/referee_simulator.py --port $uartPort --mode split --once --verbose

# 改在第 1 字节后拆开；可选择 1～14
python tools/referee_simulator.py --port $uartPort --mode split --split-at 1 --once

# 同时到期的完整帧合并为一次串口写入
python tools/referee_simulator.py --port $uartPort --mode burst --once --verbose

# 每条模拟帧故意损坏 CRC16；预期不更新本地状态，也没有对应 0x501
python tools/referee_simulator.py --port $uartPort --mode bad-crc --once --verbose

# 载荷多一个字节但重新计算正确 CRC；预期不更新该命令本地状态，仍原样回传 0x501
python tools/referee_simulator.py --port $uartPort --mode bad-length --once --verbose
```

拆分与合并控制的是电脑写入方式，USB 缓冲和实际 UART 空闲中断边界仍以 C 板观测为准。每种异常测试后重新运行正常模式，确认恢复；不要在两次模式切换之间混淆发送日志和对应回包。

## 如何判断通过

1. `--dry-run --once` 默认从帧内 seq 0 开始，列出 10 条完整 UART 原始帧。
2. 运行真实 `--once --verbose` 后，连接 CAN 适配器的上位机应出现 0x501 分包，非末包 DLC 8，末包按实际长度发送，内容包括整帧编号、片序和最多 6 字节原始数据。
3. 完整收齐一组应有 10 条裁判原始帧，对应 38 条 0x501 CAN 分包；首次上电且没有其他裁判输入时线上整帧编号从 1 开始，脚本重启不会使 C 板编号归零。
4. 参考 [NUC 通信协议](../nuc通信协议.md) 重组、核对原始字节及 CRC。达妙上位机仅显示原始 CAN 帧，不自动理解本工程分包；本脚本没有实现 CAN 接收或重组，不能仅凭发送计数宣布通过。
5. 用 Keil 调试器检查 `Get_Referee_Instance()` 返回的实例字段和 `Get_NUC_Ctrl_Instance()->referee_drop_count`，区分本地解析失败与回传丢帧。通信联调模式不输出电机 VOFA 数据。
6. 完成测试后关闭脚本，保留原始 UART 日志和 CAN 接收记录，记录固件版本及测试模式；再更新 [开发待办](../开发待办.md) 中相应项目。

脚本只模拟本工程当前关注的常规链路数据，不模拟官方定位模块、图传链路或现场无线网络延迟；实际裁判设备及整车验证仍需后续完成。

## 环形 DMA 修复后的实机回归

2026-11-25 修复前实测：206 字节连续发送仅回传最后 4 条裁判帧（13 个 CAN 分包）；每帧拆分并增加 5 ms 间隔时，新增 38 个分包，10 条原始帧收齐。该结果定位到 BSP 满 DMA 缓冲未交付的问题，不是模块拼帧或 CAN 分包格式错误。

新固件采用 USART1/6 环形 DMA 的 HT/TC/IDLE 交付，USART3 DBUS 仍使用原路径。重新构建并烧录后，先清空 CAN 上位机接收列表，再依次验证：

```powershell
# 不额外插入间隔，预期一组共 38 个 0x501 分包
python tools/referee_simulator.py --port $uartPort --once --verbose
# 一次串口写入完整 206 字节，同样预期 38 个分包
python tools/referee_simulator.py --port $uartPort --mode burst --once --verbose
# 持续发送，记录原始帧，与 CAN 接收日志逐帧对照
python tools/referee_simulator.py --port $uartPort --duration 30 --log "$env:TEMP\referee_ring_tx.jsonl"
```

每组都应重组为 10 条原始帧，核对长度、内容和 CRC，而不是只看分包数量；检查 `referee_drop_count` 是否增加。最后重做拆帧、坏 CRC、错误长度测试。整车条件具备后，还需验证 USART6 上的 IM900、USART1 上的 HI12，以及 USART3 DBUS 不受影响；当前只接两块 DM Tool，不能据此确认其他模块实测通过。
