# FM03 交接快照

更新日期：2026-10-07，北京时间。后续 agent 必须先读 `AGENTS.md`，每轮更新本文件并追加 `WORK_LOG.md`。

## 目标与当前阶段

用 ESP32 被动监听 RS3 ↔ DJI FM03，保存原始 CAN 抓包，基于对照实验分析控制协议。监听和上位机已跑通；目前在定位拨轮相关字段，尚未实现或验证独立驱动 FM03。

## 仓库与交接文件

- 工作目录：`J:\New soft project\FM03`。
- Git 分支：`main`；本次交接同时建立本地初始提交。提交版本以 `git log -1 --oneline` 为准，不在自身快照内硬编码其 SHA。
- 没有配置远程仓库；没有推送。
- `WORK_LOG.md`：持续追加的工作历史；`captures/INDEX.md`：抓包语义及用途索引。
- Git 保存源代码、交接文档、原始抓包及分析结果。`.venv/`、`.pio/`、`backups/` 被忽略。
- `.gitattributes` 禁止原始 `.log` 的换行转换，避免 Windows Git 修改抓包字节。首次提交前逐个比对原始文件与暂存对象哈希。

## 硬件与运行

- ESP32-S3-DevKitC-1 N16R8；esptool 实测为 ESP32-S3 revision v0.2、16MB Flash、8MB PSRAM。
- SN65HVD230，3.3V 供电。GPIO4 → TXD，GPIO5 ← RXD；GND 共地；CANH/CANL 并联到对应总线。
- 用户已拆除跨 CANH/CANL 的 120Ω 电阻。Rs 电路、原总线终端及 USB-C 原引脚对应关系未完整记录，不要据型号或 D+/D− 标记猜线序。
- OLED 为 0.49 寸 64×32，暂未参与固件；接口、控制器、实际引脚未确认。
- 已烧录 `listen_1m`：TWAI Listen Only、CAN 1Mbps、USB-UART 921600 baud。发送队列为零，代码无 CAN 发送调用。
- 2026-10-07 已确认 COM17 的 CH343 USB-UART 对应这块 ESP32；插拔后端口可能变化，应重新检测。
- 上位机：双击 `启动上位机.cmd`，访问 `http://127.0.0.1:8765/`。只绑定本机地址；重复启动复用现有服务。
- 开始抓包连接串口并写日志；停止保存摘要并释放串口。关闭网页不会停止采集。烧录前先确认未在抓包。

## 已验证的抓包

### 一次未精确标记的拨轮转动

`captures/20261007_154446_862533_capture.log`，30.44 秒，52666 帧，5 个标准 CAN ID：0x106、0x201、0x232、0x503、0x504。

用户明确说向一个方向转过一次拨轮，但未标记具体方向或动作时间；不能将这整段当作静止或严格正转样本。帧序号连续，无重置；已记录固件状态的丢帧/总线错误计数为零。开头有非帧字节，末尾有 22 字节未完成行，不宣称采集边界无损。

### 静止基线

`captures/20261007_154729_002678_idle.log`，用户确认全程静止，10.41 秒，17987 帧，同样 5 个 ID，帧序号连续，无重置，末尾 37 字节未完成行。

首末固件累计值均为 logger_dropped=273、bus_errors=1、rx_missed=0、rx_overrun=0。本段已记录状态中没有新增丢帧或总线错误；273 和 1 在首条状态时已经存在，不是本段新增，也不能描述成累计零。

## 协议结果与限制

- 各 ID 的 data 分别串接后，可按 DJI DUML 0x55 包格式重组，报头 CRC8 和整包 CRC16 均通过。转动样本重组14633包，静止样本4998包。
- CRC8 初值0x77、反射多项式0x8C；CRC16 初值0x3692、反射多项式0x8408。参考实现：[dji-firmware-tools](https://github.com/o-gs/dji-firmware-tools/blob/master/comm_dat2pcap.py)。
- 重点候选：CAN 0x503，逻辑 E5→BF，命令04:69，应用 payload 6字节；0x201 的 E5→52 同命令也有相同变化规律。
- 静止时 payload 固定 `0F 04 08 19 00 00`；转动样本中出现428种payload，仅最后两字节变化，即 payload 零起始偏移4、5（不是 CAN 单帧偏移）。
- 0x503 静止时约每10ms出现一个此命令。暂按有符号小端16位解读候选两字节，转动样本范围-557～388，开头/末尾为零。
- 这些事实支持“拨轮相关候选”；还不能确定它是增量、速度、位置或别的编码，不能确定符号、量纲或对应的物理设备。没有重放或主动控制验证。
- 0x106 的04:05 在静止时也一直变化；变化不必然意味着拨轮控制。
- `.analysis.json` 保存命令分组/可变偏移/样例，`.vs-idle.json` 保存对照；早期 `.protocol.json` 保存首次重组结果。原始 `.log` 是复核依据。

## 接手后先做什么

1. 检查 Git 状态与最新抓包，避免忽略用户刚保存的新文件。
2. 等待或分析独立 `slow_forward`、`slow_reverse` 记录。每段10秒：前2秒静止、中间3秒缓慢转动、最后5秒静止。由用户固定两个物理方向并说明，避免自行定义已有数据的正负方向。
3. 用静止记录作参考重组，比较04:69末尾两字节的符号、动作区间、停止后值和相邻状态报文；区分计数器/校验与应用数据。
4. 找到规律后用重复动作/速度梯度做被动验证。独立主动控制阶段需要另行确认范围，当前不要发 CAN 帧。
5. 每次新结果更新本文件、追加工作日志及抓包索引，记录检查和 Git 状态。

## 常用命令（项目目录，PowerShell）

```powershell
git status --short --branch
git log -1 --oneline
.\.venv\Scripts\python.exe -m serial.tools.list_ports -v
.\.venv\Scripts\python.exe tools\gui_server.py --open
.\.venv\Scripts\python.exe tools\analyze_protocol.py --self-check
.\.venv\Scripts\python.exe tools\analyze_protocol.py captures\20261007_154729_002678_idle.log captures\20261007_154446_862533_capture.log
.\.venv\Scripts\python.exe -m unittest discover -s tools -p test_gui.py -v
```

此前验证：两个速率版本1Mbps/500kbps编译通过；实际烧录/总线接收只验证1Mbps。采集自检、解析自检、5个模拟串口/HTTP测试、真实COM17开始/停止/保存/释放串口均通过。本次交接只修改记录文件，核对证据和Git，不额外重烧录。

## 本机备份与运行局限

- 原固件全量备份：`backups/esp32s3_before_listener_20261007.bin`，16777216字节；SHA256 `8822C84EFF9E59C5231930BBE3E4FC2F5C010051ADDA6FD6EC723B983941D00F`。不在Git内，跨机器交接需单独保留。
- 原始输出是串口字符行，吞吐存在上限；曾看到累计输出队列丢帧273，必须持续看增量与积压。
- 采集错误日志在 `.pio/gui-server.log`。2026-10-07 15:43的一次零帧采集曾因串口读取拒绝访问而终止，根因未独立确认；随后成功收到总线数据。
- 生成的GUI JSON内部 `summary_file` 曾记录为null（序列化时尚未赋值）；实际摘要文件通常存在，应用状态会给出下载链接。不要把这个字段的null当作文件丢失。
