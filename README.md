# FM03 协议分析

## Agent 接手入口

先读 [HANDOFF.md](HANDOFF.md) 获取当前交接快照，再按 [AGENTS.md](AGENTS.md) 维护记录。历史工作见 [WORK_LOG.md](WORK_LOG.md)，每份抓包的动作语义和用途见 [captures/INDEX.md](captures/INDEX.md)。

Git 已初始化在 `main`；本次交接建立本地初始提交，没有远程或推送。当前版本使用 `git log -1 --oneline` 查询；每次完成工作更新交接与日志，检查并记录 Git 状态。原固件备份不在 Git 中。

## 目标

被动监听 RS3 ↔ DJI FM03 的正常控制链路，保留原始抓包，再依据证据分析协议。后续主动控制另立阶段。

## 已知硬件与待确认事项

- 开发板：ESP32-S3-DevKitC-1 N16R8。
- CAN 收发器：SN65HVD230 已确认；2026-10-07 用户确认已拆除跨接 CANH/CANL 的 120 Ω 电阻并完成接线，随后补充总线侧尚未接上。具体模块电路、Rs 接法及原总线终端仍待核实。
- 显示屏：0.49 英寸、64 × 32 OLED。接口、控制器、地址及引脚待确认。
- 已在 RS3 ↔ FM03 链路收到经典 CAN 标准数据帧，1 Mbps 实测可正常解码；连接器引出线对应的原始 USB-C 引脚及直流电压尚未记录。

旧聊天[《查找大疆驱动方案》](https://chatgpt.com/c/6abb8489-1ea4-83ee-b3a0-8857f2c41a51)曾依据示波器图推测可能为 CAN，并提出 1 Mbps / 500 kbps 候选。该图尚未重新核验，这些均不是已确认事实。

## 工作阶段

1. **核实硬件与链路。** 确认 ESP32 芯片完整型号、收发器电平/终端/静默能力、连接器线序和电压、总线类型，以及 OLED 接口、控制器、地址和引脚。
2. **建立被动监听。** 仅在确认是 CAN 后，使用外接收发器和 ESP32 TWAI Listen Only 模式。不要将 GPIO 直接接入 CAN，也不要盲目添加终端电阻。先输出串口原始日志，之后再加入 OLED 状态显示。

   候选接线：开发板 3V3 → SN65HVD230 VCC，GND 共地；GPIO4（TWAI TX）→ D/TXD，GPIO5（TWAI RX）← R/RXD；CANH/CANL 只接到线序已核实的总线。Rs 拉高可关闭驱动并保留接收，但必须先核实模块电路；如果 Rs 已接地，不要直接拉高，也不要求为此改板。先核查模块是否带 120 Ω 终端及原总线终端，不新增第三个终端。OLED 仅在确认为四针 3.3 V I²C 模块时，候选接线为 GPIO8 → SDA、GPIO9 → SCL、3V3 → VCC、GND → GND；控制器和 I²C 地址待确认。候选 GPIO 需与实物引脚及其他占用核对。
3. **分场景重复抓包。** 分别记录静止、慢速正转、慢速反转、快速正转、快速反转及校准；每次标记动作时间。原始日志保留时间戳、ID、扩展帧标记、帧类型、DLC、数据字节及丢帧统计。
4. **基于证据分析协议。** 比较 ID、周期、字段和校验。不能仅凭 ID 判断发送方向；需要结合物理连接、动作时序与多次抓包验证。

规划：gpt-6-astra。执行：gpt-6-sol，medium。

## 第一版监听固件

`src/main.cpp` 固定使用 GPIO4 TX / GPIO5 RX、TWAI Listen Only、接收所有 ID，发送队列长度为零，代码不调用 CAN 发送 API。串口经开发板 USB-UART 口输出，921600 baud；OLED 和 PSRAM 暂不启用。板型沿用 DevKitC-1，Flash 容量和分区覆盖为 N16R8 的 16 MB。

PowerShell（项目目录）：

```powershell
.\.venv\Scripts\python.exe -m platformio run -e listen_1m
.\.venv\Scripts\python.exe -m platformio run -e listen_1m -t upload --upload-port COM17
.\.venv\Scripts\python.exe tools\capture.py --port COM17 --seconds 15 --label idle
```

2026-10-07 已通过 esptool 确认 COM17 为 ESP32-S3（revision v0.2），8 MB 内置 PSRAM、16 MB Flash，与 N16R8 对应。需要验证 500 kbps 时，将编译/烧录环境改为 `listen_500k`。速率不会自行切换，日志包含当前 bitrate。

首次烧录前的全量 Flash 备份保存在 `backups/esp32s3_before_listener_20261007.bin`（16 MB），读取过程日志为 `backups/read_flash.log`。备份目录不纳入 Git，保留在本机。

首次建立环境：用本机 Python 创建 `.venv`，随后 `python -m pip install -r requirements.txt`。当前机器已有 PlatformIO 的 Espressif 工具链缓存。

原始串口字节原样保存到 `captures/*.log`，旁边的 JSON 记录时间（北京时间）、端口、帧数、ID 分布、序号缺口、首末状态和固件错误。不同动作分别用 `idle`、`slow_forward`、`slow_reverse` 标签，标记实际动作时间。

帧行格式：`t_us,sequence,id_hex,extended,rtr,dlc,data_hex`。`t_us` 是高优先级接收任务从驱动取帧时的开机后微秒数，**不是硬件帧边沿时间**。远程帧保留 DLC，但 data_hex 为空。序号缺口只反映接收任务之后的丢帧；驱动之前的丢帧另看状态。

每秒 `# STATUS` 包含 `rx`（任务收到的累计帧数）、`logger_dropped`（串口输出队列丢帧）、`rx_missed`（驱动接收队列/勘误相关丢帧）、`rx_overrun`（硬件 FIFO 溢出）、`bus_errors` 和两级队列积压。这些计数从启动起累计；连续抓包应比较首末值。串口字符行吞吐有限，不能保证满负荷 1 Mbps 总线无损；出现丢帧时先保存证据再调整采集方式。烧录、更换速率和复位会产生采集空档。

主机工具自检：`.\.venv\Scripts\python.exe tools\capture.py --self-check`。成功编译只证明软件可构建，真实链路、速率和完整性必须通过抓包验证。

## 当前验证结果（2026-10-07）

- `listen_1m` 和 `listen_500k` 均编译成功，抓包脚本自检通过。
- 原 Flash 已完整备份（16777216 字节）；SHA256：`8822C84EFF9E59C5231930BBE3E4FC2F5C010051ADDA6FD6EC723B983941D00F`。
- COM17 已成功烧录 `listen_1m`，烧录数据校验通过；当前串口为 921600 baud，CAN 配置为 1 Mbps Listen Only。
- 尚未连接真实总线。`captures/20261007_151952_378136_no_bus_startup.log` 记录了约 5 秒启动后状态：rx=0、各丢帧/总线错误计数为 0，没有固件错误。这仅验证监听固件和串口采集能运行，**不确认线序、实际 CAN 速率或总线接收能力**。
- 下一步：断电接入已核实的 CANH/CANL/GND 支路，保持 RS3 ↔ FM03 原链路，开机确认原拨轮控制正常，再抓静止、慢速正转和慢速反转数据。没有总线数据时暂不尝试 500 kbps。

## 可视化上位机

双击项目根目录的 **`启动上位机.cmd`**，浏览器会打开 <http://127.0.0.1:8765>。服务只监听本机地址，使用现有 Python 环境和 pyserial，无额外界面依赖。服务已运行时，重复启动会打开现有界面。

1. 选择串口（当前为 COM17，USB 串口优先显示），选本次场景。
2. 点击 **开始抓包**：连接串口，实时显示接收帧数、最近一秒帧率、时间、ID 分布、最新 150 帧、固件状态和错误计数。
3. 点击 **停止并保存**：停止电脑端记录，关闭并释放串口，生成 `.log` 原始日志和 `.json` 摘要。点击界面上的下载按钮可获取本次文件；原件始终保存在项目 `captures/` 中。

关闭页面不会停止正在运行的采集，请先停止再关闭。ESP32 固件继续被动监听，电脑端按钮不发送 CAN 控制帧。CAN 速率由已烧录的固件决定，界面显示的是固件配置值，不能据此认定实际总线速率。

显示用列表有 150 帧上限，完整原始串口字节仍写入文件。固件丢帧/总线错误显示本次首条状态之后的增量，详情显示开机后累计值；序号缺口单独计数，避免重复相加。固件重启后计数增量继续累计。端口被其他工具占用或设备断开时显示错误，已经接收的日志保留。

手动启动与测试：

```powershell
.\.venv\Scripts\python.exe tools\gui_server.py --open
.\.venv\Scripts\python.exe -m unittest discover -s tools -p test_gui.py -v
```

上位机测试覆盖分片串口数据、开始/停止及重新抓包、串口异常后保存、计数重置、输入校验和 HTTP 控制/下载。真实 COM17 的开始/停止验证已通过，收到 1 Mbps 固件状态，尚未接总线，因此仍为 0 帧。运行错误日志保存在 `.pio/gui-server.log`。

## 首次总线抓包成功（2026-10-07 15:44，北京时间）

原始记录：`captures/20261007_154446_862533_capture.log`。用户确认本次约 30 秒期间曾向一个方向转动拨轮一次，具体方向及动作时间未标记，不能按文件名 `capture` 将整段当作静止基线。

- 已解析 52666 个有效 CAN 标准数据帧，DLC 为 2–8。记录时长 30.44 秒，有效帧时间跨度约 30.42 秒。
- ID / 帧数：0x106 / 12294；0x201 / 12434；0x232 / 6688；0x503 / 9090；0x504 / 12160。
- 帧序号连续、无重置。收到的固件状态中 logger_dropped、rx_missed、rx_overrun 和 bus_errors 全部为 0；这不等同于证明采集边界无损。文件开头有少量非帧字节，结尾有 22 字节未完成行，均保留在原始记录中。
- 将各 CAN ID 的 data 字节分别按顺序拼接，按 DJI DUML 0x55 包格式检查报头 CRC8 和整包 CRC16，重组出 14633 个双校验通过的包。CRC8：初值 0x77、反射多项式 0x8C；CRC16：初值 0x3692、反射多项式 0x8408。参考来源：[dji-firmware-tools 原始实现](https://github.com/o-gs/dji-firmware-tools/blob/master/comm_dat2pcap.py)。使用一个实际完整 19 字节包验证了两种校验计算，再扫描全量数据。
- 初步命令组：0x503 包含 04:69；0x201 包含 04:69、04:2F；0x106 包含 04:05、04:10；其他 ID 包含 04:2F、24:20、00:0E。两个 ID 上的 04:69 均出现 428 种 payload，值得在静止/方向对照中优先比较，**目前未确定控制意义或物理发送方向**。

包长度、命令组计数及完整包样例保存在 `captures/20261007_154446_862533_capture.protocol.json`。此次重组使用原始文件，有效帧序号连续；后续出现丢帧/重启时必须在断点处重新建立重组边界。

下一组对照：在上位机依次选择静止、慢速正转、慢速反转，分别保存独立记录。静止录 10 秒；两个方向各录 10 秒，前 2 秒静止、中间约 3 秒缓慢转动、最后约 5 秒静止。保持接线、速率和其他操作一致，再比较重组后的应用 payload。

## 静止基线与首次转动对照（2026-10-07 15:47，北京时间）

用户确认 `captures/20261007_154729_002678_idle.log` 为静止记录。时长 10.41 秒，17987 帧、5 个 CAN ID，帧序号连续、无重置；重组出 4998 个 CRC8/CRC16 均通过的包。

本段首末固件状态均为 logger_dropped=273、bus_errors=1、rx_missed=0、rx_overrun=0，**本段已记录状态中均无新增**。273 次输出队列丢帧和 1 次总线错误在首条状态时已经存在，不能算作本段新增，也不能把累计计数描述成零。日志结尾 37 字节未完成行已保留。

重点候选：

- CAN 0x503，逻辑地址 E5→BF，命令 04:69，6 字节应用 payload；静止期间共 1037 包，内容始终为 `0F 04 08 19 00 00`，报文取帧间隔中位数约 10.002 ms。
- CAN 0x201 上 E5→52 的同一命令、同长度 payload 也有相同规律。
- 首次含拨轮转动的记录中，两路 payload 均出现 428 种内容，仅 payload 第 5、6 字节（零起始偏移 4、5）变化，前四字节始终为 `0F 04 08 19`。这是一处明确的拨轮相关候选，尚未证明它直接控制电机。
- 如暂按有符号小端 16 位解读这两个字节，首次记录取值范围为 -557～388，记录开头/末尾为 0。由于动作时间、方向没有精确标记，不能据此认定符号对应某个物理方向，也未确认它是增量、速度或其他编码。
- CAN 0x106 的 04:05 报文在静止时也持续变化，不能把任何变化都解释为拨轮或电机控制字段。

离线分析工具：`tools/analyze_protocol.py`。它按 CAN ID、帧格式和可靠性断点分别重组，校验报头/整包，输出命令组、可变字节偏移及对照 JSON，不连接串口。CRC、受损包、分片重组及丢帧断点自检已通过。

```powershell
.\.venv\Scripts\python.exe tools\analyze_protocol.py --self-check
.\.venv\Scripts\python.exe tools\analyze_protocol.py captures\20261007_154729_002678_idle.log captures\20261007_154446_862533_capture.log
```

结果保存为各原始日志旁的 `.analysis.json`，以及转动记录的 `.vs-idle.json`。接下来仍需慢速正转、慢速反转独立记录，按此前 2 秒静止 / 3 秒转动 / 5 秒静止的节奏，验证最后两字节与物理方向和动作时序的关系。
