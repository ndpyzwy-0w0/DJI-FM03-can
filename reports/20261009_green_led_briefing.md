# 绿灯问题分析简报（供其他 AI 接手）

日期：2026-10-09。目标：让 ESP32-S3 独立驱动 DJI RS 跟焦电机（FM03），**不依赖 RS3**。当前卡点：电机指示灯不进入"绿灯"（官方语义：绿灯常亮 = 可正常工作；红灯常亮 = 连接异常）。

相关文档：[HANDOFF.md](../HANDOFF.md)、[HANDOFF_FOR_NEXT_AGENT.md](../HANDOFF_FOR_NEXT_AGENT.md)、[20261008_green_led_analysis.md](20261008_green_led_analysis.md)、[WORK_LOG.md](../WORK_LOG.md)。

---

## 1. 一句话现状

**有一个能稳定让电机"闪绿"的历史固件（`replay_hist`），和一个配置上已对齐、却不绿的重建版（`replay_bisect5`）。** 二者在总线上发出的报文**内容相同**，但**重建版只发出每个报文的首帧、后续帧全部失败**，历史版则**完整发出**。这个"发送完整度"差异是目前唯一被证实的行为差异。

---

## 2. 硬件与链路

- ESP32-S3 DevKitC-1 + SN65HVD230 收发器（**保留 120Ω** 的那块），GPIO4=TX / GPIO5=RX，**1 Mbps**，Arduino core 2.0.17 / PlatformIO `espressif32@7.0.1`（IDF v4.4 预编译库）。
- 串口 `COM17` @ 921600（**动态，使用前核实**）。
- 台架：**总线上只有电机**（适配器单独供 5V），**RS3 不参与、不供电**。监听用另一块模块（120Ω 已拆、Rs 接 3.3V 关断驱动器），**同一时刻只允许一块模块在总线上**。
- 电机单独上电会以约 7600 帧/秒**刷屏**（同一帧无限重复、0 个 CRC 有效包）。

---

## 3. 已知的协议事实（字节均已核验双 CRC）

| 报文 | CAN ID | 逻辑地址 | 分片 | 备注 |
|---|---|---|---|---|
| `00:01` 请求 | `0x503` | E5→BF | 13 B = **8+5** | `55 0D 04 33 E5 BF 00 00` + `40 00 01 DD FE`，seq 0 |
| `04:69` 拨轮命令 | `0x503` | E5→BF | 19 B = **8+8+3** | payload `0F 04 08 19 <int16LE>` |
| `04:69` 拨轮命令 | `0x201` | E5→52 | 19 B = **8+8+3** | 同上，目标与 ID 不同 |

- CRC8 头：init `0x77`、反射多项式 `0x8C`；CRC16 包：init `0x3692`、反射多项式 `0x8408`。
- 真实 RS3 开机时序号**从 0 重新开始**（`04:05` seq 62 起、`04:69` 启动态 seq 6）。
- 用本项目工具重算 seq=0 的 `04:69` 得到 `55 13 04 03 E5 BF 00 00 00 04 69 0F 04 08 19 00 00 6B 56`，与历史固件硬编码的字节**完全一致**。

---

## 4. 对照组与实验组

| | `replay_hist`（**能闪绿**） | `replay_bisect5`（不绿） |
|---|---|---|
| 源码 | `src/transmit_replay_hist.cpp`（提交 `7e261d4` 原文） | `src/transmit_replay.cpp`（当前版） |
| 发送内容 | 只有 `00:01` + 稳态 `04:69` | 同（已用宏关闭其余报文） |
| `tx_queue_len` | **8 → 崩溃循环** | **8 → 崩溃循环** |
| `REPLAY_INTERVAL_MS` | 500 | 500 |
| `REPLAY_SECONDS` | 120 | 120 |
| `04:69` 序号 | 恒 0 | 恒 0（`REPLAY_SEQ_START=0`、`STEP=0`）|
| `04:69` 路由 | 仅 `0x503` | 仅 `0x503`（`REPLAY_SINGLE_ROUTE=1`）|
| `send_packet` | 无自旋门、`pdMS_TO_TICKS(20)` 超时 | 同（`REPLAY_LEGACY_SEND=1`）|

**穷尽宏对比结论**：两者**都读取**的 7 个宏（`CAN_BITRATE`、`LED_RGB_GPIO`、`REPLAY_AUTO_START`、`REPLAY_INCLUDE_0469`、`REPLAY_INTERVAL_MS`、`REPLAY_SECONDS`、`REPLAY_STAGE`）取值**完全一致**；另有 14 个宏只有新版源码读取，历史版源码**不读**它们。

---

## 5. 决定性证据：自收回显对比

方法：给固件设 `message.self = 1`（自收，M4 已验证可用），使**自己发出的帧也进入日志**，从而直接看总线上的实际字节。

- 当前版 + 自收：`replay_echo` → `captures/20261008_echo.log`
- 历史版 + 自收：`replay_hist_echo` → `captures/20261008_hist_echo.log`

| 我方在 CAN `0x503` 上实际发出的帧 | `bisect5` | `hist`（能绿） |
|---|---|---|
| `55 0D 04 33 E5 BF 00 00`（`00:01` 首帧） | **49** | 13 |
| `40 00 01 DD FE`（`00:01` **尾帧**） | **0** | **14** |
| `55 13 04 03 E5 BF 00 00`（`04:69` 首帧） | **0** | **14** |
| `00 04 69 0F 04 08 19 00`（`04:69` 帧2） | **0** | **14** |
| `00 6B 56`（`04:69` 帧3） | **0** | **14** |

- `bisect5` 日志持续出现 `# WARN transmit_rejected total=1,3,5,7,…`（约每秒一条，限速过的）。**即：只有首帧发出，其余帧全部 `twai_transmit` 失败。**
- `hist` 日志**没有任何 `WARN`**，且 14 次完整发出两包。

**结论：不绿的那一版在总线上发的是残缺报文。**

---

## 6. 关键机制（读 IDF v4.4 源码得出）

### 6.1 `tx_queue_len = 0` 时 `twai_transmit` 的行为

```c
if (p_twai_obj->tx_queue == NULL) {
    if (p_twai_obj->tx_msg_count != 0) {   // 仍有帧在途
        ret = ESP_FAIL;                     // 立即失败，不阻塞、不入队
    }
    ...
}
```

即：**发送缓冲被占用时，后续帧直接 `ESP_FAIL`**，必须等前一帧完成（或放弃）才能发下一帧。

### 6.2 已定位的 IDF 缺陷

`tx_queue_len > 0` 时，任务的 `xQueueSend` 与 TX 完成 ISR 的 `xQueueReceiveFromISR` **竞争同一队列链表**，导致 panic：

```
uxListRemove <- xTaskRemoveFromEventList <- xQueueGenericSend <- twai_transmit (twai.c:574)
```

**这正是两组固件"崩溃循环"的来源**（它们都用 `tx_queue_len = 8`）。已实测**无效**的规避：`NO_ACK` 模式、`TX_QUEUE_LEN=1`、降低发送速率、发送失败后退避。

---

## 7. 未解决的核心矛盾（**请重点判断这里**）

两版固件的**报文内容相同、宏配置相同**，但：

- `hist` **每轮都能完整发出 5 帧**（`00:01` 2 帧 + `04:69` 3 帧）
- `bisect5` **每轮只发出 1 帧**，后续 `ESP_FAIL`

**在 `tx_queue_len = 0` 下，若前一帧一直等不到 ACK，发送缓冲会一直被占用，后续帧必然全部失败。** 因此有一个强假设：

> **电机可能不 ACK 我方的帧。** 那么任何"多帧报文"都只能发出首帧——这与 `bisect5` 的观测完全吻合。

但这留下一个**未解释的矛盾**：`hist` 同样多帧、同样配置，为何能完整发出 5 帧？

**候选解释（均未验证）**：
1. **`hist` 的崩溃更快**：每次重启后 TX 控制器被重新初始化，缓冲区状态被清空，于是每轮能"趁干净"发出整个报文；而 `bisect5` 崩溃较慢（实测 5 秒内 6–8 次），缓冲区被占住后一路失败。
2. **发送顺序不同**：`hist` 先发 `00:01`；`bisect5` 先发 `04:69` 再发 `00:01`（实现顺序不同）。若某个报文能促成电机的 ACK 行为，顺序就可能有影响。
3. 两版 `send_packet` 的**实际编译结果仍有差异**（本 agent 未能用 objdump 取出该符号做指令级比对——**这是明确的待办**）。

**已排除**（各自独立实验，用户目视判定）：崩溃循环本身 ❌、反复重放启动序列 ❌、序号恒 0 ❌、单路由 ❌、报文种类 ❌、发送节奏（500ms）❌。

---

## 8. 建议的下一步（按可行性排序）

1. **指令级比对 `send_packet`**：用 `xtensa-esp32s3-elf-objdump` 分别反汇编 `replay_hist` 与 `replay_bisect5` 的 `send_packet`，确认两版是否真的生成等价代码。（本 agent 取符号失败，需换方法：`-d` 全量反汇编后按地址段比对，或用 `nm` 先取地址。）
2. **验证电机是否 ACK 我方帧**：这是第 7 节假设的核心。方法：`twai_read_alerts()` 读 `TWAI_ALERT_TX_SUCCESS` / `TWAI_ALERT_BUS_ERROR`；或直接读 `error_code_capture_reg` 的 LEC 字段（0=无错、3=ACK 错误）。
3. **单帧直发试验**：绕过多帧问题，只发**一个单帧报文**（如 `04:69` 的首帧 `55 13 04 03 E5 BF 00 00`），看电机有无任何反应。可排除"多帧发不全"这一变量。
4. **换发送通道**：用独立 USB-CAN 适配器从 PC 发送同样字节，看电机是否变绿。若能，则问题确定在 ESP32/TWAI 侧；若不能，则问题在报文内容或物理层。
5. **用示波器**（非逻辑分析仪——CAN 是差分，逻辑分析仪测不了 ACK 位）直接观察 ACK 位是否被电机拉低。

---

## 9. 复现方式

```powershell
# 烧录能绿的对照组
.\.venv\Scripts\python.exe -m platformio run -e replay_hist -t upload --upload-port COM17

# 烧录不绿的重建版
.\.venv\Scripts\python.exe -m platformio run -e replay_bisect5 -t upload --upload-port COM17

# 自收回显（用于对比实际发出的字节）
.\.venv\Scripts\python.exe -m platformio run -e replay_echo      -t upload --upload-port COM17
.\.venv\Scripts\python.exe -m platformio run -e replay_hist_echo -t upload --upload-port COM17
```

**注意**：`replay_hist` / `replay_bisect5` 都会**崩溃循环**（`tx_queue_len = 8`），这是对照组自带的特征，不必修复；**`replay_echo` 用的是稳定配置**（`tx_queue_len = 0`）。

**安全边界**：绝不在 RS3 与电机通话的总线上发送；每个"开始发送"的子阶段都需要新的明确授权；原始抓包不编辑。
