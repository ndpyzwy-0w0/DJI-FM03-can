# DJI FM03 CAN

**简体中文** | [English](README.en.md)

DJI FM03 跟焦电机的 CAN 协议逆向与独立控制。项目提供 ESP32 适配器固件、桌面上位机、抓包分析工具和可复算的研究证据，支持速度控制、自动行程标定、坐标反馈及实验性闭环定位。

核心控制协议已有实机验证：可脱离 RS3 控制电机、触发自动寻限位并读取标定后坐标。自动到位和严格到点已完成软件实现，实际精度与可靠性仍需实机验证。

## 功能

- **独立运动控制**：正反转、速度调整、零值停止、定时运动与脉冲。
- **自动行程标定**：触发电机内部寻限位，显示阶段变化；已有用户实测成功记录和校准后软限位证据。
- **坐标反馈与记录**：显示有效性和数据年龄，记录 A/B 坐标；当前观察范围为 0～4096。
- **目标坐标到位**：指定目标、返回 A/B、可选接近减速、可设置最低速度与超时。
- **严格到点**：低速纠偏，只有坐标等于目标且归零后稳定才算成功；无进展、失效或超时明确失败。
- **自动实验日志**：保存 UART 原始接收字节、命令/写入结果、状态/坐标、A/B 和到位过程，生成便于离线分析的摘要。
- **协议研究工具**：CAN 抓包、按 CAN ID 重组 DUML、CRC 校验、报文构造与对照分析。

当前控制固件为 **1.2**，上位机为 **1.6**。定位闭环运行在上位机，通过普通速度命令控制电机；PID 尚未实现。

## 硬件与运行方式

已验证硬件为 DJI RS 跟焦电机（2022 款，FM03）、ESP32-S3-DevKitC-1 N16R8 和 SN65HVD230 CAN 收发器。CAN 速率为 **1 Mbps**，UART 为 **921600 baud**；当前控制固件使用 **GPIO4/TX、GPIO5/RX**。

- **被动监听**：保留 RS3 ↔ FM03 原链路，使用 `listen_1m` 环境。
- **独立控制**：使用 `motor_control` 环境，发送时 RS3 必须物理脱离测试总线。

确认实际模块的电源、公共地、CANH/CANL、终端与静默配置。监听模块的接法不能直接用于主动发送；接线与早期验证保留在[历史说明](README.history.md)。FM03 原固件保持不变，本项目固件运行在外接 ESP32 上。

## 快速开始

已验证软件环境为 Windows / Python 3.12。在 PowerShell 中执行：

```powershell
git clone https://github.com/ndpyzwy-0w0/DJI-FM03-can.git
cd DJI-FM03-can
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe tools/motor_control.py
```

程序启动不会自动连接串口、ARM、标定或运动。需要相应适配器固件；固件只编译，不自动烧录：

```powershell
.\.venv\Scripts\python.exe -m platformio run -e motor_control
# 仅监听使用：
.\.venv\Scripts\python.exe -m platformio run -e listen_1m
```

烧录前核实设备与是否存在活动采集。历史回放/诊断环境有各自的触发行为，不用于替代上述两个正常入口。

控制前阅读[最新操作指南](reports/20261010_strict_positioning_guide.md)。独立台架确认接线和机械条件后，手动连接、ARM、STOP 等普通状态归零；需要时手动标定，取得新鲜阶段 3 坐标后再使用 A/B 或目标到位。

**内部校准没有已验证的取消命令。** 普通 STOP、DISARM 和看门狗不保证取消校准；普通停止与短时命令也不是通信/固件故障时的硬件停转保证。

## 构建 Windows 单文件程序

本机已验证 PyInstaller 6.22.3：

```powershell
.\.venv\Scripts\python.exe -m pip install pyinstaller==6.22.3
.\.venv\Scripts\python.exe tools/build_host_app.py --name FM03_MotorControl_StrictPosition
```

产物为 `dist/FM03_MotorControl_StrictPosition.exe`。`dist/`、虚拟环境、编译缓存和本机备份不在源码仓库中，不代表已有可下载的发布附件。

## 代码与证据

- [src/motor_control.cpp](src/motor_control.cpp)：适配器控制固件与 UART 命令。
- [src/motor_status.h](src/motor_status.h)：电机状态解析。
- [tools/motor_control.py](tools/motor_control.py)：桌面上位机。
- [tools/position_control.py](tools/position_control.py)：无 GUI/串口依赖的通用坐标控制模块，可用于其他控制项目接入。
- [tools/control_session.py](tools/control_session.py)：自动日志与离线摘要。
- [captures/INDEX.md](captures/INDEX.md)、[CAPTURE_FILE_MAP.md](CAPTURE_FILE_MAP.md)：原始证据和动作对应关系。
- [reports/](reports/)：协议分析、验证记录及操作指南。

坐标模块需要调用者提供最新状态、反馈、发送与记录回调；接口说明见[严格到点指南](reports/20261010_strict_positioning_guide.md)。嵌入式独立运行仍需移植，本仓库只维护 FM03 协议与通用控制能力。

## 验证与已知限制

最新版已有 **67 项相关离线软件测试通过**，包括假串口、日志、状态、校准及定位。下面运行其中的定位测试（23 项）和协议自检，不连接真实串口：

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tools -p test_position_control.py -v
.\.venv\Scripts\python.exe tools/capture.py --self-check
.\.venv\Scripts\python.exe tools/analyze_protocol.py --self-check
```

自动寻限位有实机成功记录；用户通过机械标记确认重复标定后的相同坐标对应相同位置，限于已测试的安装/行程条件。软件测试和严格成功判据不代表已测量的机械定位精度。

仍待确认：

- 自动定位的成功率、双向回差、负载影响及最低可用速度。
- 坐标来自真实编码器还是内部参考，以及 4096 的物理含义；不能直接换算角度、毫米或编码器分辨率。
- 改变行程后的映射、掉电/复位后的坐标与限位保留、校准取消和完整故障语义。
- 其他未知报文、跨设备兼容性与长期运行可靠性。

FM03 的 GD32F303CCT6 原固件尚未提取；ESP32 备份不是电机原固件。`firmware-extract` 研究分支尚未合并或上传。完整范围见[PROJECT_STATUS.md](PROJECT_STATUS.md)。

## 进一步阅读

- [严格到点与最低速度](reports/20261010_strict_positioning_guide.md)
- [自动实验日志](reports/20261010_control_logging_guide.md)
- [自动寻限位实测](reports/20261010_183747_calibration_findings.md)
- [坐标与机械位置确认](reports/20261010_mechanical_coordinate_confirmation.md)
- [软限位证据](reports/20261009_virtual_limits_findings.md)
- [历史 README](README.history.md)、[工作日志](WORK_LOG.md)、[当前交接](HANDOFF.md)

历史文档包含当时的假设和阶段状态；当前结论以项目状态和最新交接为准。原始抓包保留不改，设备时间是驱动取帧时间，UART 日志不等于完整 CAN 总线记录。研究与详细操作文档目前主要为中文。
