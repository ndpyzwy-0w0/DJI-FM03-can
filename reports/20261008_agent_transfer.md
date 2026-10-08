# FM03 可复制交接文档

整理时间：2026-10-08 18:54:58 北京时间。以下正文可整体复制给新 agent。

接续任务：通过 ESP32-S3 独立控制 DJI RS 跟焦电机（2022款，FM03），不依赖 RS3。先读取项目交接，再继续推进；不要重复已完成实验。

一、项目与必读入口
项目目录：J:\New soft project\FM03。
先读 AGENTS.md、HANDOFF.md 最新入口、WORK_LOG.md 与 captures/INDEX.md 末尾，再读：
- reports/20261008_probe_minus16_motion_result.md 和同名 .json（最新运动/方向结果）
- reports/20261008_probe16_motion_result.md 和同名 .json（此前 +16 结果）
- reports/20261008_probe_minus16_upload_preparation.json（此次 −16 上传核验）
- reports/20261008_probe_minus16_prepared.json、reports/20261008_probe16_prepared.json（备份/镜像身份）
- reports/20261008_external_research_and_probe_plan.md（一手资料与旧动作字段复核）
旧准备报告中的“未上传”仅指当时状态；旧段落中的“非零未验证”已被最新运动证据更新。以最新结果和HANDOFF入口为准。
先核实 git status --short --branch 和 git log，保留已有修改。串口、进程、板上运行状态与接线/供电使用前重新确认，不凭旧快照操作。

二、已完成的成果
1. 零值常绿组合：replay_gate0_stable_50ms_echo，00:01请求＋零值04:69，50ms节奏。完整120秒无panic，2398请求＋2398拨轮包分片及双CRC通过。用户确认绿灯常亮、手拧有阻力；自动停发板子蓝灯、电机回红。最小必要报文集未确认。
2. +16短时运动：replay_gate0_probe16_echo，RST只监听、BOOT长按2秒启动；零值约5秒、+16软件选择200ms、回零至15秒、50ms。用户确认两次启动，电机绿灯下稍后转一下并停止，最终板蓝/电机红。两个完整15秒日志窗口各4个+16完整包，首非零首片到首回零首片200.000/199.999ms，无panic。原始+16观察未逐窗同步记录方向/幅度。
3. −16单变量对照已经获准、上传并完成，不能再写成“待授权/未上传”。实际配置仅将+16改为−16。用户原话：
“都按完了 电机转了”
“现在停了 逆时针 与上次相反 这次我也按了两次boot 现在板子恢复蓝灯电机恢复红灯了”。
方向是在“从齿轮端看”的问题语境下回答；支持当前台架/报文组合中字段符号对应相反方向运动。旧+16方向未独立测量，不能倒填成同步测量数据。
4. −16抓包：1START/1READY，2手动窗口、2window_done、1stopped_by_button，0AUTO_START/panic/异常Rebooting/WARN/ERROR。首窗口sent224由按键提前结束，次窗口sent300完整15秒；每窗各4个完整−16包，双CRC/8+8+3/独立构包一致，非零首片到首零分别205.000/200.000ms。首窗口软件转换elapsed5015→5220，次5009→5209；200ms软件选择不是精确物理时长，首段205ms不能隐去。首提前结束的具体按键动作/时刻未说明；不把“两次BOOT”解释成仅两个按下动作。
5. 当前结论：已有无RS3的常绿、短时运动、±16反向及用户确认停止证据。完整任意控制、字段速度/位置/增量语义、单位、位移量、重复性和最小报文集仍未完成。不要把±16称为16度或16rpm，不宣布已完成完整独立控制。

三、关键原始证据及边界
+16：captures/20261008_161953_149324_probe16_manual_window.log（旁边有自动JSON）
SHA256 604941e23210107c728d8e6184615661309b06819bf544e93c8c537d9134e899
UART375.156秒、固件两15秒窗口。3START的具体来源未知，不认定为panic；每窗标记内299请求＋299拨轮、结束后额外1零包；启动切换缺口3164/3169。
−16：captures/20261008_184054_626320_probe_minus16_manual_window.log（旁边有自动JSON）
SHA256 f14934a791cf268b3daea71e21dadddbd8532ec757d1033ca0a4d572314d8f58
UART210.032秒，窗口结束且listen之后Ctrl-C关闭，尾30字节保留。interrupted=true/PTY退出码1只是UART关闭，不是panic或发送中断。
−16首/次窗口标记内223/299请求＋223/299拨轮，结束后各额外1完整零包。窗口结束rejected/bus_errors/tx_errors/rx_missed均0、TX_STATUS错误/失败最大0。两个脉冲区间各149帧行、0序号缺口，但整窗启动处有监听队列旧帧/缺口3130/2974，不能宣称整份无损或224/300轮全被UART记录。
logger_dropped累计含监听刷屏；−16结束累计101769/137555，首replay STATUS之后增0，prior-listen→结束增935/1151含未知切换边界。driver计数和sent跨mode/window复位，不跨边界差分。self echo、成功事件位、submitted/idle不等于应用ACK或物理停止保证；物理停止来自用户反馈。第一停发段主要0x504，第二主要0x232，刷屏签名不固定，不当唯一指纹。
原始log/视频不编辑、不覆盖，分析另存；运行抓包不暂存，结束后核对原始字节/hash/Git属性。

四、最后已知板上与台架
板上应用：replay_gate0_probe_minus16_echo；应用0x10000、284912字节、esptool hash verified。运行CONFIG/PROBE_CONFIG/READY匹配−16，未测活体ELF身份。
bin SHA256 08ff3e8c0a44169925d9c5513de40c801898e5facb256f9f7ae86ac07b575bcc
ELF SHA256 0d3dfccdda6a069d4c47bb37b64e63cd1a4f00f49f21fd83a415202fcf02b58a
RST只监听，不自动发送；每次BOOT长按2秒会开启新窗口，先不要让用户重复按键。
最后用户确认板蓝/电机红/已停转；常亮还是闪烁未说明。采集已关闭，旧session94247已退出，不复用旧会话句柄。
最近枚举COM17=CH343，VID1A86:PID55D3，SER5C83447762；这是历史快照，使用前重新核实。本次交接文档整理时未发现相关采集/上传进程，没有新开串口或操作板子。
用户明确RS3一直物理未接，先前台架只有一块测试收发器和电机；新回复“通电了 rs3一直没连 情况都是正常的”。正常RS3监听时前置拨轮已设跟焦电机。当前准确电压、接线、灯色是动态信息，实验前核实；不反复问已确认的设备型号或历史设置。

五、备份与操作要求
−16备份：backups/probe_minus16_prepared_20261008_163117_512786
+16成功备份：backups/probe16_prepared_20261008_161123_477101
原常绿备份：backups/stable_50ms_candidate_20261008_153208_442438
备份已核验完整，只在本机、不入Git；若换机器，必须另带backups，Git克隆不能恢复它们。
保留TX_QUEUE_LEN=0、逐分片有界等待、RX同步和驱动生命周期保护、单路由、seq0。CAN单帧最多8字节，00:01为8+5、04:69为8+8+3。
04:69字段为整包零基索引15、16，小端int16，改值后重算CRC；−16实录包55130403e5bf00000004690f040819f0ff1b25。
AGENTS旧“IDF竞争根因已证明”措辞证据不足，仅保留已验证TX0规避；不重复NO_ACK、队列1、降速/退避等失败实验。
不用独立软件RTS复位，不用pio device monitor做无扰观察。串口DTR/RTS在open前false；上传用esptool自身下载时序、after=no_reset，先开采集再请用户RST。
先核对CONFIG、PROBE_CONFIG、READY/listen，再提示BOOT。此次用户自行在agent核验前RST/BOOT，日志启动配置是事后核验，不编造流程已经逐步执行。
官方R SDK通过云台（0E:12/0xAA/223↔222）与电机直连04:69不同，不能套其CAN地址或0–4095位置单位。

六、下一步与授权
本次±16方向对照已完成，不自动再发，也不重做已完成实验。下一阶段优先固定齿轮端视角、可见轴标记、整窗视频与CAN时序对应，逐窗口记录方向/位移/回零/物理停止，以量化重复性；之后再研究幅度/时长/最小必要报文集。下一阶段尚未新增候选代码或完成运动测量。
用户此前“可以继续”仅批准已结束的−16阶段；这条交接请求不批准新发送、更大幅值、更长时长或取消时限。
可以先自主完成离线分析、方案和候选核验，再简短确认具体新发送阶段和当前工况；获准后不重复索要同一授权，直接完成上传/采集准备并给明确步骤。不要只给空泛计划，也不要追问交接已有的信息。

七、工具、记录与Git
使用现有.venv/Scripts/python.exe；正式采集tools/capture.py，解析tools/analyze_protocol.py。
离线回归：.venv/Scripts/python.exe -m unittest discover -s tools -p test_replay_timing.py -v
采集GUI改动：.venv/Scripts/python.exe -m unittest discover -s tools -p test_gui.py -v（不误用真实串口）
解析改动：capture.py --self-check、analyze_protocol.py --self-check。固件改动编译对应环境；未变代码不为接续重复跑全部编译/实验。仅文档改动git diff --check，原始串口空白不修剪。
每次代码/分析/硬件反馈后更新HANDOFF，WORK_LOG追加北京时间/用户确认/证据/验证/未知/下一步，维护captures/INDEX。实现简单，优先现有工具与标准库；.pio辅助脚本仅本机忽略文件，不当版本库正式工具。
本会话普通sandbox exec/Node启动失败helper_unknown_error；经自动审查require_escalated可用，未改全局权限。新agent根据自己的实际工具状态处理，不凭旧故障盲目改权限。
用户已授权按需勤提交Git，有意义阶段本地提交，无需每次询问；只提交本轮内容，不推送远程、不改写历史。
核心成果最新提交：748aae5 Record negative motor pulse motion and opposite direction confirmation
此前：cf0a660（−16上传/准备）、097a65e（接续核验）、2d794e3（+16运动/−16候选）、70ba0af（常绿）。
本文整理前核验main干净、没有采集进程，未推送；交接文档及持续记录将另作本地提交，最新提交号以git log为准。本文没有新烧录、复位或CAN发送。

请读取后以最新成果继续，不把“尚未见运动/−16未上传”旧状态当当前，也不夸大为完整独立控制已完成。
