# 独立FM03台架 −16符号对照结果（2026-10-08 18:49:13 北京时间）

本次对照已获用户“可以继续”授权，沿用无RS3、单测试收发器+电机台架。用户先确认“通电了 rs3一直没连 情况都是正常的”，后报告“都按完了 电机转了”，并补充“现在停了 逆时针 与上次相反 这次我也按了两次boot 现在板子恢复蓝灯电机恢复红灯了”。方向回答是在要求从齿轮端观察的语境中给出；没有与CAN同步的视觉时刻、角度测量或逐窗口单独动作结果。灯色已确认，常亮/闪烁未说明。

## 结论

在当前台架和既有00:01＋04:69报文组合下，−16有实际转动正向证据，用户明确报告逆时针、与此前+16相反，并确认现在已停转。这支持候选小端int16字段的符号与运动方向关联。仍不能把16解释为角度/rpm，不能确定字段代表速度/位置/增量，也未完成任意运动控制、重复位移量和最小必要报文集。回零包与停止有对应观察，但没有隔离“回零”和“停发”对物理停止的因果。

## 启动与原始证据

- 固件replay_gate0_probe_minus16_echo，应用0x10000上传284912字节/hash verified/after=no_reset。bin08ff3e8c0a44169925d9c5513de40c801898e5facb256f9f7ae86ac07b575bcc，ELF0d3dfccdda6a069d4c47bb37b64e63cd1a4f00f49f21fd83a415202fcf02b58a；只把+16改为−16。成功+16、常绿镜像备份保留。
- captures/20261008_184054_626320_probe_minus16_manual_window.log及自动JSON，18:40:54.626320+08:00开始，UART210.032秒。两窗口都已结束并回listen后Ctrl-C关闭；interrupted=true/PTY退出码1为UART采集停止，不是固件panic。原始SHA256 `f14934a791cf268b3daea71e21dadddbd8532ec757d1033ca0a4d572314d8f58`，尾部30字节保留，不修剪。
- 1START/1READY、2手动开始/2window_done、1stopped_by_button，0AUTO_START/panic/异常Rebooting/WARN/ERROR。用户确认BOOT两次，首提前结束的具体按键动作及时刻仍未知，不能用“按两次”擅自解释成仅按下两次或两次等幅运动。
- CONFIG为TX0/echo1/delay0/legacy0/seq0/sync_rx1；START为stage4/1Mbps/15秒/50ms；PROBE_CONFIG为value−16/start5000/duration200/auto_start0，READY listen_only。用户在agent完成启动核验前自行RST/BOOT，因此本次是事后核对配置，不捏造曾先核验再发BOOT指令；后续继续先核验READY再开始。上传/CONFIG相符不是活体ELF测量。

## 分窗口验证

第一窗口被BOOT提前结束，window_done sent224/rejected0，标记内223请求＋223拨轮（219零/4个−16），结束后另1完整零拨轮。软件转换elapsed5015→5220ms，非零首片24.278144s→首回零24.483144s，205.000ms。

第二窗口完成15秒，window_done sent300/rejected0，标记内299请求＋299拨轮（295零/4个−16），结束后另1完整零拨轮。软件转换elapsed5009→5209ms，非零首片42.332144s→首回零42.532144s，200.000ms。

两段非零包均为`55130403e5bf00000004690f040819f0ff1b25`，04:69是19字节/8+8+3，00:01是13字节/8+5。逐ID重组、CRC8/CRC16及独立build_packet逐字节核对通过。两个脉冲区间各149完整帧行，0序号缺口；发送窗口结束bus_errors/tx_errors/rx_missed/rejected均0，TX_STATUS错误/失败最大0、pending最大1。每段final_zero_submitted1/final_tx_idle1，额外完整零包已自收；不是应用ACK或物理停止保证，物理停止证据来自用户。

200ms是软件选择区间，首个窗口首次调度到回零发生在5220ms，实录205ms不隐藏。上述时间是驱动取帧/自收时间，不是物理帧边沿或实际电机运动持续时间。

## 计数及丢帧边界

两窗口标记内序号缺口3130/2974，含开始前监听队列积压，不能宣布全份日志或224/300调度轮数全被串口记录。首replay STATUS之后logger_dropped增量均0；window_done累计101769/137555继承监听丢帧。最后prior-listen STATUS→window_done的logger增935/1151包含未知开始前/切换边界，不全算主动发送损失。driver和sent跨mode/window复位，不做跨界无意义减法。

全文件162071帧行/415633缺口/0序号回退，不代表总线物理总帧数。第一停发后的非自收帧主要0x504，第二主要0x232，不能把刷屏签名当唯一指纹。结束listen bus_errors为12/15，不能归入主动发送期或单凭它推ACK根因。以TX0已验证规避为事实，不声称IDF竞争根因已证。

## 下一步

先保全当前±16方向证据，下一阶段优先固定齿轮端视角、加可见轴标记、整窗视频与CAN对应，每窗口单独记录方向/位移/停止；复用现有±16手动镜像及全部时限，暂不增大值或时长。既有目视方向相反已有新证据，不要求重做本轮。后续同步测量用于重复性和量化，另行确认新发送阶段，不自动重开窗口。详细机器数据见20261008_probe_minus16_motion_result.json；本机.pio/analyze_minus16_result.py仅一次性分析助手，不作为版本库正式工具。
