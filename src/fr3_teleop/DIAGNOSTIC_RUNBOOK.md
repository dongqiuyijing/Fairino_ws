# FR3 ServoCart 诊断部署记录（2026-09-28）

## 实际反馈诊断准备更新（16:10以后）

用户已将本次范围明确缩小为 actual-feedback-only：不以 target_TCP_pos 缺失阻断测试，不推断控制器目标是否变化。解码器保留原 valid（完整诊断）语义，另加 actual_fields_valid/actual_missing/target_available，绝不补造目标字段。数值有效与反馈新鲜分别检查，帧头count固定0不冒充计数递增。

重新检查发现原硬件与Manager均已退出，抓包仍在。现场再次确认安全并允许恢复后，现运行硬件 PID 78514、Manager PID 78865；未改SDK或运动参数。后续现场明确“无需重启”，未再次重启两臂。

恢复后已重新设置速度10、Manual、operator-stop。新日志位于 `/tmp/fr3-diagnostic-20260928/actual-feedback-run/`。只读监视器实际测到普通抓包到软件可见的年龄约52–457ms；不能将其按实时100ms新鲜门槛判为通过。因此请求现场开独立 `--immediate-mode` 抓包，不涉及SDK或机器人状态改变。

实际观察到现场曾重开原 tcpdump 命令，仍写 session.pcap，未带 --immediate-mode，导致原路径重写、原监视器读取停止推进。此前硬件CSV和解码JSONL保留，但不能声称原始pcap从最初启动起完整保留。新的配置溯源仅使用原解码日志中真实配置握手及首次成功反馈，对同一TCP四元组做校验；新抓包初始不完整片段明确标注为测试前连续性缺口，之后仍严格拒绝TCP缺口。不得借此伪造字段或忽略测试窗口内丢包。

新实际反馈安全检查模块 `fr3_teleop/actual_feedback_guard.py`：实际六关节、六维TCP有限值、机器人时间推进、两臂反馈年龄与诊断writer年龄<=100ms、安全/故障/使能字段完整且正常。缺目标TCP可接受；缺实际反馈、安全字段、时间不推进或过期不可接受。

单次程序 `/tmp/fr3-diagnostic-20260928/actual-feedback-run/single_trial.py`：默认仅检查，不创建命令发布器；`--execute-once --operator-clearance` 仅在现场明确开始后使用，固定Arm A / 实际Tool X+ / 10mm/s / scale=1 / 0.5s / 20Hz输入，目标约5mm。创建独占motion-permit-consumed文件后才允许非零，禁止自动重试。SIGINT/SIGTERM和异常进入停止分支，停止后继续观测1秒、重复发布停止而非运动。记录实际发布速度、采样时刻及反馈；完整SDK调用仍由硬件CSV保存。

软件中止阈值（不是硬件限位/独立保护）：A TCP总位移>7mm、反向投影<-0.3mm、侧偏>0.8mm、姿态变化>0.5deg、任一关节变化>1deg；B TCP变化>0.2mm或关节>0.05deg；停止200ms后继续移动>0.2mm；SDK错误、诊断丢记录/过期、ROS反馈过期或竞争输入。现场仍须核实实际Tool X+空间、人员和急停，并明确开始。本次未消耗运动授权。

当前在线检查仍未通过：即时抓包指定路径尚未出现。不可把离线测试通过或旧静止数据当作新的试验就绪声明。

## 现场启动与无运动验证更新（15:54–16:01，UTC+07）

下文“尚无抓包/未启动”是实施前记录，已由本节更新。
现场 tcpdump PID 63793 持续写入 session.pcap，配置握手已捕获。现场明确授权后已启动：

- 双臂 launch PID 65069，硬件 PID 65109，进程 maps 指向 `/tmp/fr3-diagnostic-20260928/install/fairino_hardware_dual/lib/libfairino_dual_hardware.so`；SDK 仍为原 `libfairino.so.2.3.7`。
- Teleop launch PID 65869，Manager PID 65954，其可执行文件、PYTHONPATH 首项和新生成诊断文件均来自隔离 overlay。
- 零运动设置脚本 `/tmp/fr3-diagnostic-20260928/prepare_zero.py` 仅允许 set_frame/set_speed/enable/stop，不含 motion 或速度发布器。已设置界面 base、速度10、进入 Manual 并发送 operator-stop。物理 mode 仍为2，界面语义不一致未在本轮修改。
- 当前 Arm A JTC inactive、Arm B JTC active，两个 teleop controller inactive。直接订阅确认硬件 MANUAL_CARTESIAN；Manager MANUAL_READY、motion=stopped、stage1_velocity=0；8秒161条状态 joint_feedback 全部 ONLINE，指令话题无其他发布者。
- 切换瞬间确实出现过 MANUAL_READY 与 joint_feedback=STALE 同时成立；随后8秒观察没有持续 STALE。不能据此认为持续状态验证缺陷已经修复。
- 已核查的硬件快照：16968次 ServoCart 返回，返回码全部0，非零调用0；10次 reason=watchdog、其余 input_zero。这是零输入基线，不是非零运动被打断的证据。最大调用包围耗时约11.824ms。
- 两臂 CNDE 配置周期8ms，解析成功、无记录到的 TCP 缺口/配置拒绝；各取625帧约5秒窗口，机器人时间持续推进，捕获最大相邻间隔约10.2ms。实际TCP平移各轴峰峰值 A<=0.011mm、B<=0.005mm，符合静止小幅反馈波动。SDK关节读取和网络反馈量级/数值相符，但尚未做逐帧精确时间配对。

关键缺口：两臂配置都不含 `target_TCP_pos`，因此解码 `valid=false`（缺必要字段），不是实际关节/TCP无法解析。帧头count始终0，不能将frame_delta=0当成反馈冻结；以机器人时间、TCP序列和捕获时间判新鲜。Arm A机器人日期是2015年但递增正常，不能直接与主机墙钟对齐。

没有执行非零运动；用户表示可测试且现场急停可用，但目标反馈缺失仍使原定完整诊断条件不成立。

新增SDK证据：当前二进制 FRRobot::AddRobotRealtimeState @0xa2742 调用 FRCNDEClient::AddCNDEState @0x84cf2；该函数检查状态并向本地vector push_back后返回，未调用SendCNDEOutputConfig。SetCNDEStateConfig @0x84b96 同样只更新本地vector/period。在现有反馈线程运行时直接调用，不能保证控制器重发新布局且有共享配置访问风险。因此未在在线进程热调用这些接口，也未增加并行 SDK 客户端。需要先验证安全的配置生效阶段及握手，再决定最小反馈补丁，不能仅凭返回0宣布配置成功。

保存的检查结果：`/tmp/fr3-diagnostic-20260928/passive-summary.json`、`zero-setup.json`、`ros-zero-check.json`，原始CSV/JSONL/pcap仍持续记录。非零运动根因仍未定位，零命令成功不构成运动能力验证。

## 当前状态与边界

已实现诊断代码和两项已批准的 Manager 输入修复；未执行运动、未修改 SDK、未修改 ServoCart 实验参数、未实施反馈错误生命周期补丁 B。

现场已明确允许启动两台机械臂。此确认不是运动测试的最终开始指令。
最新只读进程检查中，原 PID 41632 已不存在，未发现双臂 bringup、Manager 或到两臂的 TCP 连接。
这些现象不能代替现场静止/碰撞空间/急停检查。本代理没有执行停止旧进程的操作。

当前部署门槛：普通用户 tcpdump 无抓包权限，sudo -n 提示需要密码；尚无真实 CNDE 配置握手和有效反馈记录。
不得绕过这个门槛启动后直接运动。需要现场开被动抓包，再启动控制进程。

## 本次文件改动

- `fr3_teleop/manager.py`：实际发布 core 已限幅、乘 scale 的速度；普通 stop/release 先检查所有权，全局 operator-stop 保留。新增命令接收/处理结束/源拒绝/实际速度发布诊断。
- `config/teleop.yaml`：澄清 legacy default 注释，不改变参数值。
- `fr3_teleop/manual_trace.py`：独立落盘线程和有界队列；记录单调时钟和实时钟对照、序号、丢记录数。
- `fairino_hardware_dual/.../manual_trace.hpp`：预分配有界队列；生产者不等待、无分配、无文件/网络操作；并发生产者冲突或队列满时丢诊断记录并计数。
- 硬件接口 `.hpp/.cpp`：接收序号、输入年龄、写周期、清零分类、ServoCart 调用前及返回、会话调用前及返回、已有关节读取结果、实际 ServoJ 调用记录。不新增 SDK 读取或连接。
- `fr3_teleop/cnde_trace.py`：被动解析 pcap 中已有 20005 连接的 CNDE 数据。SDK Python 文件只经 AST 读取类型映射，绝不导入/执行。
- 测试与 CMake：新增诊断测试、输入修复回归测试。

原有脏工作区修改全部保留；不能把当前 git diff 的全部内容误认为本次修改。

## 时序、有效性与限制

`FR3_TRACE_DIR` 未设置时不启动诊断线程。启用时硬件文件按 IP/PID 区分；Manager 写 JSONL。
两条硬件队列分别记录输入和控制事件；文件行不保证跨队列时间排序，分析必须按 `begin_ns` 排序，不能按行号推断连续性。

硬件 CSV 的 kind：

| 值 | 含义 |
|---|---|
| 1 | 硬件收到速度，rx_seq、原始值 p0、限幅后 vx |
| 2 / 8 | ServoCart 返回 / 调用前；同一 begin_ns 配对 |
| 3 | 原有 GetActualJointPosDegree 结果；p0..5 为度，ret 非零时数值无效 |
| 4 / 9 | ServoMoveStart 返回 / 调用前 |
| 5 / 10 | ServoMoveEnd 返回 / 调用前 |
| 6 | ServoJ 返回，不能再靠字符串 servoJ=0 排除混发 |
| 7 | write 周期入口和 runtime mode |

CSV 固定参数写入头部：pos_gain 六个 1、exaxis 四个 0、acc=0、vel=60、filterT=0、gain=0。
mode、float 转换后的 cmdT、六维 pose 每次记录。完成计数与调用前计数分别分析；若卡在 SDK 内，只有调用前记录，不能算成已返回。
begin/end 包围诊断入队和 SDK 调用，耗时包含入队的微小开销，不是控制器执行耗时。
health 中计数为已返回 ServoCart 总数/非零数/零数。丢记录非零或文件停止更新时，不得声称完整时间序列。

清零 reason：0 非零输入、1 输入为零、2 未收到输入、3 看门狗过期、4 SDK 错误抑制、5 夹爪占用、6 退出边界。
此分类是该周期决策；需关联此前命令和状态事件判断原因，不把 stale 且 vx=0 一概视为打断非零运动。

被动反馈必须来自同一连接捕获的配置及成功应答。未知字段、长度不符、TCP 缺口/乱序、帧损坏都拒绝继续解码，不猜偏移。
此最小解码器保守拒绝乱序，不是通用 TCP 重组器；发生拒绝时保留 pcap，但本次数据不满足在线测试门槛。
`valid` 仅表示字段完整且数值有限，不代表新鲜。还必须验证 frame_delta 连续、robot_time_changed 持续为真和抓包时间持续推进。
至少需要 actual_joint_pos、actual_TCP_pos、target_TCP_pos、robot_time；若控制器原有配置缺少字段，不得拿 last_servoJ_target 冒充 ServoCart TCP 目标。
缺字段时停止准备并单独评审最小反馈配置变更，不修改运动参数。

SDK 关节读取 ret=0 只说明缓存读取成功，不证明反馈更新。使用网络反馈帧和机器人时间进行交叉验证。
pcap 是实时钟，Manager/硬件是 CLOCK_MONOTONIC。通过 Manager health 的双时钟样本做最近邻偏移关联，检查时钟跳变；不得用离线解码时刻替代采样时刻。
部署前还须用静止数据对照已有 SDK 关节反馈，验证类型映射和单位；不通过不能运动。

## 已有证据

Arm A 只读 GetSoftwareVersion 返回：FR3-V1-001(V6.0)、v3.9.7、V3.9.33-QX。
官方 C++ 版本记录显示 V3.9.3 给 ServoCart 增加扩展轴参数：
https://fairino-doc-zhs.readthedocs.io/latest/SDKManual/CPPVersionIntro.html
因此目前没有足够依据删除 exaxis 或更换 SDK。这不是控制器内部执行正确性的证明。

已检查 SDK ServoCart 返回路径：成功返回 XML-RPC 整数结果，并不等于完成了实际位移。
不运动根因仍未确定；输入缺陷修复不能被冒充为根因修复。

## 离线验证及恢复材料

隔离构建目录：`/tmp/fr3-diagnostic-20260928/build`、`install`。没有覆盖原工作区 build/install 中的硬件共享库。
部署采用该隔离 overlay；硬件回退可在停止新进程后重新使用原 overlay。Manager 的原安装是源码链接，单纯切回原 overlay 不会恢复旧 Python：必须以源快照另建隔离回退包，核对导入路径，不能覆盖用户后续修改。快照代表诊断实施前状态（已包含输入修复），不声称能够逐字节恢复此前进程内的 Python。不得同时运行两套硬件或 Manager。

原始材料位于 `/tmp/fr3-diagnostic-20260928/`：

- `preimplementation.patch`、`preimplementation-source.tar.gz`：诊断实施前的完整脏状态，包含当时已完成的输入修复。
- `postimplementation.patch`、`postimplementation-source.tar.gz`：最终 tracked diff 和包括 untracked 的源代码快照。
- `pytest.xml`、`trace-tests.xml`、`cartesian-tests.xml`、各次构建日志。

Teleop 全套测试 99 项通过（包含离线 trace 与 pcap 合成帧测试）。
原硬件 Cartesian 测试 9/10 通过；唯一失败是遗留断言 mode==1，而用户指定保留的实验代码 mode==2。没有为使测试变绿而改变控制条件。
新增诊断测试单独覆盖当前 mode=2、动态 cmdT、dx 计算、队列溢出/并发顺序、落盘计数。
这些测试不等于真机实时性、真实协议布局或停止能力验证。

## 现场下一步

现场终端先运行（sudo 密码只在现场输入）：

```bash
sudo tcpdump -i any -n -s 0 -U -w /tmp/fr3-diagnostic-20260928/session.pcap '((host 192.168.58.2 or host 192.168.58.5) and tcp and (port 20003 or port 20005))'
```

确认监听已启动且落盘路径可读后，再加载隔离 overlay 启动唯一一套双臂 bringup 和 Manager，设置 FR3_TRACE_DIR。
启动会对两臂执行既有 ResetAllError、RobotEnable、StopMotion、ServoMoveEnd/Start；Arm B 的 AUTO 会话也受影响。
启动后核查新 PID、/proc/maps、插件哈希、Manager 导入路径、trace 文件 PID、Controller、命令发布者；确认未加载旧代码。

解码工具只读文件：

```bash
PYTHONPATH=src/fr3_teleop python3 -m fr3_teleop.cnde_trace /tmp/fr3-diagnostic-20260928/session.pcap --sdk-source /home/cyberbraindualarm/fairino-python-sdk/linux/fairino/Robot.py --follow
```

先做零运动检查：Manual 状态、set_speed=10、scale=1 的离线链路与在线设置、连续有效实际/目标反馈、零丢记录、无其他指令源。
准备带正常停止/finally/信号退出处理的一次性测试程序；在上述检查完成前，不启动发布非零命令的程序。

最终展示参数并等待现场单独明确“开始”：Arm A，实际 Tool X+，mode=2，10 mm/s，0.5 s，名义约5 mm，其他参数不变，不自动重试。
现场核实 Tool X+ 无碰撞空间、急停可用、人员在场。软件阈值不是独立安全保护，也不保证硬件硬限位。
异常返回、反馈失效、方向错误、位置偏差或停止后仍运动立即中止；软件停止无效由现场执行机器人安全停止。

结束后保留 pcap、捕获丢包统计、全部 trace、进程/版本/Controller 快照和测试程序输出，逐项回答非零连续性、零增量穿插、帧/时间更新、实际/目标变化及返回值关系。未运动也不得修改参数后重试。
