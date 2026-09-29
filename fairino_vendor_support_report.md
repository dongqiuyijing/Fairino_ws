# FR3 ServoCart 连续增量执行不足问题报告

## 1. 环境与版本

- 机器人：FAIRINO FR3，Arm A，控制器 IP `192.168.58.2`
- Arm B：`192.168.58.5`，测试期间保持静止
- 机器人/控制器版本：`FR3-V1-001(V6.0)`、`v3.9.7`、`V3.9.33-QX`
- C++ SDK：Fairino C++ SDK v3.9.7
- 实际链接 SDK：`libfairino.so.2.3.7`
- Hardware Interface 动态库：
  `/home/cyberbraindualarm/fairino_ws/build/fairino_hardware_dual/libfairino_dual_hardware.so`
- Hardware Interface SHA-256：
  `c2e72f710ab075cc520cfe19f79a2762f01dc6fbb22edd968719d473af2c7fe7`

## 2. 实际调用方式

每次测试前，Hardware Interface 使用同一 FRRobot 对象调用：

```text
ServoMoveStart()
```

返回码为 `0`。运动期间调用：

```cpp
ServoCart(
    mode,
    &desc,
    exaxis,
    pos_gain,
    acc,
    vel,
    cmdT,
    filterT,
    gain)
```

实际参数：

```text
mode       = 2
DescPose   = [dx, 0, 0, 0, 0, 0]
             dx 约 0.05～0.11 mm，通常约 0.08 mm
pos_gain   = [1, 1, 1, 1, 1, 1]
exaxis     = [0, 0, 0, 0]
acc        = 0
vel        = 60
cmdT       = 实际周期，约 0.005～0.011 s，通常约 0.008 s
filterT    = 0
gain       = 0
```

SDK C++ 头文件中的函数声明为 `exaxis` 在 `pos_gain` 之前；但 XML-RPC wire 请求中参数顺序为 `pos_gain` 在 `exaxis` 之前。已通过 TCP 序列号重组确认 XML-RPC 请求和响应均完整，所有响应返回 `0`。请厂商核对该 C++ ABI 与 XML-RPC 协议顺序是否为当前版本的预期行为。

## 3. 两次测试结果

同一运行目录中记录了两个完整运动窗口：

```text
运行目录：/tmp/fr3-cnde-test-20260928-190547
Arm A CSV：/tmp/fr3-cnde-test-20260928-190547/trace/hardware_192.168.58.2_122136_37943287355620.csv
Manager JSONL：/tmp/fr3-cnde-test-20260928-190547/trace/manager_122475_38044669622488.jsonl
CNDE pcap：/tmp/fr3-cnde-test-20260928-190547/traffic.pcap
```

| 项目 | 测试 1 | 测试 2 |
|---|---:|---:|
| 运动时长 | 约 0.509 s | 约 0.510 s |
| 非零 ServoCart | 64 次 | 64 次 |
| 调用周期 | 约 8 ms | 约 8 ms |
| 单次 dx | 约 0.08 mm | 约 0.08 mm |
| 理论累计增量 | 约 5.1 mm | 约 5.1 mm |
| XML-RPC 响应 | 全部 `0` | 全部 `0` |

实际网络重组结果：两个窗口对应 128 个非零请求；在包含窗口边界的 pcap 时间包络中恢复到 130 个 ServoCart 请求和 130 个匹配响应，额外 2 个为边界相邻调用。

## 4. 控制器反馈

两个窗口内 CNDE 反馈均有效，RobotTime 持续推进。典型状态：

```text
program_state       = 2（运行）
robot_state         = 2（运行）
robot_mode          = 1（手动）
rbt_enable_state    = 1（已使能）
emergency_stop      = 0
safety_stop0_state  = 0
safety_stop1_state  = 0
collision_state     = 0
```

反馈结果：

```text
测试 1：TargetTCPPos 变化约 0.14～0.17 mm；ActualTCPPos 约 0.1 mm 级变化
测试 2：TargetTCPPos 变化约 0.14 mm；ActualTCPPos 约 0.1 mm 级变化
```

由于 mode=2 为工具坐标系增量，已将 TCP 工具 X 增量转换到同一笛卡尔坐标系后比较；目标变化模长仍远小于理论 5.1 mm。

`motion_queue_len` 在运动期间约为 `28～30`，停止后约为 `28～31`；`motion_done` 始终为 `0`。现有字段定义只说明这是通用运动命令队列长度，不能确认它是否专属于 ServoCart。

## 5. 已排除的问题

已排除或已有直接证据不支持：

1. ROS 命令未发送或 Manager 未接收；
2. Manager 在运动窗口内提前清零；
3. 看门狗在两个运动窗口内触发；
4. Hardware Interface 未调用 ServoCart；
5. ServoMoveStart 未建立；
6. XML-RPC 请求参数顺序、数值或类型与 C++ 调用记录不一致；
7. SDK 响应不是逐条匹配；
8. 机器人未使能、急停、安全停止或碰撞状态阻止执行。

## 6. 请厂商明确回答

1. 在当前 FR3 固件和 v3.9.7 SDK 中，`mode=2` 的连续 ServoCart 增量如何累积和执行？
2. `mc_queue_len` 是否包含 ServoCart 指令？保持在 `28～31` 是否正常？
3. `TargetTCPPos` 表示当前正在执行的目标、当前控制目标，还是队列末端目标？
4. 当前固件是否支持约 8 ms 周期、动态 `cmdT` 的 XML-RPC ServoCart？
5. C++ 函数签名中 `exaxis` 位于 `pos_gain` 之前，而 XML-RPC 请求中 `pos_gain` 位于 `exaxis` 之前，这是否符合当前协议？
6. 在 64 次非零请求、理论累计约 5.1 mm，但目标/实际 TCP 仅变化约 0.1～0.17 mm 的情况下，控制器端可能有哪些原因？
7. 是否存在直接查询 ServoCart 已接收数量、已执行数量、当前队列尾端目标或 ServoCart 执行状态的接口？
8. `motion_done=0` 且 `mc_queue_len` 长时间保持非零时，是否表示 ServoCart 队列尚未消费，还是这些字段不适用于 ServoCart？

## 7. 最小证据附件

建议只提交以下材料，不提交完整 1.1 GB pcap：

1. 本报告；
2. Arm A Hardware CSV 中两个窗口对应的 `kind=8` ServoCart 调用和 `kind=2` 响应行；
3. Manager JSONL 中两个 `motion`、`velocity_published` 和 `operator-stop` 时间线；
4. CNDE 解码后的两个窗口反馈摘要，包含 `TargetTCPPos`、`ActualTCPPos`、`target_TCP_vel`、`motion_queue_len`、`motion_done`、`RobotTime` 和安全状态；
5. 每个窗口各选一条完整 XML-RPC ServoCart 请求及对应返回 `0` 的文本片段；
6. 当前 v3.9.7 `robot.h` 中的 ServoCart 声明和 Hardware Interface 调用片段。

原始证据保留在本机运行目录中。如需提供 pcap，应仅导出两个运动窗口及其前后少量 TCP 流量，并先检查是否包含账号、密码、控制器内部地址或其他敏感字段。目前已确认的 payload 主要包含 IP、XML-RPC 方法和运动参数，仍应由现场人员在发送前复核。

## 8. 当前结论

目前没有足够证据认定是 C++ SDK 参数编码错误，也没有足够证据认定 `mc_queue_len` 非零就是 ServoCart 队列堵塞。

决定性缺口是：控制器没有公开 ServoCart 队列尾端目标或已执行计数，官方资料也没有明确 `mc_queue_len`、`TargetTCPPos` 对连续 ServoCart 的具体语义。

请厂商先确认上述接口和字段含义，再决定是否需要进一步测试或控制器日志。
