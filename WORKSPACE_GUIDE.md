# FAIRINO FR3 Workspace Guide

本文档是对 `~/fairino_ws` 的只读梳理结果（READ → UNDERSTAND → MAP → DOCUMENT）。  
生成日期：2026-09-11。  
**不修改现有功能代码。** 启动命令均来自实际 `package.xml` / `setup.py` / `CMakeLists.txt` / launch 文件，并用 `ros2 pkg executables` 验证过。

---

## 1. Project Goal

最终系统：

```text
FR3 机械臂 + 末端双指夹爪 + 末端深度相机
```

自动完成：深度相机找零件 → 计算位姿 → FR3 抓取 → 固定检测位置 P1 → Face1/Face2/Face3 依次朝 D1。  
第二阶段才会加入第二台机械臂检测另外三面。

**当前尚未进入深度相机阶段。**  
仿真侧已做到已知位姿抓取 + P1 三面朝向。  
真机侧已打通 MoveIt + ros2_control，但 Stage 1~5 任务程序尚未完整放到真机上执行。真机夹爪 backend 仍是占位实现。

---

## 2. Workspace Packages

在干净环境中执行：

```bash
cd ~/fairino_ws
source /opt/ros/humble/setup.bash
source ~/fairino_ws/install/setup.bash
colcon list --base-paths ~/fairino_ws/src
```

当前 **实际已编译进 `~/fairino_ws/install` 的包** 只有：

| 已安装 package | 说明 |
| --- | --- |
| `fairino3_gazebo` | 自定义 Gazebo 仿真 |
| `fairino3_v6_moveit2_config` | FR3 MoveIt 配置（官方包，已加入真机 launch） |
| `fairino_description` | 官方 URDF/网格（已加入 HKV 夹爪 xacro） |
| `fairino_hardware_v3_9_7` | 当前真机 hardware 插件 |
| `fairino_msgs` | 官方消息 |
| `fr_control` | 自定义任务层 |
| `fr3_part_demo` | 早期 C++ 翻面演示，非当前主路径 |
| `gz_ros2_control` | 第三方 Gazebo↔ros2_control 桥 |

源码目录里还有大量 **未安装** 的官方机型 / 旧 hardware 版本，见下表。

### 2.1 Package Table

| Package | 来源 | 主要用途 | Gazebo | MoveIt | 真机 | Stage 任务 |
| --- | --- | --- | --- | --- | --- | --- |
| `fairino_description` | 官方（夹爪为后加） | FR3 URDF、网格、HKV 夹爪模型 | 是（被 SIM xacro include） | 是 | 是 | 否 |
| `fairino_msgs` | 官方 | FAIRINO 服务/消息 | 否 | 否 | 是（hardware 依赖） | 否 |
| `fairino3_v6_moveit2_config` | 官方 MoveIt 包 + 自定义真机文件 | SRDF、kinematics、move_group、RViz、真机 bringup | SIM 用其 move_group/RViz | **主配置** | **主配置** | 间接 |
| `fairino5/10/16/20/30/3mt_v6_moveit2_config` | 官方 | 其它机型 MoveIt，本项目不用 | 否 | 否 | 否 | 否 |
| `fairino_hardware` | 官方旧桩 | 仅 `ros2_cmd_server` 版本节点，**未安装** | 否 | 否 | 否 | 否 |
| `fairino_hardware_master` 及 `v3_8_2_11`~`v3_9_9`（除 3.9.7） | 官方多版本备份 | 同名插件源码，**当前未安装** | 否 | 否 | 否 | 否 |
| `fairino_hardware_v3_9_7` | 官方（当前选用） | `FairinoHardwareInterface` + `ros2_cmd_server` | 否 | 间接 | **是** | 否 |
| `gz_ros2_control` | 第三方（ros-controls） | Gazebo `GazeboSimSystem` 插件 | **是** | 间接 | 否 | 否 |
| `gz_ros2_control_demos` / `ign_ros2_control*` | 第三方 demo | 示例，本项目不跑 | 否 | 否 | 否 | 否 |
| `fairino3_gazebo` | **自定义** | Gazebo world、spawn、夹爪物理、controllers.yaml | **是** | SIM 用其 overlay | 否 | Stage 2~4 环境 |
| `fr_control` | **自定义** | MoveIt 任务层、夹爪、抓取、Stage 4 | 调用 SIM | **是** | 可复用运动层 | **Stage 1~4** |
| `fr3_part_demo` | **自定义早期** | C++ `part_rotate_demo`，硬编码翻面 | 可连 MoveIt | 是 | 不建议 | 被 Stage 4 取代 |

---

## 3. System Architecture

任务程序不直接连电机。统一走：

```text
任务 Python（fr_control）
        ↓  MoveGroup action / Cartesian service
move_group
        ↓  FollowJointTrajectory
fairino3_controller
        ↓  ros2_control hardware plugin
SIM: gz_ros2_control/GazeboSimSystem → Gazebo FR3
REAL: fairino_hardware/FairinoHardwareInterface → 真实 FR3 ServoJ
```

夹爪是另一条链，**不经过 MoveIt arm group**：

```text
任务 Python  create_gripper(backend=...)
        ↓
SIM:  /gripper_left_controller/gripper_cmd
      /gripper_right_controller/gripper_cmd
      → effort PID → Gazebo 独立指关节
REAL: RealFairinoGripper  （NotImplementedError，尚未实现）
```

规划组与末端（SIM/REAL 共用 SRDF）：

- planning group：`fairino3_v6_group`（j1~j6）
- EE link：`gripper_tcp`
- 夹爪 SRDF group：`gripper`（`gripper_joint`）
- Gazebo 物理驱动关节：`rail_2_slider_l` / `rail_2_slider_r`

---

## 4. Simulation Architecture

日常入口是一条命令：

```text
ros2 launch fairino3_gazebo sim_full.launch.py
```

它只 **include** 现有 launch，不另起控制栈。内部顺序：

```text
sim_full.launch.py
├─ sim.launch.py
│     Gazebo Fortress（inspection_world.sdf）
│     robot_state_publisher（fairino3_gazebo.urdf.xacro）
│     spawn fairino3
│     ros_gz_bridge（/clock, attach/detach）
│     spawner: joint_state_broadcaster,
│              fairino3_controller,
│              gripper_left/right_controller
└─ 等待 moveit_delay（默认 8 s）
      ├─ move_group.launch.py（官方 move_group + Gazebo 超时放宽）
      └─ moveit_rviz.launch.py（可用 use_rviz:=false 关掉）
```

Stage 4 世界不同，用 `ros2 launch fr_control stage4_full.launch.py`。  
分终端启动仍然可用，不必删。

SIM `robot_description` 来自：

```text
fairino3_gazebo/urdf/fairino3_gazebo.urdf.xacro
├── fairino_description/urdf/fairino3_v6.urdf          # 官方 FR3
├── world → base_link 固定关节
├── hkv_gripper_physics.xacro                          # 独立双指物理夹爪
├── gz_ros2_control.xacro                              # GazeboSimSystem
└── gazebo plugin: gz_ros2_control::GazeboSimROS2ControlPlugin
        parameters = fairino3_gazebo/config/controllers.yaml
```

Stage 4 世界 **不使用** `inspection_world.sdf`，而由 `stage4_sim.launch.py` 按 YAML 生成临时 SDF，并 `enable_grasp_weld:=true`。

---

## 5. Real Robot Architecture

一键启动（会运动真实 FR3）：

```text
fairino3_v6_moveit2_config/launch/real_bringup.launch.py
├─ static TF  world → base_link     （real_installation.yaml）
├─ robot_state_publisher            （fairino3_v6_robot.real.urdf.xacro）
├─ ros2_control_node                （ros2_controllers_real.yaml）
│     hardware 初始 unconfigured: FR3RealSystem
├─ hardware_spawner --activate FR3RealSystem
│     → FairinoHardwareInterface::on_activate()
│        RPC / 同步关节 / RobotEnable(1) / ServoMoveStart()
├─ joint_state_broadcaster
├─ fairino3_controller              （fairino3_controller_real.yaml）
├─ move_group                       （allow_trajectory_execution=True）
└─ rviz2                            （config/moveit.rviz）
```

REAL `robot_description` 来自：

```text
fairino3_v6_robot.real.urdf.xacro
├── fairino_description/urdf/fairino3_v6.urdf
├── hkv_gripper.xacro               # 运动学/可视化；mimic 滑块
└── fairino3_v6_robot.real.ros2_control.xacro
        ros2_control name="FR3RealSystem"
        plugin=fairino_hardware/FairinoHardwareInterface
        仅 j1~j6，无夹爪 command interface
```

真机控制器 IP **写死在 C++ 头文件**，不是 YAML：

```text
fairino_hardware_v3_9_7/include/fairino_hardware/fairino_hardware_interface.hpp
#define CONTROLLER_IP_ADDRESS "192.168.58.2"
```

辅助/分步 launch（调试用，不是日常一键）：

| 文件 | 作用 |
| --- | --- |
| `real_hardware_check.launch.py` | 只起 hardware + controllers，不起 MoveIt |
| `real_move_group_check.launch.py` | 只起 move_group，**禁止执行**（`allow_trajectory_execution=False`） |
| `real_move_group_execute.launch.py` | 只起可执行的 move_group |
| `real_moveit_rviz_check.launch.py` | 只起 RViz |

`ros2_cmd_server`（XML-RPC 命令 API）是 `fairino_hardware_v3_9_7` 的另一个可执行文件，**`real_bringup` 不会启动它**。

---

## 6. Important Files

### 6.1 Launch（本项目真正会用的）

| 完整路径 | Gazebo | MoveIt | RViz | hardware | URDF | controller YAML |
| --- | --- | --- | --- | --- | --- | --- |
| `src/fairino3_gazebo/launch/sim.launch.py` | 是 | 否 | 否 | GazeboSimSystem | `fairino3_gazebo.urdf.xacro` | `fairino3_gazebo/config/controllers.yaml` |
| `src/fairino3_gazebo/launch/move_group.launch.py` | 否 | 是 | 否 | 无（连已有 controller） | 官方 MoveItConfigsBuilder 默认 URDF（参数内） | 官方 `moveit_controllers.yaml` |
| `src/frcobot_ros2/fairino3_v6_moveit2_config/launch/moveit_rviz.launch.py` | 否 | 否 | 是 | 无 | MoveIt 配置 | 无 |
| `src/frcobot_ros2/fairino3_v6_moveit2_config/launch/move_group.launch.py` | 否 | 是 | 否 | 无 | 默认 mock xacro | `moveit_controllers.yaml` |
| `src/frcobot_ros2/fairino3_v6_moveit2_config/launch/demo.launch.py` | **否** | 是 | 是 | **mock GenericSystem** | `fairino3_v6_robot.urdf.xacro` | `ros2_controllers.yaml` |
| `src/frcobot_ros2/fairino3_v6_moveit2_config/launch/real_bringup.launch.py` | 否 | 是 | 是 | FairinoHardwareInterface | `fairino3_v6_robot.real.urdf.xacro` | `ros2_controllers_real.yaml` + `fairino3_controller_real.yaml` |
| `src/fr_control/launch/pose_sequence.launch.py` | 否 | 调用已有 | 否 | 无 | 无 | 无 |
| `src/fr_control/launch/gripper_test.launch.py` | 否 | 否 | 否 | 无 | 无 | 连已有夹爪 action |
| `src/fr_control/launch/grasp_test.launch.py` | 否 | 调用已有 | 否 | 无 | 无 | 无 |
| `src/fr_control/launch/stage4_sim.launch.py` | 是（include sim） | 否 | 否 | Gazebo | 同 sim | 同 sim |
| `src/fr_control/launch/stage4.launch.py` | 否 | 调用已有 | 否 | 无 | 无 | 无 |
| `src/fr_control/launch/gripper_contact_test.launch.py` | 否 | 否 | 否 | 无 | 无 | 夹爪接触实验 |
| `src/fr_control/launch/collision_drop_test.launch.py` | 否 | 否 | 否 | 无 | 无 | 碰撞掉落实验 |

官方其它机型 `demo.launch.py` / `spawn_controllers.launch.py` / `rsp.launch.py` / `warehouse_db.launch.py` / `setup_assistant.launch.py` 不是本 FR3 项目日常入口。

### 6.2 URDF / Xacro include 关系

**SIM**

```text
fairino3_gazebo.urdf.xacro
├── FR3: fairino_description/urdf/fairino3_v6.urdf
├── gripper: hkv_gripper_physics.xacro（独立双指，effort）
├── gazebo tags: GazeboSimROS2ControlPlugin + 可选 DetachableJoint
└── ros2_control: gz_ros2_control.xacro → GazeboSimSystem
```

**REAL**

```text
fairino3_v6_robot.real.urdf.xacro
├── FR3: 同一个 fairino3_v6.urdf
├── gripper: hkv_gripper.xacro（mimic 运动学模型，无真机驱动）
├── 无 Gazebo plugin
└── ros2_control: fairino3_v6_robot.real.ros2_control.xacro → FairinoHardwareInterface
```

**官方 MoveIt demo / mock（不要当 Gazebo 仿真用）**

```text
fairino3_v6_robot.urdf.xacro
├── 同一个 FR3 + hkv_gripper.xacro
└── fairino3_v6_robot.ros2_control.xacro → mock_components/GenericSystem
```

相同点：FR3 连杆/关节名、`gripper_tcp`、MoveIt group `fairino3_v6_group`。  
不同点：夹爪物理实现、hardware plugin、是否有 `world` 固定关节、是否有夹爪 controller。

兼容包装（不要当主文件改）：

- `fairino3_gazebo/urdf/gazebo_gripper.xacro` → 转调 `hkv_gripper_physics.xacro`
- `fairino3_v6_moveit2_config/config/virtual_gripper.xacro` → 转调官方 `hkv_gripper.xacro`

### 6.3 Python 任务文件

#### Robot Motion

| 路径 | 功能 | 输入 | 输出 | 依赖 | 谁调用 | SIM/REAL | 是否继续用 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `fr_control/moveit_arm.py` | 向已运行的 move_group 发 Plan/Execute、Cartesian | 关节角或 Pose | 轨迹执行 | move_action, execute_trajectory, compute_cartesian_path | pose_sequence / grasp_test / stage4 | **共用** | **是，核心** |
| `fr_control/pose_sequence.py` | Home → Pose A → Pose B → Home | `poses.yaml` | MoveIt 运动 | MoveItArm | `pose_sequence` executable | 共用（默认 use_sim_time=true） | **Stage 1 正式入口** |
| `fairino3_gazebo/joint_demo.py` | 绕过 MoveIt，直接 FollowJointTrajectory | 硬编码关节 | Gazebo 臂动 | fairino3_controller | `joint_demo` | SIM ONLY | 早期调试，非正式任务 |
| `fairino3_gazebo/joint_axis_test.py` | 逐轴 ±11.5° | 硬编码 Home | 单轴验证 | fairino3_controller | `joint_axis_test` | SIM ONLY | 调试用 |

#### Gripper

| 路径 | 功能 | 输入 | 输出 | 依赖 | 谁调用 | SIM/REAL | 是否继续用 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `fr_control/gripper.py` | `open/close/move`；工厂 `create_gripper` | backend=gazebo/real | 夹爪动作 | 左右 GripperCommand action | gripper_test / grasp / stage4 | 接口共用；real 未实现 | **是** |
| `fr_control/gripper_test.py` | OPEN→CLOSE→OPEN→MID→OPEN | 参数 backend | 日志 | GripperInterface | `gripper_test` | SIM（gazebo） | **Stage 2 正式入口** |
| `fr_control/gripper_contact_test.py` | 臂静止，夹爪夹 20×40×20 mm 块 | Gazebo spawn SDF | 接触/stall 日志 | gripper + ign | `gripper_contact_test` | SIM ONLY | 夹爪物理实验 |
| `fr_control/collision_drop_test.py` | 向静止夹指扔盒子 | ign spawn | 碰撞观察 | TF + ign | `collision_drop_test` | SIM ONLY | 碰撞实验 |

#### Grasp

| 路径 | 功能 | 输入 | 输出 | 依赖 | 谁调用 | SIM/REAL | 是否继续用 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `fr_control/grasp_test.py` | Home→开爪→Pre-Grasp→直线接近→闭合→attach→Lift | `grasp.yaml` | 抬起零件 | MoveItArm, gripper, planning_scene, sim_grasp | `grasp_test` | 逻辑可共用；默认 SIM | **Stage 3 正式入口** |
| `fr_control/grasp_poses.py` | 由物体位姿算 pre-grasp / grasp / lift | 物体 Pose + offset | GraspPoses | 四元数数学 | grasp_test, stage4 | **共用** | **是** |
| `fr_control/sim_grasp.py` | Gazebo DetachableJoint weld | backend gazebo/none | attach/detach topic | `/fr3/grasp/*` | grasp_test, stage4 | SIM ONLY（real 用 none） | SIM 翻面需要 |
| `fr_control/planning_scene.py` | 桌子/零件 collision、attach | 尺寸与 Pose | PlanningScene | ApplyPlanningScene | grasp_test, stage4 | **共用** | **是** |
| `fairino3_gazebo/grasp_reset.py` | spawn 后 detach 初始焊死 | 无 | `/fr3/grasp/detach` | sim.launch 条件启动 | sim.launch（enable_grasp_weld） | SIM ONLY | Stage 4 环境 |

#### Inspection

| 路径 | 功能 | 输入 | 输出 | 依赖 | 谁调用 | SIM/REAL | 是否继续用 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `fr_control/stage4_inspection_test.py` | 抓取 + P1 上 Face1/2/3 | `stage4_config.yaml` | 报告 PASS/FAIL | 上述全部 + gazebo_world_pose | `stage4_inspection_test` | 逻辑可共用；验收读 Gazebo | **Stage 4 正式入口** |
| `fr_control/inspection_poses.py` | Face 法向对齐 D1；Object→TCP | P1, D1, face 定义, T_tcp_object | object/TCP Pose | grasp_poses | stage4 | **共用** | **是** |
| `fr_control/stage4_config.py` | 解析 stage4 YAML | YAML | Home/P1/面顺序等 | yaml | stage4, stage4_sim | 共用配置层 | **是** |
| `fr_control/stage4_world.py` | YAML → 临时 world SDF | stage4 YAML | SDF 路径 | tempfile | stage4_sim.launch | SIM ONLY | Stage 4 仿真 |
| `fr_control/gazebo_world_pose.py` | `ign topic` 读/写模型位姿 | 模型名 | Pose | IGN_PARTITION=fairino3_gazebo | stage4 验收 | **SIM ONLY** | 真机需换成相机/TF |

#### Utilities

| 路径 | 功能 | SIM/REAL | 是否继续用 |
| --- | --- | --- | --- |
| `fr_control/constants.py` | group、TCP、夹爪关节/action 名 | 共用常量；夹爪 action 是 SIM 名 | 是 |
| `fr_control/preflight.py` | 检测重复 move_group / 异常 TCP 高度 | SIM 更关键 | 是 |

#### 不应再作为正式任务入口

| 路径 | 原因 |
| --- | --- |
| `fairino3_gazebo/pick_inspect_demo.py` | 硬编码关节；夹爪仍走已废弃的 `/gripper_controller/follow_joint_trajectory` |
| `fr3_part_demo/src/part_rotate_demo.cpp` | 早期 C++ 翻面；已被 Stage 4 Python 取代 |

---

## 7. Stage 1~5 Code Mapping

workspace **没有**名为 Stage 5 的独立代码。编号以源码注释和 README 为准，不为凑数虚构。

### Stage 1

目标：Python → MoveIt → FR3 自动运动（不拖 RViz 绿球）

对应文件：

- `fr_control/fr_control/pose_sequence.py`
- `fr_control/fr_control/moveit_arm.py`
- `fr_control/config/poses.yaml`
- `fr_control/launch/pose_sequence.launch.py`

如何运行（仿真环境已按第 9 节启动后）：

```bash
ros2 launch fr_control pose_sequence.launch.py
# 等价：ros2 run fr_control pose_sequence
```

### Stage 2

目标：Python → Gazebo 夹爪 OPEN → CLOSE → OPEN

对应文件：

- `fr_control/fr_control/gripper.py`（GazeboGripper）
- `fr_control/fr_control/gripper_test.py`
- `fr_control/launch/gripper_test.launch.py`

只需 Gazebo（`sim.launch.py`），**不需要** MoveIt：

```bash
ros2 launch fr_control gripper_test.launch.py
# 等价：ros2 run fr_control gripper_test
```

### Stage 3

目标：固定已知位置零件抓取（Pre-Grasp → Grasp → Close → Lift）

对应文件：

- `fr_control/fr_control/grasp_test.py`
- `fr_control/fr_control/grasp_poses.py`
- `fr_control/config/grasp.yaml`
- `fr_control/launch/grasp_test.launch.py`
- 世界：`fairino3_gazebo/worlds/inspection_world.sdf`
- `grasp.yaml` 中 `use_sim_attach: false`（纯摩擦，不焊）

```bash
# 先 sim.launch.py + move_group.launch.py
ros2 launch fr_control grasp_test.launch.py
# 等价：ros2 run fr_control grasp_test
```

### Stage 4

目标：抓取后把零件中心保持在 P1，Face1/2/3 依次朝 D1

对应文件：

- `fr_control/fr_control/stage4_inspection_test.py`
- `fr_control/fr_control/inspection_poses.py`
- `fr_control/config/stage4_config.yaml`
- `fr_control/launch/stage4_sim.launch.py`（按 YAML 生成世界）
- `fr_control/launch/stage4.launch.py`
- `fr_control/README_STAGE4.md`
- `use_sim_attach: true`（翻面用 DetachableJoint）

```bash
# Terminal 1
ros2 launch fr_control stage4_sim.launch.py \
  config_file:=$HOME/fairino_ws/src/fr_control/config/stage4_config.yaml

# Terminal 2
ros2 launch fairino3_gazebo move_group.launch.py

# Terminal 3
ros2 launch fr_control stage4.launch.py \
  config_file:=$HOME/fairino_ws/src/fr_control/config/stage4_config.yaml
```

### Stage 5

**不存在独立 Stage 5 程序。**  
没有 `stage5*` 文件。当前最高任务层就是 Stage 4。  
`pick_inspect_demo` / `part_rotate_demo` 是更早的演示，不是 Stage 5。

---

## 8. Configuration Files

### PARAMETER LOCATION TABLE

| 参数 | 当前实际文件 | SIM/REAL | 是否建议用户修改 |
| --- | --- | --- | --- |
| Robot initial joint pose (Home) | `fr_control/config/poses.yaml` `home` | SIM 任务 | Stage 1 改这里 |
| 同上 | `fr_control/config/grasp.yaml` `home.joints` | SIM Stage 3 | **DUPLICATED** |
| 同上 | `fr_control/config/stage4_config.yaml` `robot.initial_joint_positions` | SIM Stage 4 | **DUPLICATED** |
| 同上 | `fairino3_gazebo/urdf/gz_ros2_control.xacro` `initial_value` | SIM 启动姿态 | 改仿真初始角时改 |
| 同上 | SRDF `pos1` in `fairino3_v6_robot.srdf` / `.real.srdf` | SIM+REAL 命名姿态 | 与 Home 重复 |
| MoveIt mock 初始角 | `fairino3_v6_moveit2_config/config/initial_positions.yaml` 全 0 | mock demo | 不要当 Gazebo Home |
| Robot base pose（仿真 spawn） | `sim.launch.py` spawn_* 默认 0；Stage 4 用 `stage4_config.yaml` `robot.base_pose` | SIM | Stage 4 改 YAML |
| 真机安装位姿 world→base | `fairino3_v6_moveit2_config/config/real_installation.yaml` | REAL | **建议改**（现 x/y/z=0） |
| Object initial pose | `grasp.yaml` `object.position`；`inspection_world.sdf`；`stage4_config.yaml` `object.initial_pose` | SIM | **DUPLICATED** |
| Object dimensions | `grasp.yaml` `object.size`；`stage4_config.yaml` `object.dimensions`；world SDF | SIM | **DUPLICATED** |
| Table pose / size | `grasp.yaml` `table`；`stage4_config.yaml` `table`；world SDF | SIM | **DUPLICATED** |
| Pre-Grasp offset | `grasp.yaml` `pre_grasp_offset`；`stage4_config.yaml` `pregrasp_distance` | SIM 任务 | **DUPLICATED 命名** |
| Lift distance | `grasp.yaml` `lift_height`；`stage4_config.yaml` `lift_distance` | SIM 任务 | **DUPLICATED 命名** |
| fingertip_from_tcp / table_clearance | 两个 YAML 都有 `0.036` / `0.015` | SIM | 夹爪几何变了再改 |
| Inspection P1 | **仅** `stage4_config.yaml` `inspection.position` | Stage 4 | **建议改实验时改这里** |
| Inspection D1 | **仅** `stage4_config.yaml` `inspection.direction` | Stage 4 | 同上 |
| Face1/2/3 | **仅** `stage4_config.yaml` `inspection.faces` | Stage 4 | 同上 |
| MoveIt velocity/acc（任务） | poses.yaml / grasp.yaml / stage4 `motion.velocity_scaling` | 任务层 | 真机应明显减小 |
| MoveIt 默认缩放 | `joint_limits.yaml` `default_velocity_scaling_factor: 0.1` | MoveIt | 全局上限 |
| 真机控制器频率 | `ros2_controllers_real.yaml` `update_rate: 125` | REAL | 一般不改 |
| 仿真控制器频率 | `fairino3_gazebo/config/controllers.yaml` `update_rate: 100` | SIM | 一般不改 |
| 真机 IP | C++ `#define CONTROLLER_IP_ADDRESS "192.168.58.2"` | REAL | IP 变了必须改源码重编译 |
| 夹爪开合行程 | `fr_control/constants.py` `GRIPPER_OPEN=0` `GRIPPER_CLOSED=0.1` | SIM | 一般不改 |
| 夹爪关节名 | `constants.py`：`rail_2_slider_l/r` | SIM | 与 URDF 绑定 |

本次 **不重构** 重复参数，只报告。Stage 4 自称 `stage4_config.yaml` 是该阶段唯一权威源；Stage 3 仍以 `grasp.yaml` + `inspection_world.sdf` 为准。

---

## 9. How to Start Simulation

```text
================================
SIMULATION
================================
```

新终端不要 `source ~/install`。`.bashrc` 已有 Humble 和 `agx_arm_ws`；再 overlay 本仓库。

**一键（Stage 1–3 世界）：**

```bash
cd ~/fairino_ws
source /opt/ros/humble/setup.bash
source ~/fairino_ws/install/setup.bash

ros2 launch fairino3_gazebo sim_full.launch.py
```

可选：`use_rviz:=false`、`moveit_delay:=10.0`、`enable_grasp_weld:=true`。

**一键（Stage 4 世界）：**

```bash
ros2 launch fr_control stage4_full.launch.py
```

不要用 `demo.launch.py` 代替。`demo.launch.py` 是 mock RViz，不启动 Gazebo。

分终端启动（调试时仍可用）：`sim.launch.py` → `fairino3_gazebo move_group.launch.py` → `fairino3_v6_moveit2_config moveit_rviz.launch.py`。`spawn_yaw` 已在 `sim.launch.py` 声明，默认 0。

### 启动成功后应看到的节点 / 控制器

节点（名称可能带命名空间前缀）：

- `robot_state_publisher`
- `create`（spawn，完成后退出）
- Gazebo / `parameter_bridge`（`ros_gz_bridge`）
- `controller_manager`（由 Gazebo 插件拉起）
- `move_group`
- `rviz2`

确认：

```bash
ros2 control list_controllers
```

期望 active：

- `joint_state_broadcaster`
- `fairino3_controller`
- `gripper_left_controller`
- `gripper_right_controller`

```bash
ros2 topic list
# 至少应有 /clock /joint_states
# /fairino3_controller/follow_joint_trajectory/_action/...
# /gripper_left_controller/gripper_cmd/_action/...
```

RViz 中 Planning Group = `fairino3_v6_group`，Plan & Execute 后 Gazebo 中 FR3 应同步运动。

---

## 10. How to Start Real Robot

```text
================================
REAL ROBOT
================================
```

**这个命令会控制真实 FR3。** 急停应随时可及。网络需能访问 `192.168.58.2`。

```bash
cd ~/fairino_ws
source /opt/ros/humble/setup.bash
source ~/fairino_ws/install/setup.bash

ros2 launch fairino3_v6_moveit2_config real_bringup.launch.py
```

与 SIM 的区别：

| 项目 | SIM | REAL |
| --- | --- | --- |
| 终端数 | 2~3 | 1 |
| robot_description | `fairino3_gazebo.urdf.xacro` | `fairino3_v6_robot.real.urdf.xacro` |
| hardware | `gz_ros2_control/GazeboSimSystem` | `fairino_hardware/FairinoHardwareInterface` |
| use_sim_time | true | false |
| 夹爪 controller | left/right effort | **无** |
| world→base | URDF 固定 0 或 spawn 参数 | `real_installation.yaml` 静态 TF |
| MoveIt 超时 | gazebo overlay 放宽 | 官方/真机配置 |
| 轨迹 | Servo 进 Gazebo | ServoJ 进控制器 |

成功后 `ros2 control list_controllers` 应有：

- `joint_state_broadcaster`
- `fairino3_controller`

没有 gripper_* controller。

---

## 11. How to Run Task Programs

已用 `ros2 pkg executables` 验证。

```text
================================
TASK COMMANDS
================================
```

前提：对应环境已启动。任务节点默认 `use_sim_time:=true`。

```text
机械臂运动测试（Stage 1）：
  ros2 launch fr_control pose_sequence.launch.py
  ros2 run fr_control pose_sequence

夹爪测试（Stage 2，只需 Gazebo）：
  ros2 launch fr_control gripper_test.launch.py
  ros2 run fr_control gripper_test

固定位置抓取（Stage 3）：
  ros2 launch fr_control grasp_test.launch.py
  ros2 run fr_control grasp_test

P1 / Face1 / Face2 / Face3（Stage 4）：
  先：ros2 launch fr_control stage4_sim.launch.py
  再：ros2 launch fairino3_gazebo move_group.launch.py
  然后：
  ros2 launch fr_control stage4.launch.py
  ros2 run fr_control stage4_inspection_test

夹爪接触实验（可选）：
  ros2 run fr_control gripper_contact_test
  ros2 launch fr_control gripper_contact_test.launch.py

碰撞掉落实验（可选）：
  ros2 run fr_control collision_drop_test

早期调试（非正式 Stage）：
  ros2 run fairino3_gazebo joint_demo
  ros2 run fairino3_gazebo joint_axis_test
  ros2 run fairino3_gazebo pick_inspect_demo     # 夹爪接口已过时，不要当正式任务
  ros2 run fr3_part_demo part_rotate_demo        # 已被 Stage 4 取代
```

真机跑 Stage 1 时必须：

```bash
ros2 launch fr_control pose_sequence.launch.py use_sim_time:=false
```

且 **不要直接复用** 当前 `poses.yaml` 里为直立 Gazebo 桌面准备的笛卡尔 Pose A/B（真机基座有 90° tilt）。

---

## 12. SIM ONLY / REAL ONLY / SHARED

### A. SIM ONLY

- `fairino3_gazebo` 全部 launch / world / gazebo xacro / controllers.yaml
- `gz_ros2_control/GazeboSimSystem` 与 Gazebo plugin
- `hkv_gripper_physics.xacro` 双指 effort 物理
- `gripper_left_controller` / `gripper_right_controller`
- `sim_grasp.py` DetachableJoint、`/fr3/grasp/attach|detach`
- `gazebo_world_pose.py`、`stage4_world.py`、`grasp_reset.py`
- `inspection_world.sdf` 与 Stage 4 临时 SDF
- `gripper_contact_test` / `collision_drop_test`
- `joint_demo` / `joint_axis_test` / `pick_inspect_demo`

### B. REAL ONLY

- `FairinoHardwareInterface`（package `fairino_hardware_v3_9_7`）
- `real_bringup.launch.py` 及 `real_*_check.launch.py`
- `fairino3_v6_robot.real.urdf.xacro` / `.real.ros2_control.xacro` / `.real.srdf`
- `ros2_controllers_real.yaml`、`fairino3_controller_real.yaml`、`moveit_controllers_real.yaml`
- `real_installation.yaml`
- 控制器 IP `192.168.58.2`
- `ros2_cmd_server`（官方 API，当前 bringup 不用）
- `RealFairinoGripper`（占位，调用即抛错）

### C. SHARED（迁移真机时应复用，不要重写）

- `MoveItArm`：`go_joints` / `go_pose` / `go_cartesian`
- 规划组 `fairino3_v6_group`、EE `gripper_tcp`、关节 `j1~j6`
- 控制器名 `fairino3_controller` + FollowJointTrajectory
- `grasp_poses.py` Pre-Grasp / Grasp / Lift 几何
- `inspection_poses.py` P1、Face→D1、Object→TCP
- `planning_scene.py` 碰撞体 / attach（真机仍可用于规划避障）
- Stage 状态机：`pose_sequence` / `grasp_test` / `stage4_inspection_test`
- `create_gripper()` 工厂（换 backend，不改任务）
- `create_sim_grasp(backend='none')` 真机必须 none

Sim → Real **真正该换的层**：

1. launch / `robot_description` / hardware plugin  
2. `use_sim_time`  
3. 夹爪 backend（目前还没有真机实现）  
4. `sim_grasp_backend:=none`  
5. 验收：不要再读 Gazebo model pose  
6. YAML 中的 Home / P1 / 物体位姿改成实验室实测值  
7. 速度缩放降到安全值  

不要重写 MoveIt 客户端或 Face 数学。

---

## 13. Current Status

已完成：

- Gazebo + MoveIt Plan & Execute 同步
- Python 自动臂运动（Stage 1）
- Gazebo 夹爪开合（Stage 2）
- 已知位姿抓取（Stage 3）
- P1 三面朝向（Stage 4，仿真焊死辅助翻面）
- 真机 `real_bringup.launch.py` + MoveIt 可运动真实 FR3

未完成：

- 真机夹爪
- 深度相机
- Stage 1~4 任务在真机上的完整执行
- 第二台机械臂 / 另外三面
- 独立 Stage 5 / Stage 6

---

## 14. Next Step

见下文 **NEXT RECOMMENDED STEP**。本次不要开始改 Stage 真机任务代码。

---

## 附：官方 demo.launch.py 的作用

```bash
ros2 launch fairino3_v6_moveit2_config demo.launch.py
```

由 `generate_demo_launch(MoveItConfigsBuilder)` 生成，会启动：

- `robot_state_publisher`
- mock `ros2_control`（`mock_components/GenericSystem`）
- `fairino3_controller` + `gripper_controller`（**单关节 gripper_joint**，与 Gazebo 双指 effort 不同）
- `move_group` + RViz
- 可选 warehouse

**不会启动 Gazebo。**  
只适合 MoveIt 配置/RViz mock 调试。  
**不是** 本项目正式 Gazebo 仿真入口。  
与自定义仿真的区别：hardware 是假的；夹爪 controller 名称和类型都不同；没有桌子和零件。

---

## 附：当前问题与重复（不在本次修复）

1. **Home / 物体 / 桌子 / pre-grasp / lift** 在 `poses.yaml`、`grasp.yaml`、`stage4_config.yaml`、`inspection_world.sdf`、`gz_ros2_control.xacro`、SRDF `pos1` 多处重复。  
2. `sim.launch.py` 使用未声明的 `spawn_yaw`。  
3. 源码中有多份 `fairino_hardware_v*`，同名插件 `fairino_hardware/FairinoHardwareInterface`；全量 `colcon build` 可能冲突。当前 install 只用 **v3_9_7**。  
4. `~/install` 里还有一份 `fairino3_gazebo` / `fr_control`。若先 source 它会 overlay 错包。请只用 `~/fairino_ws/install`。  
5. `fairino3_v6_robot.urdf copy.xacro`、`fairino3_v6_robot copy.srdf` 为备份文件。  
6. `real_installation.yaml` 注释写明位置尚未实测，xyz=0。  
7. SIM 的 `move_group` 仍用 MoveItConfigsBuilder **默认 mock URDF 作节点参数**，Gazebo 的 `robot_description` 由 `robot_state_publisher` 发布。能工作是因为 j1~j6 与 `gripper_tcp` 一致；夹爪模型并不相同。  
8. `pick_inspect_demo` 仍调用已删除的 `gripper_controller` FollowJointTrajectory。  
9. 真机 IP 不能用 YAML 改。  
10. Stage 4 翻面依赖仿真 weld；真机没有等价层。
