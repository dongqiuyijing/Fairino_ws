# 阶段 4：抓住零件后把三个面依次朝向固定检测方向 D1

阶段 4 只使用已知位姿，不用相机、检测或第二台机械臂。

## 零件坐标系

Gazebo 模型 `small_part` 是盒子。`orientation_rpy` 全 0、平放在桌面上时：

| object_frame | 物理含义 | 尺寸 |
| --- | --- | --- |
| +X / -X | 夹持宽度方向的一对侧面 | 20 mm × 20 mm 高度 |
| +Y / -Y | 沿桌面长度的两个端面 | 20 mm × 20 mm |
| +Z | 顶面 | 20 mm × 40 mm |
| -Z | 底面（与桌面接触） | 20 mm × 40 mm |

夹爪沿 ±X 闭合，所以阶段 4 检测：

- Face1 = +Z 顶面
- Face2 = +Y 端面
- Face3 = -Y 端面

改面定义只改 `config/stage4_config.yaml` 里的 `inspection.faces`。

阶段 4 的 Face1 需要把顶面转到水平检测方向 D1。纯摩擦夹持会沿夹指缝滑落，因此阶段 4 仿真启用已有的 Gazebo `DetachableJoint`（`grasp.use_sim_attach: true`）。验收仍然读取 Gazebo 里 `small_part` 的真实模型位姿，不是 MoveIt PlanningScene。阶段 3 默认仍是纯物理抓取。

## 姿态计算

1. 用 `inspection.direction`（D1）和 `inspection.up_direction` 唯一确定检测姿态。
2. 每个 Face 把 `normal_in_object` 对齐到 D1，把 `up_in_object` 对齐到 up。
3. 三个 Object Pose 的位置都是 P1，只改朝向。
4. 抓取抬起后从 Gazebo 真实零件位姿和 TCP TF 计算 `T_tcp_object`。
5. `T_base_tcp = T_base_object_target * inverse(T_tcp_object)`，再交给 MoveIt。

不要把阶段 4 理解成「把 j6 转 90°」。

## 启动

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_ws/install/setup.bash

# 终端 1：Gazebo（底座、桌子、零件都来自 YAML）
ros2 launch fr_control stage4_sim.launch.py \
  config_file:=$HOME/fairino_ws/src/fr_control/config/stage4_config.yaml

# 终端 2：MoveIt
ros2 launch fairino3_gazebo move_group.launch.py

# 终端 3：抓取 + 三面检测
ros2 launch fr_control stage4.launch.py \
  config_file:=$HOME/fairino_ws/src/fr_control/config/stage4_config.yaml
```

无界面仿真把终端 1 改成 `headless:=true`。
