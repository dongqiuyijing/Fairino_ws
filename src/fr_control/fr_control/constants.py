"""
FR3 MoveIt control-layer constants.

These values match fairino3_v6_moveit2_config SRDF.
Change the planning group or EE link here, not in task nodes.
"""

PLANNING_GROUP = "fairino3_v6_group"
EE_LINK = "gripper_tcp"
BASE_FRAME = "base_link"
ARM_JOINTS = ("j1", "j2", "j3", "j4", "j5", "j6")

MOVE_ACTION = "move_action"
EXECUTE_ACTION = "execute_trajectory"
CARTESIAN_SERVICE = "/compute_cartesian_path"
APPLY_SCENE_SERVICE = "/apply_planning_scene"
GET_SCENE_SERVICE = "/get_planning_scene"

# 上层仍用闭合行程：0.0 张开，0.1 完全闭合。
# 底层左右滑块独立：left = +0.5 * travel，right = -0.5 * travel。
GRIPPER_JOINT = "gripper_joint"
GRIPPER_LEFT_JOINT = "rail_2_slider_l"
GRIPPER_RIGHT_JOINT = "rail_2_slider_r"
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.1
GRIPPER_LEFT_ACTION = "/gripper_left_controller/gripper_cmd"
GRIPPER_RIGHT_ACTION = "/gripper_right_controller/gripper_cmd"
GRIPPER_ACTION = GRIPPER_LEFT_ACTION

# 抓取后 PlanningScene 附着到 TCP；touch_links 允许夹爪与零件接触。
ATTACH_LINK = "gripper_tcp"
GRIPPER_TOUCH_LINKS = (
    "gripper_base_link",
    "rail_155",
    "slider_l",
    "slider_r",
    "finger_l",
    "finger_r",
    "gripper_gap_link",
)

# 仅 Gazebo DetachableJoint 使用；真实夹爪任务不得依赖这两个 topic。
GRASP_ATTACH_TOPIC = "/fr3/grasp/attach"
GRASP_DETACH_TOPIC = "/fr3/grasp/detach"
