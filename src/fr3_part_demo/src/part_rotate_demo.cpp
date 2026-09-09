#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;


// --------------------------------------------------
// 执行一个笛卡尔位姿目标
// --------------------------------------------------
bool moveToPose(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &target_pose)
{
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose, "gripper_tcp");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
        (move_group.plan(plan) ==
         moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("fr3_part_demo"),
            "Planning failed!");
        return false;
    }

    auto result = move_group.execute(plan);

    move_group.clearPoseTargets();

    return result == moveit::core::MoveItErrorCode::SUCCESS;
}


// --------------------------------------------------
// 沿工具自身 Z 轴平移
// --------------------------------------------------
geometry_msgs::msg::Pose translateLocalZ(
    const geometry_msgs::msg::Pose &pose,
    double distance)
{
    geometry_msgs::msg::Pose result = pose;

    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);

    tf2::Matrix3x3 rotation(q);

    tf2::Vector3 local_offset(0.0, 0.0, distance);
    tf2::Vector3 world_offset = rotation * local_offset;

    result.position.x += world_offset.x();
    result.position.y += world_offset.y();
    result.position.z += world_offset.z();

    return result;
}


// --------------------------------------------------
// 绕工具自身 Y 轴旋转
// --------------------------------------------------
geometry_msgs::msg::Pose rotateLocalY(
    const geometry_msgs::msg::Pose &pose,
    double angle_rad)
{
    geometry_msgs::msg::Pose result = pose;

    tf2::Quaternion q_current;
    tf2::fromMsg(pose.orientation, q_current);

    tf2::Quaternion q_rotation;
    q_rotation.setRPY(0.0, angle_rad, 0.0);

    // 右乘 = 绕工具自身坐标系旋转
    tf2::Quaternion q_result =
        q_current * q_rotation;

    q_result.normalize();

    result.orientation = tf2::toMsg(q_result);

    return result;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "fr3_part_rotate_demo",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    // MoveGroupInterface 需要节点持续 spin
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });


    // --------------------------------------------------
    // 创建 MoveIt 控制接口
    // --------------------------------------------------

    static const std::string PLANNING_GROUP =
        "fairino3_v6_group";

    moveit::planning_interface::MoveGroupInterface move_group(
        node,
        PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;


    move_group.setEndEffectorLink("gripper_tcp");

    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);

    move_group.setMaxVelocityScalingFactor(0.15);
    move_group.setMaxAccelerationScalingFactor(0.15);

    RCLCPP_INFO(
        node->get_logger(),
        "Planning frame: %s",
        move_group.getPlanningFrame().c_str());

    RCLCPP_INFO(
        node->get_logger(),
        "End effector: %s",
        move_group.getEndEffectorLink().c_str());


    // --------------------------------------------------
    // STEP 1
    // 先移动到 FR3 官方 SRDF 中的 pos1
    // --------------------------------------------------

    RCLCPP_INFO(
        node->get_logger(),
        "Step 1: moving to pos1...");

    move_group.setNamedTarget("pos1");

    moveit::planning_interface::MoveGroupInterface::Plan
        initial_plan;

    if (move_group.plan(initial_plan) !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Cannot plan to pos1.");

        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    move_group.execute(initial_plan);

    std::this_thread::sleep_for(2s);


    // --------------------------------------------------
    // STEP 2
    // 获取当前夹爪 TCP 位姿
    // --------------------------------------------------

    auto tcp_pose_stamped =
        move_group.getCurrentPose("gripper_tcp");

    geometry_msgs::msg::Pose tcp_pose =
        tcp_pose_stamped.pose;


    // --------------------------------------------------
    // STEP 3
    // 在夹爪内部生成 5 x 5 x 20 mm 零件
    //
    // 零件长轴沿 gripper_tcp 的 Z 轴
    //
    // 我们之前的虚拟夹爪：
    // TCP 位于指尖附近
    // 所以把零件中心放在 TCP 后方 20 mm
    // --------------------------------------------------

    geometry_msgs::msg::Pose part_pose =
        translateLocalZ(tcp_pose, -0.020);


    moveit_msgs::msg::CollisionObject part;

    part.header.frame_id =
        move_group.getPlanningFrame();

    part.id = "small_part";


    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type =
        shape_msgs::msg::SolidPrimitive::BOX;

    primitive.dimensions.resize(3);

    // 单位是米
    primitive.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_X] = 0.005;

    primitive.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.005;

    primitive.dimensions[
        shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.020;


    part.primitives.push_back(primitive);
    part.primitive_poses.push_back(part_pose);

    part.operation =
        moveit_msgs::msg::CollisionObject::ADD;


    planning_scene_interface.applyCollisionObject(part);

    RCLCPP_INFO(
        node->get_logger(),
        "Step 2: 5x5x20 mm part added.");

    std::this_thread::sleep_for(1s);


    // --------------------------------------------------
    // STEP 4
    // 模拟夹爪夹住零件
    //
    // 注意：
    // 这里不是物理抓取
    // attach 后零件会跟随机器人
    // --------------------------------------------------

    std::vector<std::string> touch_links =
    {
        "gripper_base_link",
        "gripper_left_finger",
        "gripper_right_finger",
        "gripper_tcp"
    };


    bool attached =
        move_group.attachObject(
            "small_part",
            "gripper_tcp",
            touch_links);


    if (!attached)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Attach object failed.");

        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }


    RCLCPP_INFO(
        node->get_logger(),
        "Step 3: part attached to gripper.");

    std::this_thread::sleep_for(2s);


    // --------------------------------------------------
    // STEP 5
    // 抬高 80 mm
    // --------------------------------------------------

    geometry_msgs::msg::Pose lifted_pose =
        move_group.getCurrentPose(
            "gripper_tcp").pose;

    lifted_pose.position.z += 0.080;


    RCLCPP_INFO(
        node->get_logger(),
        "Step 4: lifting part...");

    if (!moveToPose(move_group, lifted_pose))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Lift failed.");

        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }


    std::this_thread::sleep_for(3s);


    // --------------------------------------------------
    // Face 1
    //
    // 当前姿态定义为 Face 1
    //
    // 这里先把“方向1”定义成：
    // Face1 当前朝向的固定方向
    // --------------------------------------------------

    geometry_msgs::msg::Pose face1_pose =
        move_group.getCurrentPose(
            "gripper_tcp").pose;


    RCLCPP_INFO(
        node->get_logger(),
        "FACE 1 ready.");

    std::this_thread::sleep_for(3s);


    // --------------------------------------------------
    // Face 2
    //
    // Face1 + 90 degree
    // --------------------------------------------------

    geometry_msgs::msg::Pose face2_pose =
        rotateLocalY(
            face1_pose,
            M_PI / 2.0);


    RCLCPP_INFO(
        node->get_logger(),
        "Moving Face1 -> Face2 (+90 deg)...");

    if (!moveToPose(move_group, face2_pose))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Face2 motion failed.");

        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }


    RCLCPP_INFO(
        node->get_logger(),
        "FACE 2 ready.");

    std::this_thread::sleep_for(3s);


    // --------------------------------------------------
    // Face 3
    //
    // Face2 反向旋转 180°
    //
    // 等效于：
    // Face1 - 90°
    // --------------------------------------------------

    geometry_msgs::msg::Pose face3_pose =
        rotateLocalY(
            face1_pose,
            -M_PI / 2.0);


    RCLCPP_INFO(
        node->get_logger(),
        "Moving Face2 -> Face3 (-180 deg)...");

    if (!moveToPose(move_group, face3_pose))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Face3 motion failed.");

        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }


    RCLCPP_INFO(
        node->get_logger(),
        "FACE 3 ready.");

    RCLCPP_INFO(
        node->get_logger(),
        "Demo finished successfully.");


    std::this_thread::sleep_for(5s);


    executor.cancel();

    if (spinner.joinable())
        spinner.join();

    rclcpp::shutdown();

    return 0;
}