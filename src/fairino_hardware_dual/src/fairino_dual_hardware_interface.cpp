#include "fairino_hardware_dual/fairino_dual_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <string>
#include <unordered_map>

namespace fairino_hardware_dual{

namespace {
constexpr double kArmSettledThresholdRad = 0.05;  // ~2.9 deg
constexpr auto kArmSettleCommandWait = std::chrono::milliseconds(1000);
constexpr auto kGripperPollPeriod = std::chrono::milliseconds(50);
constexpr auto kStaleDoneGrace = std::chrono::milliseconds(300);
constexpr auto kServoRestartMargin = std::chrono::milliseconds(4000);
constexpr auto kRestartLogThrottle = std::chrono::seconds(1);
}  // namespace

hardware_interface::CallbackReturn FairinoDualHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量
    
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {

        //指令部分
        if (joint.command_interfaces.size() != 1) {//开放servoJ
            RCLCPP_FATAL(rclcpp::get_logger("FairinoDualHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoDualHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoDualHardwareInterface"),
        //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        //关节状态部分
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoDualHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoDualHardwareInterface"),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoDualHardwareInterface"),
        //                 "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoDualHardwareInterface"),
        //                 "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

    }

    if (info_.joints.size() != 6) {
        RCLCPP_FATAL(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "Expected 6 arm joints, got %zu. Real backend must not include gripper_joint.",
            info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto & params = info_.hardware_parameters;
    auto read_param = [&params](const std::string & key, std::string & dest) {
        const auto it = params.find(key);
        if (it != params.end() && !it->second.empty()) {
            dest = it->second;
        }
    };
    read_param("controller_ip", _controller_ip);
    read_param("gripper_service", _gripper_service_name);
    read_param("gripper_node", _gripper_node_name);

    RCLCPP_INFO(
        rclcpp::get_logger("FairinoDualHardwareInterface"),
        "configured controller_ip=%s gripper_service=%s gripper_node=%s",
        _controller_ip.c_str(),
        _gripper_service_name.c_str(),
        _gripper_node_name.c_str()
    );
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoDualHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state.at(i)));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoDualHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoDualHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "Starting ...please wait...");
    //做变量的初始化工作
    _ptr_robot = std::make_unique<FRRobot>();//创建机器人实例
    for(int i=0;i<6;i++){//初始化变量
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
    }
    _control_mode = 0;//默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
    RCLCPP_INFO(
        rclcpp::get_logger("FairinoDualHardwareInterface"),
        "Connecting Fairino SDK at %s",
        _controller_ip.c_str()
    );
    errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str());//建立xmlrpc连接
    rclcpp::sleep_for(200ms);//等待一段时间让控制器的rpc连接建立完毕
    if(returncode != 0){
        RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "机械臂SDK连接失败！请检查端口时候被占用");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "机械臂SDK连接成功！");
    }
    //做第一步的工作，读取当前状态数据
    JointPos jntpos;
    returncode = _ptr_robot->GetActualJointPosDegree(0,&jntpos);
    /*
    获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
    因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
    */
    if(returncode == 0){
        for(int j=0;j<6;j++){
            _jnt_position_command[j] = jntpos.jPos[j]/180.0*M_PI;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"),"初始指令位置: %f,%f,%f,%f,%f,%f",_jnt_position_command[0],\
        _jnt_position_command[1],_jnt_position_command[2],_jnt_position_command[3],_jnt_position_command[4],_jnt_position_command[5]);    
        // RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "机械臂硬件启动成功!");
        // return hardware_interface::CallbackReturn::SUCCESS;
        // 在开始周期性 ServoJ 控制之前，先进入伺服运动模式。
        // 清除机器人错误
        errno_t ret = _ptr_robot->ResetAllError();
        if (ret != 0) {
            RCLCPP_WARN(
                rclcpp::get_logger("FairinoDualHardwareInterface"),
                "ResetAllError返回错误码:%d",
                ret
            );
        }

        // 自动使能机器人
        ret = _ptr_robot->RobotEnable(1);
        if (ret != 0) {
            RCLCPP_ERROR(
                rclcpp::get_logger("FairinoDualHardwareInterface"),
                "RobotEnable(1)失败，错误码:%d",
                ret
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "机器人自动使能成功"
        );

        // 确保旧运动状态结束
        _ptr_robot->StopMotion();
        _ptr_robot->ServoMoveEnd();
        errno_t servo_ret = _ptr_robot->ServoMoveStart();

        if (servo_ret != 0) {
            RCLCPP_ERROR(
                rclcpp::get_logger("FairinoDualHardwareInterface"),
                "ServoMoveStart失败，错误码:%d，拒绝激活硬件接口",
                servo_ret
            );

            return hardware_interface::CallbackReturn::ERROR;
        }

        _runtime_mode.store(RuntimeMode::SERVO_ACTIVE);
        RCLCPP_INFO(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "ServoMoveStart成功，ServoJ模式已启用"
        );

        RCLCPP_INFO(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "机械臂硬件启动成功!"
        );

        try {
            start_gripper_bridge();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(
                rclcpp::get_logger("FairinoDualHardwareInterface"),
                "夹爪桥接启动失败（六轴 ServoJ 仍继续）：%s",
                e.what()
            );
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoDualHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "Stopping ...please wait...");
    // _ptr_robot->StopMotion();//停止机器人
    // _ptr_robot->CloseRPC();//销毁实例，连接断开
    // _ptr_robot.release();
    if (_ptr_robot) {
        stop_gripper_bridge();
        errno_t servo_ret = _ptr_robot->ServoMoveEnd();
    
        if (servo_ret != 0) {
            RCLCPP_WARN(
                rclcpp::get_logger("FairinoDualHardwareInterface"),
                "ServoMoveEnd返回错误码:%d",
                servo_ret
            );
        }
    
        _ptr_robot->StopMotion();
        _ptr_robot->CloseRPC();
        _ptr_robot.reset();
    }
    RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoDualHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
    if (!_ptr_robot) {
        return hardware_interface::return_type::OK;
    }
    JointPos state_data;
    error_t returncode = _ptr_robot->GetActualJointPosDegree(1,&state_data);
    if(returncode == 0){
        for(int i=0;i<6;i++){
            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;//注意单位转换，moveit统一用弧度
            //_jnt_torque_state[i] = state_data.jt_cur_tor[i];//注意单位转换
        }
    }else{
        hardware_interface::return_type::ERROR;
    }
    //RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
    _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FairinoDualHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    if (!_ptr_robot) {
        return hardware_interface::return_type::OK;
    }

    const RuntimeMode mode = _runtime_mode.load();
    if (mode != RuntimeMode::SERVO_ACTIVE) {
        process_pending_gripper();
        return hardware_interface::return_type::OK;
    }
    if (gripper_pending_active()) {
        // FollowJointTrajectory may complete before hardware feedback reaches
        // the bridge's existing 0.05 rad safety threshold.  Re-check the
        // pending request without pausing ServoJ, so feedback can converge.
        process_pending_gripper();
        if (_runtime_mode.load() != RuntimeMode::SERVO_ACTIVE) {
            return hardware_interface::return_type::OK;
        }
    }

    if(_control_mode == 0){//位置控制模式
        if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[0] + 6,\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        JointPos cmd;
        ExaxisPos extcmd{0,0,0,0};
        for(auto j=0;j<6;j++){
            cmd.jPos[j] = _jnt_position_command[j]/M_PI*180; //注意单位转换
        }
        //RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "ServoJ下发位置:%f,%f,%f,%f,%f,%f",\
            cmd.jPos[0],cmd.jPos[1],cmd.jPos[2],cmd.jPos[3],cmd.jPos[4],cmd.jPos[5]);
        int returncode = _ptr_robot->ServoJ(&cmd,&extcmd,0,0,0.008,0,0);
        if(returncode != 0){
            RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "ServoJ指令下发错误,错误码:%d",returncode);
        }
    }else if(_control_mode == 1){//扭矩控制模式
        if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[0] + 6,\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        //_ptr_robot->write(_jnt_torque_command);//注意单位转换
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoDualHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}


void FairinoDualHardwareInterface::start_gripper_bridge()
{
    if (_gripper_node) {
        return;
    }
    _gripper_node = std::make_shared<rclcpp::Node>(_gripper_node_name);
    _gripper_service = _gripper_node->create_service<fairino_msgs::srv::GripperBridge>(
        _gripper_service_name,
        std::bind(
            &FairinoDualHardwareInterface::handle_gripper,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    _gripper_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _gripper_executor->add_node(_gripper_node);
    _gripper_spin_thread = std::thread([this]() {
        _gripper_executor->spin();
    });
    RCLCPP_INFO(
        rclcpp::get_logger("FairinoDualHardwareInterface"),
        "夹爪桥接已启动：%s（idle，不会自动 reset/activate/运动）",
        _gripper_service_name.c_str()
    );
}

void FairinoDualHardwareInterface::stop_gripper_bridge()
{
    if (_gripper_executor) {
        _gripper_executor->cancel();
    }
    if (_gripper_spin_thread.joinable()) {
        _gripper_spin_thread.join();
    }
    {
        std::lock_guard<std::mutex> lock(_gripper_mutex);
        if (_gripper_pending) {
            _gripper_pending->error_code = -1;
            _gripper_pending->message = "hardware deactivating";
            _gripper_pending->finished = true;
            _gripper_pending->cv.notify_all();
            _gripper_pending.reset();
        }
    }
    _gripper_service.reset();
    if (_gripper_executor && _gripper_node) {
        _gripper_executor->remove_node(_gripper_node);
    }
    _gripper_executor.reset();
    _gripper_node.reset();
}

void FairinoDualHardwareInterface::handle_gripper(
    const std::shared_ptr<fairino_msgs::srv::GripperBridge::Request> request,
    std::shared_ptr<fairino_msgs::srv::GripperBridge::Response> response)
{
    const std::string command = request->command;
    if (command == "ping") {
        response->error_code = _ptr_robot ? 0 : -1;
        response->message = _ptr_robot ? "ok" : "robot not connected";
        return;
    }

    auto pending = std::make_shared<PendingGripperCommand>();
    pending->command = command;
    pending->gripper_id = request->gripper_id;
    pending->position = request->position;
    pending->velocity = request->velocity;
    pending->force = request->force;
    pending->max_time_ms = request->max_time_ms;
    pending->block = request->block;
    pending->gripper_type = request->gripper_type;
    pending->rot_num = request->rot_num;
    pending->rot_vel = request->rot_vel;
    pending->rot_torque = request->rot_torque;
    pending->arm_settle_deadline = std::chrono::steady_clock::now() + kArmSettleCommandWait;

    {
        std::lock_guard<std::mutex> lock(_gripper_mutex);
        if (_gripper_pending) {
            response->error_code = -1;
            response->message = "gripper command busy";
            return;
        }
        if (_runtime_mode.load() != RuntimeMode::SERVO_ACTIVE) {
            response->error_code = -1;
            response->message = "gripper command busy";
            return;
        }
        _gripper_pending = pending;
    }

    std::unique_lock<std::mutex> lock(_gripper_mutex);
    const bool finished = pending->cv.wait_for(
        lock,
        gripper_service_wait(pending->max_time_ms),
        [&pending]() { return pending->finished; });
    if (!finished) {
        // 仲裁器仍可能在 GRIPPER_WAIT / SERVO_RESTART；不要抢走 pending。
        if (_runtime_mode.load() == RuntimeMode::SERVO_ACTIVE &&
            _gripper_pending == pending) {
            _gripper_pending.reset();
        }
        response->error_code = -1;
        response->message =
            "gripper command timed out waiting for motion done and ServoJ resume";
        return;
    }
    response->error_code = pending->error_code;
    response->message = pending->message;
}

bool FairinoDualHardwareInterface::gripper_pending_active()
{
    std::lock_guard<std::mutex> lock(_gripper_mutex);
    return static_cast<bool>(_gripper_pending) && !_gripper_pending->finished;
}

std::chrono::milliseconds FairinoDualHardwareInterface::gripper_service_wait(
    int max_time_ms) const
{
    const int motion_ms = std::max(max_time_ms, 1000);
    return std::chrono::milliseconds(motion_ms) + kServoRestartMargin;
}

double FairinoDualHardwareInterface::max_command_state_error() const
{
    double max_err = 0.0;
    for (int j = 0; j < 6; ++j) {
        max_err = std::max(
            max_err,
            std::abs(_jnt_position_command[j] - _jnt_position_state[j]));
    }
    return max_err;
}

void FairinoDualHardwareInterface::finish_gripper_pending(
    int error_code, const std::string & message)
{
    std::lock_guard<std::mutex> lock(_gripper_mutex);
    if (!_gripper_pending) {
        return;
    }
    _gripper_pending->error_code = error_code;
    _gripper_pending->message = message;
    _gripper_pending->finished = true;
    _gripper_pending->cv.notify_all();
    _gripper_pending.reset();
}

bool FairinoDualHardwareInterface::sync_joints_from_actual(const char * log_line)
{
    JointPos jntpos{};
    const errno_t ret = _ptr_robot->GetActualJointPosDegree(0, &jntpos);
    if (ret != 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "[GRIPPER ARBITER] GetActualJointPosDegree ret=%d (%s)",
            ret,
            log_line
        );
        return false;
    }
    for (int j = 0; j < 6; ++j) {
        const double rad = jntpos.jPos[j] / 180.0 * M_PI;
        _jnt_position_state[j] = rad;
        _jnt_position_command[j] = rad;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("FairinoDualHardwareInterface"),
        "[GRIPPER ARBITER] %s: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
        log_line,
        _jnt_position_command[0],
        _jnt_position_command[1],
        _jnt_position_command[2],
        _jnt_position_command[3],
        _jnt_position_command[4],
        _jnt_position_command[5]
    );
    return true;
}

void FairinoDualHardwareInterface::process_pending_gripper()
{
    if (!_ptr_robot) {
        return;
    }

    const RuntimeMode mode = _runtime_mode.load();
    if (mode == RuntimeMode::GRIPPER_WAIT) {
        poll_gripper_wait();
        return;
    }
    if (mode == RuntimeMode::SERVO_RESTART) {
        restart_servo_after_gripper();
        return;
    }

    std::shared_ptr<PendingGripperCommand> cmd;
    {
        std::lock_guard<std::mutex> lock(_gripper_mutex);
        cmd = _gripper_pending;
    }
    if (!cmd || cmd->finished) {
        if (mode == RuntimeMode::GRIPPER_COMMAND) {
            _runtime_mode.store(RuntimeMode::SERVO_RESTART);
            restart_servo_after_gripper();
        }
        return;
    }
    begin_gripper_command(cmd);
}

void FairinoDualHardwareInterface::begin_gripper_command(
    const std::shared_ptr<PendingGripperCommand> & cmd)
{
    const double max_err = max_command_state_error();
    RCLCPP_INFO(
        rclcpp::get_logger("FairinoDualHardwareInterface"),
        "[GRIPPER ARBITER] request received cmd=%s id=%d pos=%d vel=%d force=%d "
        "max_time_ms=%d max(|command-state|)=%.6f rad",
        cmd->command.c_str(),
        cmd->gripper_id,
        cmd->position,
        cmd->velocity,
        cmd->force,
        cmd->max_time_ms,
        max_err
    );

    if (cmd->command != "activate" && cmd->command != "reset" &&
        cmd->command != "move") {
        finish_gripper_pending(-1, "unknown gripper command");
        _runtime_mode.store(RuntimeMode::SERVO_ACTIVE);
        return;
    }

    if (max_err > kArmSettledThresholdRad) {
        const auto now = std::chrono::steady_clock::now();
        if (now < cmd->arm_settle_deadline) {
            if (!cmd->arm_settle_wait_logged) {
                cmd->arm_settle_wait_logged = true;
                const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                    cmd->arm_settle_deadline - now).count();
                RCLCPP_INFO(
                    rclcpp::get_logger("FairinoDualHardwareInterface"),
                    "[GRIPPER ARBITER] READY_WAIT command_state_error=%.6f rad "
                    "threshold=%.6f rad timeout_remaining_ms=%ld; ServoJ continues",
                    max_err, kArmSettledThresholdRad, static_cast<long>(remaining));
            }
            return;
        }
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "[GRIPPER ARBITER] READY_WAIT_TIMEOUT command_state_error=%.6f rad "
            "threshold=%.6f rad; gripper command rejected",
            max_err, kArmSettledThresholdRad
        );
        finish_gripper_pending(-1, "arm command did not settle before gripper command timeout");
        _runtime_mode.store(RuntimeMode::SERVO_ACTIVE);
        return;
    }

    _runtime_mode.store(RuntimeMode::GRIPPER_COMMAND);

    const errno_t end_ret = _ptr_robot->ServoMoveEnd();
    RCLCPP_INFO(
        rclcpp::get_logger("FairinoDualHardwareInterface"),
        "[GRIPPER ARBITER] ServoMoveEnd ret=%d",
        end_ret
    );
    if (end_ret != 0) {
        finish_gripper_pending(end_ret, "ServoMoveEnd failed");
        _runtime_mode.store(RuntimeMode::SERVO_ACTIVE);
        return;
    }

    if (!sync_joints_from_actual("synchronized joint commands")) {
        _gripper_result_code = -1;
        _gripper_result_message = "GetActualJointPosDegree failed after ServoMoveEnd";
        _runtime_mode.store(RuntimeMode::SERVO_RESTART);
        restart_servo_after_gripper();
        return;
    }

    int code = -1;
    std::string message = "unknown gripper command";
    bool wait_for_motion = false;

    if (cmd->command == "activate") {
        code = _ptr_robot->ActGripper(cmd->gripper_id, 1);
        message = "ActGripper(1)";
        RCLCPP_INFO(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "[GRIPPER ARBITER] ActGripper(1) ret=%d",
            code
        );
    } else if (cmd->command == "reset") {
        code = _ptr_robot->ActGripper(cmd->gripper_id, 0);
        message = "ActGripper(0)";
        RCLCPP_INFO(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "[GRIPPER ARBITER] ActGripper(0) ret=%d",
            code
        );
    } else if (cmd->command == "move") {
        // SDK：block 0=阻塞，1=非阻塞。write() 里必须非阻塞。
        if (cmd->block == 0) {
            RCLCPP_WARN(
                rclcpp::get_logger("FairinoDualHardwareInterface"),
                "忽略 MoveGripper block=0，强制非阻塞以免卡住 write()"
            );
        }
        code = _ptr_robot->MoveGripper(
            cmd->gripper_id,
            cmd->position,
            cmd->velocity,
            cmd->force,
            cmd->max_time_ms,
            1,
            cmd->gripper_type,
            cmd->rot_num,
            cmd->rot_vel,
            cmd->rot_torque
        );
        message = "MoveGripper";
        RCLCPP_INFO(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "[GRIPPER ARBITER] MoveGripper ret=%d id=%d pos=%d vel=%d force=%d",
            code,
            cmd->gripper_id,
            cmd->position,
            cmd->velocity,
            cmd->force
        );
        wait_for_motion = (code == 0);
    }

    _gripper_result_code = code;
    _gripper_result_message = message;

    if (wait_for_motion) {
        const auto now = std::chrono::steady_clock::now();
        const int motion_ms = std::max(cmd->max_time_ms, 1000);
        _gripper_wait_start = now;
        _gripper_wait_deadline = now + std::chrono::milliseconds(motion_ms);
        _gripper_last_poll = now;
        _gripper_seen_in_motion = false;
        _logged_waiting_gripper = false;
        _runtime_mode.store(RuntimeMode::GRIPPER_WAIT);
        poll_gripper_wait();
        return;
    }

    if (code != 0) {
        _gripper_result_message = message + " failed";
    }
    _runtime_mode.store(RuntimeMode::SERVO_RESTART);
    restart_servo_after_gripper();
}

void FairinoDualHardwareInterface::poll_gripper_wait()
{
    const auto now = std::chrono::steady_clock::now();
    if (!_logged_waiting_gripper) {
        RCLCPP_INFO(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "[GRIPPER ARBITER] waiting gripper..."
        );
        _logged_waiting_gripper = true;
    }

    if (now >= _gripper_wait_deadline) {
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "[GRIPPER ARBITER] gripper done / fault timeout"
        );
        _gripper_result_code = -1;
        _gripper_result_message = "GetGripperMotionDone timeout";
        _runtime_mode.store(RuntimeMode::SERVO_RESTART);
        restart_servo_after_gripper();
        return;
    }

    if (now - _gripper_last_poll < kGripperPollPeriod) {
        return;
    }
    _gripper_last_poll = now;

    uint16_t fault = 0;
    uint8_t status = 0;
    const errno_t ret = _ptr_robot->GetGripperMotionDone(&fault, &status);
    if (ret != 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "[GRIPPER ARBITER] gripper done / fault sdk_ret=%d",
            ret
        );
        _gripper_result_code = ret;
        _gripper_result_message = "GetGripperMotionDone error";
        _runtime_mode.store(RuntimeMode::SERVO_RESTART);
        restart_servo_after_gripper();
        return;
    }
    if (fault != 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoDualHardwareInterface"),
            "[GRIPPER ARBITER] gripper done / fault fault=%u status=%u",
            static_cast<unsigned>(fault),
            static_cast<unsigned>(status)
        );
        _gripper_result_code = -1;
        _gripper_result_message = "gripper fault";
        _runtime_mode.store(RuntimeMode::SERVO_RESTART);
        restart_servo_after_gripper();
        return;
    }
    if (status == 0) {
        _gripper_seen_in_motion = true;
        return;
    }
    if (status != 1) {
        return;
    }

    const bool already_at_target =
        !_gripper_seen_in_motion && (now - _gripper_wait_start >= kStaleDoneGrace);
    if (!_gripper_seen_in_motion && !already_at_target) {
        return;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("FairinoDualHardwareInterface"),
        "[GRIPPER ARBITER] gripper done / fault fault=0 status=1 seen_motion=%d",
        _gripper_seen_in_motion ? 1 : 0
    );
    _gripper_result_code = 0;
    _gripper_result_message = "MoveGripper done";
    _runtime_mode.store(RuntimeMode::SERVO_RESTART);
    restart_servo_after_gripper();
}

void FairinoDualHardwareInterface::restart_servo_after_gripper()
{
    if (!sync_joints_from_actual("synchronized joints before restart")) {
        const auto now = std::chrono::steady_clock::now();
        if (now - _last_restart_error_log >= kRestartLogThrottle) {
            _last_restart_error_log = now;
            RCLCPP_ERROR(
                rclcpp::get_logger("FairinoDualHardwareInterface"),
                "[GRIPPER ARBITER] skip ServoMoveStart until joint read succeeds"
            );
        }
        return;
    }

    const errno_t start_ret = _ptr_robot->ServoMoveStart();
    RCLCPP_INFO(
        rclcpp::get_logger("FairinoDualHardwareInterface"),
        "[GRIPPER ARBITER] ServoMoveStart ret=%d",
        start_ret
    );
    if (start_ret != 0) {
        const auto now = std::chrono::steady_clock::now();
        if (now - _last_restart_error_log >= kRestartLogThrottle) {
            _last_restart_error_log = now;
            RCLCPP_ERROR(
                rclcpp::get_logger("FairinoDualHardwareInterface"),
                "[GRIPPER ARBITER] ServoMoveStart failed, will retry"
            );
        }
        return;
    }

    _runtime_mode.store(RuntimeMode::SERVO_ACTIVE);
    RCLCPP_INFO(
        rclcpp::get_logger("FairinoDualHardwareInterface"),
        "[GRIPPER ARBITER] ServoJ resumed"
    );

    const int code = _gripper_result_code;
    const std::string message = (code == 0)
        ? (_gripper_result_message.empty() ? "ok" : _gripper_result_message)
        : _gripper_result_message;
    finish_gripper_pending(code, message);
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware_dual::FairinoDualHardwareInterface, hardware_interface::SystemInterface)
