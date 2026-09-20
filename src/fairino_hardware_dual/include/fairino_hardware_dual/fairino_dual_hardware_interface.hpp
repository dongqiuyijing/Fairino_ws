#ifndef _FR_DUAL_HARDWARE_INTERFACE_
#define _FR_DUAL_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "fairino_hardware_dual/visibility_control.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "libfairino/include/robot.h"
#include "fairino_msgs/srv/gripper_bridge.hpp"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"

namespace fairino_hardware_dual
{

class FairinoDualHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoDualHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  struct PendingGripperCommand {
    std::string command;
    int gripper_id{1};
    int position{0};
    int velocity{20};
    int force{20};
    int max_time_ms{5000};
    int block{1};
    int gripper_type{0};
    double rot_num{0.0};
    int rot_vel{0};
    int rot_torque{0};
    int error_code{-1};
    std::string message;
    bool finished{false};
    std::condition_variable cv;
  };

  enum class RuntimeMode {
    SERVO_ACTIVE,
    GRIPPER_COMMAND,
    GRIPPER_WAIT,
    SERVO_RESTART
  };

  void start_gripper_bridge();
  void stop_gripper_bridge();
  void process_pending_gripper();
  void handle_gripper(
    const std::shared_ptr<fairino_msgs::srv::GripperBridge::Request> request,
    std::shared_ptr<fairino_msgs::srv::GripperBridge::Response> response);
  bool gripper_pending_active();
  void begin_gripper_command(const std::shared_ptr<PendingGripperCommand> & cmd);
  void poll_gripper_wait();
  void restart_servo_after_gripper();
  bool sync_joints_from_actual(const char * log_line);
  void finish_gripper_pending(int error_code, const std::string & message);
  double max_command_state_error() const;
  std::chrono::milliseconds gripper_service_wait(int max_time_ms) const;

  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  int _control_mode;
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;
  std::string _gripper_service_name = "/fairino_gripper/command";
  std::string _gripper_node_name = "fairino_gripper_bridge";
  std::unique_ptr<FRRobot> _ptr_robot;

  std::mutex _gripper_mutex;
  std::shared_ptr<PendingGripperCommand> _gripper_pending;
  std::atomic<RuntimeMode> _runtime_mode{RuntimeMode::SERVO_ACTIVE};
  std::chrono::steady_clock::time_point _gripper_wait_start{};
  std::chrono::steady_clock::time_point _gripper_wait_deadline{};
  std::chrono::steady_clock::time_point _gripper_last_poll{};
  std::chrono::steady_clock::time_point _last_restart_error_log{};
  int _gripper_result_code{-1};
  std::string _gripper_result_message;
  bool _gripper_seen_in_motion{false};
  bool _logged_waiting_gripper{false};
  rclcpp::Node::SharedPtr _gripper_node;
  rclcpp::Service<fairino_msgs::srv::GripperBridge>::SharedPtr _gripper_service;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _gripper_executor;
  std::thread _gripper_spin_thread;
};

} //end namespace


#endif
