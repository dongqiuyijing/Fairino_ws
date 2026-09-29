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
#include "fairino_hardware_dual/manual_cartesian.hpp"
#include "fairino_hardware_dual/manual_trace.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"


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
    std::chrono::steady_clock::time_point arm_settle_deadline{};
    bool arm_settle_wait_logged{false};
    std::condition_variable cv;
  };

  struct PendingManualRequest {
    bool enter{false};
    bool finished{false};
    bool success{false};
    std::string message;
    std::condition_variable cv;
  };

  void start_gripper_bridge();
  void stop_gripper_bridge();
  void start_manual_cartesian_bridge();
  void clear_manual_command_locked();
  std::string manual_namespace() const;
  void publish_runtime_mode();
  void publish_cartesian_error(int code);
  void on_manual_velocity(const std_msgs::msg::Float64::SharedPtr msg);
  void handle_manual_enter(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_manual_exit(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void wait_for_manual_request(
    const std::shared_ptr<PendingManualRequest> & pending,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool process_manual_transition();
  int issue_servocart(const ServoCartCommand & command);
  int traced_servo_session(bool start);
  void note_servocart_error(int code);
  void apply_boundary(const BoundaryOutcome & outcome, const char * action);
  void finish_manual_request(bool success, const std::string & message);
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
  ServoKind _resume_kind{ServoKind::SERVOJ};
  std::mutex _manual_mutex;
  double _manual_vx_mm_s{0.0};
  std::chrono::steady_clock::time_point _manual_command_time{};
  bool _manual_command_stamped{false};
  uint64_t _manual_rx_seq{0};
  ManualTrace _trace;
  ManualTraceRecord _cart_trace;
  bool _cartesian_output_inhibited{false};
  bool _watchdog_logged{false};
  bool _manual_rx_logged{false};
  std::chrono::steady_clock::time_point _last_manual_diag_log{};
  bool _exit_zero_sent{false};
  int _last_servocart_error{0};
  std::chrono::steady_clock::time_point _last_servocart_error_log{};
  std::mutex _manual_request_mutex;
  std::shared_ptr<PendingManualRequest> _manual_request;
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _manual_enter_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _manual_exit_service;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _manual_velocity_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _runtime_mode_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _cartesian_error_pub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _gripper_executor;
  std::thread _gripper_spin_thread;
};

} //end namespace


#endif
