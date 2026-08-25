#ifndef CBR_BASE_HARDWARE_INTERFACE__MARIOLA_SYSTEM_HPP_
#define CBR_BASE_HARDWARE_INTERFACE__MARIOLA_SYSTEM_HPP_

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "interfaces/msg/wheel_command.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace base_hardware_interface
{

class MariolaSystemTestPeer;

class MariolaSystem final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MariolaSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  ~MariolaSystem() override;

private:
  friend class MariolaSystemTestPeer;
  void state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void stop_ros_io();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<interfaces::msg::WheelCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Executor::SharedPtr executor_;
  std::vector<std::string> joint_names_;
  std::vector<double> commands_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> received_positions_;
  std::vector<double> received_velocities_;
  std::mutex mutex_;
  bool state_received_{false};
  bool state_stale_{false};
  std::chrono::steady_clock::time_point last_state_time_;
  std::chrono::duration<double> state_timeout_{0.25};
  double max_wheel_velocity_rad_s_{7.0};
  std::string state_topic_;
  std::string command_topic_;
};

}  // namespace base_hardware_interface

#endif
