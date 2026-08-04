#ifndef SO_ARM_101_HARDWARE_INTERFACE__SO101_SYSTEM_HPP_
#define SO_ARM_101_HARDWARE_INTERFACE__SO101_SYSTEM_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace so_arm_101_hardware_interface
{

class SO101System final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SO101System)

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

  ~SO101System() override;

private:
  void state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void stop_ros_io();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Executor::SharedPtr executor_;

  std::vector<std::string> joint_names_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> received_positions_;
  std::vector<double> received_velocities_;
  std::vector<bool> received_joints_;
  std::mutex state_mutex_;
  bool state_received_{false};
  std_msgs::msg::Float64MultiArray command_message_;
  std::string state_topic_;
  std::string command_topic_;
};

}  // namespace so_arm_101_hardware_interface

#endif  // SO_ARM_101_HARDWARE_INTERFACE__SO101_SYSTEM_HPP_
