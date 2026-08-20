#include "so_arm_101_hardware_interface/so101_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace so_arm_101_hardware_interface
{

hardware_interface::CallbackReturn SO101System::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.empty()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(rclcpp::get_logger("so101_system"),
        "Joint '%s' must expose exactly one position command interface.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto has_state_interface = [&joint](const std::string & name) {
        return std::any_of(
          joint.state_interfaces.begin(), joint.state_interfaces.end(),
          [&name](const auto & interface) { return interface.name == name; });
      };
    if (!has_state_interface(hardware_interface::HW_IF_POSITION) ||
      !has_state_interface(hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(rclcpp::get_logger("so101_system"),
        "Joint '%s' must expose position and velocity state interfaces.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  joint_names_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  executor_ = params.executor.lock();
  if (!executor_) {
    RCLCPP_ERROR(rclcpp::get_logger("so101_system"), "Controller manager executor indisponível.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto get_parameter = [this](const std::string & name, const std::string & fallback) {
      const auto it = info_.hardware_parameters.find(name);
      return it == info_.hardware_parameters.end() ? fallback : it->second;
    };
  state_topic_ = get_parameter("state_topic", "/so101_hardware/raw_joint_states");
  command_topic_ = get_parameter("command_topic", "/so101_hardware/command_positions");

  const auto size = joint_names_.size();
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  hw_commands_.assign(size, nan);
  hw_positions_.assign(size, 0.0);
  hw_velocities_.assign(size, 0.0);
  received_positions_.assign(size, nan);
  received_velocities_.assign(size, 0.0);
  received_joints_.assign(size, false);
  command_message_.data.assign(size, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SO101System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> SO101System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn SO101System::on_activate(
  const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_received_ = false;
    std::fill(received_joints_.begin(), received_joints_.end(), false);
  }
  node_ = std::make_shared<rclcpp::Node>("so101_system_io");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  command_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, qos);
  state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, qos,
    std::bind(&SO101System::state_callback, this, std::placeholders::_1));

  executor_->add_node(node_);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_received_) {
      hw_commands_ = received_positions_;
      hw_positions_ = received_positions_;
      hw_velocities_ = received_velocities_;
    } else {
      hw_commands_.assign(hw_commands_.size(), std::numeric_limits<double>::quiet_NaN());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SO101System::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_ros_io();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SO101System::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!state_received_) {
    return hardware_interface::return_type::OK;
  }
  hw_positions_ = received_positions_;
  hw_velocities_ = received_velocities_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SO101System::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!command_publisher_) {
    return hardware_interface::return_type::ERROR;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!state_received_) {
    return hardware_interface::return_type::OK;
  }
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    command_message_.data[i] = std::isfinite(hw_commands_[i]) ? hw_commands_[i] : hw_positions_[i];
  }
  command_publisher_->publish(command_message_);
  return hardware_interface::return_type::OK;
}

void SO101System::state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::fill(received_joints_.begin(), received_joints_.end(), false);
  for (size_t i = 0; i < message->name.size(); ++i) {
    const auto it = std::find(joint_names_.begin(), joint_names_.end(), message->name[i]);
    if (it == joint_names_.end() || i >= message->position.size()) {
      continue;
    }
    const auto index = static_cast<size_t>(std::distance(joint_names_.begin(), it));
    received_positions_[index] = message->position[i];
    if (i < message->velocity.size()) {
      received_velocities_[index] = message->velocity[i];
    }
    received_joints_[index] = true;
  }
  state_received_ = std::all_of(received_joints_.begin(), received_joints_.end(), [](bool value) {
      return value;
    });
}

void SO101System::stop_ros_io()
{
  if (node_ && executor_) {
    executor_->remove_node(node_);
  }
  state_subscription_.reset();
  command_publisher_.reset();
  node_.reset();
}

SO101System::~SO101System()
{
  stop_ros_io();
}

}  // namespace so_arm_101_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  so_arm_101_hardware_interface::SO101System,
  hardware_interface::SystemInterface)
