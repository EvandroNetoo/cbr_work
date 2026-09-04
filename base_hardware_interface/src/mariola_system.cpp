#include "base_hardware_interface/mariola_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace base_hardware_interface
{

hardware_interface::CallbackReturn MariolaSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("mariola_system"), "MariolaSystem requer quatro rodas.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(rclcpp::get_logger("mariola_system"),
        "A roda '%s' deve ter um único comando velocity.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto has_state = [&joint](const std::string & name) {
        return std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
          [&name](const auto & interface) {return interface.name == name;});
      };
    if (!has_state(hardware_interface::HW_IF_POSITION) ||
      !has_state(hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(rclcpp::get_logger("mariola_system"),
        "A roda '%s' deve expor estados position e velocity.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }
  executor_ = params.executor.lock();
  if (!executor_) {
    RCLCPP_ERROR(rclcpp::get_logger("mariola_system"), "Executor do controller_manager indisponível.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto parameter = [this](const std::string & name, const std::string & fallback) {
      const auto found = info_.hardware_parameters.find(name);
      return found == info_.hardware_parameters.end() ? fallback : found->second;
    };
  state_topic_ = parameter("state_topic", "/base_hardware/raw_joint_states");
  command_topic_ = parameter("command_topic", "/base_hardware/command_velocities");
  try {
    state_timeout_ = std::chrono::duration<double>(std::stod(parameter("state_timeout_sec", "0.25")));
    max_wheel_velocity_rad_s_ = std::stod(parameter("max_wheel_velocity_rad_s", "11.0"));
  } catch (const std::exception &) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (state_timeout_.count() <= 0.0 ||
    !std::isfinite(max_wheel_velocity_rad_s_) || max_wheel_velocity_rad_s_ <= 0.0)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto size = joint_names_.size();
  commands_.assign(size, std::numeric_limits<double>::quiet_NaN());
  positions_.assign(size, 0.0);
  velocities_.assign(size, 0.0);
  received_positions_.assign(size, 0.0);
  received_velocities_.assign(size, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MariolaSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  result.reserve(joint_names_.size() * 2);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    result.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &positions_[i]);
    result.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
  }
  return result;
}

std::vector<hardware_interface::CommandInterface> MariolaSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  result.reserve(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    result.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &commands_[i]);
  }
  return result;
}

hardware_interface::CallbackReturn MariolaSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_received_ = false;
  std::fill(commands_.begin(), commands_.end(), std::numeric_limits<double>::quiet_NaN());
  node_ = std::make_shared<rclcpp::Node>("mariola_system_io");
  // Local latest-value bridge: never stall controller_manager::write() waiting
  // for DDS reliability acknowledgements. The hardware watchdog supplies the
  // command-delivery safety contract.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  command_publisher_ = node_->create_publisher<interfaces::msg::WheelCommand>(command_topic_, qos);
  state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, qos, std::bind(&MariolaSystem::state_callback, this, std::placeholders::_1));
  executor_->add_node(node_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MariolaSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_ros_io();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MariolaSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_received_) {
    return hardware_interface::return_type::OK;
  }
  if (std::chrono::steady_clock::now() - last_state_time_ > state_timeout_) {
    RCLCPP_ERROR(rclcpp::get_logger("mariola_system"), "Estado das rodas expirou.");
    return hardware_interface::return_type::ERROR;
  }
  positions_ = received_positions_;
  velocities_ = received_velocities_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MariolaSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_publisher_) {
    return hardware_interface::return_type::ERROR;
  }
  if (!state_received_) {
    return hardware_interface::return_type::OK;
  }
  interfaces::msg::WheelCommand message;
  message.header.stamp = node_->now();
  message.name = joint_names_;
  message.velocity.resize(commands_.size());
  double largest_velocity = 0.0;
  for (size_t i = 0; i < commands_.size(); ++i) {
    message.velocity[i] = std::isfinite(commands_[i]) ? commands_[i] : 0.0;
    largest_velocity = std::max(largest_velocity, std::abs(message.velocity[i]));
  }
  if (largest_velocity > max_wheel_velocity_rad_s_) {
    const double scale = max_wheel_velocity_rad_s_ / largest_velocity;
    for (auto & velocity : message.velocity) {
      velocity = std::clamp(
        velocity * scale, -max_wheel_velocity_rad_s_, max_wheel_velocity_rad_s_);
    }
  }
  command_publisher_->publish(message);
  return hardware_interface::return_type::OK;
}

void MariolaSystem::state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (message->name.size() != message->position.size() ||
    message->name.size() != message->velocity.size())
  {
    return;
  }
  std::unordered_map<std::string, size_t> indices;
  for (size_t i = 0; i < message->name.size(); ++i) {
    if (!indices.emplace(message->name[i], i).second) {
      return;
    }
  }
  std::vector<double> new_positions(joint_names_.size());
  std::vector<double> new_velocities(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto found = indices.find(joint_names_[i]);
    if (found == indices.end()) {
      return;
    }
    new_positions[i] = message->position[found->second];
    new_velocities[i] = message->velocity[found->second];
    if (!std::isfinite(new_positions[i]) || !std::isfinite(new_velocities[i])) {
      return;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  received_positions_ = std::move(new_positions);
  received_velocities_ = std::move(new_velocities);
  last_state_time_ = std::chrono::steady_clock::now();
  state_received_ = true;
}

void MariolaSystem::stop_ros_io()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (node_ && executor_) {
    executor_->remove_node(node_);
  }
  state_subscription_.reset();
  command_publisher_.reset();
  node_.reset();
  state_received_ = false;
}

MariolaSystem::~MariolaSystem()
{
  stop_ros_io();
}

}  // namespace base_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  base_hardware_interface::MariolaSystem,
  hardware_interface::SystemInterface)
