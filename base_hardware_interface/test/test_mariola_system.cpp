#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "base_hardware_interface/mariola_system.hpp"
#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"

using namespace std::chrono_literals;

namespace base_hardware_interface
{

class MariolaSystemTestPeer
{
public:
  static void configure(
    MariolaSystem & system, const rclcpp::Executor::SharedPtr & executor,
    std::chrono::duration<double> timeout = 0.25s)
  {
    system.executor_ = executor;
    system.joint_names_ = {
      "front_left_wheel_joint", "front_right_wheel_joint",
      "rear_left_wheel_joint", "rear_right_wheel_joint"};
    system.commands_.assign(4, 0.0);
    system.positions_.assign(4, 0.0);
    system.velocities_.assign(4, 0.0);
    system.received_positions_.assign(4, 0.0);
    system.received_velocities_.assign(4, 0.0);
    system.state_topic_ = "/test_mariola/raw_joint_states";
    system.command_topic_ = "/test_mariola/command_velocities";
    system.state_timeout_ = timeout;
  }

  static void set_commands(MariolaSystem & system, std::vector<double> commands)
  {
    system.commands_ = std::move(commands);
  }
};

class MariolaSystemTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(MariolaSystemTest, lifecycle_interfaces_and_stale_state)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto test_node = std::make_shared<rclcpp::Node>("mariola_system_test_node");
  executor->add_node(test_node);

  MariolaSystem system;
  MariolaSystemTestPeer::configure(system, executor, 0.10s);
  EXPECT_EQ(system.export_command_interfaces().size(), 4u);
  EXPECT_EQ(system.export_state_interfaces().size(), 8u);
  EXPECT_EQ(
    system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  std::vector<interfaces::msg::WheelCommand> commands;
  auto command_subscription = test_node->create_subscription<interfaces::msg::WheelCommand>(
    "/test_mariola/command_velocities", rclcpp::QoS(1).best_effort(),
    [&commands](const interfaces::msg::WheelCommand & message) {
      commands.push_back(message);
    });
  auto state_publisher = test_node->create_publisher<sensor_msgs::msg::JointState>(
    "/test_mariola/raw_joint_states", rclcpp::QoS(1).best_effort());

  // Activation alone must not generate commands before the first complete state.
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  executor->spin_some();
  EXPECT_TRUE(commands.empty());

  sensor_msgs::msg::JointState state;
  state.name = {
    "front_left_wheel_joint", "front_right_wheel_joint",
    "rear_left_wheel_joint", "rear_right_wheel_joint"};
  state.position = {1.0, 2.0, 3.0, 4.0};
  state.velocity = {0.1, 0.2, 0.3, 0.4};
  state_publisher->publish(state);
  for (int attempt = 0; attempt < 10; ++attempt) {
    executor->spin_some();
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  MariolaSystemTestPeer::set_commands(system, {14.0, 7.0, -14.0, -3.5});
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  executor->spin_some();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front().name, state.name);
  EXPECT_EQ(commands.front().velocity, std::vector<double>({7.0, 3.5, -7.0, -1.75}));

  std::this_thread::sleep_for(110ms);
  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(
    system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  executor->remove_node(test_node);
}

}  // namespace base_hardware_interface
