"""Fail-safe Xbox joystick teleoperation for the mecanum base."""

from __future__ import annotations

import math

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


def shaped_axis(axes, index: int, deadzone: float) -> float:
    """Return a finite joystick axis with a rescaled deadzone."""
    if index < 0 or index >= len(axes):
        return 0.0
    value = float(axes[index])
    if not math.isfinite(value) or abs(value) <= deadzone:
        return 0.0
    magnitude = (abs(value) - deadzone) / (1.0 - deadzone)
    return math.copysign(min(magnitude, 1.0), value)


def button_pressed(buttons, index: int) -> bool:
    """Safely read one joystick button."""
    return 0 <= index < len(buttons) and bool(buttons[index])


def limit_planar_velocity(linear_x: float, linear_y: float, maximum: float):
    """Scale mecanum planar velocity without changing its direction."""
    magnitude = abs(linear_x) + abs(linear_y)
    if magnitude <= maximum or magnitude == 0.0:
        return linear_x, linear_y
    scale = maximum / magnitude
    return linear_x * scale, linear_y * scale


def limit_mecanum_command(
    linear_x: float, linear_y: float, angular_z: float,
    wheel_linear_speed: float, kinematic_lever: float,
):
    """Scale X/Y/yaw together so no mecanum wheel exceeds its limit."""
    requested = (
        abs(linear_x) + abs(linear_y) + kinematic_lever * abs(angular_z))
    if requested <= wheel_linear_speed or requested == 0.0:
        return linear_x, linear_y, angular_z
    scale = wheel_linear_speed / requested
    return linear_x * scale, linear_y * scale, angular_z * scale


class XboxBaseTeleop(Node):
    """Publish mecanum velocity commands only while the enable button is held."""

    def __init__(self) -> None:
        super().__init__('xbox_base_teleop')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_angular_z', 3)
        self.declare_parameter('enable_button', 5)
        self.declare_parameter('turbo_button', 4)
        self.declare_parameter('deadzone', 0.10)
        self.declare_parameter('max_linear_x', 0.35)
        self.declare_parameter('max_linear_y', 0.35)
        self.declare_parameter('max_angular_z', 1.20)
        self.declare_parameter('max_linear_speed', 0.238)
        self.declare_parameter('wheel_linear_speed_limit', 0.238)
        self.declare_parameter('kinematic_lever', 0.2225)
        self.declare_parameter('turbo_linear_x', 0.70)
        self.declare_parameter('turbo_linear_y', 0.70)
        self.declare_parameter('turbo_angular_z', 2.00)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_sec', 0.30)

        self._axis_x = int(self.get_parameter('axis_linear_x').value)
        self._axis_y = int(self.get_parameter('axis_linear_y').value)
        self._axis_yaw = int(self.get_parameter('axis_angular_z').value)
        self._enable_button = int(self.get_parameter('enable_button').value)
        self._turbo_button = int(self.get_parameter('turbo_button').value)
        self._deadzone = float(self.get_parameter('deadzone').value)
        self._max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self._wheel_linear_speed_limit = float(
            self.get_parameter('wheel_linear_speed_limit').value)
        self._kinematic_lever = float(self.get_parameter('kinematic_lever').value)
        self._normal = (
            float(self.get_parameter('max_linear_x').value),
            float(self.get_parameter('max_linear_y').value),
            float(self.get_parameter('max_angular_z').value),
        )
        self._turbo = (
            float(self.get_parameter('turbo_linear_x').value),
            float(self.get_parameter('turbo_linear_y').value),
            float(self.get_parameter('turbo_angular_z').value),
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._timeout = float(self.get_parameter('joy_timeout_sec').value)
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError('deadzone must be in [0, 1)')
        if rate <= 0.0 or self._timeout <= 0.0:
            raise ValueError('publish_rate_hz and joy_timeout_sec must be positive')
        if (min(*self._normal, *self._turbo) <= 0.0
                or self._max_linear_speed <= 0.0
                or self._wheel_linear_speed_limit <= 0.0
                or self._kinematic_lever <= 0.0):
            raise ValueError('all velocity limits must be positive')

        self._last_joy_ns: int | None = None
        self._enabled = False
        self._stop_sent = True
        self._command = (0.0, 0.0, 0.0)
        self._publisher = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_vel_topic').value, 1)
        self.create_subscription(
            Joy, self.get_parameter('joy_topic').value,
            self._joy_callback, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self._publish_cycle)
        self.get_logger().info(
            'Xbox base teleop ready: hold RB to drive; hold LB for turbo.')

    def _joy_callback(self, message: Joy) -> None:
        self._last_joy_ns = self.get_clock().now().nanoseconds
        enabled = button_pressed(message.buttons, self._enable_button)
        if not enabled:
            if self._enabled:
                self._publish_stop()
            self._enabled = False
            self._command = (0.0, 0.0, 0.0)
            return

        turbo = button_pressed(message.buttons, self._turbo_button)
        limits = self._turbo if turbo else self._normal
        linear_x = limits[0] * shaped_axis(message.axes, self._axis_x, self._deadzone)
        linear_y = limits[1] * shaped_axis(message.axes, self._axis_y, self._deadzone)
        linear_x, linear_y = limit_planar_velocity(
            linear_x, linear_y, self._max_linear_speed)
        angular_z = limits[2] * shaped_axis(
            message.axes, self._axis_yaw, self._deadzone)
        self._command = limit_mecanum_command(
            linear_x,
            linear_y,
            angular_z,
            self._wheel_linear_speed_limit,
            self._kinematic_lever,
        )
        self._enabled = True
        self._stop_sent = False

    def _publish_cycle(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        connected = (
            self._last_joy_ns is not None
            and (now_ns - self._last_joy_ns) / 1e9 <= self._timeout)
        if self._enabled and connected:
            self._publish(*self._command)
        elif not self._stop_sent:
            self._enabled = False
            self._command = (0.0, 0.0, 0.0)
            self._publish_stop()

    def _publish_stop(self) -> None:
        self._publish(0.0, 0.0, 0.0)
        self._stop_sent = True

    def _publish(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_footprint'
        message.twist.linear.x = linear_x
        message.twist.linear.y = linear_y
        message.twist.angular.z = angular_z
        self._publisher.publish(message)


def main(args=None) -> int:
    """Run the Xbox base teleoperation node."""
    rclpy.init(args=args)
    node = XboxBaseTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
