"""ROS 2 node exposing a LeRobot SO101Follower as a simple arm interface."""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from .lerobot_adapter import (
    LEROBOT_TO_ROS,
    make_follower,
    observation_to_ros,
    ros_to_action,
)


class SO101HardwareNode(Node):
    def __init__(self):
        super().__init__('so101_hardware')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('robot_id', 'so101_follower')
        self.declare_parameter('use_degrees', False)
        self.declare_parameter('read_rate_hz', 30.0)
        self.declare_parameter('arm_command_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('gripper_command_topic', '/gripper_controller/joint_trajectory')
        self._lock = threading.Lock()
        self._connected = False
        self._last_positions = {}
        self._use_degrees = bool(self.get_parameter('use_degrees').value)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.arm_sub = self.create_subscription(
            JointTrajectory, self.get_parameter('arm_command_topic').value,
            self._trajectory_callback, 10)
        self.gripper_sub = self.create_subscription(
            JointTrajectory, self.get_parameter('gripper_command_topic').value,
            self._trajectory_callback, 10)
        self.calibrate_srv = self.create_service(Trigger, '~/calibrate', self._calibrate)
        self.follower = None

        try:
            self.follower = make_follower(
                self.get_parameter('port').value,
                self.get_parameter('robot_id').value,
                use_degrees=self._use_degrees,
            )
            self.follower.connect(calibrate=False)
            self._connected = True
            self.get_logger().info('SO-101 conectado via LeRobot/Feetech.')
        except Exception as error:  # Hardware errors must be visible in ROS logs.
            self.get_logger().fatal(f'Falha ao conectar o SO-101: {error}')

        period = 1.0 / float(self.get_parameter('read_rate_hz').value)
        self.timer = self.create_timer(period, self._read_observation)

    def _read_observation(self):
        if not self._connected:
            return
        try:
            with self._lock:
                observation = self.follower.get_observation()
            positions = observation_to_ros(observation, use_degrees=self._use_degrees)
            self._last_positions.update(positions)
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(self._last_positions)
            message.position = [self._last_positions[name] for name in message.name]
            self.joint_state_pub.publish(message)
        except Exception as error:
            self.get_logger().error(f'Falha ao ler os motores: {error}', throttle_duration_sec=2.0)

    def _trajectory_callback(self, message: JointTrajectory):
        if not self._connected or not message.points:
            return
        point = message.points[-1]
        values = dict(zip(message.joint_names, point.positions))
        if not values:
            return
        try:
            with self._lock:
                self.follower.send_action(ros_to_action(values, use_degrees=self._use_degrees))
        except Exception as error:
            self.get_logger().error(f'Falha ao comandar os motores: {error}', throttle_duration_sec=2.0)

    def _calibrate(self, request, response):
        del request
        if not self._connected:
            response.success = False
            response.message = 'Braço não conectado.'
            return response
        try:
            with self._lock:
                self.follower.calibrate()
            response.success = True
            response.message = 'Calibração concluída.'
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def destroy_node(self):
        if self._connected:
            try:
                with self._lock:
                    self.follower.disconnect()
            except Exception as error:
                self.get_logger().error(f'Falha ao desconectar o braço: {error}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SO101HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch may already have shut down the global context after SIGINT.
        if rclpy.ok():
            rclpy.shutdown()
