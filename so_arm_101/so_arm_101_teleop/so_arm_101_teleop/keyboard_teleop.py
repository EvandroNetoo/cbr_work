"""Keyboard teleoperation for all six actuators of the SO-ARM-101."""

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import String

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = (
    'base_link_to_link1',
    'link1_to_link2',
    'link2_to_link3',
    'link3_to_link4',
    'link4_to_link5',
)
GRIPPER_JOINT = 'right_clamp'
GRIPPER_OPEN_POSITION = 0.037
GRIPPER_CLOSED_POSITION = 0.0
GRIPPER_STEP = 0.005
JOINT_LIMITS = {
    'base_link_to_link1': (-2.094395, 2.094395),
    'link1_to_link2': (-3.228859, 0.174533),
    'link2_to_link3': (0.0, 3.316126),
    'link3_to_link4': (-1.658063, 1.658063),
    'link4_to_link5': (-4.276057, 1.570796),
    GRIPPER_JOINT: (GRIPPER_CLOSED_POSITION, GRIPPER_OPEN_POSITION),
}
KEY_BINDINGS = {
    'q': (ARM_JOINTS[0], 1), 'a': (ARM_JOINTS[0], -1),
    'w': (ARM_JOINTS[1], 1), 's': (ARM_JOINTS[1], -1),
    'e': (ARM_JOINTS[2], 1), 'd': (ARM_JOINTS[2], -1),
    'r': (ARM_JOINTS[3], 1), 'f': (ARM_JOINTS[3], -1),
    't': (ARM_JOINTS[4], 1), 'g': (ARM_JOINTS[4], -1),
    'y': (GRIPPER_JOINT, -1), 'h': (GRIPPER_JOINT, 1),
}


def clamp(value, limits):
    """Return value constrained to the inclusive joint limits."""
    return max(limits[0], min(limits[1], value))


def gripper_target(current_position, direction, step=GRIPPER_STEP):
    """Return the next gradual gripper position for a key direction."""
    return clamp(
        current_position + direction * step,
        JOINT_LIMITS[GRIPPER_JOINT],
    )


class KeyboardTeleop(Node):
    """Turn key presses into trajectory and gripper position commands."""

    def __init__(self):
        """Create publishers, subscriptions and the key reader."""
        super().__init__('keyboard_teleop')
        self.declare_parameter('arm_step', 0.10)
        self.declare_parameter('gripper_step', GRIPPER_STEP)
        self.declare_parameter('trajectory_duration', 0.15)
        self.arm_step = self.get_parameter('arm_step').value
        self.gripper_step = self.get_parameter('gripper_step').value
        self.trajectory_duration = self.get_parameter(
            'trajectory_duration').value

        self.targets = {}
        self.controller_manager_seen = False
        self.controller_manager_missing_checks = 0
        self.arm_publisher = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(
            String, '/keyboard_teleop/key', self._key_message_cb, 10)

        self.terminal_fd = None
        self.previous_terminal_settings = None
        self._configure_terminal()
        self.create_timer(0.02, self._read_keyboard)
        self.create_timer(0.5, self._check_controller_manager)
        self._print_help()

    def _configure_terminal(self):
        try:
            self.terminal_fd = os.open(
                '/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            self.previous_terminal_settings = termios.tcgetattr(
                self.terminal_fd)
            tty.setcbreak(self.terminal_fd)
        except (OSError, termios.error) as error:
            if self.terminal_fd is not None:
                os.close(self.terminal_fd)
                self.terminal_fd = None
            self.get_logger().error(
                f'Nao foi possivel acessar o teclado pelo terminal: {error}')

    def destroy_node(self):
        """Restore the terminal before destroying the ROS node."""
        if self.terminal_fd is not None:
            if self.previous_terminal_settings is not None:
                termios.tcsetattr(
                    self.terminal_fd, termios.TCSADRAIN,
                    self.previous_terminal_settings)
            os.close(self.terminal_fd)
            self.terminal_fd = None
        super().destroy_node()

    def _print_help(self):
        self.get_logger().info(
            '\nControle do SO-ARM-101 (a primeira tecla aumenta):\n'
            '  q/a: base       w/s: ombro     e/d: cotovelo\n'
            '  r/f: punho      t/g: rotacao   y/h: fechar/abrir garra\n'
            '  Ctrl-C: encerrar')

    def _joint_state_cb(self, message):
        for name, position in zip(message.name, message.position):
            if name in JOINT_LIMITS and name not in self.targets:
                self.targets[name] = clamp(
                    position, JOINT_LIMITS[name])

    def _read_keyboard(self):
        if self.terminal_fd is None:
            return
        readable, _, _ = select.select([self.terminal_fd], [], [], 0.0)
        if not readable:
            return
        try:
            key = os.read(self.terminal_fd, 1).decode(errors='ignore').lower()
        except BlockingIOError:
            return
        if key in KEY_BINDINGS:
            self.command_key(key)

    def _key_message_cb(self, message):
        """Accept a key over ROS, primarily for automated integration tests."""
        key = message.data[:1].lower()
        if key in KEY_BINDINGS:
            self.command_key(key)

    def _check_controller_manager(self):
        services = {
            name for name, _ in self.get_service_names_and_types()
        }
        manager_available = '/controller_manager/list_controllers' in services
        if manager_available:
            self.controller_manager_seen = True
            self.controller_manager_missing_checks = 0
        elif self.controller_manager_seen:
            self.controller_manager_missing_checks += 1
            if self.controller_manager_missing_checks >= 4:
                self.get_logger().info(
                    'Controller manager encerrou; fechando o teleop.')
                rclpy.shutdown()

    def command_key(self, key):
        """Apply a supported key and report whether it was commanded."""
        joint, direction = KEY_BINDINGS[key]
        if joint not in self.targets:
            self.get_logger().warning(
                'Aguardando /joint_states antes de aceitar comandos...',
                throttle_duration_sec=2.0)
            return False

        if joint == GRIPPER_JOINT:
            # Each key press advances the target, just like the arm joints.
            # Keeping the target locally also allows repeated presses while
            # the measured joint state is still catching up.
            self.targets[joint] = gripper_target(
                self.targets[joint], direction, self.gripper_step)
            message = JointTrajectory()
            message.joint_names = [GRIPPER_JOINT]
            point = JointTrajectoryPoint()
            point.positions = [self.targets[joint]]
            duration_ns = int(
                self.trajectory_duration * 1_000_000_000)
            point.time_from_start.sec = duration_ns // 1_000_000_000
            point.time_from_start.nanosec = duration_ns % 1_000_000_000
            message.points = [point]
            self.gripper_publisher.publish(message)
        else:
            self.targets[joint] = clamp(
                self.targets[joint] + direction * self.arm_step,
                JOINT_LIMITS[joint],
            )
            if not all(name in self.targets for name in ARM_JOINTS):
                self.get_logger().warning(
                    'Estado das cinco juntas ainda incompleto.')
                return False
            message = JointTrajectory()
            message.joint_names = list(ARM_JOINTS)
            point = JointTrajectoryPoint()
            point.positions = [self.targets[name] for name in ARM_JOINTS]
            duration_ns = int(
                self.trajectory_duration * 1_000_000_000)
            point.time_from_start.sec = duration_ns // 1_000_000_000
            point.time_from_start.nanosec = duration_ns % 1_000_000_000
            message.points = [point]
            self.arm_publisher.publish(message)

        self.get_logger().info(f'{joint}: {self.targets[joint]:.4f}')
        return True


def main(args=None):
    """Run the keyboard teleoperation node."""
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main(sys.argv)
