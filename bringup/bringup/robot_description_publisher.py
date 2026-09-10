"""Publish the robot description without duplicating the TF publisher."""

from __future__ import annotations

import traceback

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('hardware_robot_description_publisher')
        self.declare_parameter('robot_description', '')
        description = str(self.get_parameter('robot_description').value)
        if not description.strip():
            raise ValueError('O parâmetro robot_description não pode estar vazio.')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            String, '/robot_description', qos)
        self._publisher.publish(String(data=description))
        self.get_logger().info(
            'Descrição do robô publicada para o controller_manager.')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = RobotDescriptionPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        logger = (
            node.get_logger() if node is not None
            else get_logger('hardware_robot_description_publisher'))
        logger.fatal(f'Falha ao publicar robot_description: {error}')
        traceback.print_exc()
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
