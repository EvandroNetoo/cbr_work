#!/usr/bin/env python3
"""Sequência de pegar e colocar um objeto usando MoveIt 2 no ROS 2 Jazzy."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Final, TypeAlias

import sys
import time
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


# ============================ CONFIGURAÇÃO ============================

JointPositions: TypeAlias = Mapping[str, float]

ARM_GROUP: Final[str] = "arm"
GRIPPER_GROUP: Final[str] = "gripper"
BASE_FRAME: Final[str] = "base_link"
END_EFFECTOR_LINK: Final[str] = "gripper_tcp"

# Estas são as únicas coordenadas necessárias para o pick-and-place.
# Todas estão em metros e no referencial base_link.
OBJECT_X: Final[float] = 0.0
OBJECT_Y: Final[float] = -0.15
OBJECT_Z: Final[float] = 0.0
OBJECT_ANGLE_DEG: Final[float] = 0.0
APPROACH_HEIGHT: Final[float] = 0.05

POSITION_TOLERANCE: Final[float] = 0.01
# Pega por cima: uma rotação de +90 graus em X aponta o eixo local -Y da
# garra para baixo. Como o TCP está 0.10 m em -Y, link5_1 fica acima do objeto.
TILT_TOLERANCE: Final[float] = 0.20
ANGLE_TOLERANCE: Final[float] = math.radians(5.0)

# Conforme os estados definidos no SO-ARM-101.srdf.
GRIPPER_OPEN_POSITION: Final[float] = 0.037
GRIPPER_CLOSED_POSITION: Final[float] = 0.023
GRIPPER_JOINT_TOLERANCE: Final[float] = 0.001

# Pose ``home`` definida em so_arm_101.srdf. Como este script envia a ação
# MoveGroup diretamente, ela é representada pelas restrições articulares.
HOME_JOINTS: Final[JointPositions] = {
    "base_link_to_link1": 1.57,
    "link1_to_link2": 1.83,
    "link2_to_link3": -1.6,
    "link3_to_link4": 1.7,
    "link4_to_link5": 0.0,
}
HOME_JOINT_TOLERANCE: Final[float] = 0.01


DEPOSIT_CUBE_LEFT_JOINTS: Final[JointPositions] = {
    "base_link_to_link1": 1.57,
    "link1_to_link2": 0.98,
    "link2_to_link3": -1.13,
    "link3_to_link4": -1.56,
    "link4_to_link5": 0.0
}


PLANNING_TIME: Final[float] = 15.0
PLANNING_ATTEMPTS: Final[int] = 10
MAX_VELOCITY: Final[float] = 0.9
MAX_ACCELERATION: Final[float] = 0.9
GRIPPER_MAX_VELOCITY: Final[float] = 1.0
GRIPPER_MAX_ACCELERATION: Final[float] = 1.0

# =======================================================================


class PickAndPlace:
    def __init__(self) -> None:
        self.node = rclpy.create_node("pegar_e_colocar")
        self.move_group = ActionClient(self.node, MoveGroup, "/move_action")
        self.current_joint_positions: dict[str, float] = {}
        self.joint_state_sequence: int = 0
        self.joint_states_subscription = self.node.create_subscription(
            JointState, "/joint_states", self._joint_states_callback, 10
        )

    def _joint_states_callback(self, message: JointState) -> None:
        self.current_joint_positions.update(zip(message.name, message.position))
        self.joint_state_sequence += 1

    def wait_for_moveit(self) -> None:
        self.node.get_logger().info("Aguardando o servidor de planejamento do MoveIt...")
        if not self.move_group.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(
                "Servidor /move_action não encontrado. Inicie o real_moveit.launch.py."
            )

    @staticmethod
    def pose(x: float, y: float, z: float, angle_deg: float) -> PoseStamped:
        # Rz(angle) * Rx(90°): mantém a pega por cima e gira os dedos para
        # acompanhar o ângulo do objeto no plano XY.
        half_angle = math.radians(angle_deg) / 2.0
        sqrt_half = math.sqrt(0.5)

        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = sqrt_half * math.cos(half_angle)
        pose.pose.orientation.y = sqrt_half * math.sin(half_angle)
        pose.pose.orientation.z = sqrt_half * math.sin(half_angle)
        pose.pose.orientation.w = sqrt_half * math.cos(half_angle)
        return pose

    @staticmethod
    def position_constraints(pose: PoseStamped) -> Constraints:
        constraints = Constraints()

        position = PositionConstraint()
        position.header = pose.header
        position.link_name = END_EFFECTOR_LINK
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [POSITION_TOLERANCE]
        position.constraint_region.primitives.append(primitive)
        position.constraint_region.primitive_poses.append(pose.pose)
        position.weight = 1.0

        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header = pose.header
        orientation.link_name = END_EFFECTOR_LINK
        orientation.orientation = pose.pose.orientation
        # X/Z limitam a inclinação e Y controla o alinhamento visto de cima.
        orientation.absolute_x_axis_tolerance = TILT_TOLERANCE
        orientation.absolute_y_axis_tolerance = ANGLE_TOLERANCE
        orientation.absolute_z_axis_tolerance = TILT_TOLERANCE
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        return constraints

    @staticmethod
    def gripper_constraints(position: float) -> Constraints:
        constraints = Constraints()
        joint = JointConstraint()
        joint.joint_name = "right_clamp"
        joint.position = position
        joint.tolerance_above = GRIPPER_JOINT_TOLERANCE
        joint.tolerance_below = GRIPPER_JOINT_TOLERANCE
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
        return constraints

    def move_to_joint_positions(
        self,
        joint_positions: JointPositions,
        description: str,
        tolerance: float = HOME_JOINT_TOLERANCE,
    ) -> None:
        """Planeja e executa um movimento para as posições das juntas dadas."""
        if not joint_positions:
            raise ValueError("Informe ao menos uma posição de junta.")

        self.node.get_logger().info(description)
        constraints = Constraints()
        for joint_name, position in joint_positions.items():
            joint = JointConstraint()
            joint.joint_name = joint_name
            joint.position = position
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        self.execute_goal(ARM_GROUP, constraints)
        time.sleep(1)

    def execute_goal(self, group: str, constraints: Constraints) -> None:
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal.request.allowed_planning_time = PLANNING_TIME
        goal.request.max_velocity_scaling_factor = (
            GRIPPER_MAX_VELOCITY if group == GRIPPER_GROUP else MAX_VELOCITY
        )
        goal.request.max_acceleration_scaling_factor = (
            GRIPPER_MAX_ACCELERATION if group == GRIPPER_GROUP else MAX_ACCELERATION
        )
        goal.request.goal_constraints = [constraints]
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True

        send_future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Objetivo rejeitado pelo MoveIt para o grupo '{group}'.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        action_result = result_future.result()
        if action_result is None or action_result.status != GoalStatus.STATUS_SUCCEEDED:
            error_code = getattr(action_result.result, "error_code", None) if action_result else None
            value = getattr(error_code, "val", "desconhecido")
            self.node.get_logger().error(
                f"MoveIt não conseguiu executar o grupo '{group}'. "
                f"Código de erro: {value}"
            )
            raise RuntimeError(f"Movimento falhou no MoveIt (código {value}).")

    def move_arm(self, pose: PoseStamped, description: str) -> None:
        self.node.get_logger().info(description)
        self.execute_goal(ARM_GROUP, self.position_constraints(pose))
        time.sleep(1)

    def move_home(self) -> None:
        self.move_to_joint_positions(HOME_JOINTS, "Indo para a pose home")

    def move_gripper(self, position: float, description: str) -> None:
        self.node.get_logger().info(description)
        self.execute_goal(GRIPPER_GROUP, self.gripper_constraints(position))
        time.sleep(1)

    def run(self) -> None:
        self.wait_for_moveit()
        self.move_home()

        # Definindo as poses para pegar o objeto
        object_pose = self.pose(OBJECT_X, OBJECT_Y, OBJECT_Z, OBJECT_ANGLE_DEG)
        object_above = self.pose(
            OBJECT_X, OBJECT_Y, OBJECT_Z + APPROACH_HEIGHT, OBJECT_ANGLE_DEG
        )

        # Executando a sequência de pegar objeto
        self.move_gripper(GRIPPER_OPEN_POSITION, "Abrindo a garra")
        self.move_arm(object_above, "Indo para cima do objeto")
        self.move_arm(object_pose, "Descendo até o objeto")
        self.move_gripper(GRIPPER_CLOSED_POSITION, "Fechando a garra")
        self.move_arm(object_above, "Levantando o objeto")

        # Executando a sequência de colocar objeto
        self.move_home()
        self.move_to_joint_positions(
            DEPOSIT_CUBE_LEFT_JOINTS, "Indo para o depósito do cubo à esquerda"
        )
        self.move_gripper(GRIPPER_OPEN_POSITION, "Abrindo a garra")

        # Retornando para a pose home
        self.move_home()
        self.node.get_logger().info("Sequência concluída")


def main() -> int:
    rclpy.init(args=sys.argv)
    task = PickAndPlace()
    try:
        task.run()
    except Exception as error:
        task.node.get_logger().error(f"SEQUÊNCIA INTERROMPIDA: {error}")
        return_code = 1
    else:
        return_code = 0
    finally:
        task.node.destroy_node()
        rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
