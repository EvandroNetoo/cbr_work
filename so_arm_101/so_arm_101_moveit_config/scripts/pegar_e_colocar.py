#!/usr/bin/env python3
"""Sequência de pegar e colocar um objeto usando MoveIt 2 no ROS 2 Jazzy."""

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
from shape_msgs.msg import SolidPrimitive


# ============================ CONFIGURAÇÃO ============================

ARM_GROUP = "arm"
GRIPPER_GROUP = "gripper"
BASE_FRAME = "base_link"
END_EFFECTOR_LINK = "gripper_tcp"

# Estas são as únicas coordenadas necessárias para o pick-and-place.
# Todas estão em metros e no referencial base_link.
OBJECT_X = -0.10
OBJECT_Y = -0.15
OBJECT_Z = 0.025
APPROACH_HEIGHT = 0.10
PLACE_X = 0.10
PLACE_Y = -0.15
PLACE_Z = 0.025
OBJECT_ANGLE_DEG = 90.0
PLACE_ANGLE_DEG = 90.0

POSITION_TOLERANCE = 0.01
# Pega por cima: uma rotação de +90 graus em X aponta o eixo local -Y da
# garra para baixo. Como o TCP está 0.10 m em -Y, link5_1 fica acima do objeto.
TILT_TOLERANCE = 0.20
ANGLE_TOLERANCE = math.radians(5.0)

# Conforme os estados definidos no SO-ARM-101.srdf.
GRIPPER_OPEN_POSITION = 0.037
GRIPPER_CLOSED_POSITION = 0.015

PLANNING_TIME = 15.0
PLANNING_ATTEMPTS = 10
MAX_VELOCITY = 0.25
MAX_ACCELERATION = 0.25

# =======================================================================


class PickAndPlace:
    def __init__(self):
        self.node = rclpy.create_node("pegar_e_colocar")
        self.move_group = ActionClient(self.node, MoveGroup, "/move_action")

    def wait_for_moveit(self):
        self.node.get_logger().info("Aguardando o servidor de planejamento do MoveIt...")
        if not self.move_group.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(
                "Servidor /move_action não encontrado. Inicie o real_moveit.launch.py."
            )

    @staticmethod
    def pose(x, y, z, angle_deg):
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
    def position_constraints(pose):
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
    def gripper_constraints(position):
        constraints = Constraints()
        joint = JointConstraint()
        joint.joint_name = "right_clamp"
        joint.position = position
        joint.tolerance_above = 0.001
        joint.tolerance_below = 0.001
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
        return constraints

    def execute_goal(self, group, constraints):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal.request.allowed_planning_time = PLANNING_TIME
        goal.request.max_velocity_scaling_factor = MAX_VELOCITY
        goal.request.max_acceleration_scaling_factor = MAX_ACCELERATION
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

    def move_arm(self, pose, description):
        self.node.get_logger().info(description)
        self.execute_goal(ARM_GROUP, self.position_constraints(pose))

    def move_gripper(self, position, description):
        self.node.get_logger().info(description)
        self.execute_goal(GRIPPER_GROUP, self.gripper_constraints(position))
        time.sleep(5)

    def run(self):
        self.wait_for_moveit()

        object_pose = self.pose(OBJECT_X, OBJECT_Y, OBJECT_Z, OBJECT_ANGLE_DEG)
        object_above = self.pose(
            OBJECT_X, OBJECT_Y, OBJECT_Z + APPROACH_HEIGHT, OBJECT_ANGLE_DEG
        )
        place_pose = self.pose(PLACE_X, PLACE_Y, PLACE_Z, PLACE_ANGLE_DEG)
        place_above = self.pose(
            PLACE_X, PLACE_Y, PLACE_Z + APPROACH_HEIGHT, PLACE_ANGLE_DEG
        )

        # self.move_gripper(GRIPPER_OPEN_POSITION, "Abrindo a garra")
        self.move_arm(object_above, "Indo para cima do objeto")
        self.move_arm(object_pose, "Descendo até o objeto")
        self.move_gripper(GRIPPER_CLOSED_POSITION, "Fechando a garra")
        self.move_arm(object_above, "Levantando o objeto")
        self.move_arm(place_above, "Indo para a aproximação do destino")
        self.move_arm(place_pose, "Indo para a posição de colocação")
        self.move_gripper(GRIPPER_OPEN_POSITION, "Abrindo a garra")
        self.move_arm(place_above, "Levantando a garra")
        self.node.get_logger().info("Sequência concluída")


def main():
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
