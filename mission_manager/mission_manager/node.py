"""ROS action server that executes finite YAML missions."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from interfaces.action import ExecuteMission, PickCube, PlaceCube, RetrieveCube, StoreCube
from interfaces.msg import MissionStepResult
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .core import FAILED, LEFT, SLOTS, Inventory, MissionConfig, Step, load_config


STATE = {
    "IDLE": 0, "VALIDATING": 1, "NAVIGATING": 2, "PICKING": 3,
    "STORING": 4, "RETRIEVING": 5, "PLACING": 6,
    "SAFE_CANCEL": 7, "FINISHED": 8,
}
COMPLETE, PARTIAL, CRITICAL_FAILURE, CANCELED = range(4)


@dataclass
class CallResult:
    success: bool
    code: int
    known: bool
    message: str


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")
        default_file = str(
            __import__("pathlib").Path(get_package_share_directory("mission_manager"))
            / "config" / "missions.yaml"
        )
        self.declare_parameter("mission_file", default_file)
        self.declare_parameter("server_timeout", 5.0)
        self.config: MissionConfig = load_config(
            str(self.get_parameter("mission_file").value)
        )
        self._group = ReentrantCallbackGroup()
        self._busy = False
        self._busy_lock = threading.Lock()
        self._active_remote_goal: Any = None
        self._odom_condition = threading.Condition()
        self._stationary_samples = 0
        self._initial_pose_sent = False
        self.create_subscription(Odometry, "/odom", self._receive_odom, 10,
                                 callback_group=self._group)
        initial_pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", initial_pose_qos
        )

        self._navigate = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self._group
        )
        self._pick = ActionClient(
            self, PickCube, "/manipulation/pick_cube", callback_group=self._group
        )
        self._store = ActionClient(
            self, StoreCube, "/manipulation/store_cube", callback_group=self._group
        )
        self._retrieve = ActionClient(
            self, RetrieveCube, "/manipulation/retrieve_cube", callback_group=self._group
        )
        self._place = ActionClient(
            self, PlaceCube, "/manipulation/place_cube", callback_group=self._group
        )
        self._server = ActionServer(
            self, ExecuteMission, "/mission/execute",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )
        self.get_logger().info(
            f"Gerenciador pronto; missões: {sorted(self.config.missions)}"
        )
        if not self.config.enabled:
            self.get_logger().warning(
                "missions.yaml está com enabled=false; goals serão rejeitados "
                "até que as poses medidas sejam configuradas."
            )

    def _receive_odom(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        stopped = math.hypot(linear.x, linear.y) <= 0.01 and abs(angular.z) <= 0.02
        with self._odom_condition:
            self._stationary_samples = self._stationary_samples + 1 if stopped else 0
            self._odom_condition.notify_all()

    def _wait_base_stopped(self, timeout: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout
        with self._odom_condition:
            while self._stationary_samples < 3:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._odom_condition.wait(timeout=remaining)
        return True

    def _publish_initial_pose(self) -> None:
        if self._initial_pose_sent:
            return
        configured = self.config.initial_pose
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = configured.x
        message.pose.pose.position.y = configured.y
        message.pose.pose.orientation.z = math.sin(configured.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(configured.yaw / 2.0)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self._initial_pose_publisher.publish(message)
        self._initial_pose_sent = True
        self.get_logger().info("Pose inicial fixa publicada para o AMCL.")

    def _goal(self, request: ExecuteMission.Goal) -> GoalResponse:
        if not self.config.enabled:
            self.get_logger().error("Missões desabilitadas na configuração.")
            return GoalResponse.REJECT
        if request.mission_name not in self.config.missions:
            self.get_logger().error(f"Missão desconhecida: {request.mission_name}")
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle: Any) -> CancelResponse:
        active = self._active_remote_goal
        if active is not None:
            active.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _feedback(
        self, goal_handle: Any, state: str, index: int, total: int,
        message: str, tag_id: int = -1,
    ) -> None:
        feedback = ExecuteMission.Feedback()
        feedback.state = STATE[state]
        feedback.step_index = index
        feedback.total_steps = total
        feedback.tag_id = tag_id
        feedback.message = message
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(message)

    @staticmethod
    def _record(index: int, step: Step, success: bool, code: int, message: str) -> MissionStepResult:
        item = MissionStepResult()
        item.index = index
        item.action = step.kind
        item.tag_id = step.tag_id if step.tag_id is not None else -1
        item.success = success
        item.code = code
        item.message = message
        return item

    async def _call(self, client: ActionClient, goal: Any) -> CallResult:
        timeout = float(self.get_parameter("server_timeout").value)
        if not client.wait_for_server(timeout_sec=timeout):
            return CallResult(False, 100, True, "Servidor de ação indisponível.")
        remote = await client.send_goal_async(goal)
        if remote is None or not remote.accepted:
            return CallResult(False, 101, True, "Objetivo rejeitado pelo servidor.")
        self._active_remote_goal = remote
        wrapped = await remote.get_result_async()
        self._active_remote_goal = None
        if wrapped is None:
            return CallResult(False, 102, False, "Servidor encerrou sem resultado.")
        result = wrapped.result
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return CallResult(False, 3, getattr(result, "cargo_state_known", True), "Operação cancelada.")
        success = wrapped.status == GoalStatus.STATUS_SUCCEEDED and getattr(result, "code", 0) == 0
        return CallResult(
            success,
            int(getattr(result, "code", 0 if success else 103)),
            bool(getattr(result, "cargo_state_known", True)),
            str(getattr(result, "message", "")),
        )

    async def _navigate_to(self, station_name: str) -> CallResult:
        station = self.config.stations[station_name]
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = station.x
        pose.pose.position.y = station.y
        pose.pose.orientation.z = math.sin(station.yaw / 2.0)
        pose.pose.orientation.w = math.cos(station.yaw / 2.0)
        goal.pose = pose
        with self._odom_condition:
            self._stationary_samples = 0
        result = await self._call(self._navigate, goal)
        if result.success and not self._wait_base_stopped():
            return CallResult(False, 104, True,
                              "Nav2 terminou, mas a base não confirmou parada em /odom.")
        if result.success:
            result.message = f"Chegou à estação '{station_name}'."
        return result

    async def _store_gripper(self, inventory: Inventory, slot: str) -> CallResult:
        goal = StoreCube.Goal()
        goal.tag_id = inventory.gripper
        goal.slot = StoreCube.Goal.LEFT if slot == LEFT else StoreCube.Goal.RIGHT
        result = await self._call(self._store, goal)
        if result.success:
            inventory.store_gripper(slot)
        return result

    async def _retrieve_slot(self, inventory: Inventory, slot: str) -> CallResult:
        goal = RetrieveCube.Goal()
        goal.tag_id = inventory.slots[slot]
        goal.slot = RetrieveCube.Goal.LEFT if slot == LEFT else RetrieveCube.Goal.RIGHT
        result = await self._call(self._retrieve, goal)
        if result.success:
            inventory.retrieve(slot)
        return result

    async def _execute(self, goal_handle: Any) -> ExecuteMission.Result:
        result = ExecuteMission.Result()
        records: list[MissionStepResult] = []
        partial = False
        critical_message: str | None = None
        mission = self.config.missions[goal_handle.request.mission_name]
        inventory = Inventory()
        navigation_ready = False
        total = len(mission.steps)
        try:
            self._feedback(goal_handle, "VALIDATING", 0, total, f"Executando '{mission.name}'.")
            self._publish_initial_pose()
            for index, step in enumerate(mission.steps):
                if goal_handle.is_cancel_requested:
                    self._feedback(goal_handle, "SAFE_CANCEL", index, total, "Cancelando com segurança.")
                    goal_handle.canceled()
                    result.outcome = CANCELED
                    result.steps = records
                    result.message = "Missão cancelada; recolhimento solicitado ao servidor ativo."
                    return result

                tag_id = step.tag_id if step.tag_id is not None else -1
                if step.kind == "navigate":
                    self._feedback(goal_handle, "NAVIGATING", index, total,
                                   f"Navegando para '{step.station}'.")
                    call = await self._navigate_to(step.station)
                    navigation_ready = call.success
                    partial |= not call.success
                    records.append(self._record(index, step, call.success, call.code, call.message))
                    continue

                if not navigation_ready:
                    partial = True
                    records.append(self._record(index, step, False, 110,
                                                "Etapa ignorada: navegação anterior falhou."))
                    continue

                if step.kind == "pick":
                    if inventory.gripper is not None:
                        slot = inventory.free_slot()
                        self._feedback(goal_handle, "STORING", index, total,
                                       f"Guardando tag {inventory.gripper} em {slot}.", inventory.gripper)
                        call = await self._store_gripper(inventory, slot)
                        if not call.success:
                            critical_message = call.message
                            break
                    self._feedback(goal_handle, "PICKING", index, total,
                                   f"Pegando tag {tag_id}.", tag_id)
                    goal = PickCube.Goal()
                    goal.tag_id = tag_id
                    call = await self._call(self._pick, goal)
                    if call.success:
                        inventory.add_to_gripper(tag_id)
                    elif call.known:
                        inventory.mark_failed(tag_id)
                        partial = True
                    else:
                        critical_message = call.message
                        break
                    records.append(self._record(index, step, call.success, call.code, call.message))
                    continue

                location = inventory.locations.get(tag_id)
                if location == FAILED or location is None:
                    partial = True
                    records.append(self._record(index, step, False, 111,
                                                f"Tag {tag_id} não foi coletada; depósito ignorado."))
                    continue
                if location in SLOTS:
                    if inventory.gripper is not None:
                        free = inventory.free_slot(excluding=location)
                        self._feedback(goal_handle, "STORING", index, total,
                                       f"Liberando a garra em {free}.", inventory.gripper)
                        call = await self._store_gripper(inventory, free)
                        if not call.success:
                            critical_message = call.message
                            break
                    self._feedback(goal_handle, "RETRIEVING", index, total,
                                   f"Retirando tag {tag_id} de {location}.", tag_id)
                    call = await self._retrieve_slot(inventory, location)
                    if not call.success:
                        critical_message = call.message
                        break
                self._feedback(goal_handle, "PLACING", index, total,
                               f"Depositando tag {tag_id} em '{step.placement}'.", tag_id)
                place_goal = PlaceCube.Goal()
                place_goal.tag_id = tag_id
                place_goal.placement = self.config.placements[step.placement]
                call = await self._call(self._place, place_goal)
                if not call.success:
                    critical_message = call.message
                    records.append(self._record(index, step, False, call.code, call.message))
                    break
                inventory.deliver_gripper(tag_id)
                records.append(self._record(index, step, True, call.code, call.message))

            if goal_handle.is_cancel_requested:
                self._feedback(goal_handle, "SAFE_CANCEL", total, total,
                               "Missão cancelada após interromper a ação ativa.")
                result.outcome = CANCELED
                result.steps = records
                result.message = "Missão cancelada."
                goal_handle.canceled()
                return result
            self._feedback(goal_handle, "FINISHED", total, total, "Sequência finalizada.")
            result.steps = records
            if critical_message is not None:
                result.outcome = CRITICAL_FAILURE
                result.message = f"Missão abortada por estado de carga inseguro: {critical_message}"
                goal_handle.abort()
            else:
                result.outcome = PARTIAL if partial else COMPLETE
                result.message = "Missão concluída parcialmente." if partial else "Missão concluída."
                goal_handle.succeed()
            return result
        except Exception as error:  # boundary: never kill the long-lived server
            self.get_logger().error(f"Falha não tratada na missão: {error}")
            result.outcome = CRITICAL_FAILURE
            result.steps = records
            result.message = str(error)
            goal_handle.abort()
            return result
        finally:
            self._active_remote_goal = None
            with self._busy_lock:
                self._busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
