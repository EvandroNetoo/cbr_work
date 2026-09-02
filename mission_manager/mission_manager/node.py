"""ROS 2 action server that executes validated mission steps sequentially."""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from interfaces.action import (
    ExecuteMission,
    MoveToDistance,
    PickObject,
    PlaceInContainer,
    PlaceOnShelf,
    PlaceOnTable,
    PrepareManipulator,
    RetrieveObject,
    StackObject,
    StoreObject,
)
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from .errors import ConfigurationError, MissionCanceled, StepFailed
from .loaders import PLAN_ID_PATTERN, load_arena, load_plan, validate_plan
from .models import Arena, Plan, Step


class MissionManager(Node):
    """Own one mission at a time and compose existing semantic action servers."""

    def __init__(self) -> None:
        super().__init__('mission_manager')
        share = Path(get_package_share_directory('mission_manager'))
        defaults = {
            'arena_file': str(share / 'config' / 'arena.yaml'),
            'plans_directory': str(share / 'config' / 'plans'),
            'execute_action': '/mission/execute',
            'navigate_action': '/navigate_to_pose',
            'alignment_action': '/vl53/move_to_distance',
            'prepare_action': '/manipulation/prepare',
            'pick_action': '/manipulation/pick',
            'store_action': '/manipulation/store',
            'retrieve_action': '/manipulation/retrieve',
            'place_on_table_action': '/manipulation/place_on_table',
            'place_in_container_action': '/manipulation/place_in_container',
            'stack_action': '/manipulation/stack',
            'place_on_shelf_action': '/manipulation/place_on_shelf',
            'server_timeout_s': 10.0,
            'navigation_timeout_s': 120.0,
            'manipulation_timeout_s': 120.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._callback_group = ReentrantCallbackGroup()
        self._cancel_event = threading.Event()
        self._lock = threading.RLock()
        self._busy = False
        self._status = 'idle'
        self._current_step_index = 0
        self._current_location = 'start'
        self._active_child = None
        self._arena: Arena | None = None

        def client(action_type, parameter_name):
            return ActionClient(
                self,
                action_type,
                str(self.get_parameter(parameter_name).value),
                callback_group=self._callback_group,
            )

        self._navigate_client = client(NavigateToPose, 'navigate_action')
        self._alignment_client = client(MoveToDistance, 'alignment_action')
        self._prepare_client = client(PrepareManipulator, 'prepare_action')
        self._pick_client = client(PickObject, 'pick_action')
        self._store_client = client(StoreObject, 'store_action')
        self._retrieve_client = client(RetrieveObject, 'retrieve_action')
        self._place_table_client = client(PlaceOnTable, 'place_on_table_action')
        self._place_container_client = client(
            PlaceInContainer, 'place_in_container_action'
        )
        self._stack_client = client(StackObject, 'stack_action')
        self._place_shelf_client = client(PlaceOnShelf, 'place_on_shelf_action')

        self._server = ActionServer(
            self,
            ExecuteMission,
            str(self.get_parameter('execute_action').value),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            'Mission manager pronto; arena e plano serão validados ao executar.'
        )

    def _goal_callback(self, goal_request: ExecuteMission.Goal) -> GoalResponse:
        plan_id = str(goal_request.plan_id)
        with self._lock:
            if self._busy or not PLAN_ID_PATTERN.fullmatch(plan_id):
                return GoalResponse.REJECT
            self._cancel_event.clear()
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        self._cancel_event.set()
        self._cancel_active_child()
        return CancelResponse.ACCEPT

    def _cancel_active_child(self) -> None:
        with self._lock:
            child = self._active_child
        if child is not None:
            child.cancel_goal_async()

    def _check_canceled(self) -> None:
        if self._cancel_event.is_set():
            self._cancel_active_child()
            raise MissionCanceled('Missão cancelada pelo cliente.')

    def _wait_future(
        self,
        future: Any,
        timeout_s: float,
        *,
        check_cancel: bool = True,
    ) -> Any:
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if check_cancel:
                self._check_canceled()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError('Operação ROS excedeu o tempo limite.')
            completed.wait(min(0.05, remaining))
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def _server_timeout(self) -> float:
        value = float(self.get_parameter('server_timeout_s').value)
        if not math.isfinite(value) or value <= 0.0:
            raise ConfigurationError('server_timeout_s deve ser positivo.')
        return value

    def _call_action(
        self,
        client: ActionClient,
        goal: Any,
        description: str,
        timeout_s: float,
        validate_result: Callable[[Any], str | None] | None = None,
    ) -> Any:
        self._check_canceled()
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ConfigurationError(f'Timeout inválido para {description}.')
        if not client.wait_for_server(timeout_sec=self._server_timeout()):
            raise StepFailed(f'Servidor indisponível: {description}.')
        try:
            send_future = client.send_goal_async(goal)
            child = self._wait_future(
                send_future, self._server_timeout(), check_cancel=False
            )
            if child is None or not child.accepted:
                raise StepFailed(f'Goal rejeitado: {description}.')
            with self._lock:
                self._active_child = child
            self._check_canceled()
            result_wrapper = self._wait_future(
                child.get_result_async(), timeout_s
            )
        except MissionCanceled:
            raise
        except StepFailed:
            raise
        except TimeoutError as error:
            self._cancel_active_child()
            raise StepFailed(f'Timeout durante {description}.') from error
        except Exception as error:
            raise StepFailed(f'Falha de comunicação em {description}: {error}') from error
        finally:
            with self._lock:
                self._active_child = None
        if (
            result_wrapper is None
            or result_wrapper.status != GoalStatus.STATUS_SUCCEEDED
        ):
            status = (
                result_wrapper.status if result_wrapper is not None else 'sem resultado'
            )
            raise StepFailed(f'{description} falhou com estado {status}.')
        result = result_wrapper.result
        if validate_result is not None:
            failure = validate_result(result)
            if failure:
                raise StepFailed(f'{description} falhou: {failure}')
        return result

    @staticmethod
    def _manipulation_failure(result: Any) -> str | None:
        if result.outcome.code == result.outcome.SUCCESS:
            return None
        return result.outcome.message or f'código {result.outcome.code}'

    @staticmethod
    def _navigation_failure(result: NavigateToPose.Result) -> str | None:
        if result.error_code == NavigateToPose.Result.NONE:
            return None
        return result.error_msg or f'código {result.error_code}'

    @staticmethod
    def _alignment_failure(result: MoveToDistance.Result) -> str | None:
        if result.has_valid_reading:
            return None
        return result.message or 'sensores de distância sem leitura válida'

    def _duration(self, seconds: float):
        goal_duration = MoveToDistance.Goal().timeout
        total_nanoseconds = round(seconds * 1_000_000_000)
        goal_duration.sec, goal_duration.nanosec = divmod(
            total_nanoseconds, 1_000_000_000
        )
        return goal_duration

    def _navigation_timeout(self) -> float:
        return float(self.get_parameter('navigation_timeout_s').value)

    def _manipulation_timeout(self) -> float:
        return float(self.get_parameter('manipulation_timeout_s').value)

    def _prepare_for_navigation(self) -> None:
        goal = PrepareManipulator.Goal()
        goal.mode = PrepareManipulator.Goal.NAVIGATION
        self._call_action(
            self._prepare_client,
            goal,
            'preparação do manipulador para navegação',
            self._manipulation_timeout(),
            self._manipulation_failure,
        )

    def _navigate(self, target: str) -> None:
        assert self._arena is not None
        pose = self._arena.pose_for(target)
        if (
            self._current_location in self._arena.service_areas
            and target != self._current_location
        ):
            departure = self._arena.service_areas[
                self._current_location
            ].departure
            departure_goal = MoveToDistance.Goal()
            departure_goal.distance_mm = departure.distance_mm
            departure_goal.tolerance_mm = departure.tolerance_mm
            departure_goal.timeout = self._duration(departure.timeout_s)
            self._call_action(
                self._alignment_client,
                departure_goal,
                f'recuo para sair de {self._current_location}',
                departure.timeout_s + 5.0,
                self._alignment_failure,
            )
        self._prepare_for_navigation()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._arena.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose.x_m
        goal.pose.pose.position.y = pose.y_m
        goal.pose.pose.orientation.z = math.sin(pose.yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose.yaw_rad / 2.0)
        self._call_action(
            self._navigate_client,
            goal,
            f'navegação até {target}',
            self._navigation_timeout(),
            self._navigation_failure,
        )
        if target in self._arena.service_areas:
            alignment = self._arena.service_areas[target].alignment
            alignment_goal = MoveToDistance.Goal()
            alignment_goal.distance_mm = alignment.distance_mm
            alignment_goal.tolerance_mm = alignment.tolerance_mm
            alignment_goal.timeout = self._duration(alignment.timeout_s)
            self._call_action(
                self._alignment_client,
                alignment_goal,
                f'alinhamento em {target}',
                alignment.timeout_s + 5.0,
                self._alignment_failure,
            )
        self._current_location = target

    def _execute_manipulation(self, step: Step) -> None:
        assert self._arena is not None
        area = self._arena.service_areas[self._current_location]
        timeout = self._manipulation_timeout()
        if step.action == 'pick':
            goal = PickObject.Goal()
            goal.tag_id = int(step.tag_id)
            goal.profile = ''
            client = self._pick_client
        elif step.action == 'store':
            goal = StoreObject.Goal()
            goal.slot_id = str(step.slot_id)
            client = self._store_client
        elif step.action == 'retrieve':
            goal = RetrieveObject.Goal()
            goal.slot_id = str(step.slot_id)
            client = self._retrieve_client
        elif step.action == 'place_on_table':
            goal = PlaceOnTable.Goal()
            goal.ws_height_cm = float(area.height_cm)
            goal.analyze_apriltags = step.analyze_apriltags
            goal.analyze_containers = step.analyze_containers
            client = self._place_table_client
        elif step.action == 'place_in_container':
            goal = PlaceInContainer.Goal()
            goal.ws_height_cm = float(area.height_cm)
            goal.container_color = (
                PlaceInContainer.Goal.RED
                if step.container_color == 'red'
                else PlaceInContainer.Goal.BLUE
            )
            client = self._place_container_client
        elif step.action == 'stack':
            goal = StackObject.Goal()
            goal.support_tag_id = int(step.support_tag_id)
            client = self._stack_client
        elif step.action == 'place_on_shelf':
            goal = PlaceOnShelf.Goal()
            client = self._place_shelf_client
        else:
            raise ConfigurationError(f'Operação não implementada: {step.action}.')
        self._call_action(
            client,
            goal,
            f"passo '{step.step_id}' ({step.action})",
            timeout,
            self._manipulation_failure,
        )

    def _execute_step(self, step: Step) -> None:
        if step.action == 'navigate':
            assert step.target is not None
            self._navigate(step.target)
        elif step.action == 'finish':
            self._navigate('finish')
        else:
            self._execute_manipulation(step)

    def _load_goal_files(self, plan_id: str) -> tuple[Arena, Plan]:
        if not PLAN_ID_PATTERN.fullmatch(plan_id):
            raise ConfigurationError('plan_id possui formato inválido.')
        arena_path = Path(str(self.get_parameter('arena_file').value))
        plans_root = Path(
            str(self.get_parameter('plans_directory').value)
        ).resolve()
        plan_path = plans_root / f'{plan_id}.yaml'
        arena = load_arena(arena_path)
        plan = load_plan(plan_path)
        if plan.plan_id != plan_id:
            raise ConfigurationError(
                f"O arquivo solicitado como '{plan_id}' declara plan_id "
                f"'{plan.plan_id}'."
            )
        validate_plan(plan, arena)
        return arena, plan

    def _feedback(
        self,
        goal_handle: Any,
        index: int,
        total: int,
        step: Step,
        description: str,
    ) -> None:
        feedback = ExecuteMission.Feedback()
        feedback.current_step_index = index
        feedback.total_steps = total
        feedback.step_id = step.step_id
        feedback.operation = step.action
        feedback.description = description
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(
        goal_handle: Any,
        code: int,
        completed_steps: int,
        failed_step_id: str,
        message: str,
    ) -> ExecuteMission.Result:
        result = ExecuteMission.Result()
        result.code = code
        result.completed_steps = completed_steps
        result.failed_step_id = failed_step_id
        result.message = message
        if code == ExecuteMission.Result.SUCCESS:
            goal_handle.succeed()
        elif code == ExecuteMission.Result.CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _execute_callback(self, goal_handle: Any) -> ExecuteMission.Result:
        self._status = 'running'
        self._current_location = 'start'
        completed = 0
        failed_step = ''
        try:
            arena, plan = self._load_goal_files(str(goal_handle.request.plan_id))
            self._arena = arena
            total = len(plan.steps)
            for index, step in enumerate(plan.steps):
                failed_step = step.step_id
                self._current_step_index = index
                self._feedback(
                    goal_handle, index, total, step,
                    f'Executando {step.action}',
                )
                self._execute_step(step)
                completed += 1
            self._status = 'succeeded'
            return self._result(
                goal_handle,
                ExecuteMission.Result.SUCCESS,
                completed,
                '',
                f"Plano '{plan.plan_id}' concluído.",
            )
        except MissionCanceled as error:
            self._status = 'canceled'
            return self._result(
                goal_handle,
                ExecuteMission.Result.CANCELED,
                completed,
                failed_step,
                str(error),
            )
        except ConfigurationError as error:
            self._status = 'failed'
            return self._result(
                goal_handle,
                ExecuteMission.Result.CONFIGURATION_ERROR,
                completed,
                failed_step,
                str(error),
            )
        except StepFailed as error:
            self._status = 'failed'
            return self._result(
                goal_handle,
                ExecuteMission.Result.STEP_FAILED,
                completed,
                failed_step,
                str(error),
            )
        except Exception as error:
            self.get_logger().error(f'Falha interna na missão: {error}')
            self._status = 'failed'
            return self._result(
                goal_handle,
                ExecuteMission.Result.INTERNAL_ERROR,
                completed,
                failed_step,
                str(error),
            )
        finally:
            self._cancel_event.clear()
            self._arena = None
            with self._lock:
                self._busy = False
                self._active_child = None

    def destroy_node(self):
        self._cancel_event.set()
        self._cancel_active_child()
        self._server.destroy()
        return super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    exit_code = 0
    try:
        node = MissionManager()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(f'Falha fatal no mission manager: {error}')
        else:
            print(f'Falha fatal no mission manager: {error}')
        exit_code = 1
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
