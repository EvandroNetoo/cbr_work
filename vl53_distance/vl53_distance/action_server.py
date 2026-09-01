"""Servidor da action que posiciona a base com dois sensores VL53L0X."""

from __future__ import annotations

import math
import threading
import time
import traceback
from typing import Iterable

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import TwistStamped
from interfaces.action import MoveToDistance
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node

from .control import ControlCommand, DistanceController
from .pid import PIDConfig, PIDController
from .sensor_pair import DistanceSample, SensorPairConfig, VL53SensorPair


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def duration_message(seconds: float) -> DurationMsg:
    seconds = max(0.0, float(seconds))
    result = DurationMsg()
    result.sec = int(seconds)
    result.nanosec = int((seconds - result.sec) * 1e9)
    return result


class VL53DistanceAction(Node):
    def __init__(self, sensor_pair=None) -> None:
        super().__init__('vl53_distance_action')
        self._declare_parameters()
        self._sensor_config = self._read_sensor_config()
        self._control_rate_hz = self._positive_float('control_rate_hz')
        command_rate = self._positive_float('command_publish_rate_hz')
        self._freshness_timeout = self._positive_float(
            'command_freshness_timeout_sec')
        self._settle_time = self._positive_float('settle_time_sec')
        self._failure_limit = int(
            self.get_parameter('max_consecutive_read_failures').value)
        if self._failure_limit <= 0:
            raise ValueError('max_consecutive_read_failures deve ser positivo.')

        self._controller = DistanceController(
            PIDController(self._read_pid_config('linear_pid')),
            PIDController(self._read_pid_config('angular_pid')),
        )
        self._sensor_pair = sensor_pair or VL53SensorPair(self._sensor_config)
        self._command_frame = str(self.get_parameter('command_frame').value)
        self._publisher = self.create_publisher(
            TwistStamped, str(self.get_parameter('cmd_vel_topic').value), 1)

        self._lock = threading.RLock()
        self._state = 'idle'
        self._desired_command = (0.0, 0.0)
        self._desired_updated = float('-inf')
        self._desired_valid = False
        self._shutdown_event = threading.Event()
        self._goal_wakeup = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._command_timer = self.create_timer(
            1.0 / command_rate, self._publish_command_cycle)
        action_name = str(self.get_parameter('action_name').value)
        self._action_server = ActionServer(
            self,
            MoveToDistance,
            action_name,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            handle_accepted_callback=self._handle_accepted_callback,
        )
        self.get_logger().info(
            f'VL53L0X pronto; aguardando goals em {action_name}.')

    def _declare_parameters(self) -> None:
        defaults = {
            'action_name': '/vl53/move_to_distance',
            'cmd_vel_topic': '/cmd_vel',
            'command_frame': 'base_footprint',
            'sensor.i2c_bus': 1,
            'sensor.mux_address': 0x70,
            'sensor.right.channel': 0,
            'sensor.right.offset_mm': 48,
            'sensor.left.channel': 1,
            'sensor.left.offset_mm': 106,
            'sensor.raw_min_mm': 30,
            'sensor.raw_max_mm': 2000,
            'sensor.median_window': 3,
            'sensor.ranging_timeout_ms': 200,
            'control_rate_hz': 10.0,
            'command_publish_rate_hz': 20.0,
            'command_freshness_timeout_sec': 0.20,
            'settle_time_sec': 0.50,
            'max_consecutive_read_failures': 3,
            'linear_pid.kp': 0.8,
            'linear_pid.ki': 0.0,
            'linear_pid.kd': 0.0,
            'linear_pid.integral_limit': 0.20,
            'linear_pid.derivative_filter_alpha': 0.20,
            'linear_pid.output_limit': 0.10,
            'angular_pid.kp': 4.0,
            'angular_pid.ki': 0.0,
            'angular_pid.kd': 0.0,
            'angular_pid.integral_limit': 0.20,
            'angular_pid.derivative_filter_alpha': 0.20,
            'angular_pid.output_limit': 0.50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_sensor_config(self) -> SensorPairConfig:
        value = lambda name: self.get_parameter(name).value
        return SensorPairConfig(
            i2c_bus=int(value('sensor.i2c_bus')),
            mux_address=int(value('sensor.mux_address')),
            right_channel=int(value('sensor.right.channel')),
            right_offset_mm=int(value('sensor.right.offset_mm')),
            left_channel=int(value('sensor.left.channel')),
            left_offset_mm=int(value('sensor.left.offset_mm')),
            raw_min_mm=int(value('sensor.raw_min_mm')),
            raw_max_mm=int(value('sensor.raw_max_mm')),
            median_window=int(value('sensor.median_window')),
            ranging_timeout_ms=int(value('sensor.ranging_timeout_ms')),
        )

    def _read_pid_config(self, prefix: str) -> PIDConfig:
        value = lambda suffix: float(
            self.get_parameter(f'{prefix}.{suffix}').value)
        return PIDConfig(
            kp=value('kp'),
            ki=value('ki'),
            kd=value('kd'),
            integral_limit=value('integral_limit'),
            derivative_filter_alpha=value('derivative_filter_alpha'),
            output_limit=value('output_limit'),
        )

    def _positive_float(self, parameter: str) -> float:
        value = float(self.get_parameter(parameter).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{parameter} deve ser positivo e finito.')
        return value

    def _goal_callback(self, request) -> GoalResponse:
        target = int(request.distance_mm)
        tolerance = int(request.tolerance_mm)
        timeout = duration_seconds(request.timeout)
        minimum = max(1, self._sensor_config.minimum_target_mm)
        maximum = self._sensor_config.maximum_target_mm
        if target < minimum or target > maximum:
            self.get_logger().warning(
                f'Goal rejeitado: distância deve estar entre {minimum} e '
                f'{maximum} mm.')
            return GoalResponse.REJECT
        if tolerance <= 0:
            self.get_logger().warning('Goal rejeitado: tolerância deve ser positiva.')
            return GoalResponse.REJECT
        if not math.isfinite(timeout) or timeout <= 0.0:
            self.get_logger().warning('Goal rejeitado: timeout deve ser positivo.')
            return GoalResponse.REJECT
        with self._lock:
            if self._state != 'idle':
                self.get_logger().warning(
                    'Goal rejeitado: outro controle de distância está ativo.')
                return GoalResponse.REJECT
            self._state = 'accepted'
            self._desired_valid = False
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._invalidate_command(publish=True)
        self._goal_wakeup.set()
        return CancelResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle) -> None:
        worker = threading.Thread(
            target=self._execute_goal,
            args=(goal_handle,),
            name='vl53-distance-action-goal',
            daemon=True,
        )
        with self._lock:
            self._worker_thread = worker
        worker.start()

    def _execute_goal(self, goal_handle):
        started = time.monotonic()
        last_iteration = started
        last_sample: DistanceSample | None = None
        consecutive_failures = 0
        settled_since: float | None = None
        self._controller.reset()
        self._sensor_pair.reset_filter()
        with self._lock:
            self._state = 'executing'
        if not goal_handle.is_cancel_requested:
            goal_handle.executing()

        try:
            while (
                rclpy.ok()
                and goal_handle.is_active
                and not self._shutdown_event.is_set()
            ):
                now = time.monotonic()
                elapsed = now - started
                if goal_handle.is_cancel_requested:
                    result = self._result(last_sample, elapsed, 'Goal cancelado.')
                    goal_handle.canceled(result)
                    return result
                if elapsed >= duration_seconds(goal_handle.request.timeout):
                    result = self._result(
                        last_sample, elapsed,
                        'Timeout antes de alcançar a distância solicitada.')
                    goal_handle.abort(result)
                    return result

                iteration_started = now
                try:
                    sample = self._sensor_pair.read()
                except Exception as error:
                    consecutive_failures += 1
                    settled_since = None
                    self._controller.reset()
                    self._invalidate_command(publish=True)
                    self.get_logger().warning(
                        f'Falha de leitura VL53L0X '
                        f'({consecutive_failures}/{self._failure_limit}): {error}')
                    self._publish_feedback(
                        goal_handle, last_sample, None, consecutive_failures,
                        time.monotonic() - started)
                    if consecutive_failures >= self._failure_limit:
                        result = self._result(
                            last_sample, time.monotonic() - started,
                            'Número máximo de falhas consecutivas atingido.')
                        goal_handle.abort(result)
                        return result
                else:
                    last_sample = sample
                    consecutive_failures = 0
                    now = time.monotonic()
                    if (
                        goal_handle.is_cancel_requested
                        or self._shutdown_event.is_set()
                    ):
                        continue
                    elapsed = now - started
                    if elapsed >= duration_seconds(goal_handle.request.timeout):
                        continue
                    dt = max(now - last_iteration, 1.0 / self._control_rate_hz)
                    command = self._controller.calculate(
                        sample.left_mm,
                        sample.right_mm,
                        int(goal_handle.request.distance_mm),
                        int(goal_handle.request.tolerance_mm),
                        dt,
                    )
                    last_iteration = now
                    if command.inside_tolerance:
                        self._controller.reset()
                        self._set_desired_command(0.0, 0.0)
                        if settled_since is None:
                            settled_since = now
                        elif now - settled_since >= self._settle_time:
                            result = self._result(
                                sample, elapsed,
                                'Distância e alinhamento alcançados.')
                            goal_handle.succeed(result)
                            return result
                    else:
                        settled_since = None
                        self._set_desired_command(
                            command.linear_velocity_mps,
                            command.angular_velocity_rad_s,
                        )
                    self._publish_feedback(
                        goal_handle, sample, command, 0, elapsed)

                remaining = (
                    1.0 / self._control_rate_hz
                    - (time.monotonic() - iteration_started))
                self._goal_wakeup.wait(timeout=max(0.0, remaining))
                self._goal_wakeup.clear()

            if goal_handle.is_active:
                result = self._result(
                    last_sample, time.monotonic() - started,
                    'Servidor encerrado durante o goal.')
                goal_handle.abort(result)
                return result
            return self._result(
                last_sample, time.monotonic() - started, 'Goal encerrado.')
        except Exception as error:
            self.get_logger().error(f'Falha inesperada na action VL53L0X: {error}')
            if goal_handle.is_active:
                result = self._result(
                    last_sample, time.monotonic() - started,
                    f'Falha inesperada: {error}')
                goal_handle.abort(result)
                return result
            return self._result(
                last_sample, time.monotonic() - started, str(error))
        finally:
            self._invalidate_command(publish=True)
            with self._lock:
                self._state = 'idle'
                self._worker_thread = None

    def _set_desired_command(self, linear: float, angular: float) -> None:
        with self._lock:
            self._desired_command = (float(linear), float(angular))
            self._desired_updated = time.monotonic()
            self._desired_valid = True

    def _invalidate_command(self, *, publish: bool) -> None:
        with self._lock:
            self._desired_command = (0.0, 0.0)
            self._desired_updated = float('-inf')
            self._desired_valid = False
        if publish:
            self._publish_twist(0.0, 0.0)

    def _publish_command_cycle(self) -> None:
        with self._lock:
            active = self._state != 'idle'
            command = self._desired_command
            fresh = (
                self._desired_valid
                and time.monotonic() - self._desired_updated
                <= self._freshness_timeout
            )
        if not active:
            return
        self._publish_twist(*(command if fresh else (0.0, 0.0)))

    def _publish_twist(self, linear: float, angular: float) -> None:
        if not hasattr(self, '_publisher'):
            return
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._command_frame
        message.twist.linear.x = float(linear)
        message.twist.angular.z = float(angular)
        try:
            self._publisher.publish(message)
        except Exception:
            if rclpy.ok():
                raise

    def _publish_feedback(
        self,
        goal_handle,
        sample: DistanceSample | None,
        command: ControlCommand | None,
        failures: int,
        elapsed: float,
    ) -> None:
        feedback = MoveToDistance.Feedback()
        if sample is not None:
            feedback.raw_left_distance_mm = sample.raw_left_mm
            feedback.raw_right_distance_mm = sample.raw_right_mm
            feedback.left_distance_mm = sample.left_mm
            feedback.right_distance_mm = sample.right_mm
            feedback.average_distance_mm = sample.average_mm
        if command is not None:
            feedback.distance_error_mm = command.distance_error_mm
            feedback.alignment_error_mm = command.alignment_error_mm
            feedback.linear_velocity_mps = command.linear_velocity_mps
            feedback.angular_velocity_rad_s = command.angular_velocity_rad_s
        feedback.consecutive_read_failures = int(failures)
        feedback.elapsed = duration_message(elapsed)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(
        sample: DistanceSample | None,
        elapsed: float,
        message: str,
    ):
        result = MoveToDistance.Result()
        result.has_valid_reading = sample is not None
        if sample is not None:
            result.final_left_distance_mm = sample.left_mm
            result.final_right_distance_mm = sample.right_mm
            result.final_average_distance_mm = sample.average_mm
        result.elapsed = duration_message(elapsed)
        result.message = message
        return result

    def destroy_node(self):
        self._shutdown_event.set()
        self._goal_wakeup.set()
        self._invalidate_command(publish=True)
        with self._lock:
            worker = self._worker_thread
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=1.0)
        try:
            self._sensor_pair.close()
        except Exception as error:
            self.get_logger().error(f'Falha ao fechar os VL53L0X: {error}')
        if hasattr(self, '_action_server'):
            self._action_server.destroy()
        return super().destroy_node()


def main(args: Iterable[str] | None = None) -> int:
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    exit_code = 0
    try:
        node = VL53DistanceAction()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        logger = node.get_logger() if node is not None else get_logger(
            'vl53_distance_action')
        logger.fatal(f'Falha no servidor VL53L0X: {error}')
        traceback.print_exc()
        exit_code = 1
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
