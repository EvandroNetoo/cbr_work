"""Action que posiciona a base com dois sensores VL53L0X e odometria."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading
import time
import traceback
from typing import Iterable

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import TwistStamped
from interfaces.action import FollowWall
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .control import (
    FollowWallCommand,
    FollowWallController,
    limit_mecanum_command,
)
from .lateral_safety import (
    lateral_clearances_from_scan,
    LateralClearances,
    PlanarTransform,
)
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


@dataclass(frozen=True)
class OdometryPose:
    x_m: float
    y_m: float
    yaw_rad: float


def odometry_pose(message: Odometry) -> OdometryPose:
    """Extract a finite planar pose and normalize its quaternion."""
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    values = (
        float(position.x),
        float(position.y),
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('A odometria contém valores não finitos.')
    norm = math.sqrt(sum(value * value for value in values[2:]))
    if norm < 1e-6:
        raise ValueError('A odometria contém quaternion nulo.')
    qx, qy, qz, qw = (value / norm for value in values[2:])
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return OdometryPose(values[0], values[1], yaw)


def rightward_displacement_mm(
    initial: OdometryPose,
    current: OdometryPose,
) -> float:
    """Project displacement on the robot's initial right-hand axis."""
    delta_x = current.x_m - initial.x_m
    delta_y = current.y_m - initial.y_m
    return 1000.0 * (
        math.sin(initial.yaw_rad) * delta_x
        - math.cos(initial.yaw_rad) * delta_y
    )


class VL53DistanceAction(Node):
    def __init__(self, sensor_pair=None) -> None:
        super().__init__('vl53_distance_action')
        self._declare_parameters()
        self._sensor_config = self._read_sensor_config()
        self._control_rate_hz = self._positive_float('control_rate_hz')
        command_rate = self._positive_float('command_publish_rate_hz')
        self._freshness_timeout = self._positive_float(
            'command_freshness_timeout_sec')
        self._odom_start_timeout = self._positive_float(
            'odom_start_timeout_sec')
        self._odom_freshness_timeout = self._positive_float(
            'odom_freshness_timeout_sec')
        self._settle_time = self._positive_float('settle_time_sec')
        self._wheel_linear_speed = self._positive_float(
            'wheel_linear_speed_limit')
        self._kinematic_lever = self._positive_float('kinematic_lever')
        self._lateral_scan_timeout = self._positive_float(
            'lateral_safety.scan_timeout_sec')
        self._lateral_scan_start_timeout = self._positive_float(
            'lateral_safety.scan_start_timeout_sec')
        self._lateral_slowdown_margin_mm = float(self.get_parameter(
            'lateral_safety.slowdown_margin_mm').value)
        self._footprint_half_length_m = self._positive_float(
            'lateral_safety.footprint_half_length_m')
        self._footprint_half_width_m = self._positive_float(
            'lateral_safety.footprint_half_width_m')
        self._lateral_longitudinal_margin_m = float(self.get_parameter(
            'lateral_safety.longitudinal_margin_m').value)
        self._lateral_minimum_points = int(self.get_parameter(
            'lateral_safety.minimum_consecutive_points').value)
        if (
            not math.isfinite(self._lateral_slowdown_margin_mm)
            or self._lateral_slowdown_margin_mm < 0.0
        ):
            raise ValueError(
                'lateral_safety.slowdown_margin_mm nao pode ser negativo.')
        if (
            not math.isfinite(self._lateral_longitudinal_margin_m)
            or self._lateral_longitudinal_margin_m < 0.0
        ):
            raise ValueError(
                'lateral_safety.longitudinal_margin_m nao pode ser negativa.')
        if self._lateral_minimum_points <= 0:
            raise ValueError(
                'lateral_safety.minimum_consecutive_points deve ser positivo.')
        self._failure_limit = int(
            self.get_parameter('max_consecutive_read_failures').value)
        if self._failure_limit <= 0:
            raise ValueError('max_consecutive_read_failures deve ser positivo.')

        self._follow_wall_controller = FollowWallController(
            PIDController(self._read_pid_config('linear_pid')),
            PIDController(self._read_pid_config('travel_pid')),
            PIDController(self._read_pid_config('angular_pid')),
            wheel_linear_speed=self._wheel_linear_speed,
            kinematic_lever=self._kinematic_lever,
        )
        # Hardware, odometria e watchdog permanecem inativos enquanto nao ha
        # goal. Isso evita inicializar o I2C e acordar o executor em standby.
        self._owns_sensor_pair = sensor_pair is None
        self._sensor_pair = sensor_pair
        self._sensor_pair_factory = lambda: VL53SensorPair(self._sensor_config)
        self._command_frame = str(self.get_parameter('command_frame').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._scan_topic = str(self.get_parameter('scan_topic').value)

        self._lock = threading.RLock()
        self._resource_lock = threading.RLock()
        self._latest_odom: OdometryPose | None = None
        self._odom_updated = float('-inf')
        self._latest_lateral_clearances: LateralClearances | None = None
        self._lateral_scan_updated = float('-inf')
        self._lateral_scan_started = float('-inf')
        self._publisher = self.create_publisher(
            TwistStamped, str(self.get_parameter('cmd_vel_topic').value), 1)
        self._odom_subscription = None
        self._scan_subscription = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._state = 'idle'
        self._desired_command = (0.0, 0.0, 0.0)
        self._desired_updated = float('-inf')
        self._desired_valid = False
        self._shutdown_event = threading.Event()
        self._goal_wakeup = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._command_timer = self.create_timer(
            1.0 / command_rate,
            self._publish_command_cycle,
            autostart=False,
        )
        action_name = str(self.get_parameter('action_name').value)
        self._action_server = ActionServer(
            self,
            FollowWall,
            action_name,
            goal_callback=self._follow_wall_goal_callback,
            cancel_callback=self._cancel_callback,
            handle_accepted_callback=self._handle_accepted_callback,
        )
        self.get_logger().info(
            f'Servidor VL53L0X pronto; recursos em standby ate receber '
            f'goals em {action_name}.')

    def _declare_parameters(self) -> None:
        defaults = {
            'action_name': '/vl53/follow_wall',
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'scan_topic': '/scan_front',
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
            'odom_start_timeout_sec': 1.0,
            'odom_freshness_timeout_sec': 0.30,
            'settle_time_sec': 0.50,
            'max_consecutive_read_failures': 3,
            'wheel_linear_speed_limit': 0.370,
            'kinematic_lever': 0.2225,
            'lateral_safety.scan_timeout_sec': 0.35,
            'lateral_safety.scan_start_timeout_sec': 1.0,
            'lateral_safety.slowdown_margin_mm': 150,
            'lateral_safety.footprint_half_length_m': 0.119,
            'lateral_safety.footprint_half_width_m': 0.155,
            # Movimento lateral puro varre somente o comprimento do footprint;
            # margem aqui poderia classificar a parede frontal como lateral.
            'lateral_safety.longitudinal_margin_m': 0.0,
            'lateral_safety.minimum_consecutive_points': 2,
            'linear_pid.kp': 0.8,
            'linear_pid.ki': 0.0,
            'linear_pid.kd': 0.0,
            'linear_pid.integral_limit': 0.20,
            'linear_pid.derivative_filter_alpha': 0.20,
            'linear_pid.output_limit': 0.155,
            'travel_pid.kp': 0.8,
            'travel_pid.ki': 0.0,
            'travel_pid.kd': 0.0,
            'travel_pid.integral_limit': 0.20,
            'travel_pid.derivative_filter_alpha': 0.20,
            'travel_pid.output_limit': 0.155,
            'angular_pid.kp': 4.0,
            'angular_pid.ki': 0.0,
            'angular_pid.kd': 0.0,
            'angular_pid.integral_limit': 0.20,
            'angular_pid.derivative_filter_alpha': 0.20,
            'angular_pid.output_limit': 0.777,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_sensor_config(self) -> SensorPairConfig:
        def value(name):
            return self.get_parameter(name).value

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
        def value(suffix):
            return float(self.get_parameter(f'{prefix}.{suffix}').value)

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

    def _odom_callback(self, message: Odometry) -> None:
        try:
            pose = odometry_pose(message)
        except ValueError as error:
            self.get_logger().warning(f'Odometria ignorada: {error}')
            return
        with self._lock:
            self._latest_odom = pose
            self._odom_updated = time.monotonic()

    def _odometry_snapshot(
        self,
        now: float | None = None,
    ) -> tuple[OdometryPose | None, bool]:
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            pose = self._latest_odom
            updated = self._odom_updated
        fresh = (
            pose is not None
            and now - updated <= self._odom_freshness_timeout
        )
        return pose, fresh

    def _scan_callback(self, message: LaserScan) -> None:
        """Atualiza as folgas laterais no frame da base."""
        source_frame = message.header.frame_id
        if not source_frame:
            self.get_logger().warning(
                'LaserScan ignorado porque header.frame_id esta vazio.',
                throttle_duration_sec=2.0,
            )
            return
        try:
            stamped = self._tf_buffer.lookup_transform(
                self._command_frame,
                source_frame,
                Time(),
            )
            translation = stamped.transform.translation
            rotation = stamped.transform.rotation
            transform = PlanarTransform.from_quaternion(
                translation.x,
                translation.y,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
            clearances = lateral_clearances_from_scan(
                message.ranges,
                angle_min_rad=float(message.angle_min),
                angle_increment_rad=float(message.angle_increment),
                range_min_m=float(message.range_min),
                range_max_m=float(message.range_max),
                transform=transform,
                footprint_half_length_m=self._footprint_half_length_m,
                footprint_half_width_m=self._footprint_half_width_m,
                longitudinal_margin_m=self._lateral_longitudinal_margin_m,
                minimum_consecutive_points=self._lateral_minimum_points,
            )
        except (TransformException, ValueError) as error:
            self.get_logger().warning(
                f'LaserScan lateral ignorado: {error}',
                throttle_duration_sec=2.0,
            )
            return
        with self._lock:
            self._latest_lateral_clearances = clearances
            self._lateral_scan_updated = time.monotonic()

    def _lateral_clearance_snapshot(
        self,
        linear_y: float,
        now: float,
    ) -> tuple[str, float | None, bool]:
        """Retorna somente o lado para o qual linear.y esta movimentando."""
        side = 'esquerdo' if linear_y > 0.0 else 'direito'
        with self._lock:
            clearances = self._latest_lateral_clearances
            updated = self._lateral_scan_updated
        fresh = (
            clearances is not None
            and now - updated <= self._lateral_scan_timeout
        )
        if not fresh:
            return side, None, False
        clearance = (
            clearances.left_mm if linear_y > 0.0 else clearances.right_mm)
        return side, clearance, True

    def _apply_lateral_safety(
        self,
        command: FollowWallCommand,
        minimum_clearance_mm: int,
        now: float,
    ) -> tuple[FollowWallCommand, str | None]:
        """Protege o lado do linear.y sem bloquear a correcao frontal."""
        linear_y = command.linear_y_velocity_mps
        if minimum_clearance_mm <= 0 or linear_y == 0.0:
            return command, None

        side, clearance, fresh = self._lateral_clearance_snapshot(
            linear_y, now)
        if not fresh:
            if now - self._lateral_scan_started <= self._lateral_scan_start_timeout:
                return replace(command, linear_y_velocity_mps=0.0), None
            stopped = replace(
                command,
                linear_x_velocity_mps=0.0,
                linear_y_velocity_mps=0.0,
                angular_velocity_rad_s=0.0,
            )
            return stopped, (
                f'LiDAR lateral indisponivel ou obsoleto ao mover para o '
                f'lado {side}.')

        # Nenhum cluster no corredor significa lado livre dentro do alcance.
        if clearance is None:
            return command, None
        if clearance <= float(minimum_clearance_mm):
            x_only = replace(
                command,
                linear_y_velocity_mps=0.0,
                angular_velocity_rad_s=0.0,
            )
            return x_only, (
                f'Obstaculo no lado {side} a {clearance:.0f} mm do footprint; '
                f'minimo solicitado: {minimum_clearance_mm} mm.')

        slowdown_end = (
            float(minimum_clearance_mm) + self._lateral_slowdown_margin_mm)
        if self._lateral_slowdown_margin_mm > 0.0 and clearance < slowdown_end:
            scale = (
                (clearance - float(minimum_clearance_mm))
                / self._lateral_slowdown_margin_mm
            )
            return replace(
                command,
                linear_y_velocity_mps=linear_y * max(0.0, min(1.0, scale)),
            ), None
        return command, None

    def _reserve_goal(self, description: str) -> GoalResponse:
        with self._lock:
            if self._state != 'idle':
                self.get_logger().warning(
                    f'Goal rejeitado: outro {description} está ativo.')
                return GoalResponse.REJECT
            self._state = 'accepted'
            self._desired_valid = False
        return GoalResponse.ACCEPT

    def _follow_wall_goal_callback(self, request) -> GoalResponse:
        target = int(request.wall_distance_mm)
        wall_tolerance = int(request.wall_tolerance_mm)
        travel_tolerance = int(request.travel_tolerance_mm)
        max_alignment_error = int(request.max_alignment_error_mm)
        recovery_distance = int(request.alignment_recovery_distance_mm)
        minimum_lateral_clearance = int(
            request.minimum_lateral_clearance_mm)
        timeout = duration_seconds(request.timeout)
        minimum = max(1, self._sensor_config.minimum_target_mm)
        maximum = self._sensor_config.maximum_target_mm
        if target < minimum or target > maximum:
            self.get_logger().warning(
                f'Goal rejeitado: distância da parede deve estar entre '
                f'{minimum} e {maximum} mm.')
            return GoalResponse.REJECT
        if wall_tolerance <= 0 or travel_tolerance <= 0:
            self.get_logger().warning(
                'Goal rejeitado: tolerâncias devem ser positivas.')
            return GoalResponse.REJECT
        if max_alignment_error < 0:
            self.get_logger().warning(
                'Goal rejeitado: limite de desalinhamento não pode ser '
                'negativo.')
            return GoalResponse.REJECT
        if recovery_distance < 0:
            self.get_logger().warning(
                'Goal rejeitado: distância de recuperação não pode ser '
                'negativa.')
            return GoalResponse.REJECT
        if minimum_lateral_clearance < 0:
            self.get_logger().warning(
                'Goal rejeitado: folga lateral minima nao pode ser negativa.')
            return GoalResponse.REJECT
        if recovery_distance > 0 and max_alignment_error == 0:
            self.get_logger().warning(
                'Goal rejeitado: recuperação requer um limite de '
                'desalinhamento positivo.')
            return GoalResponse.REJECT
        if recovery_distance > 0 and int(request.travel_distance_mm) == 0:
            self.get_logger().warning(
                'Goal rejeitado: recuperação lateral requer percurso '
                'lateral diferente de zero.')
            return GoalResponse.REJECT
        if not math.isfinite(timeout) or timeout <= 0.0:
            self.get_logger().warning('Goal rejeitado: timeout deve ser positivo.')
            return GoalResponse.REJECT
        return self._reserve_goal('controle VL53')

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._invalidate_command(publish=True)
        self._goal_wakeup.set()
        return CancelResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle) -> None:
        worker = threading.Thread(
            target=self._execute_follow_wall_goal,
            args=(goal_handle,),
            name='vl53-follow-wall-action-goal',
            daemon=True,
        )
        with self._lock:
            self._worker_thread = worker
        worker.start()

    def _activate_goal_resources(
        self,
        lateral_safety_enabled: bool = False,
    ) -> None:
        """Ativa somente os recursos necessarios durante um goal."""
        with self._resource_lock:
            with self._lock:
                self._latest_odom = None
                self._odom_updated = float('-inf')
                self._latest_lateral_clearances = None
                self._lateral_scan_updated = float('-inf')
                self._lateral_scan_started = time.monotonic()

            try:
                if self._odom_subscription is None:
                    self._odom_subscription = self.create_subscription(
                        Odometry,
                        self._odom_topic,
                        self._odom_callback,
                        qos_profile_sensor_data,
                    )
                if lateral_safety_enabled and self._scan_subscription is None:
                    self._scan_subscription = self.create_subscription(
                        LaserScan,
                        self._scan_topic,
                        self._scan_callback,
                        qos_profile_sensor_data,
                    )
                if self._sensor_pair is None:
                    self._sensor_pair = self._sensor_pair_factory()
                self._command_timer.reset()
            except Exception:
                self._deactivate_goal_resources()
                raise

    def _deactivate_goal_resources(self) -> None:
        """Cancela wakeups, remove odometria e libera o hardware do goal."""
        with self._resource_lock:
            if hasattr(self, '_command_timer'):
                self._command_timer.cancel()

            subscription = self._odom_subscription
            self._odom_subscription = None
            if subscription is not None:
                self.destroy_subscription(subscription)

            scan_subscription = self._scan_subscription
            self._scan_subscription = None
            if scan_subscription is not None:
                self.destroy_subscription(scan_subscription)

            with self._lock:
                self._latest_odom = None
                self._odom_updated = float('-inf')
                self._latest_lateral_clearances = None
                self._lateral_scan_updated = float('-inf')
                self._lateral_scan_started = float('-inf')

            if self._owns_sensor_pair and self._sensor_pair is not None:
                sensor_pair = self._sensor_pair
                self._sensor_pair = None
                sensor_pair.close()

    def _execute_follow_wall_goal(self, goal_handle):
        started = time.monotonic()
        last_iteration = started
        last_sample: DistanceSample | None = None
        initial_odom: OdometryPose | None = None
        current_odom: OdometryPose | None = None
        traveled_mm = 0.0
        consecutive_failures = 0
        settled_since: float | None = None
        lateral_blocked_since: float | None = None
        lateral_blocked_message: str | None = None
        recovery_target_mm: float | None = None
        recovery_distance_mm = int(
            goal_handle.request.alignment_recovery_distance_mm)
        minimum_lateral_clearance_mm = int(
            goal_handle.request.minimum_lateral_clearance_mm)
        requested_travel_mm = int(goal_handle.request.travel_distance_mm)
        active_travel_target_mm = float(requested_travel_mm)
        try:
            if goal_handle.is_cancel_requested:
                result = self._follow_wall_result(
                    None, False, 0.0, 0.0,
                    'Goal cancelado antes de ativar os sensores.')
                goal_handle.canceled(result)
                return result

            goal_handle.executing()
            self._activate_goal_resources(
                lateral_safety_enabled=minimum_lateral_clearance_mm > 0)
            self._follow_wall_controller.reset()
            assert self._sensor_pair is not None
            self._sensor_pair.reset_filter()
            with self._lock:
                self._state = 'executing_follow_wall'

            while (
                rclpy.ok()
                and goal_handle.is_active
                and not self._shutdown_event.is_set()
                and initial_odom is None
            ):
                now = time.monotonic()
                elapsed = now - started
                if goal_handle.is_cancel_requested:
                    result = self._follow_wall_result(
                        last_sample, False, traveled_mm, elapsed,
                        'Goal cancelado antes da odometria inicial.')
                    goal_handle.canceled(result)
                    return result
                current_odom, fresh = self._odometry_snapshot(now)
                if fresh:
                    initial_odom = current_odom
                    break
                if elapsed >= duration_seconds(goal_handle.request.timeout):
                    result = self._follow_wall_result(
                        last_sample, False, traveled_mm, elapsed,
                        'Timeout aguardando a odometria inicial.')
                    goal_handle.abort(result)
                    return result
                if elapsed >= self._odom_start_timeout:
                    result = self._follow_wall_result(
                        last_sample, False, traveled_mm, elapsed,
                        'Odometria inicial indisponível ou obsoleta.')
                    goal_handle.abort(result)
                    return result
                self._publish_follow_wall_feedback(
                    goal_handle, last_sample, None, 0, elapsed,
                    travel_target_mm=int(
                        goal_handle.request.travel_distance_mm),
                )
                self._goal_wakeup.wait(timeout=1.0 / self._control_rate_hz)
                self._goal_wakeup.clear()

            while (
                rclpy.ok()
                and goal_handle.is_active
                and not self._shutdown_event.is_set()
            ):
                now = time.monotonic()
                elapsed = now - started
                if goal_handle.is_cancel_requested:
                    result = self._follow_wall_result(
                        last_sample, initial_odom is not None, traveled_mm,
                        elapsed, 'Goal cancelado.')
                    goal_handle.canceled(result)
                    return result
                if elapsed >= duration_seconds(goal_handle.request.timeout):
                    result = self._follow_wall_result(
                        last_sample, initial_odom is not None, traveled_mm,
                        elapsed,
                        'Timeout antes de concluir o seguimento da parede.')
                    goal_handle.abort(result)
                    return result

                current_odom, odom_fresh = self._odometry_snapshot(now)
                if not odom_fresh or current_odom is None:
                    self._follow_wall_controller.reset()
                    self._invalidate_command(publish=True)
                    result = self._follow_wall_result(
                        last_sample, initial_odom is not None, traveled_mm,
                        elapsed, 'Odometria ficou indisponível ou obsoleta.')
                    goal_handle.abort(result)
                    return result
                assert initial_odom is not None
                traveled_mm = rightward_displacement_mm(
                    initial_odom, current_odom)

                iteration_started = now
                try:
                    sample = self._sensor_pair.read()
                except Exception as error:
                    consecutive_failures += 1
                    settled_since = None
                    self._follow_wall_controller.reset()
                    self._invalidate_command(publish=True)
                    self.get_logger().warning(
                        f'Falha de leitura VL53L0X '
                        f'({consecutive_failures}/{self._failure_limit}): '
                        f'{error}')
                    self._publish_follow_wall_feedback(
                        goal_handle, last_sample, None, consecutive_failures,
                        time.monotonic() - started,
                        traveled_mm=traveled_mm,
                        travel_target_mm=round(active_travel_target_mm),
                    )
                    if consecutive_failures >= self._failure_limit:
                        result = self._follow_wall_result(
                            last_sample, True, traveled_mm,
                            time.monotonic() - started,
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
                    max_alignment_error = int(
                        goal_handle.request.max_alignment_error_mm)
                    alignment_error = sample.right_mm - sample.left_mm
                    alignment_outside = (
                        max_alignment_error > 0
                        and abs(alignment_error) > max_alignment_error
                    )
                    if recovery_target_mm is None and alignment_outside:
                        if recovery_distance_mm == 0:
                            self._follow_wall_controller.reset()
                            self._invalidate_command(publish=True)
                            self._publish_follow_wall_feedback(
                                goal_handle, sample, None, 0, elapsed,
                                traveled_mm=traveled_mm,
                                travel_target_mm=round(
                                    active_travel_target_mm),
                            )
                            result = self._follow_wall_result(
                                sample, True, traveled_mm, elapsed,
                                f'Desalinhamento de {abs(alignment_error)} mm '
                                f'excede o limite de '
                                f'{max_alignment_error} mm.')
                            goal_handle.abort(result)
                            return result

                        travel_direction = (
                            1.0 if requested_travel_mm > 0 else -1.0)
                        recovery_target_mm = (
                            traveled_mm
                            - travel_direction * recovery_distance_mm
                        )
                        active_travel_target_mm = recovery_target_mm
                        settled_since = None
                        self._follow_wall_controller.reset()
                        self._invalidate_command(publish=True)
                        self.get_logger().warning(
                            f'Desalinhamento de {abs(alignment_error)} mm; '
                            f'iniciando retorno lateral de '
                            f'{recovery_distance_mm} mm até '
                            f'{recovery_target_mm:.1f} mm de odometria.')

                    dt = max(now - last_iteration, 1.0 / self._control_rate_hz)
                    use_sensor_alignment = (
                        recovery_target_mm is None or not alignment_outside)
                    control_left_mm = (
                        sample.left_mm
                        if use_sensor_alignment
                        else int(goal_handle.request.wall_distance_mm)
                    )
                    control_right_mm = (
                        sample.right_mm
                        if use_sensor_alignment
                        else int(goal_handle.request.wall_distance_mm)
                    )
                    command = self._follow_wall_controller.calculate(
                        control_left_mm,
                        control_right_mm,
                        int(goal_handle.request.wall_distance_mm),
                        int(goal_handle.request.wall_tolerance_mm),
                        traveled_mm,
                        round(active_travel_target_mm),
                        int(goal_handle.request.travel_tolerance_mm),
                        dt,
                    )
                    command, lateral_safety_error = self._apply_lateral_safety(
                        command,
                        minimum_lateral_clearance_mm,
                        now,
                    )
                    last_iteration = now
                    lateral_safety_handled = False
                    if lateral_safety_error is not None:
                        is_lateral_obstacle = lateral_safety_error.startswith(
                            'Obstaculo no lado ')
                        if (
                            is_lateral_obstacle
                            and command.linear_x_velocity_mps != 0.0
                        ):
                            lateral_blocked_since = None
                            if lateral_blocked_message != lateral_safety_error:
                                self.get_logger().warning(
                                    f'{lateral_safety_error} Mantendo somente '
                                    'a correção frontal em linear.x.')
                            lateral_blocked_message = lateral_safety_error
                            settled_since = None
                            self._set_desired_command(
                                command.linear_x_velocity_mps, 0.0, 0.0)
                            self._publish_follow_wall_feedback(
                                goal_handle, sample, command, 0, elapsed)
                            lateral_safety_handled = True
                        elif is_lateral_obstacle:
                            if lateral_blocked_since is None:
                                lateral_blocked_since = now
                                self._set_desired_command(0.0, 0.0, 0.0)
                                self._publish_follow_wall_feedback(
                                    goal_handle, sample, command, 0, elapsed)
                                lateral_safety_handled = True
                            elif now - lateral_blocked_since < self._settle_time:
                                self._set_desired_command(0.0, 0.0, 0.0)
                                self._publish_follow_wall_feedback(
                                    goal_handle, sample, command, 0, elapsed)
                                lateral_safety_handled = True
                            else:
                                lateral_safety_error += (
                                    ' Aproximacao frontal concluida; '
                                    'deslocamento lateral interrompido.')
                        if not lateral_safety_handled:
                            self._follow_wall_controller.reset()
                            self._invalidate_command(publish=True)
                            self._publish_follow_wall_feedback(
                                goal_handle, sample, command, 0, elapsed)
                            result = self._follow_wall_result(
                                sample, True, traveled_mm, elapsed,
                                lateral_safety_error)
                            goal_handle.abort(result)
                            return result
                    else:
                        lateral_blocked_since = None
                        lateral_blocked_message = None
                    if (
                        not lateral_safety_handled
                        and command.inside_tolerance
                    ):
                        self._follow_wall_controller.reset()
                        self._set_desired_command(0.0, 0.0, 0.0)
                        if recovery_target_mm is not None:
                            result = self._follow_wall_result(
                                sample, True, traveled_mm, elapsed,
                                f'Recuperação concluída após retorno lateral '
                                f'de {recovery_distance_mm} mm.')
                            goal_handle.abort(result)
                            return result
                        if settled_since is None:
                            settled_since = now
                        elif now - settled_since >= self._settle_time:
                            result = self._follow_wall_result(
                                sample, True, traveled_mm, elapsed,
                                'Parede e percurso lateral alcançados.')
                            goal_handle.succeed(result)
                            return result
                    elif not lateral_safety_handled:
                        settled_since = None
                        self._set_desired_command(
                            command.linear_x_velocity_mps,
                            command.linear_y_velocity_mps,
                            command.angular_velocity_rad_s,
                        )
                    self._publish_follow_wall_feedback(
                        goal_handle, sample, command, 0, elapsed)

                remaining = (
                    1.0 / self._control_rate_hz
                    - (time.monotonic() - iteration_started))
                self._goal_wakeup.wait(timeout=max(0.0, remaining))
                self._goal_wakeup.clear()

            result = self._follow_wall_result(
                last_sample, initial_odom is not None, traveled_mm,
                time.monotonic() - started, 'Servidor encerrado durante o goal.')
            if goal_handle.is_active:
                goal_handle.abort(result)
            return result
        except Exception as error:
            self.get_logger().error(
                f'Falha inesperada na action de seguimento de parede: {error}')
            result = self._follow_wall_result(
                last_sample, initial_odom is not None, traveled_mm,
                time.monotonic() - started, f'Falha inesperada: {error}')
            if goal_handle.is_active:
                goal_handle.abort(result)
            return result
        finally:
            self._invalidate_command(publish=True)
            try:
                self._deactivate_goal_resources()
            except Exception as error:
                self.get_logger().error(
                    f'Falha ao liberar recursos VL53L0X: {error}')
            with self._lock:
                self._state = 'idle'
                self._worker_thread = None

    def _set_desired_command(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
    ) -> None:
        command = limit_mecanum_command(
            float(linear_x),
            float(linear_y),
            float(angular_z),
            self._wheel_linear_speed,
            self._kinematic_lever,
        )
        with self._lock:
            self._desired_command = command
            self._desired_updated = time.monotonic()
            self._desired_valid = True

    def _invalidate_command(self, *, publish: bool) -> None:
        with self._lock:
            self._desired_command = (0.0, 0.0, 0.0)
            self._desired_updated = float('-inf')
            self._desired_valid = False
        if publish:
            self._publish_twist(0.0, 0.0, 0.0)

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
        self._publish_twist(*(command if fresh else (0.0, 0.0, 0.0)))

    def _publish_twist(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
    ) -> None:
        if not hasattr(self, '_publisher'):
            return
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._command_frame
        message.twist.linear.x = float(linear_x)
        message.twist.linear.y = float(linear_y)
        message.twist.angular.z = float(angular_z)
        try:
            self._publisher.publish(message)
        except Exception:
            if rclpy.ok():
                raise

    def _publish_follow_wall_feedback(
        self,
        goal_handle,
        sample: DistanceSample | None,
        command: FollowWallCommand | None,
        failures: int,
        elapsed: float,
        *,
        traveled_mm: float = 0.0,
        travel_target_mm: int = 0,
    ) -> None:
        feedback = FollowWall.Feedback()
        if sample is not None:
            feedback.raw_left_distance_mm = sample.raw_left_mm
            feedback.raw_right_distance_mm = sample.raw_right_mm
            feedback.left_distance_mm = sample.left_mm
            feedback.right_distance_mm = sample.right_mm
            feedback.average_distance_mm = sample.average_mm
            feedback.alignment_error_mm = float(
                sample.right_mm - sample.left_mm)
        if command is not None:
            feedback.wall_distance_error_mm = command.wall_distance_error_mm
            if sample is None:
                feedback.alignment_error_mm = command.alignment_error_mm
            feedback.traveled_distance_mm = command.traveled_distance_mm
            feedback.travel_error_mm = command.travel_error_mm
            feedback.linear_x_velocity_mps = command.linear_x_velocity_mps
            feedback.linear_y_velocity_mps = command.linear_y_velocity_mps
            feedback.angular_velocity_rad_s = command.angular_velocity_rad_s
        else:
            feedback.traveled_distance_mm = float(traveled_mm)
            feedback.travel_error_mm = (
                float(travel_target_mm) - float(traveled_mm))
        feedback.consecutive_read_failures = int(failures)
        feedback.elapsed = duration_message(elapsed)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _follow_wall_result(
        sample: DistanceSample | None,
        has_valid_odometry: bool,
        traveled_mm: float,
        elapsed: float,
        message: str,
    ):
        result = FollowWall.Result()
        result.has_valid_reading = sample is not None
        result.has_valid_odometry = bool(has_valid_odometry)
        if sample is not None:
            result.final_left_distance_mm = sample.left_mm
            result.final_right_distance_mm = sample.right_mm
            result.final_average_distance_mm = sample.average_mm
        result.traveled_distance_mm = float(traveled_mm)
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
            self._deactivate_goal_resources()
            if self._sensor_pair is not None:
                self._sensor_pair.close()
                self._sensor_pair = None
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
