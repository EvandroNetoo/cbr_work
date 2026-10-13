"""Persistent directed Nav2 distances, keyed by map, costmap and planner state."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import tempfile

import yaml

from .errors import ConfigurationError


class UnreachablePath(ConfigurationError):
    """A permanent path failure for the current map snapshot, not a ROS failure."""


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def map_fingerprint(path):
    path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict) or not isinstance(raw.get('image'), str):
        raise ConfigurationError(f'Mapa YAML inválido: {path}.')
    image = (path.parent / raw['image']).resolve()
    return digest({'metadata': {k: v for k, v in raw.items() if k != 'image'},
                   'image': hashlib.sha256(image.read_bytes()).hexdigest()})


def verify_map(path, grid, frame_id):
    """Compare a local map with GetMap data, without reading remote filenames.

    Mirrors nav2_map_server's grayscale occupancy conversion and vertical flip.
    Timestamps and filenames are deliberately excluded from the comparison.
    """
    from PIL import Image
    path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(path.read_text())
    try:
        resolution = float(raw['resolution'])
        x, y, yaw = map(float, raw['origin'])
        free, occupied = float(raw['free_thresh']), float(raw['occupied_thresh'])
        mode = raw.get('mode', 'trinary')
        negate = raw['negate']
        if (mode not in {'trinary', 'scale', 'raw'} or negate not in (0, 1)
                or not 0 <= free < occupied <= 1 or not resolution > 0
                or not all(math.isfinite(v) for v in (resolution, x, y, yaw))):
            raise ValueError('metadados inválidos')
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigurationError(f'Mapa YAML inválido: {path}: {exc}.') from exc
    with Image.open(path.parent / raw['image']) as image:
        image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
        width, height = image.size
        pixels = image.convert('L').tobytes()
        alpha = image.convert('RGBA').getchannel('A').tobytes()
    origin = grid.info.origin
    actual = (grid.info.resolution, origin.position.x, origin.position.y,
              origin.position.z, origin.orientation.x, origin.orientation.y,
              origin.orientation.z, origin.orientation.w)
    expected = (resolution, x, y, 0, 0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    metadata_match = (grid.header.frame_id == frame_id
                      and (grid.info.width, grid.info.height) == (width, height)
                      and all(math.isclose(a, b, abs_tol=1e-6, rel_tol=1e-6)
                              for a, b in zip(actual, expected)))
    table = []
    for pixel in range(256):
        probability = pixel / 255 if negate else 1 - pixel / 255
        if mode == 'raw':
            cell = pixel if pixel <= 100 else 255
        elif probability <= free:
            cell = 0
        elif probability >= occupied:
            cell = 100
        elif mode == 'scale':
            cell = math.floor((probability - free) / (occupied - free) * 100 + 0.5)
        else:
            cell = 255  # unsigned representation of UNKNOWN (-1)
        table.append(cell)
    cells = pixels.translate(bytes(table))
    if mode != 'raw' and any(a < 255 for a in alpha):
        cells = bytes(cell if a == 255 else 255 for cell, a in zip(cells, alpha))
    remote = bytes(value & 255 for value in grid.data)
    if not metadata_match or cells != remote:
        raise ConfigurationError(
            f'--map ({path}) difere do mapa publicado pelo map server. '
            'Use no PC uma cópia do mapa carregado no robô; --map não troca o mapa do Nav2.')


class DistanceCache:
    """Write each successful pair immediately; a failed request is never cached."""
    def __init__(self, path, context):
        self.path = Path(path).expanduser()
        self.context = context
        self.hits = 0
        self.misses = 0
        self.entries = self._read()

    def _read(self):
        if not self.path.exists():
            return {}
        try:
            raw = json.loads(self.path.read_text())
            if raw.get('version') != 1 or not isinstance(raw.get('distances'), dict):
                raise ValueError('schema desconhecido')
            return raw['distances']
        except (ValueError, AttributeError) as exc:
            raise ConfigurationError(f'Cache inválido {self.path}: {exc}. Use outro --cache.') from exc

    def get(self, start, goal, compute):
        key = digest({'context': self.context, 'start': asdict(start), 'goal': asdict(goal)})
        if key in self.entries:
            value = self.entries[key]
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ConfigurationError(f'Distância inválida no cache {self.path}.')
            self.hits += 1
            return value
        self.misses += 1
        value = compute()
        if not math.isfinite(value) or value < 0:
            raise ConfigurationError('Nav2 retornou distância inválida.')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Serialize merging/writing across simultaneous CLI invocations.
        import fcntl
        with self.path.with_suffix(self.path.suffix + '.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            self.entries = self._read()
            self.entries[key] = value
            with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent,
                                             prefix=self.path.name + '.', delete=False) as output:
                temporary = Path(output.name)
                try:
                    json.dump({'version': 1, 'distances': self.entries}, output, allow_nan=False)
                    output.flush()
                    temporary.replace(self.path)
                finally:
                    temporary.unlink(missing_ok=True)
        return value


def path_length(path, start, goal, frame_id, tolerance=0.10):
    """Reject empty/truncated paths rather than storing zero or a partial distance."""
    if path.header.frame_id != frame_id or not path.poses:
        raise ConfigurationError('Nav2 retornou caminho vazio ou em outro frame.')
    positions = [p.pose.position for p in path.poses]
    if any(not math.isfinite(v) for p in positions for v in (p.x, p.y)):
        raise ConfigurationError('Nav2 retornou coordenadas não finitas.')
    first, last = positions[0], positions[-1]
    if (math.hypot(first.x - start.x_m, first.y - start.y_m) > tolerance
            or math.hypot(last.x - goal.x_m, last.y - goal.y_m) > tolerance):
        raise ConfigurationError('Caminho Nav2 não alcançou as poses solicitadas (tolerância 10 cm).')
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(positions, positions[1:]))


class Nav2Distances:
    def __init__(self, node, arena, map_path, cache_path, *, planner_id='',
                 action_name='/compute_path_to_pose', map_server='/map_server',
                 planner_server='/planner_server', costmap_node='/global_costmap/global_costmap',
                 costmap_service='/global_costmap/get_costmap', timeout=30.0):
        from nav2_msgs.action import ComputePathToPose
        from nav2_msgs.srv import GetCostmap
        from nav_msgs.srv import GetMap
        from rclpy.action import ActionClient

        self.node, self.arena, self.timeout = node, arena, timeout
        self.planner_id = planner_id
        self.action = ActionClient(node, ComputePathToPose, action_name)
        if not self.action.wait_for_server(timeout_sec=timeout):
            raise ConfigurationError(f'Nav2 indisponível em {action_name}; inicie a navegação.')
        expected = map_fingerprint(map_path)
        published = self._service(map_server.rstrip('/') + '/map', GetMap, GetMap.Request()).map
        verify_map(map_path, published, arena.frame_id)
        self.costmap_client = node.create_client(GetCostmap, costmap_service)
        if not self.costmap_client.wait_for_service(timeout_sec=timeout):
            raise ConfigurationError(f'Serviço indisponível: {costmap_service}.')
        self.costmap = self._costmap_digest()
        from rclpy.serialization import serialize_message
        configurations = {}
        for name in (planner_server, costmap_node):
            parameters = self._all_parameters(name)
            configurations[name] = [serialize_message(p).hex() for p in parameters]
        self.cache = DistanceCache(cache_path, {
            'map': expected, 'costmap': self.costmap, 'parameters': configurations,
            'planner_id': planner_id, 'action': action_name, 'frame': arena.frame_id,
            'metric': 'nav2_path_xy_v1',
        })

    def _await(self, future):
        import rclpy
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        if not future.done():
            raise ConfigurationError(f'Timeout de {self.timeout:g}s consultando Nav2.')
        result = future.result()
        if result is None:
            raise ConfigurationError('Resposta vazia do Nav2.')
        return result

    def _service(self, name, service_type, request):
        client = self.node.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise ConfigurationError(f'Serviço indisponível: {name}.')
            return self._await(client.call_async(request))
        finally:
            self.node.destroy_client(client)

    def _parameters(self, name, names):
        from rcl_interfaces.srv import GetParameters
        return self._service(name.rstrip('/') + '/get_parameters', GetParameters,
                             GetParameters.Request(names=names)).values

    def _all_parameters(self, name):
        from rcl_interfaces.srv import ListParameters
        response = self._service(name.rstrip('/') + '/list_parameters', ListParameters,
                                 ListParameters.Request(depth=0))
        return self._parameters(name, sorted(response.result.names))

    def _costmap_digest(self):
        from nav2_msgs.srv import GetCostmap
        grid = self._await(self.costmap_client.call_async(GetCostmap.Request())).map
        if grid.header.frame_id != self.arena.frame_id or not grid.data:
            raise ConfigurationError('Costmap vazio ou em frame diferente da arena.')
        m = grid.metadata
        return digest({'frame': grid.header.frame_id, 'resolution': m.resolution,
                       'size': [m.size_x, m.size_y],
                       'origin': [m.origin.position.x, m.origin.position.y, m.origin.position.z,
                                  m.origin.orientation.x, m.origin.orientation.y,
                                  m.origin.orientation.z, m.origin.orientation.w],
                       'data': hashlib.sha256(bytes(grid.data)).hexdigest()})

    def _pose(self, pose):
        from geometry_msgs.msg import PoseStamped
        result = PoseStamped()
        result.header.frame_id = self.arena.frame_id
        result.header.stamp = self.node.get_clock().now().to_msg()
        result.pose.position.x, result.pose.position.y = pose.x_m, pose.y_m
        result.pose.orientation.z = math.sin(pose.yaw_rad / 2)
        result.pose.orientation.w = math.cos(pose.yaw_rad / 2)
        return result

    def verify_costmap(self):
        if self._costmap_digest() != self.costmap:
            raise ConfigurationError('Costmap mudou durante o cálculo. Repita com a arena estável.')

    def distance(self, start_name, goal_name):
        start, goal = self.arena.pose_for(start_name), self.arena.pose_for(goal_name)

        def compute():
            from action_msgs.msg import GoalStatus
            from nav2_msgs.action import ComputePathToPose
            self.verify_costmap()
            self.node.get_logger().info(f'Calculando distância: {start_name} → {goal_name}')
            request = ComputePathToPose.Goal()
            request.start, request.goal = self._pose(start), self._pose(goal)
            request.use_start = True
            request.planner_id = self.planner_id
            handle = self._await(self.action.send_goal_async(request))
            if not handle.accepted:
                raise ConfigurationError(f'Nav2 rejeitou {start_name} → {goal_name}.')
            try:
                response = self._await(handle.get_result_async())
            except BaseException:
                handle.cancel_goal_async()
                raise
            if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code:
                error = response.result.error_code
                kind = UnreachablePath if error in {203, 204, 205, 206, 208} else ConfigurationError
                raise kind(f'Nav2 falhou em {start_name} → {goal_name}: '
                           f'{error} {response.result.error_msg}')
            value = path_length(response.result.path, start, goal, self.arena.frame_id)
            self.verify_costmap()
            return value

        try:
            return self.cache.get(start, goal, compute)
        except UnreachablePath as exc:
            self.node.get_logger().warning(str(exc))
            return math.inf
