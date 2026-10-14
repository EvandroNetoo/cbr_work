"""Start a mission-only launch on demand and stop only processes we own."""
from __future__ import annotations

import fcntl
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import psutil

from .errors import ConfigurationError


class MissionPlannerProcess:
    def __init__(self, node, args):
        self.node, self.args = node, args
        self.process = None
        self.log = None
        self.lock = None
        self.children = {}
        self.prefix = args.planner_server.rsplit('/', 1)[0]

    def _ready(self, client, action, timeout):
        import rclpy
        from lifecycle_msgs.msg import State
        from lifecycle_msgs.srv import GetState
        if not action.server_is_ready() or not client.service_is_ready():
            rclpy.spin_once(self.node, timeout_sec=min(timeout, 0.1))
            return False
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            client.remove_pending_request(future)
            return False
        response = future.result()
        return response is not None and response.current_state.id == State.PRIMARY_STATE_ACTIVE

    def _acquire_lock(self):
        key = hashlib.sha256(f'{os.getuid()}:{os.environ.get("ROS_DOMAIN_ID", "0")}:{self.prefix}'.encode()).hexdigest()[:16]
        self.lock = (Path(tempfile.gettempdir()) / f'mission_planner_{key}.lock').open('a')
        deadline = time.monotonic() + self.args.timeout
        while True:
            try:
                fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise ConfigurationError('Outro gerador está usando o planner de missão. Aguarde e repita.')
                if not self.node.context.ok():
                    raise KeyboardInterrupt
                time.sleep(0.1)

    def _launch(self):
        args = self.args
        expected = {
            'path_action': self.prefix + '/compute_path_to_pose',
            'map_server': self.prefix + '/map_server',
            'costmap_node': self.prefix + '/global_costmap/global_costmap',
            'costmap_service': self.prefix + '/global_costmap/get_costmap',
            'keepout_server': self.prefix + '/keepout_filter_mask_server',
            'filter_info_server': self.prefix + '/keepout_costmap_filter_info_server',
        }
        if not self.prefix or any(getattr(args, name) != value for name, value in expected.items()):
            raise ConfigurationError('Para endpoints remapeados, inicie o planner separadamente '
                                     'e use --external-planner.')
        map_path = args.map.expanduser().resolve()
        mask = args.keepout_mask.expanduser().resolve() if args.keepout_mask else map_path.with_name(
            map_path.stem + '_keepout.yaml')
        params = args.planner_params.expanduser().resolve()
        for path in (map_path, mask, params):
            if not path.is_file():
                raise ConfigurationError(f'Arquivo do planner não encontrado: {path}. '
                                         'Confira --map, --keepout-mask e --planner-params.')
        self.log = tempfile.NamedTemporaryFile(prefix='mission_planner_', suffix='.log', delete=False)
        command = ['ros2', 'launch', 'mission_manager', 'mission_planner.launch.py',
                   f'namespace:={self.prefix.strip("/")}', f'map:={map_path}',
                   f'keepout_mask:={mask}', f'arena:={args.arena.expanduser().resolve()}',
                   f'params_file:={params}']
        self.node.get_logger().info('Iniciando planner de missão local...')
        # A separate session keeps terminal Ctrl+C in the CLI, which owns cleanup.
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)

    def _track_children(self, *, include_orphans=False):
        if self.process is None:
            return
        try:
            for child in psutil.Process(self.process.pid).children(recursive=True):
                self.children[child.pid] = child
        except psutil.NoSuchProcess:
            pass
        if include_orphans:
            # A launch interrupted while spawning can exit before announcing its
            # children. They retain the session created exclusively for our launch.
            for child in psutil.process_iter():
                try:
                    if child.pid != self.process.pid and os.getsid(child.pid) == self.process.pid:
                        self.children[child.pid] = child
                except (ProcessLookupError, PermissionError, psutil.NoSuchProcess):
                    pass

    def start(self):
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient
        self._acquire_lock()
        action = ActionClient(self.node, ComputePathToPose, self.args.path_action)
        client = self.node.create_client(GetState, self.args.planner_server + '/get_state')
        try:
            deadline = time.monotonic() + self.args.timeout
            # Give discovery a short opportunity to find an existing launch.
            probe_deadline = min(deadline, time.monotonic() + 0.7)
            while time.monotonic() < probe_deadline:
                if not self.node.context.ok():
                    raise KeyboardInterrupt
                if self._ready(client, action, 0.1):
                    self.node.get_logger().info('Usando planner de missão já ativo.')
                    return
            existing = any(namespace == self.prefix or namespace.startswith(self.prefix + '/')
                           for _name, namespace in self.node.get_node_names_and_namespaces())
            if not self.args.external_planner and not existing:
                self._launch()
            while time.monotonic() < deadline:
                if not self.node.context.ok():
                    raise KeyboardInterrupt
                if self.process is not None and self.process.poll() is not None:
                    raise ConfigurationError(f'Planner de missão encerrou durante a inicialização '
                                             f'(código {self.process.returncode}). Log: {self.log.name}')
                self._track_children()
                if self._ready(client, action, 0.2):
                    return
            detail = f' Log: {self.log.name}' if self.log else ''
            raise ConfigurationError(f'Planner de missão não ficou ativo em {self.args.timeout:g}s.' + detail)
        finally:
            self.node.destroy_client(client)
            action.destroy()

    def close(self):
        # Do not let a second Ctrl+C interrupt termination of owned children.
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if self.process is not None:
                self._track_children()
                if self.process.poll() is None:
                    try:
                        os.killpg(self.process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                for timeout, signum in ((10, signal.SIGTERM), (5, signal.SIGKILL)):
                    try:
                        self.process.wait(timeout=timeout)
                        break
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(self.process.pid, signum)
                        except ProcessLookupError:
                            pass
                self.process.wait(timeout=5)
                self._track_children(include_orphans=True)
                # Launch may put children in separate sessions, and Ctrl+C while
                # spawning can arrive before its shutdown handlers track them.
                # psutil verifies PID creation times; only owned descendants or
                # members of our dedicated session are ever signaled here.
                _, alive = psutil.wait_procs(list(self.children.values()), timeout=2)
                for signum in (signal.SIGTERM, signal.SIGKILL):
                    for child in alive:
                        try:
                            child.send_signal(signum)
                        except psutil.NoSuchProcess:
                            pass
                    _, alive = psutil.wait_procs(alive, timeout=2)
        finally:
            if self.log is not None:
                self.log.close()
            if self.lock is not None:
                self.lock.close()
            signal.signal(signal.SIGINT, previous)
