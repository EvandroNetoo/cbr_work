"""Persistent planner supervisor; ROS stays in a separate, sourced process."""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

ENDPOINTS = {
    'path_action': 'compute_path_to_pose', 'map_server': 'map_server',
    'planner_server': 'planner_server', 'costmap_node': 'global_costmap/global_costmap',
    'costmap_service': 'global_costmap/get_costmap',
    'keepout_server': 'keepout_filter_mask_server',
    'filter_info_server': 'keepout_costmap_filter_info_server',
}


def identity(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return stat[19] if stat[0] != 'Z' else None
    except (FileNotFoundError, ProcessLookupError):
        return None


def fingerprint(settings):
    digest = hashlib.sha256()
    paths = [Path(settings[key]).expanduser().resolve()
             for key in ('setup', 'arena', 'map', 'planner_params')]
    map_path = paths[2]
    paths.append(map_path.with_name(map_path.stem + '_keepout.yaml'))
    for path in list(paths[2:3]) + paths[4:]:
        try:
            content = yaml.safe_load(path.read_text())
        except yaml.YAMLError as exc:
            raise ValueError(f'Mapa YAML inválido: {path}') from exc
        if not isinstance(content, dict) or not isinstance(content.get('image'), str):
            raise ValueError(f'O mapa precisa informar o caminho da imagem: {path}')
        paths.append((path.parent / content['image']).resolve())
    for path in paths:
        digest.update(str(path).encode())
        digest.update(path.read_bytes())
    for name in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI'):
        digest.update(f'{name}={os.environ.get(name, "")}'.encode())
    return digest.hexdigest()


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False))
    temporary.replace(path)


def service_status(directory):
    try:
        status = json.loads((directory / 'status.json').read_text())
        worker = json.loads((directory / 'worker.json').read_text())
    except FileNotFoundError:
        return {'state': 'stopped', 'message': 'Servidor parado.'}
    if not worker['identity'] or identity(worker['pid']) != worker['identity']:
        return status if status['state'] == 'error' else {'state': 'error', 'message': 'Servidor encerrado. Consulte os logs e tente novamente.'}
    return status


def ensure_service(root, settings, *, busy=False, retry=False):
    root = Path(root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    key = fingerprint(settings)
    directory = root / 'planner'
    directory.mkdir(exist_ok=True)
    with (root / 'planner.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        status = service_status(directory)
        request_path = directory / 'request.json'
        previous = json.loads(request_path.read_text()) if request_path.exists() else {}
        if previous.get('key') == key and status['state'] != 'stopped' and not retry:
            return {**status, 'directory': str(directory), 'key': key}
        if busy:
            return {'state': 'waiting', 'message': 'Aguarde a geração terminar para atualizar o servidor.'}
        worker_path = directory / 'worker.json'
        if worker_path.exists():
            worker = json.loads(worker_path.read_text())
            if worker['identity'] and identity(worker['pid']) == worker['identity']:
                os.kill(worker['pid'], signal.SIGTERM)
                deadline = time.monotonic() + 30
                while identity(worker['pid']) == worker['identity']:
                    if time.monotonic() >= deadline:
                        raise ValueError('O servidor anterior ainda está encerrando. Tente novamente.')
                    time.sleep(.1)
        namespace = '/dashboard_planning_' + hashlib.sha256(str(root).encode()).hexdigest()[:10]
        request = {'settings': settings, 'key': key, 'namespace': namespace,
                   'parent': os.getpid(), 'parent_identity': identity(os.getpid())}
        save(request_path, request)
        save(directory / 'status.json', {'state': 'starting', 'message': 'Iniciando servidor de planejamento…'})
        with (directory / 'worker.log').open('w') as log:
            process = subprocess.Popen([
                'bash', '-c', 'source "$1" && exec /usr/bin/python3 "$2" "$3"',
                'dashboard-planner', str(Path(settings['setup']).expanduser().resolve()),
                str(Path(__file__).resolve()), str(directory)],
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True, env={**os.environ, 'ROS_LOG_DIR': str(directory / 'ros_logs')})
        save(worker_path, {'pid': process.pid, 'identity': identity(process.pid)})
        return {'state': 'starting', 'message': 'Iniciando servidor de planejamento…',
                'directory': str(directory), 'key': key}


def planner_arguments(directory):
    request = json.loads((Path(directory) / 'request.json').read_text())
    return ['--external-planner', *[part for name, suffix in ENDPOINTS.items()
            for part in ('--' + name.replace('_', '-'), request['namespace'] + '/' + suffix)]]


def serve(directory):
    planner = node = None
    initialized = False
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from mission_manager.generate_plan import parser
        from mission_manager.planner_process import MissionPlannerProcess
        request = json.loads((directory / 'request.json').read_text())
        settings = request['settings']
        args = parser().parse_args(['--task', '/unused', '--output', '/unused.yaml',
                                   *planner_arguments(directory)[1:]])
        args.map = Path(settings['map'])
        args.arena = Path(settings['arena'])
        args.planner_params = Path(settings['planner_params'])
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('dashboard_planner_supervisor')
        planner = MissionPlannerProcess(node, args)
        planner.start()
        # Generators acquire the same lock while computing distances.
        planner.lock.close()
        planner.lock = None
        save(directory / 'status.json', {'state': 'ready', 'message': 'Servidor de planejamento pronto.',
                                                   'log': planner.log.name if planner.log else None})
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient
        action = ActionClient(node, ComputePathToPose, args.path_action)
        client = node.create_client(GetState, args.planner_server + '/get_state')
        checked = time.monotonic()
        failures = 0
        while identity(request['parent']) == request['parent_identity']:
            if planner.process is not None and planner.process.poll() is not None:
                raise RuntimeError('O servidor de planejamento encerrou inesperadamente.')
            rclpy.spin_once(node, timeout_sec=1)
            if time.monotonic() - checked >= 5:
                failures = 0 if planner._ready(client, action, 1) else failures + 1
                checked = time.monotonic()
                if failures >= 3:
                    raise RuntimeError('O planner deixou de responder. Tente iniciar novamente.')
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        save(directory / 'status.json', {'state': 'error', 'message': str(exc),
                                        'log': planner.log.name if planner and planner.log else None})
    finally:
        if planner is not None:
            planner.close()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()


if __name__ == '__main__':
    serve(Path(sys.argv[1]))
