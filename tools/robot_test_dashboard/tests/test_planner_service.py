import json
from pathlib import Path

from generation import DEFAULTS
from planner_service import ENDPOINTS, ensure_service, fingerprint, identity, planner_arguments, save, service_status


def settings_files(tmp_path):
    settings = dict(DEFAULTS)
    for name in ('setup', 'arena', 'planner_params'):
        path = tmp_path / name
        path.write_text('test')
        settings[name] = str(path)
    for name in ('map', 'map_keepout'):
        (tmp_path / (name + '.yaml')).write_text('image: ' + name + '.pgm\n')
        (tmp_path / (name + '.pgm')).write_bytes(b'pixels')
    settings['map'] = str(tmp_path / 'map.yaml')
    return settings


def test_cache_identity_tracks_images_params_and_not_seed(tmp_path):
    settings = settings_files(tmp_path)
    original = fingerprint(settings)
    assert fingerprint({**settings, 'seed': '999'}) == original
    for name in ('map.pgm', 'map_keepout.pgm', 'planner_params', 'arena'):
        path = tmp_path / name
        previous = path.read_bytes()
        path.write_bytes(previous + b'changed')
        assert fingerprint(settings) != original
        path.write_bytes(previous)


def test_reuses_service_and_defers_restart_while_busy(tmp_path, monkeypatch):
    settings = settings_files(tmp_path)
    root = tmp_path / 'generations'
    processes = []
    class Process:
        pid = 12345
    def launch(*args, **kwargs):
        processes.append(args[0])
        return Process()
    monkeypatch.setattr('planner_service.subprocess.Popen', launch)
    monkeypatch.setattr('planner_service.identity', lambda pid: 'alive')
    first = ensure_service(root, settings)
    assert first['state'] == 'starting'
    directory = Path(first['directory'])
    save(directory / 'status.json', {'state': 'ready', 'message': 'ok'})
    assert ensure_service(root, settings)['state'] == 'ready'
    assert len(processes) == 1
    (tmp_path / 'planner_params').write_text('new parameters')
    assert ensure_service(root, settings, busy=True)['state'] == 'waiting'
    assert len(processes) == 1
    args = planner_arguments(directory)
    assert args[0] == '--external-planner'
    assert len(args) == 1 + 2 * len(ENDPOINTS)
    assert '/dashboard_planning_' in args[2]


def test_dead_server_keeps_failure_details(tmp_path):
    save(tmp_path / 'status.json', {'state': 'error', 'message': 'missing ROS'})
    save(tmp_path / 'worker.json', {'pid': 999999999, 'identity': 'old'})
    assert service_status(tmp_path)['message'] == 'missing ROS'
    assert identity(999999999) is None


def test_configuration_change_stops_owned_worker_before_relaunch(tmp_path, monkeypatch):
    settings = settings_files(tmp_path)
    calls = []
    alive = {12345: True}
    class Process:
        pid = 12345
    def launch(*args, **kwargs):
        calls.append('launch')
        alive[12345] = True
        return Process()
    def stop(pid, signum):
        assert pid == 12345
        calls.append('stop')
        alive[pid] = False
    monkeypatch.setattr('planner_service.subprocess.Popen', launch)
    monkeypatch.setattr('planner_service.identity', lambda pid: 'alive' if alive.get(pid, True) else None)
    monkeypatch.setattr('planner_service.os.kill', stop)
    root = tmp_path / 'jobs'
    first = ensure_service(root, settings)
    (tmp_path / 'map.pgm').write_bytes(b'changed')
    second = ensure_service(root, settings)
    assert first['key'] != second['key']
    assert calls == ['launch', 'stop', 'launch']
