import signal
import subprocess
from types import SimpleNamespace

import pytest

from mission_manager.generate_plan import parser
from mission_manager.planner_process import MissionPlannerProcess

TRACK_CHILDREN = MissionPlannerProcess._track_children


@pytest.fixture(autouse=True)
def fake_process_tree(monkeypatch):
    monkeypatch.setattr(MissionPlannerProcess, '_track_children', lambda self, **kwargs: None)


def setup_args(tmp_path):
    for name in ('arena.yaml', 'map.yaml', 'map_keepout.yaml', 'params.yaml'):
        (tmp_path / name).write_text('test')
    return parser().parse_args([
        '--task', str(tmp_path / 'task.yaml'), '--output', str(tmp_path / 'result.yaml'),
        '--map', str(tmp_path / 'map.yaml'), '--arena', str(tmp_path / 'arena.yaml'),
        '--planner-params', str(tmp_path / 'params.yaml')])


def node():
    return SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda *args: None))


def test_owned_launch_forwards_inputs_and_stops_only_its_process_group(tmp_path, monkeypatch):
    args = setup_args(tmp_path)
    captured = {}
    signals = []
    process = SimpleNamespace(pid=12345, poll=lambda: None, wait=lambda timeout: 0)
    def popen(command, **kwargs):
        captured.update(command=command, **kwargs)
        return process
    monkeypatch.setattr(subprocess, 'Popen', popen)
    monkeypatch.setattr('os.killpg', lambda pid, sig: signals.append((pid, sig)))
    managed = MissionPlannerProcess(node(), args)
    managed._launch()
    managed.close()
    assert captured['start_new_session'] is True
    assert captured['command'][:4] == ['ros2', 'launch', 'mission_manager', 'mission_planner.launch.py']
    assert f'map:={tmp_path / "map.yaml"}' in captured['command']
    assert f'keepout_mask:={tmp_path / "map_keepout.yaml"}' in captured['command']
    assert f'arena:={tmp_path / "arena.yaml"}' in captured['command']
    assert f'params_file:={tmp_path / "params.yaml"}' in captured['command']
    assert signals == [(12345, signal.SIGINT)]


def test_unresponsive_owned_launch_gets_bounded_termination(tmp_path, monkeypatch):
    signals, waits = [], []
    def wait(timeout):
        waits.append(timeout)
        if len(waits) < 3:
            raise subprocess.TimeoutExpired('ros2 launch', timeout)
        return 0
    managed = MissionPlannerProcess(node(), setup_args(tmp_path))
    managed.process = SimpleNamespace(pid=12345, poll=lambda: None, wait=wait)
    monkeypatch.setattr('os.killpg', lambda pid, sig: signals.append((pid, sig)))
    managed.close()
    assert signals == [(12345, signal.SIGINT), (12345, signal.SIGTERM), (12345, signal.SIGKILL)]
    assert waits == [10, 5, 5]


def test_children_in_other_sessions_are_also_cleaned_up(tmp_path, monkeypatch):
    signals = []
    child = SimpleNamespace(pid=34567, send_signal=lambda sig: signals.append(('child', sig)))
    managed = MissionPlannerProcess(node(), setup_args(tmp_path))
    managed.process = SimpleNamespace(pid=12345, poll=lambda: None, wait=lambda timeout: 0)
    managed.children = {child.pid: child}
    calls = []
    def wait_procs(children, timeout):
        calls.append(children)
        return ([], [child]) if len(calls) == 1 else ([child], [])
    monkeypatch.setattr('psutil.wait_procs', wait_procs)
    monkeypatch.setattr('os.killpg', lambda pid, sig: signals.append(('launch', sig)))
    managed.close()
    assert signals == [('launch', signal.SIGINT), ('child', signal.SIGTERM)]


def test_orphans_are_identified_by_owned_session_and_unrelated_processes_excluded(tmp_path, monkeypatch):
    managed = MissionPlannerProcess(node(), setup_args(tmp_path))
    managed.process = SimpleNamespace(pid=12345)
    orphan = SimpleNamespace(pid=34567)
    unrelated = SimpleNamespace(pid=45678)
    monkeypatch.setattr('psutil.Process', lambda pid: SimpleNamespace(children=lambda **kwargs: []))
    monkeypatch.setattr('psutil.process_iter', lambda: [orphan, unrelated])
    monkeypatch.setattr('os.getsid', lambda pid: 12345 if pid == orphan.pid else 45678)
    TRACK_CHILDREN(managed, include_orphans=True)
    assert managed.children == {orphan.pid: orphan}


@pytest.mark.parametrize('external', [False, True])
def test_existing_active_planner_is_reused_and_never_stopped(tmp_path, monkeypatch, external):
    args = setup_args(tmp_path)
    args.external_planner = external
    fake_node = node()
    fake_node.context = SimpleNamespace(ok=lambda: True)
    fake_node.create_client = lambda *args: SimpleNamespace()
    fake_node.destroy_client = lambda *args: None
    fake_node.get_node_names_and_namespaces = lambda: [('planner_server', '/mission_planning')]
    monkeypatch.setattr('rclpy.action.ActionClient',
                        lambda *args: SimpleNamespace(destroy=lambda: None))
    monkeypatch.setattr(MissionPlannerProcess, '_ready', lambda *args: True)
    monkeypatch.setattr(MissionPlannerProcess, '_acquire_lock', lambda self: None)
    monkeypatch.setattr(subprocess, 'Popen', lambda *args, **kwargs: pytest.fail('Must reuse active planner'))
    monkeypatch.setattr('os.killpg', lambda *args: pytest.fail('Must not stop existing planner'))
    managed = MissionPlannerProcess(fake_node, args)
    managed.start()
    managed.close()
    assert managed.process is None
