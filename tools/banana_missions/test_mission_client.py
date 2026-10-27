"""Exercise the real action client against an isolated fake server, without hardware."""

import os
from pathlib import Path
import subprocess
import sys
import threading
import time

from interfaces.action import ExecuteMission
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


SCRIPT = Path(__file__).with_name('mission_client.py')


@pytest.fixture(autouse=True)
def isolated_ros(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '178')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)
    monkeypatch.delenv('ROS_STATIC_PEERS', raising=False)


def launch_client(cancel_file):
    environment = os.environ.copy()
    environment['CBR_MISSION_CANCEL_FILE'] = str(cancel_file)
    return subprocess.Popen(
        [sys.executable, '-u', str(SCRIPT), 'test_plan'], env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


@pytest.mark.parametrize('before_acceptance', [False, True])
def test_back_cancels_exact_goal_and_waits_for_server_cleanup(tmp_path, before_acceptance):
    context = Context()
    rclpy.init(context=context)
    node = Node('fake_mission_cancel_test', context=context)
    goal_received = threading.Event()
    allow_acceptance = threading.Event()
    cancel_received = threading.Event()
    allow_finish = threading.Event()
    requests = []
    if not before_acceptance:
        allow_acceptance.set()

    def accept(request):
        assert request.plan_id == 'test_plan'
        assert request.start_pose == 'start'
        goal_received.set()
        assert allow_acceptance.wait(10)
        return GoalResponse.ACCEPT

    def cancel(handle):
        requests.append(bytes(handle.goal_id.uuid))
        cancel_received.set()
        return CancelResponse.ACCEPT

    def execute(handle):
        deadline = time.monotonic() + 10
        while not handle.is_cancel_requested:
            assert time.monotonic() < deadline
            time.sleep(0.01)
        assert allow_finish.wait(10)
        handle.canceled()
        return ExecuteMission.Result(code=ExecuteMission.Result.CANCELED, message='cancelada')

    server = ActionServer(
        node, ExecuteMission, '/mission/execute', execute,
        goal_callback=accept, cancel_callback=cancel,
        callback_group=ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    cancel_file = tmp_path / 'cancelar'
    process = launch_client(cancel_file)
    try:
        assert goal_received.wait(10)
        cancel_file.touch()
        allow_acceptance.set()
        assert cancel_received.wait(10)
        time.sleep(0.2)
        assert process.poll() is None  # Must wait for the operation to finish canceling.
        allow_finish.set()
        output, _ = process.communicate(timeout=10)
        assert process.returncode == 130, output
        assert 'Missão cancelada' in output
        assert len(requests) == 1
    finally:
        allow_acceptance.set()
        allow_finish.set()
        if process.poll() is None:
            process.kill()
            process.wait()
        executor.shutdown(timeout_sec=12)
        thread.join(timeout=12)
        server.destroy()
        node.destroy_node()
        context.try_shutdown()


def test_back_before_start_does_not_wait_for_server_or_send_goal(tmp_path):
    cancel_file = tmp_path / 'cancelar'
    cancel_file.touch()
    process = launch_client(cancel_file)
    try:
        output, _ = process.communicate(timeout=10)
        assert process.returncode == 130, output
        assert 'antes do envio' in output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_success_returns_to_menu_without_requesting_cancel(tmp_path):
    context = Context()
    rclpy.init(context=context)
    node = Node('fake_successful_mission_test', context=context)
    cancel_requests = []

    def execute(handle):
        handle.succeed()
        return ExecuteMission.Result(code=ExecuteMission.Result.SUCCESS, message='concluída')

    server = ActionServer(
        node, ExecuteMission, '/mission/execute', execute,
        cancel_callback=lambda handle: cancel_requests.append(handle) or CancelResponse.ACCEPT)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    process = launch_client(tmp_path / 'cancelar')
    try:
        output, _ = process.communicate(timeout=10)
        assert process.returncode == 0, output
        assert 'concluída' in output
        assert cancel_requests == []
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        executor.shutdown(timeout_sec=10)
        thread.join(timeout=10)
        server.destroy()
        node.destroy_node()
        context.try_shutdown()
