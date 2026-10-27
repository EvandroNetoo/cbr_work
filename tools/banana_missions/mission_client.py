"""Run a mission and forward the menu's Back request to its exact action goal."""

import argparse
import os
from pathlib import Path
import signal
import threading

from action_msgs.msg import GoalStatus
from interfaces.action import ExecuteMission
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


def run(plan_id):
    cancel_requested = threading.Event()
    cancel_path = os.environ.get('CBR_MISSION_CANCEL_FILE')
    signal.signal(signal.SIGINT, lambda *_: cancel_requested.set())
    signal.signal(signal.SIGTERM, lambda *_: cancel_requested.set())

    def canceled():
        return cancel_requested.is_set() or (cancel_path and Path(cancel_path).exists())

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('banana_mission_client')
    client = ActionClient(node, ExecuteMission, '/mission/execute')
    try:
        print('Aguardando servidor /mission/execute...')
        while not client.wait_for_server(timeout_sec=0.1):
            if canceled():
                print('Início da missão cancelado antes do envio.')
                return 130
        if canceled():
            print('Início da missão cancelado antes do envio.')
            return 130
        pending_goal = client.send_goal_async(
            ExecuteMission.Goal(plan_id=plan_id, start_pose='start'),
            feedback_callback=lambda msg: print(
                f'{msg.feedback.step_id}: {msg.feedback.description}'))
        # If Back arrives while acceptance is pending, cancel as soon as accepted.
        while not pending_goal.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        goal = pending_goal.result()
        if not goal.accepted:
            print('Missão rejeitada pelo servidor.')
            return 1
        print(f'Missão aceita: {plan_id}. Voltar solicita cancelamento.')
        result_future = goal.get_result_async()
        cancel_future = None
        cancel_reported = False
        while not result_future.done():
            if canceled() and cancel_future is None:
                print('Solicitando cancelamento da missão; aguardando encerramento...')
                cancel_future = goal.cancel_goal_async()
            rclpy.spin_once(node, timeout_sec=0.1)
            if cancel_future is not None and cancel_future.done() and not cancel_reported:
                response = cancel_future.result()
                print(f'Resposta do cancelamento: código {response.return_code}.')
                cancel_reported = True
        response = result_future.result()
        print(f'Resultado: código {response.result.code}; {response.result.message}')
        if response.status == GoalStatus.STATUS_CANCELED:
            print('Missão cancelada. Retornando ao menu.')
            return 130
        return 0 if response.status == GoalStatus.STATUS_SUCCEEDED else 1
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan_id')
    raise SystemExit(run(parser.parse_args().plan_id))
