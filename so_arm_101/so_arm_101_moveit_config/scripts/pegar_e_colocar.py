#!/usr/bin/env python3
"""Cliente finito de compatibilidade para iniciar uma missão configurada."""

from __future__ import annotations

import sys

import rclpy
from interfaces.action import ExecuteMission
from rclpy.action import ActionClient


def main() -> int:
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("pegar_e_colocar_client")
    client = ActionClient(node, ExecuteMission, "/mission/execute")
    mission_name = sys.argv[1] if len(sys.argv) > 1 else "exemplo"
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(
                "Gerenciador /mission/execute indisponível; habilite enable_mission."
            )
            return 2
        goal = ExecuteMission.Goal()
        goal.mission_name = mission_name

        def feedback(message) -> None:
            node.get_logger().info(message.feedback.message)

        sent = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            node.get_logger().error(f"Missão '{mission_name}' rejeitada.")
            return 3
        finished = handle.get_result_async()
        rclpy.spin_until_future_complete(node, finished)
        wrapped = finished.result()
        if wrapped is None:
            node.get_logger().error("Gerenciador encerrou sem resultado.")
            return 4
        node.get_logger().info(wrapped.result.message)
        return 0 if wrapped.result.outcome in (0, 1) else 1
    except KeyboardInterrupt:
        node.get_logger().warning("Cancelando missão...")
        if 'handle' in locals() and handle is not None:
            handle.cancel_goal_async()
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
