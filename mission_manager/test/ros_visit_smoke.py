"""Live ROS smoke test using isolated simulated child action servers.

Run after building/sourcing the workspace, with ROS_DOMAIN_ID set to an unused
local domain and ROS_LOCALHOST_ONLY=1. No physical capability node is needed.
"""
from pathlib import Path
import threading

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from interfaces.action import (
    AnalyzeScene, ExecuteMission, FollowWall, PickObject, PlaceInContainer,
    PlaceOnShelf, PlaceOnTable, PrepareManipulator, RetrieveObject, StackObject,
    StoreObject,
)
from interfaces.msg import AprilTagStampedDetection, ContainerStampedDetection, ManipulationResult
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool

from mission_manager.models import DeliveryOutcome
from mission_manager.node import MissionManager
from mission_manager.world_state import EMPTY


def wait(future, timeout=30):
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    assert done.wait(timeout), 'ROS future timed out'
    return future.result()


def main():
    package = Path(__file__).resolve().parents[1]
    # All capability names are private to the smoke test, even in this domain.
    prefix = '/visit_smoke'
    action_types = {
        'navigate_action': (NavigateToPose, '/navigate_to_pose'),
        'wall_control_action': (FollowWall, '/vl53/follow_wall'),
        'prepare_action': (PrepareManipulator, '/manipulation/prepare'),
        'pick_action': (PickObject, '/manipulation/pick'),
        'store_action': (StoreObject, '/manipulation/store'),
        'retrieve_action': (RetrieveObject, '/manipulation/retrieve'),
        'place_on_table_action': (PlaceOnTable, '/manipulation/place_on_table'),
        'place_in_container_action': (PlaceInContainer, '/manipulation/place_in_container'),
        'stack_action': (StackObject, '/manipulation/stack'),
        'place_on_shelf_action': (PlaceOnShelf, '/manipulation/place_on_shelf'),
        'vision_action': (AnalyzeScene, '/vision/analyze_scene'),
    }
    services = ('camera_capture_service', 'vision_led_hold_off_service', 'vision_led_service')
    args = ['--ros-args', '-p', f'arena_file:={package / "config/arena.yaml"}',
            '-p', f'plans_directory:={package / "test/fixtures"}',
            '-p', f'execute_action:={prefix}/execute', '-p', f'state_topic:={prefix}/state']
    for parameter in (*action_types, *services):
        args.extend(['-p', f'{parameter}:={prefix}/{parameter}'])
    rclpy.init(args=args)
    simulator = Node('visit_smoke_simulator')
    manager = None
    executor = MultiThreadedExecutor(num_threads=6)
    thread = None
    servers = []
    clients = []
    scenes = {'ws_1': {1, 2, 3}, 'ws_2': {4}, 'ws_3': set()}
    picked = []
    direct_picks = []
    arrivals = []
    operations = []
    group = ReentrantCallbackGroup()
    try:
        manager = MissionManager()
        def execute(action_type, handle):
            goal = handle.request
            result = action_type.Result()
            operations.append(action_type.__name__)
            if action_type is AnalyzeScene:
                assert goal.duration.sec == 2
                assert manager._world_state.snapshot()[1] == EMPTY
                result.frames_processed = 10
                result.frames_with_base_transform = 10
                for tag in sorted(scenes.get(manager._current_location, set())):
                    item = AprilTagStampedDetection()
                    item.id = tag
                    item.header.frame_id = 'arm_base_link'
                    item.pose.orientation.w = 1.0
                    item.pose.position.y = -0.22
                    result.best_apriltags_base.append(item)
                container = ContainerStampedDetection()
                container.color = container.RED
                container.pose.position.y = -0.22
                container.observation_count = 10
                result.best_containers_base = [container]
            elif action_type is FollowWall:
                result.has_valid_reading = True
                result.has_valid_odometry = True
                result.final_left_distance_mm = goal.wall_distance_mm
                result.final_right_distance_mm = goal.wall_distance_mm
                result.final_average_distance_mm = float(goal.wall_distance_mm)
                result.traveled_distance_mm = float(goal.travel_distance_mm)
            elif action_type is NavigateToPose:
                arrivals.append(manager._world_state.snapshot())
            else:
                result.outcome.code = ManipulationResult.SUCCESS
                result.outcome.effect_known = True
                locations = {
                    PickObject: ManipulationResult.LOCATION_GRIPPER,
                    StoreObject: ManipulationResult.LOCATION_CARGO,
                    RetrieveObject: ManipulationResult.LOCATION_GRIPPER,
                    PrepareManipulator: ManipulationResult.LOCATION_UNKNOWN,
                }
                result.outcome.final_object_location = locations.get(
                    action_type, ManipulationResult.LOCATION_DESTINATION)
                if action_type is PickObject:
                    direct_picks.append(goal.use_observed_detection)
                    result.used_observed_detection = goal.use_observed_detection
                    picked.append(goal.tag_id)
                    scenes[manager._current_location].discard(goal.tag_id)
            handle.succeed()
            return result
        for parameter, (action_type, _default) in action_types.items():
            servers.append(ActionServer(
                simulator, action_type, f'{prefix}/{parameter}',
                execute_callback=lambda handle, t=action_type: execute(t, handle),
                callback_group=group))
        def service(_request, response):
            response.success = True
            return response
        for name in services:
            simulator.create_service(SetBool, f'{prefix}/{name}', service, callback_group=group)
        executor.add_node(simulator)
        executor.add_node(manager)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        client = ActionClient(simulator, ExecuteMission, f'{prefix}/execute', callback_group=group)
        clients.append(client)
        assert client.wait_for_server(timeout_sec=5)
        goal = ExecuteMission.Goal()
        goal.plan_id = 'ros_visit_smoke'
        feedback = []
        handle = wait(client.send_goal_async(goal, feedback_callback=lambda item: feedback.append(item.feedback)))
        assert handle.accepted
        result = wait(handle.get_result_async()).result
        assert result.code == ExecuteMission.Result.SUCCESS, result.message
        assert result.completed_steps == 12
        assert len(arrivals) == 4
        assert arrivals[1][1] in {1, 2} and 3 in arrivals[1][2].values()
        assert set(picked) == {1, 2, 3, 4}
        assert 'StoreObject' in operations and 'RetrieveObject' in operations
        assert operations.count('AnalyzeScene') == 2, operations
        assert direct_picks == [True, False, False, True], direct_picks
        assert any(f.operation == 'store' for f in feedback)
        assert manager._world_state.snapshot() == (True, EMPTY, {'left': EMPTY, 'right': EMPTY})
        assert all(isinstance(item, DeliveryOutcome) for item in manager._delivery_outcomes)
        print(f'Live ROS visits smoke passed: {result.completed_steps} steps; picks={picked}')
    finally:
        executor.shutdown(timeout_sec=5)
        if thread is not None:
            thread.join(timeout=5)
        for client in clients:
            client.destroy()
        for server in servers:
            server.destroy()
        if manager is not None:
            manager.destroy_node()
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
