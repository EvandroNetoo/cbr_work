#!/bin/bash
# Called by menuPrincipal; the ROS stack must already be running.
set -e
if [[ $# -ne 1 ]]; then
    echo 'Uso: bash run_mission.sh PLAN_ID' >&2
    exit 2
fi
source /home/banana/ros2_ws/.venv/bin/activate
source /opt/ros/jazzy/setup.bash
source /home/banana/ros2_ws/install/setup.bash
source /home/banana/ros2_ws/src/cbr_work/bringup/scripts/dds_environment.bash banana

exec ros2 action send_goal /mission/execute interfaces/action/ExecuteMission \
    "{plan_id: $1, start_pose: start}" --feedback
