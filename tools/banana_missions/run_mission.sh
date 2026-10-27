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

exec python -u /home/banana/ros2_ws/src/cbr_work/tools/banana_missions/mission_client.py "$1"
