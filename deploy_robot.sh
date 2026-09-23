#!/bin/bash

set -euo pipefail

# BANANA="banana@172.20.10.8"
BANANA="banana@10.223.230.44"
# BANANA="banana@192.168.1.216"

# RASPBERRY="rasp@172.20.10.13"
RASPBERRY="rasp@10.223.230.248"
# RASPBERRY="rasp@192.168.1.114"

WORKSPACE="$HOME/ros2_ws"
SOURCE_DIR="$WORKSPACE/src/cbr_work/"

sync_sources() {
  local host="$1"

  # Do not preserve source mtimes: a changed CMakeLists.txt must be newer than
  # the remote CMake cache so newly registered ROS interfaces are regenerated.
  rsync -av \
    --delete \
    --exclude build \
    --exclude install \
    --exclude log \
    --exclude .git \
    --exclude __pycache__ \
    --exclude .pyc \
    --exclude .pyo \
    --exclude .pyd \
    "$SOURCE_DIR" \
    "$host":~/ros2_ws/src/cbr_work/
}

sync_sources "$BANANA"
sync_sources "$RASPBERRY"
