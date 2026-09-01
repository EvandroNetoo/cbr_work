#!/bin/bash

set -e

# ROBOT="banana@172.20.10.8"
ROBOT="banana@192.168.1.216"
# ROBOT="banana@10.108.64.44"

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
  ~/ros2_ws/src/cbr_work/ \
  "$ROBOT":~/ros2_ws/src/cbr_work/