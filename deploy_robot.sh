#!/bin/bash

set -e

# BANANA="banana@172.20.10.8"
BANANA="banana@10.23.247.44"
# BANANA="banana@192.168.1.216"

RASPBERRY="rasp@10.23.247.248"
# RASPBERRY="rasp@10.12.217.248"
# RASPBERRY="rasp@192.168.1.114"

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
  "$BANANA":~/ros2_ws/src/cbr_work/

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
  "$RASPBERRY":~/ros2_ws/src/cbr_work/