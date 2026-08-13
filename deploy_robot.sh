#!/bin/bash

set -e

ROBOT="banana@192.168.1.111"

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