#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
robot_test_python="${ROBOT_TEST_PYTHON:-}"
if [[ -z "$robot_test_python" ]]; then
  if [[ -x .venv/bin/python ]]; then
    robot_test_python="$PWD/.venv/bin/python"
  elif [[ -x ../../../../.venv/bin/python ]]; then
    robot_test_python="$(cd ../../../.. && pwd)/.venv/bin/python"
  else
    robot_test_python=python3
  fi
fi
exec "$robot_test_python" -m streamlit run app.py \
  --server.address=127.0.0.1 --server.port=8501 \
  --browser.gatherUsageStats=false --server.headless=true
