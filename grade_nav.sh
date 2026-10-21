#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STARTUP_WAIT="${GRADE_STARTUP_WAIT:-15}"
START_PROJECT=1

if [[ "${1:-}" == "--no-start" ]]; then
  START_PROJECT=0
  shift
fi

if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
  echo "[评分器] 未找到 $ROOT_DIR/install/setup.bash，请先编译项目。" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

if (( START_PROJECT )); then
  echo "[评分器] 启动完整导航项目……"
  "$ROOT_DIR/bash/sim_nav_all_start.sh"
  echo "[评分器] 等待 ${STARTUP_WAIT}s，确保节点和窗口完成初始化……"
  sleep "$STARTUP_WAIT"
else
  echo "[评分器] --no-start：使用当前已经运行的导航项目。"
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="$ROOT_DIR/grade/results/$RUN_ID"

echo "[评分器] 发布一次目标并开始 70 分评测。"
python3 "$ROOT_DIR/grade/nav_grader.py" \
  --output-dir "$OUTPUT_DIR" \
  "$@"
