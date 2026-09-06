#!/bin/bash
# Runtime only. Run run_a5_sim.py separately in its ROS environment.
set -euo pipefail

if [[ "${1:-}" == --help || $# -eq 0 ]]; then
  printf '%s\n' \
    'Usage: P450_PX4_ROOT=DIR [SIM_ROOT=DIR] bash scripts/run_a5_gazebo.bash RUN_DIR [roslaunch arguments...]' \
    'Starts the existing SIM runtime with MID360 and without its original demo node.' \
    'A5_ROS_PORT=11951 and A5_GAZEBO_PORT=11952 select dedicated local ports.' \
    'Use Ctrl-C to stop roslaunch and its nodes. Run the A5 adapter separately.'
  exit 0
fi

sim_root="${SIM_ROOT:-/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation}"
run_dir="$1"
shift
mkdir -p "$run_dir"
run_dir="$(realpath "$run_dir")"
: "${P450_PX4_ROOT:?Set P450_PX4_ROOT to the existing compatible PX4 checkout}"
export P450_GAZEBO_DISPLAY="${P450_GAZEBO_DISPLAY:-${DISPLAY:-}}"
export P450_GAZEBO_XAUTHORITY="${P450_GAZEBO_XAUTHORITY:-${XAUTHORITY:-}}"

exec "$sim_root/scripts/with_p450_env.bash" /usr/bin/env \
  ROS_MASTER_URI="http://127.0.0.1:${A5_ROS_PORT:-11951}" \
  GAZEBO_MASTER_URI="http://127.0.0.1:${A5_GAZEBO_PORT:-11952}" \
  ROS_LOG_DIR="$run_dir/ros" \
  roslaunch air_ground_pick_demo air_ground_pick_demo.launch \
  gui:=false bunker_yaw:=3.141592653589793 \
  px4_workdir:="sitl_a5_$(date -u +%Y%m%dT%H%M%SZ)_$$" \
  "$@" run_demo:=false enable_mid360:=true
