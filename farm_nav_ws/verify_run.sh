#!/usr/bin/env bash
# Headless end-to-end verification run (intended to run INSIDE the container).
# Launches the full stack, polls nav status + plant count, saves artifacts.
#
#   ros2 ... is sourced here; outputs go to /output.
set -u
source /opt/ros/humble/setup.bash
source /farm_nav_ws/install/setup.bash

DURATION="${1:-600}"     # max seconds to wait
mkdir -p /output

echo ">> launching stack (headless) ..."
ros2 launch farm_bringup bringup.launch.py headless:=true use_rviz:=false \
     out_path:=/output/farm_run_path.png \
     video_path:=/output/farm_run_overlay.mp4 \
     > /output/launch.log 2>&1 &
LAUNCH_PID=$!

trap 'kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null' EXIT

best_count=0
elapsed=0
step=10
while [ "$elapsed" -lt "$DURATION" ]; do
  sleep "$step"; elapsed=$((elapsed+step))
  status=$(timeout 5 ros2 topic echo /perception/nav_status std_msgs/msg/String --once 2>/dev/null \
           | sed -n 's/^data: //p' | tr -d '"')
  count=$(timeout 5 ros2 topic echo /perception/plant_count std_msgs/msg/Int32 --once 2>/dev/null \
          | sed -n 's/^data: //p')
  [ -n "${count:-}" ] && [ "$count" -gt "$best_count" ] 2>/dev/null && best_count=$count
  echo "[${elapsed}s] nav='${status:-?}' plants='${count:-?}' (max ${best_count})"
  case "$status" in
    SUCCEEDED|ENDED*|REJECTED|NO_*) echo ">> mission ended: $status"; break;;
  esac
done

echo ">> FINAL plant count (max observed): $best_count   (ground truth = 26)"
echo ">> saving path plot (SIGINT to recorder) ..."
kill -INT $LAUNCH_PID 2>/dev/null
sleep 8
echo ">> artifacts in /output:"; ls -la /output
