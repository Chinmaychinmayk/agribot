#!/usr/bin/env bash
# Build (if needed) and run the farm-nav container.
#
# Uses the NVIDIA GPU for Gazebo/RViz rendering when available, falling back to
# software GL otherwise. Forwards the host X server so Gazebo's camera sensors
# get a GL context (needed even when the Gazebo GUI is hidden).
#
#   ./run.sh                 # GUI (Gazebo + RViz)
#   ./run.sh headless:=true use_rviz:=false   # no GUI windows
set -e

IMAGE=farm_nav
EXTRA_ARGS="$*"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo ">> building image $IMAGE ..."
  docker build -t "$IMAGE" .
fi

GPU_FLAGS=""
GL_ENV="-e LIBGL_ALWAYS_SOFTWARE=1"
if docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia' && command -v nvidia-smi >/dev/null 2>&1; then
  echo ">> NVIDIA GPU detected: using hardware rendering"
  GPU_FLAGS="--gpus all"
  GL_ENV="-e LIBGL_ALWAYS_SOFTWARE=0 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all"
fi

xhost +local:root >/dev/null 2>&1 || true

docker run -it --rm \
  $GPU_FLAGS \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  $GL_ENV \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:rw" \
  --env="XAUTHORITY=/root/.Xauthority" \
  --volume="$(pwd)/output:/output:rw" \
  --name farm_nav \
  "$IMAGE" \
  bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && \
            ros2 launch farm_bringup bringup.launch.py \
              out_path:=/output/farm_run_path.png \
              video_path:=/output/farm_run_overlay.mp4 ${EXTRA_ARGS}"
