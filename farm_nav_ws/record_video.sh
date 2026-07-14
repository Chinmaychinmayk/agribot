#!/usr/bin/env bash
# Optional: screen-capture Gazebo + RViz to MP4 while the sim runs (host side).
# The annotated perception video is produced automatically by video_recorder_node;
# this is only if you also want a full-screen capture.
#
#   ./record_video.sh [output.mp4] [seconds]
set -e
OUT="${1:-output/farm_screen_capture.mp4}"
DUR="${2:-120}"
mkdir -p "$(dirname "$OUT")"
echo ">> recording screen for ${DUR}s -> ${OUT}"
ffmpeg -y -video_size 1920x1080 -framerate 25 -f x11grab -i "${DISPLAY:-:0}" \
       -t "$DUR" -c:v libx264 -preset ultrafast -pix_fmt yuv420p "$OUT"
echo ">> saved ${OUT}"
