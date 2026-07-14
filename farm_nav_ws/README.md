# Autonomous Farm Lane Navigation & Plant Counting (ROS 2 Humble)

Versor Robotics / ARTPARK — Robotics Software Intern, Round 2.

A Husky A200 with a **single RealSense D435 RGB-D camera** autonomously drives
from the blue start marker to the green goal marker through irregular farm lanes,
detecting and counting plants — using **ROS 2 Humble + Gazebo + Nav2 + RTAB-Map**.
Lane perception is **purely geometric (depth-based), with no colour/HSV
thresholding** anywhere (per the brief, colour thresholding = disqualification).

See **[WRITEUP.md](WRITEUP.md)** for the full technical explanation (perception,
navigation, hardcoded-vs-learned, evaluation).

---

## Why Docker

ROS 2 **Humble** targets Ubuntu 22.04 and has no native packages on Ubuntu
24.04. The provided `Dockerfile` builds on `osrf/ros:humble-desktop-full`, so it
runs identically regardless of host OS.

### Build & run (GUI)

```bash
cd farm_nav_ws
# Linux host with an X server:
./run.sh
```

`run.sh` builds the image (first time), forwards X11 for Gazebo + RViz, mounts
`./output`, and launches everything. Deliverable artifacts land in `./output/`:

* `farm_run_overlay.mp4` — annotated camera video (RGB + drivable region +
  obstacles + plant count + nav status).
* `farm_run_path.png` — planned vs. actual path + plant positions (written on
  Ctrl-C).

Headless (no GUI, e.g. CI/cloud):

```bash
docker run -it --rm --net=host -v $(pwd)/output:/output farm_nav \
  bash -lc "source install/setup.bash && \
            ros2 launch farm_bringup bringup.launch.py headless:=true use_rviz:=false \
              out_path:=/output/farm_run_path.png video_path:=/output/farm_run_overlay.mp4"
```

### Native build (if you already have Humble on 22.04)

```bash
cd farm_nav_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch farm_bringup bringup.launch.py
```

---

## What launches

`ros2 launch farm_bringup bringup.launch.py` starts, in order:

1. **Gazebo** with the five-lane farm world (`farm_bringup/worlds/`).
2. **robot_state_publisher** — Husky + RealSense URDF/Xacro.
3. **robot_spawner_node** — spawns the Husky at the **blue marker (read at runtime)**.
4. **RTAB-Map** — RGB-D SLAM (camera only) → drift-corrected `map → odom`.
5. **Nav2** — plans/drives to the goal on the perceived obstacle costmap.
6. **Perception + mission** — lane/free-space, plant counting, path/video
   recording, and **goal sending from the green marker (read at runtime)**.
7. **RViz** — live view (overlay image, obstacle cloud, costmap, paths, plants).

### Launch arguments

| arg | default | meaning |
|---|---|---|
| `use_rviz` | `true` | start RViz |
| `use_slam` | `true` | RTAB-Map SLAM; `false` → static `map=odom` (wheel odom only) |
| `headless` | `false` | no Gazebo client GUI |
| `world` | five_lane_farm.world | world file |
| `out_path` | `/tmp/farm_run_path.png` | path-plot output |
| `video_path` | `/tmp/farm_run_overlay.mp4` | overlay video output |

---

## Key topics

| topic | type | meaning |
|---|---|---|
| `/camera/color/image_raw` | Image | RGB stream |
| `/camera/depth/image_rect_raw` | Image | depth stream |
| `/camera/depth/camera_info` | CameraInfo | intrinsics |
| `/perception/obstacles` | PointCloud2 | geometric obstacle cloud (costmap source) |
| `/perception/overlay/image_raw` | Image | annotated video frame |
| `/perception/plant_count` | Int32 | running unique plant count |
| `/perception/plant_markers` | MarkerArray | detected plant positions |
| `/plan` `/actual_path` | Path | planned / driven trajectory |

---

## Package layout

```
farm_nav_ws/
├── Dockerfile, run.sh, record_video.sh
├── README.md, WRITEUP.md
└── src/
    ├── farm_description/   # Husky + RealSense xacro, Gazebo plugins, RViz, RSP launch
    ├── farm_perception/    # depth-geometric perception, plant counting, mission, recorders
    └── farm_bringup/       # world, Nav2 + RTAB-Map config, launch files
```

---

## Recording the demo video

The overlay MP4 is written automatically by `video_recorder_node`. For a full
screen capture of Gazebo + RViz as well, run `./record_video.sh` on the host
while the simulation is running (requires `ffmpeg`).

## Notes / ground truth

* Plant ground truth in this world: **25 upright plants** (5 each in beds
  0,2,3,4,5) **+ 1 fallen plant = 26**. The count reflects plants actually seen
  along the driven route.
* Start ≈ world (5.8, −3.0) (blue), Goal ≈ world (−5.8, 2.7) (green) — both read
  at runtime, never hardcoded in the nav logic.
