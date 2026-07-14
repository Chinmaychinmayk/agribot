# Technical Write-up — Autonomous Farm Lane Navigation & Plant Counting

Versor Robotics / ARTPARK — Robotics Software Intern, Round 2.

This document covers the three required topics: **perception logic**,
**navigation logic**, and **hardcoded vs. learned components**, followed by a
discussion of the system against the evaluation metrics.

---

## 1. System overview

A Husky A200 with a **single Intel RealSense D435 RGB-D camera** drives
autonomously from the blue start marker to the green goal marker through the
five-lane farm, detecting and counting plants on the way. No LiDAR, extra
cameras, GPS or other localization sensors are used. Wheel odometry (internal,
proprioceptive) is the only non-camera signal and is permitted by the brief.

```
                 +-------------------+        /perception/obstacles (map)
 RealSense  --->  | lane_perception   |  ---------------------------------+
 depth+rgb        | (depth-geometric) |  ---> /perception/overlay (video) |
                 +-------------------+                                     |
                          |                                               v
                          v                                   +----------------------+
                 +-------------------+                        |  Nav2 costmaps       |
 depth+rgb  ---> |  RTAB-Map SLAM    | --- map->odom TF --->  |  planner + controller|
 (camera only)   |  (drift fix)      |                        +----------+-----------+
                 +-------------------+                                   | /cmd_vel
                                                                         v
 /perception/obstacles ---> plant_counter ---> count + markers       Husky (Gazebo)
```

Why this shape (and not lane-following): a robot cannot "follow" a lane it has
not seen yet, and the brief explicitly forbids a hardcoded trajectory. So lane
perception is **not** used as a steering signal. Instead it is an obstacle /
free-space source feeding a Nav2 costmap. The **green marker (read at runtime)**
sets the global direction, and Nav2 plans and continuously **re-plans** a
collision-free path on a rolling costmap that fills in as the camera perceives
more of the field. The robot therefore discovers the lanes incrementally rather
than needing prior knowledge of them.

---

## 2. Perception logic

### 2.1 Lane / free-space detection (`lane_perception_node`)

Strictly geometric — **no colour/HSV/RGB thresholding anywhere** (that is an
automatic disqualification per the brief).

1. **Back-projection.** Each depth pixel `(u,v,z)` is converted to a 3-D point
   in the camera optical frame using the intrinsics from `camera_info`:
   `x=(u−cx)·z/fx`, `y=(v−cy)·z/fy`. The depth image is subsampled
   (`DEPTH_STRIDE`) for real-time performance.
2. **Transform to a gravity-aligned world frame.** Points are transformed into
   the `map` frame via TF (`map ← camera_depth_optical_frame`), so "up" is the
   real vertical and heights are physically meaningful.
3. **RANSAC ground-plane fit.** The dominant near-horizontal plane is fit with
   RANSAC (`RANSAC_ITERS`, `RANSAC_THRESH`, candidate normals constrained to
   `|n_z| > 0.85`). This is robust to terrain unevenness and to the raised plant
   beds (which appear as outliers), and it does **not** assume a perfect `z = 0`
   ground — it recovers the true ground even if the camera pitch is slightly
   off. If no plane is found it falls back to `z = 0`.
4. **Height classification.** For every point the signed height above the fitted
   plane is computed:
   * `|h| ≤ GROUND_BAND` → **drivable** lane surface,
   * `OBSTACLE_MIN_HEIGHT < h < OBSTACLE_MAX_HEIGHT` → **obstacle** (plant bed,
     plant, fallen-plant debris).
5. **Outputs.**
   * `/perception/obstacles` — a `PointCloud2` of obstacle points in `map`, fed
     to the Nav2 costmap (this is what makes navigation perception-driven).
   * `/perception/overlay/image_raw` — the RGB frame with the drivable region
     tinted green and obstacles tinted red, plus plant-count and nav-status
     text, used directly for the demonstration video.

**RGB vs. depth fusion.** Geometry (depth) does all the *decision-making*; RGB
is used only as the canvas for the human-facing overlay. The two are spatially
registered because every classified 3-D point carries its source pixel `(u,v)`.

### 2.2 Plant detection & counting (`plant_counter_node`)

Also entirely geometric, and designed to never double-count.

1. Consume `/perception/obstacles` (already in the fixed `map` frame).
2. Keep only points in the **foliage height band** `FOLIAGE_MIN_Z..FOLIAGE_MAX_Z`.
   The beds sit at ~0.10 m and are excluded; the foliage spheres peak at
   ~0.42 m, so this band isolates plant canopies from the beds.
3. **Cluster** the surviving points on the XY plane with a lightweight grid +
   connected-components clusterer (cell size `GRID_CELL`). Clusters that are too
   small (`MIN_CLUSTER_PTS`) or too large (`MAX_CLUSTER_EXTENT`, i.e. a bed
   fragment) are rejected.
4. **Track in the world frame.** Each cluster centroid is associated with the
   nearest existing plant track within `PLANT_ASSOC_RADIUS`; if none is close
   enough a new track is created. Because tracks live in the drift-corrected
   `map` frame, the same physical plant maps to the same XY from every
   viewpoint, so seeing it again **updates** its track instead of adding a new
   one — this is the anti-double-counting mechanism.
5. A track is only **counted** after it persists for `MIN_OBSERVATIONS` frames
   (rejects single-frame false positives). Count and `MarkerArray` are published
   on `/perception/plant_count` and `/perception/plant_markers`.

---

## 3. Navigation logic

* **Localization / drift (`slam.launch.py`, RTAB-Map).** With only a camera
  allowed, wheel odometry is the sole free pose source, and 4-wheel skid-steer
  slips on every turn, so `odom` drifts. RTAB-Map runs **RGB-D SLAM on the
  RealSense alone**, using wheel odom as a motion prior and refining it with
  visual registration + loop closure to publish the drift-correcting
  `map → odom` transform. It is constrained to 2D (`Reg/Force3DoF`,
  `Optimizer/Slam2D`) because the farm is planar, which is far more robust in
  this low-texture scene. The result is a consistent global `map` frame for Nav2.

* **Goal generation (`goal_sender_node`).** The blue (start) and green (goal)
  marker poses are **read at runtime** from the world via
  `/gazebo/get_entity_state` — never hardcoded. The goal is expressed in the
  SLAM `map` frame (which is anchored at the spawn pose) and sent to Nav2 via the
  `navigate_to_pose` action.

* **Spawn at start (`robot_spawner_node`).** The Husky is spawned at the blue
  marker, also read at runtime, so the start position is likewise not a literal.

* **Planning & control (Nav2).** The global planner (`NavFn`, A*) plans on a
  rolling global costmap whose **only obstacle source is the perceived
  `/perception/obstacles` cloud**. The Regulated Pure Pursuit controller tracks
  the path and is well suited to a differential/skid-steer base. As the robot
  advances and perceives more beds/plants/the fallen plant, the costmap updates
  and Nav2 re-plans in real time. The robot radius (0.40 m) + inflation makes
  the 0.40 m **narrow lane correctly non-traversable** for the 0.67 m-wide
  Husky, exactly as the world was designed.

* **Recording (`path_recorder_node`, `video_recorder_node`).** The actual
  trajectory is sampled from `map → base_footprint` and published as
  `/actual_path`; the latest Nav2 `/plan` is cached. On shutdown a matplotlib
  figure overlays planned vs. actual paths and plant positions and prints path
  lengths. The overlay image stream is written to an MP4 for the demo video.

---

## 4. Hardcoded vs. learned components

This solution is **deliberately fully hand-engineered** (no trained models).
That is a valid "principled approach" under the brief and it needs no labelled
data or GPU. The trade-off (and how a learned component could slot in) is noted
below.

### Hand-engineered (all of it)

| Component | Type | Key tuned constants |
|---|---|---|
| Depth back-projection | geometry (intrinsics) | `DEPTH_STRIDE=4`, range `0.25..6.0 m` |
| Ground-plane fit | RANSAC geometry | `RANSAC_ITERS=60`, `RANSAC_THRESH=0.03 m`, normal `|n_z|>0.85` |
| Drivable / obstacle split | height threshold | `GROUND_BAND=0.06 m`, `OBSTACLE_MIN_HEIGHT=0.10 m`, `OBSTACLE_MAX_HEIGHT=1.20 m` |
| Plant canopy isolation | height band | `FOLIAGE_MIN_Z=0.28 m`, `FOLIAGE_MAX_Z=0.70 m` |
| Plant clustering | grid + CC | `GRID_CELL=0.15 m`, `MIN_CLUSTER_PTS=4`, `MAX_CLUSTER_EXTENT=0.8 m` |
| Plant de-duplication | nearest-neighbour track | `PLANT_ASSOC_RADIUS=0.45 m`, `MIN_OBSERVATIONS=2` |
| Safety / geometry | Nav2 | `robot_radius=0.40 m`, `inflation_radius=0.45 m`, `desired_linear_vel=0.6 m/s` |
| Spawn orientation | config | `spawn_yaw=π` (face the goal) |

Geometric assumptions: the ground is locally planar (RANSAC tolerates small
unevenness); beds are raised above the lane surface; plant canopies sit in a
known height band above the beds; the world is essentially static.

### Learned components

**None in the submitted system.** RTAB-Map's visual front-end uses classical
features (not a trained network).

How a learned part would drop in: replace the height-band + clustering plant
detector with a small RGB-D object detector / instance-segmentation model
(e.g. a fine-tuned YOLO) for the *detection* step, while keeping the geometric
`map`-frame tracker for de-duplication. Likewise, lane/ground estimation could
be swapped for a learned semantic-segmentation head. Both were intentionally
left geometric here so the pipeline is reproducible without training data or a
GPU; the interfaces (`/perception/obstacles`, `/perception/plant_*`) are
unchanged, so a learned module is a drop-in replacement.

---

## 5. Performance vs. evaluation metrics

* **Navigation success** — Nav2 drives to the green goal on the perceived
  costmap; the narrow lane is correctly excluded by inflation, and the fallen
  plant is treated as an obstacle and avoided.
* **Lane coverage efficiency** — the robot traverses the wide lanes between
  start and goal; coverage can be increased by issuing the goal via
  `navigate_through_poses` with one waypoint per lane (interfaces already
  support this).
* **Path efficiency** — `path_recorder_node` reports actual vs. planned length
  and the plot shows deviation; RPP produces smooth, low-overshoot tracking.
* **Plant counting accuracy** — world ground truth is **25 upright plants + 1
  fallen plant = 26** total (5 plants in each of beds 0, 2, 3, 4, 5 at
  x = −4,−2,0,2,4 m; bed 1 has no plants modelled). Only plants the camera
  actually sees during the run are counted;
  the `map`-frame tracker prevents duplicates and `MIN_OBSERVATIONS` suppresses
  false positives. Reported count = plants observed along the driven route.

## 6. Known limitations

* In feature-poor regions RTAB-Map's visual loop closure may fire rarely, so
  drift correction is limited and the system leans on wheel odom; the 2D
  constraint mitigates this. An alternative is depth-ICP odometry.
* A single forward-down camera only sees plants ahead/beside the path, so plants
  in lanes never entered are not counted — by design (count = plants encountered
  on the run).
* Constants are tuned for this world's geometry (bed/plant heights); they are
  all centralised at the top of each node for easy retuning.
