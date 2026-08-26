# ros2_ws_gp — Technology Overview

**The ROS 2 stack that runs on the V-PACE Jetson Orin node**

---

## Scope

This workspace holds the ROS 2 Humble source for the Jetson Orin node of V-PACE. The
Yocto layer [`meta-vpace`](https://github.com/ITIGP-ROS) packages a subset of these
packages into the vehicle image; this document covers **that subset only**, at a level
meant to explain *what technology is used and why*, not to serve as an API reference.

Two of those subsystems are machine-learning systems rather than conventional software — a
3D LiDAR object detector and a camera-based traffic-sign detector — and they get the most
space here: what the networks are, how they were trained for this vehicle, how they were
compiled for the target GPU, and what they measure.

For how the packages are built, packaged and deployed, see `meta-vpace/docs/ARCHITECTURE.md`.
For the details of individual subsystems, follow the links in
[§9](#9-related-documents) — several packages carry their own, deeper notes.

### Packages covered

| Package | Origin | Role |
|---|---|---|
| `ackermann_description` | this repo | URDF/xacro vehicle model |
| `ackermann_hardware` | this repo | `ros2_control` hardware interface over CAN |
| `ackermann_bringup` | this repo | Launch files, controller and Nav2 configuration |
| `camera_sign_detect_bringup` | this repo | Traffic-sign detection pipeline and model assets |
| `update_coordinator` | this repo | SecOC-authenticated OTA coordination over CAN |
| `cuda_pointpillars_ros` | `ros2-lidar-perception` | TensorRT 3D LiDAR detector with in-process tracking |
| `lidar_tracking` | `ros2-lidar-perception` | C++ AB3DMOT multi-object tracker |
| `object_detection_msgs` | `ros2-lidar-perception` | 3D detection message definitions |
| `object_visualization` | `ros2-lidar-perception` | RViz marker publisher |
| `lidar_perception_bringup` | `ros2-lidar-perception` | Launch file and sensor profiles |
| `ros2_yolos_cpp_trt` | submodule | TensorRT-backed YOLO inference nodes |
| `livox_ros_driver2` | submodule | Livox LiDAR driver |

### Packages not covered

`ros2-lidar-object-detection`, `lidar_object_detect_bringup` and `CUDA-PointPillars-ROS2`
are earlier iterations superseded by `ros2-lidar-perception`, and are not built into the
image. `ros2_yolos_cpp` is the ONNX Runtime sibling of `ros2_yolos_cpp_trt`; the shipped
configuration selects the TensorRT backend, so only that one is described here.

---

## Table of contents

1. [ROS 2 concepts this stack leans on](#1-ros-2-concepts-this-stack-leans-on)
2. [The Ackermann control stack](#2-the-ackermann-control-stack)
3. [The LiDAR perception pipeline](#3-the-lidar-perception-pipeline)
4. [The camera sign-detection pipeline](#4-the-camera-sign-detection-pipeline)
5. [The update coordinator](#5-the-update-coordinator)
6. [Interfaces](#6-interfaces)
7. [Recurring design themes](#7-recurring-design-themes)
8. [Building](#8-building)
9. [Related documents](#9-related-documents)

---

# 1. ROS 2 concepts this stack leans on

Four ROS 2 mechanisms carry most of the weight here. Each is used deliberately and each
shapes a package's structure, so they are worth naming before the packages themselves.

## 1.1 Node composition and intra-process communication

A ROS 2 node normally runs in its own process and talks to others through DDS, which means
every message is serialised, handed to the middleware and deserialised. **Composable nodes**
are compiled as shared libraries and loaded into a shared `component_container` process
instead. Within that container, ROS 2 can pass messages as `std::shared_ptr<const T>` —
no serialisation, no copy.

That matters most where the payload is large and the producer/consumer pair is fixed. The
camera pipeline uses it for exactly that reason: a 640×480 YUYV frame moves from the V4L2
driver node to the detector node as a pointer rather than as ~600 KB through the
middleware.

It has a cost that shows up elsewhere in this system. A composable node that throws during
construction does **not** take the container down — the loader logs the failure and the
launch continues. A pipeline can therefore report itself healthy while a node inside it
never loaded, which is why the systemd unit in `meta-vpace` for the camera pipeline
declares a hard dependency on the CAN interface being up (§4.4).

## 1.2 Managed (lifecycle) nodes

A lifecycle node has explicit states — `unconfigured`, `inactive`, `active`, `finalized` —
with transitions the system can drive. Resource acquisition happens in `configure`, and
publishing only happens in `active`.

The detector nodes are lifecycle nodes because their `configure` step is expensive: it
loads a model onto the GPU and, for TensorRT, may build an engine. Separating that from
activation means the load can be done once, and the node can be deactivated and reactivated
without repeating it. The camera pipeline's launch file drives the transitions
automatically so the pipeline comes up without operator interaction.

## 1.3 `ros2_control`

`ros2_control` splits vehicle control into three replaceable layers:

| Layer | What it is | Here |
|---|---|---|
| **Hardware interface** | A `pluginlib` plugin exposing *state* and *command* interfaces for each joint | `ackermann_hardware` |
| **Controller manager** | Runs a fixed-rate loop: `read()` from hardware, `update()` controllers, `write()` back | Configured at 30 Hz |
| **Controllers** | Consume command topics, produce joint commands | `ackermann_steering_controller`, `joint_state_broadcaster`, `imu_sensor_broadcaster` |

The value of the split is that the same controllers run unchanged against simulation or
real hardware — only the plugin named in the URDF changes. `ackermann_description` carries
both variants behind a xacro conditional (§2.1).

## 1.4 DDS and QoS

ROS 2's transport is DDS. Two consequences are visible in this stack:

- **Discovery is automatic but interface-bound.** A participant enumerates network
  interfaces when it is created and does not rescan, so a node that starts before the
  network has an address never sees peers that arrive later.
- **Transport choice is configurable per participant.** The shipped image restricts the IVI
  head unit to UDP because its user cannot complete a shared-memory channel with root-owned
  publishers. The perception nodes keep the default transports.

Both are described in full in the `meta-vpace` document; they are mentioned here because
they explain behaviour that otherwise looks like a bug in these packages.

---

# 2. The Ackermann control stack

Three packages that together turn a velocity command into wheel and steering commands on
the CAN bus, and turn encoder and IMU readings back into odometry.

## 2.1 `ackermann_description` — the vehicle model

A URDF assembled from xacro fragments:

| File | Contents |
|---|---|
| `robot.xacro` | Top-level assembly |
| `base.xacro` | Chassis and wheel links, joints, inertial and visual geometry (STL meshes) |
| `sensors.xacro` | Sensor frames — LiDAR, camera, IMU |
| `control.xacro` | The `ros2_control` block |

`control.xacro` is the interesting one. It contains **two** `ros2_control` blocks behind a
`hardware` argument:

- **Simulation** — plugin `gazebo_ros2_control/GazeboSystem`.
- **Hardware** — plugin `ackermann_hardware/AckermannHardwareSystem`, parameterised with
  the joint names, the CAN interface and the encoder counts per revolution.

Both declare the same interface set, which is what makes the controllers portable between
them:

| Joint | Command | State |
|---|---|---|
| `front_left_steering_joint`, `front_right_steering_joint` | position | position |
| `rear_left_wheel_joint`, `rear_right_wheel_joint` | velocity | position, velocity |
| `front_left_wheel_joint`, `front_right_wheel_joint` | — | position, velocity |

Plus an `imu_sensor` exporting linear acceleration, angular velocity and orientation.

The shape encodes the vehicle: **rear-wheel drive, front-wheel steering**. Rear wheels take
velocity commands; front steering joints take position commands; front wheels are passive
and only report state.

## 2.2 `ackermann_hardware` — the CAN hardware interface

A `SystemInterface` plugin. The controller manager calls it at a fixed rate; it translates
between the framework's abstract interfaces and the vehicle's actual actuators.

Structure:

| File | Role |
|---|---|
| `ackermann_system.cpp` | The `SystemInterface` implementation and lifecycle callbacks |
| `can_comms.cpp` | SocketCAN transport |
| `v_pace_db.c` | Generated signal pack/unpack for the vehicle's CAN database |
| `drive_wheel.hpp`, `steering_wheel.hpp` | Per-joint state: encoder counts, positions, velocities, commands |

Configuration arrives through the URDF rather than through parameters, which is the
`ros2_control` convention: joint names, `can_interface` (default `can0`) and
`enc_counts_per_rev` (2464 — a 616 PPR encoder read in quadrature).

The unusual part is packaging rather than code: a `pluginlib` plugin is `dlopen`ed under its
unversioned `.so` name, and the default OpenEmbedded packaging rules would put that file in
the `-dev` package and leave it off the image. `meta-vpace` corrects this explicitly.

The `SystemInterface` skeleton derives from the ros2_control demo hardware interfaces and
carries their Apache-2.0 header, crediting the
[ros2_control Development Team](https://github.com/ros-controls/ros2_control_demos); the CAN
transport, the signal database and the wheel models are specific to this vehicle.

## 2.3 `ackermann_bringup` — launch and configuration

The launch and configuration package, and in practice the specification of the running
system.

### Controllers

`config/controllers.yaml` runs the controller manager at **30 Hz** with three controllers:

| Controller | Purpose |
|---|---|
| `joint_state_broadcaster` | Publishes `/joint_states` from the hardware's state interfaces |
| `ackermann_controller` (`ackermann_steering_controller`) | Converts a `Twist` into rear-wheel velocities and front steering angles |
| `imu_broadcaster` (`imu_sensor_broadcaster`) | Publishes the IMU as `sensor_msgs/Imu` with static covariances |

The vehicle geometry lives here:

| Parameter | Value |
|---|---|
| `wheelbase` | 0.23529 m |
| `rear_wheel_track` | 0.18179 m |
| `front_wheel_track` | 0.12 m |
| Wheel radius (front and rear) | 0.0325 m |
| `max_steering_angle` | 0.286 rad |

**Front and rear track differ**, and that is not incidental. Humble's
`steering_controllers_library` shares a single track width between the traction-axle
differential-speed formula and the steering-axle angle-splitting formula, which is only
correct when the two are equal. `meta-vpace` carries a pair of backported patches that
split them. Without those patches every steering angle on this vehicle is computed from the
wrong track.

`max_steering_angle` is a deliberately conservative symmetric clamp. The mechanism's real
travel is asymmetric (roughly +0.242 rad left, −0.304 rad right); a symmetric limit keeps
Nav2's model consistent with what the vehicle will actually do.

`reference_timeout` is set to 0.4 s and the reasoning is recorded in the file: measured
teleop publishing showed gaps up to 0.215 s during continuous key repeat, larger than the
previous 0.1 s timeout, so the controller was zeroing the command on almost every repeat
cycle — which presented as jitter. The value must also stay *below* `twist_mux`'s 0.5 s
per-source timeout, because `twist_mux` does not publish a zero `Twist` when a source times
out; it simply stops relaying. That makes `reference_timeout` the only real deadman that
halts the wheels when input stops.

`enable_odom_tf` is false. The `odom → base_footprint` transform is owned by
`robot_localization`'s EKF (`config/ekf.yaml`), and exactly one publisher of a transform is
the rule.

### Command arbitration

`twist_mux` selects between velocity sources by priority:

| Source | Topic | Priority | Timeout |
|---|---|---|---|
| Teleop | `cmd_vel_teleop` | 150 | 0.5 s |
| Recovery | `cmd_vel_recovery` | 100 | 0.5 s |
| Navigation | `cmd_vel_nav_smoothed` | 10 | 0.5 s |

Teleop sits above recovery deliberately. Recovery behaviours run precisely when navigation
has already failed, which is when a human most wants control; with recovery on top, operator
input was ignored until the behaviour finished.

There is a caveat recorded in the file that anyone adding a joystick node needs to read:
because teleop is now the highest priority, a node that publishes *continuously* — including
zero `Twist` messages when idle — would permanently lock out both recovery and navigation.
The 0.5 s timeout releases only once publishing stops, so a teleop source must publish only
on real input or behind a deadman.

A separate lock channel, `config/twist_mux_locks.yaml`, defines `emergency_stop` on topic
`emergency_lock` at priority 255 with no timeout. **This is the mechanism the OTA path uses
to immobilise the vehicle during an update** (§5).

### Navigation

Nav2 is configured for a curated subset rather than the full metapackage: AMCL localisation,
the behaviour tree navigator, the MPPI controller, Smac planners, costmap 2D with the
spatio-temporal voxel layer, a smoother, a velocity smoother and waypoint following. RViz,
teleop and `slam_toolbox` are deliberately excluded from the vehicle image — mapping is done
off-vehicle.

`pointcloud_to_laserscan` bridges the 3D LiDAR into Nav2's 2D costmap machinery by
flattening the cloud into a synthetic `LaserScan`.

Pre-built maps for several environments ship in `maps/`.

### Support nodes

`emergency_stop_server.cpp` is the vehicle's stop authority: it offers
`/emergency_stop/lock` and `/emergency_stop/unlock` as `std_srvs/Trigger` services, publishes
the `Bool` on `emergency_lock` that the mux locks on, and cancels any active
`navigate_to_pose` goal when engaged. The OTA path is its main caller (§5).

`twist_stamper.py`, `imu_scale.py` and `camera_compressor.py` are smaller helpers;
`road_classification_node.cpp` is present with its own configuration.

---

# 3. The LiDAR perception pipeline

This is where most of the machine-learning work in the project lives: a 3D object detector
trained on the vehicle's own sensor data, compiled to a GPU-specific inference engine, and
paired with a classical tracker — running on the vehicle at better than three times sensor
rate.

```
Livox sensor → livox_ros_driver2 → /livox/lidar (PointCloud2)
             → cuda_pointpillars_node   [CUDA preprocessing → TensorRT → decode → AB3DMOT]
             → /object_detections_3d (object_detection_msgs/Object3dArray)
             → IVI head unit
```

Detection and tracking run in **one process**. The earlier design had a separate tracker
node; merging them removes a serialisation hop and, more importantly, removes any
possibility of detector and tracker disagreeing about message definitions. The node
publishes detections with `track_id` already populated.

## 3.1 Why a neural network, and which one

A LiDAR frame is an unordered set of roughly 120,000 3D points with no colour and no fixed
structure. The task is to find objects in it and say what they are. Classical approaches —
ground-plane removal followed by Euclidean clustering — find *that something is there* but
not *what*, and they fail on partially occluded and closely spaced objects, which is most of
a street scene.

The constraint that decided the architecture is the target: a Jetson Orin NX sharing its GPU
with a camera detector and a 3D-rendering head unit. That rules out anything that treats the
cloud as a dense 3D volume; 3D convolutions over a voxel grid are accurate and far too
expensive here.

**PointPillars** is the compromise that fits. Its insight is that a LiDAR scene is
effectively 2.5-dimensional — objects sit on a ground plane and do not stack — so the
vertical axis can be collapsed and the expensive part of the network can be an ordinary 2D
convolution, which GPUs and TensorRT are extremely good at.

## 3.2 How PointPillars works

Four stages, and knowing them makes the configuration parameters legible:

**1. Pillarisation.** The horizontal plane is divided into a grid of cells — 0.16 × 0.16 m
here. All points falling in one cell form a *pillar*, unbounded in z. Pillars are capped at
32 points and the frame at 10,000 non-empty pillars, so the network sees a fixed-size tensor
regardless of scene density.

**2. Feature encoding.** Each point is augmented from 4 raw values (x, y, z, intensity) to 9:
the offsets from its pillar's point-mean and the offsets from the pillar's geometric centre
are appended. A small learned network reduces each pillar to a single feature vector. This
runs as a hand-written CUDA kernel here, not in the network.

**3. Scatter to pseudo-image.** Pillar features are scattered back to their grid positions,
producing what is effectively a multi-channel image. From here on, the problem is 2D. This
step is a custom TensorRT plugin registered by the node itself.

**4. 2D backbone and detection head.** A convolutional backbone extracts features and an
anchor-based head predicts, per grid cell and anchor, a class score (`cls_preds`), a box
regression (`box_preds`) and a direction classification (`dir_cls_preds`). Non-maximum
suppression reduces overlapping predictions to final oriented 3D boxes.

The configuration exposes every one of these: `voxel_size`, `max_points_per_voxel`,
`max_voxels`, `point_cloud_range`, `anchor_sizes`, `anchor_bottom_heights`,
`nms_iou_threshold`, `nms_pre`, `max_num`.

## 3.3 Training the model for this vehicle

The pipeline was first brought up on a **KITTI** model — the standard automotive LiDAR
benchmark, three classes (Pedestrian, Cyclist, Car), a 64-beam roof-mounted scanner. That
gave a known-good reference to validate the deployment against, but it is not the vehicle's
sensor: a Livox Mid-360 at 0.30 m mounting height sees a different world.

Training the replacement is documented in `docs/TRAINING_HANDOFF.md`, and three changes
define it — each one a decision with a measured consequence.

### Quartering the detection grid

```
POINT_CLOUD_RANGE:  ±40.96 m  →  ±20.48 m
```

| | grid | head | cells |
|---|---|---|---|
| ±40.96 m | 512×512 | 256×256 | 262,144 |
| **±20.48 m** | **256×256** | **128×128** | **65,536 — 4× fewer** |

This is the single largest inference-latency lever on the Orin. The vehicle is slow and
indoor-scale; 20 m of range is ample.

### A dataset finding that overturned the design

The original plan was **forward-only** range — `[0, -20.48, …, 40.96, 20.48, …]` — on the
reasoning that nothing behind the vehicle needs detecting. That reasoning did not survive
contact with the labelled data. Measured against 356 hand-drawn boxes from one run:

| Range | Boxes kept | Car | Pedestrian |
|---|---|---|---|
| Forward-only | 171/356 | 171/305 | **0/51** |
| **Symmetric ±20.48** | **356/356** | 305/305 | **51/51** |

**Every pedestrian in the labelled set was behind the sensor** (x ∈ [−2.69, −0.63]). The
robot is slow, so people walk up behind it. Forward-only would have left the Pedestrian class
with zero training examples — while looking entirely reasonable in the configuration file.
Both ranges produce the same 256×256 grid and 128×128 head, so the ONNX shapes, the engine
and the latency budget are identical. There was never a cost to keeping the rear.

### Two classes, and anchors measured rather than inherited

`CLASS_NAMES` becomes `['Pedestrian', 'Car']` — Cyclist dropped. That changes the head's
output *channels*, not just the class count: 6 anchors × {3, 7, 2} → 4 anchors × {2, 7, 2},
so `cls/box/dir` go from 18/42/12 to 8/28/8, and the export tooling must be patched for both
the spatial dimensions and the channels.

Anchor priors were originally inherited from KITTI. Medians over the labelled run show what
that cost:

| Class | n | Measured l×w×h | Measured bottom z | Inherited KITTI value |
|---|---|---|---|---|
| Car | 305 | 4.34 × 1.87 × 1.50 | **−0.355** | 3.9 × 1.6 × 1.56 @ −0.19 |
| Pedestrian | 51 | 0.60 × 0.63 × 1.70 | **−0.197** | 0.8 × 0.6 × 1.73 @ −0.10 |

The KITTI bottoms sat roughly 0.15 m too high for a 0.30 m sensor mount — an anchor prior
systematically offset from the data it is meant to match.

### The contract between training and runtime

The runtime decoder mirrors the configuration the model was trained and exported with: class
order, anchor geometry, voxel grid, pillar capacity. **When they disagree, nothing crashes —
the detector emits confident, plausible-looking boxes that are noise.**

That failure mode is why these values are treated as a contract rather than as preferences,
and why the node now cross-checks the loaded engine against its compiled parameters at
startup and refuses to run on a mismatch.

## 3.4 From PyTorch to TensorRT

The training framework is PyTorch; the vehicle runs TensorRT. The path between them is ONNX,
and it is where most of the engineering effort went.

**TensorRT** is a compiler, not an interpreter. Given an ONNX network it produces an
*engine*: a plan with selected kernel implementations, fused layers and chosen precisions,
tuned by actually timing candidate kernels on the device. That has consequences:

- **Engines cannot be cross-compiled.** Tuning is a measurement, so it must happen on the
  target. An x86 build host cannot produce an Orin engine.
- **Engines are bound to a tuple** of (model, TensorRT version, GPU architecture, precision).
  The cache filename encodes all four: `<model>.onnx.<hash>.trt10.3.sm87.fp16.engine`.
- **Therefore only the `.onnx` ships**, and each device builds its own engine once.

Measured on Orin NX:

| | Time |
|---|---|
| Cold build from `.onnx` | **215 s** |
| Cached load | **< 1 s** |
| Engine size | 9.93 MB, FP16, sm87, TRT 10.3 |

**Precision.** The Orin path uses **FP16**, gated on `__aarch64__`; x86 workstations stay
FP32. Half precision roughly doubles arithmetic throughput on Orin's tensor cores and halves
weight bandwidth, and the validation below shows the accuracy cost is not observable at this
task.

One export detail matters enough to be listed as a training requirement: `max_voxels` in the
ONNX export must match the training configuration's test-time value. A mismatch truncates the
scene, and truncation follows grid order — which surfaced during bring-up as detections
appearing only on one side of the vehicle. NVIDIA's reference model accepts 10,000 voxels
while the scene needed 19,647, so roughly half of every frame was being discarded before the
network saw it.

## 3.5 Proving the port is faithful

Reimplementing a network's preprocessing in CUDA and its inference in TensorRT invites
silent divergence: nothing errors, the output is simply subtly wrong. The port was therefore
validated at two levels.

**Tensor level** — raw network outputs for one real cloud (19,663 pillars), TensorRT pipeline
versus the PyTorch reference, same cloud, same weights:

| Tensor | Correlation | Mean abs diff |
|---|---|---|
| `cls_preds` | **0.999911** | 0.0199 |
| `box_preds` | **0.999647** | 0.0024 |
| `dir_cls_preds` | **0.998964** | 0.0219 |

`argmax` of `cls_preds` lands on the *same cell* in both. These are FP32 kernel-ordering
differences, not behavioural ones — which establishes that the CUDA voxelisation, the nine
pillar features and the engine faithfully reproduce the trained model.

**That was necessary but not sufficient.** The network was right while the *decode* still had
four defects: missing `nms_pre`/`max_num` caps, argmax-only class emission, an
endpoint-to-endpoint anchor grid instead of bin centres, and a centre-versus-bottom z origin.
Each produced plausible output. End-to-end comparison against the original PyTorch node
after fixing them:

| Metric | Value |
|---|---|
| Original detections reproduced within 0.5 m | **94.0%** (1327/1411) |
| Median 3D centre error | **0.0001 m** |
| 90th percentile centre error | 0.0006 m |
| Class agreement on matches | **99.8%** |
| Distinct track ids over 33 frames | 383 → 392 |

A sub-millimetre median centre error is the signal that this is the same model, not a
reimplementation that happens to behave similarly.

The same investigation produced a genuinely useful negative result about the model itself.
On the KITTI bag the PyTorch reference's maximum sigmoid score is 0.49 with nothing above
0.5 — so "everything is classified Pedestrian" is **the model's real behaviour on that data**,
not a decode bug. The score threshold of 0.1 was admitting noise (464 cells above 0.1 against
13 above 0.3); raising it to 0.35 yields 5–13 objects per frame at scores 0.35–0.67.

## 3.6 Measured performance

| | Workstation (RTX 3050, FP32) | **Jetson Orin NX (FP16)** |
|---|---|---|
| Callback, median | 20.80 ms | **31.20 ms** |
| Throughput from median | 48.1 Hz | **32.1 Hz** |
| Sustained at 30 Hz demand | — | **28.87 Hz** |
| Headroom over the 10 Hz sensor | 4.8× | **3.2×** |

Against the original PyTorch pipeline on the same workstation, same bag, same timing method:
**135.9 ms → 20.8 ms per callback, 7.4 Hz → 48.1 Hz, a 6.5× speedup.**

Where the time goes (workstation, per-stage build):

| Stage | ms |
|---|---|
| Repack 124k points (CPU) | 0.30 |
| `generateVoxels` (CUDA) | 0.11 |
| `generateFeatures` (CUDA) | 0.21 |
| **`doinfer` (TensorRT)** | **19.22 — 92% of the frame** |
| Decode + NMS + track + publish | ~0.9 |

**The pipeline is GPU-bound**, which is the useful conclusion: no amount of host-side tuning
matters until the engine itself gets faster. FP16 and the quartered grid are the levers;
CPU optimisation is not.

On Orin at three times sensor demand the per-callback cost is unchanged from the 1× run,
confirming a genuinely saturating workload rather than a median hiding a backlog. Steady-state
footprint is **1.76 GB RSS** (Orin's unified memory puts CUDA and TensorRT allocations inside
process RSS) at **~4% of one CPU core** — the host does orchestration only.

## 3.7 Tracking — a deliberately classical choice

Detection is per-frame and stateless: it says what is present now, with no notion that the car
in this frame is the car from the last one. Tracking supplies identity over time, which is
what makes velocity, intent and "the same object" expressible at all.

**AB3DMOT** is the algorithm, and it is emphatically *not* a neural network. It combines two
textbook components:

| Component | File | Role |
|---|---|---|
| Constant-velocity Kalman filter | `kalman.hpp` | Predicts each track forward one frame; fuses the prediction with a matched detection |
| Hungarian assignment | `assignment.hpp` | Optimal one-to-one matching of detections to predicted tracks |
| 3D IoU / distance metrics | `metrics.hpp` | The association cost |
| Track lifecycle | `track3d.hpp` | Age, hit/miss counts, birth and death thresholds |

Per frame: predict every track, associate detections to predictions by 3D IoU, update matched
tracks, age out unmatched tracks, birth tracks for unmatched detections.

Choosing a classical tracker over a learned one is the right trade here. It requires no
training data and no second model on the GPU, it costs well under a millisecond per frame
against the detector's 31 ms, and its failure modes are interpretable — a track that breaks
can be traced to an association cost. Its accuracy is bounded by the detector's, which on a
GPU-bound pipeline is where the budget should go.

Validated behaviour: at threshold 0.35 over 40 frames, 87 track ids of which **74 lived four
frames or more**. One track was followed 12.49 m in a straight line at constant lateral
position under a single stable id — physically coherent motion through the whole C++ chain.

This is also a C++ port of a Python original, and the port deliberately drops PCL, OpenCV and
the NumPy-shaped helpers the original leaned on; Eigen is the only third-party dependency.

**A licence discrepancy is unresolved and documented rather than hidden.** `package.xml`
declares MIT; the `LICENSE` file in the package is the AB3DMOT agreement restricting use to
academic and non-profit research. `meta-vpace` declares the restrictive one so the image
manifest is accurate, and records the three ways to settle it.

## 3.8 The output contract

`object_detection_msgs` defines what the rest of the vehicle sees:

```
Object3dArray:  Header header, Object3d[] objects
Object3d:       uint8 label            (PEDESTRIAN=0, CYCLIST=1, CAR=2)
                float32 confidence_score
                int32 track_id         (-1 if untracked)
                BoundingBox3d bounding_box
BoundingBox3d:  geometry_msgs/Point[8] corners
```

The box is eight explicit corners rather than centre-plus-extent-plus-yaw. That makes the
consumer trivial — a renderer draws twelve edges with no trigonometry and no orientation
convention to get wrong — at the cost of a larger message. At the observed traffic (2.5–2.8 KB
per message, ~10 Hz) that cost is not significant.

`track_id` being carried in the detection message, rather than published separately, is the
visible consequence of the merged detector/tracker design.

The messages live in the same repository as the detector and tracker that produce them,
precisely so the two are versioned together.

## 3.9 Sensor driver, profiles and visualisation

**`livox_ros_driver2`** is the vendor driver, publishing `sensor_msgs/PointCloud2` on
`/livox/lidar`. The sensor is on Ethernet, configured by `MID360s_config.json` in
`ackermann_bringup/config/`. The upstream repository supports ROS 1 and ROS 2 from one tree
and expects a build script to select between them; `meta-vpace` reproduces that selection
inside the recipe's unpack step.

**`lidar_perception_bringup`** carries `perception.launch.py` and two parameter profiles,
which is what lets one build serve two sensors without a rebuild:

| | `livox.yaml` | `kitti.yaml` |
|---|---|---|
| Input topic | `/livox/lidar` | `/kitti/velo` |
| Classes | Pedestrian, Car | Pedestrian, Cyclist, Car |
| Point cloud range | ±20.48 m, −0.5…3.0 m | 0…69.12 m, ±39.68 m, −3.0…1.0 m |
| `intensity_scale` | 255.0 | 1.0 |

`intensity_scale` is why the profiles cannot be merged: KITTI intensity is already normalised
to [0, 1], while Livox and RoboSense drivers publish [0, 255]. Dividing twice degrades the
input silently. `class_remap` translates the model's own output ordering into the message
enum — a KITTI-ordered model such as NVIDIA's reference needs `[2, 0, 1]` where this one
needs the identity.

**`object_visualization`** converts `Object3dArray` into RViz `MarkerArray` with its own
confidence threshold (0.35). It is disabled on the vehicle — the IVI head unit consumes
`/object_detections_3d` directly — and enabled when debugging over the network.

# 4. The camera sign-detection pipeline

A composable-node pipeline that detects traffic signs and publishes the recognised class
onto the vehicle CAN bus.

```
/dev/camera-front → v4l2_camera ──(zero-copy)──▶ yolos_detector
                                                 ├─▶ /yolos_detector/detections
                                                 └─▶ /yolos_detector/debug_image
                                                        │
                                                 yolo_class_can ──▶ CAN 0x215
```

All nodes run in one `component_container`, so frames pass as shared pointers.

## 4.1 The model and how it was trained

Where the LiDAR detector answers "what solid objects are around the vehicle", this network
answers a different question — "what does the road furniture *say*" — so it is a 2D image
classifier-detector rather than a 3D one.

**YOLO** ("You Only Look Once") is a single-shot detector: one forward pass over the whole
image predicts every box and class simultaneously, rather than proposing regions and then
classifying them. That is what makes it fast enough to run alongside a 3D detector on a
shared GPU.

| Property | Value |
|---|---|
| Architecture | YOLO26n (Ultralytics) — the *nano* variant |
| Parameters | 2,377,761 |
| Input size | 640×640 |
| Classes | 15 |
| Export | ONNX, opset 12, static shapes |

The **nano** variant is the deliberate choice: 2.4 M parameters against tens of millions for
the larger YOLO sizes. Traffic signs are high-contrast, rigid, and drawn from a small closed
set — a task where a small network is not a compromise, and the saved GPU time belongs to the
LiDAR detector.

### Classes

Fifteen, covering two decision types the vehicle acts on:

- **Traffic lights** — Red Light, Green Light
- **Regulatory signs** — Stop, and Speed Limit 10 through 120 in steps of 10

Speed limits are separate classes rather than one class plus digit recognition. For a closed
set of twelve known plates that is simpler, and it avoids an OCR stage that would be its own
failure mode.

### Training

Transfer learning from Ultralytics' pretrained `yolo26n.pt` on a public Roboflow
self-driving-car dataset, using AdamW at 640×640, batch 16, with early stopping (patience 5)
and a second continuation run from the first run's weights — a standard two-phase schedule
where the second phase refines at a lower effective learning rate. The notebooks and script
are in `training/`.

Starting from pretrained weights rather than from scratch is what makes a dataset of this
size viable: the backbone already knows edges, shapes and textures, and only the head has to
learn what a speed-limit plate looks like.

### Validation

| Metric | Value |
|---|---|
| mAP@50, all classes | **0.958** |
| mAP@50-95, all classes | **0.829** |

Per-class figures are in the package README and they show a consistent, explainable pattern:
speed-limit plates and Stop score 0.96–0.99 at mAP@50, while **Green Light (0.883) and Red
Light (0.847) are the weakest classes** — and their mAP@50-95 falls to around 0.50 against
0.86–0.93 for the signs. Traffic lights are small, self-illuminated, and their bounding box
is far less crisply defined than a sign plate's, which is exactly what a metric averaged over
tighter IoU thresholds punishes.

### Inference cost

Measured on an RTX 3050: preprocess 0.2 ms, inference **3.1 ms**, postprocess 0.1 ms —
about 3.4 ms end to end. Against a pipeline throttled to 2 Hz (§4.3), inference is not the
constraint; the frame rate is set deliberately, not by the model.

`models/` holds `best.onnx`, `classes.names`, and a `best.engine`. The engine is a TensorRT
plan built on one specific device — like the LiDAR detector's (§3.4) it is not portable and
is rebuilt on the target.

## 4.2 Inference backend

`ros2_yolos_cpp_trt` is a ROS 2 wrapper around the YOLOs-CPP inference library, providing
composable, lifecycle-managed nodes that publish `vision_msgs/Detection2DArray`. It supports
the YOLO family generally; the shipped configuration uses detection only.

It has an ONNX Runtime sibling (`ros2_yolos_cpp`). The vehicle configuration selects
TensorRT, which is also the reason the package is licensed AGPL-3.0 in the image manifest —
the wrapper carries that licence.

`detector_params_trt.yaml` is minimal: `dla_core: -1` selects the GPU engine rather than a
Deep Learning Accelerator core. The Orin has DLA cores; using them would offload inference
from the GPU, which matters when the GPU is also running the LiDAR detector.

## 4.3 Camera configuration and pipeline optimization

The pipeline was configured for 2 Hz but ran at 30 Hz: every setting meant to throttle it
was silently ineffective. The full investigation is in
`meta-vpace/docs/deep-dives/camera-pipeline-optimization.md`. The settings that resulted:

| Setting | Value | Reason |
|---|---|---|
| `video_device` | `/dev/camera-front` | `/dev/videoN` moves across replugs; a udev rule provides the stable name |
| `image_size` | `[640, 480]` | 640×640 is not an advertised mode — V4L2 silently substituted 640×480 and logged success |
| `output_encoding` | `yuv422_yuy2` | Publishing the sensor's native format removes one of two colour conversions per frame |
| `time_per_frame` | `[1, 5]` | 5 fps is the slowest interval this camera advertises; `[1, 2]` was never reachable |

`time_per_frame` **requires a patch** carried by `meta-vpace`: no upstream release of
`v4l2_camera` calls `VIDIOC_S_PARM` at all, and rclcpp keeps unknown YAML keys as parameter
overrides and discards them without warning — so on stock the line is silently ignored.

Together these changes cut the camera's USB traffic from 18.4 MB/s to 3.07 MB/s, removed
one of two colour conversions per frame, and brought TensorRT inference down from 30 Hz to
the intended 2 Hz — a fifteenfold reduction in GPU work for the same detection output.

## 4.4 CAN output

`yolo_class_node.cpp` and `can_comms.cpp` publish detections to the vehicle bus:

| Parameter | Value |
|---|---|
| `can_interface` | `can0` |
| `detections_topic` | `/yolos_detector/detections` |
| `conf_threshold` | 0.7 |
| `can_id` | 533 (`0x215`) |

The confidence threshold here is higher than the detector's own, so only confident
detections reach the vehicle.

This node is the reason the deployed systemd unit requires `can0` to be *up* rather than
merely present: the node throws in its constructor if the interface cannot be opened, and
because it is composable, that failure is logged while the container stays up and detections
keep flowing on the ROS topic — never reaching the bus.

`launch/yolo_auto_launch.py` assembles the container and drives the lifecycle transitions to
`active`.

---

# 5. The update coordinator

`update_coordinator` is a Python ROS 2 node that arbitrates firmware updates across the
vehicle's ECUs. It is the only node in this workspace that is enabled at boot on the vehicle.

## 5.1 What it does

Two other ECUs can request permission to update themselves, and the Jetson's own OTA agent
registers its updates through the same node. While any update is active, the coordinator
holds the vehicle still — but it does so indirectly, through a chain worth following:

```
update_coordinator  ──Trigger──▶  /emergency_stop/lock
                                        │
                            emergency_stop_server
                                        ├──▶ emergency_lock  (std_msgs/Bool) ──▶ twist_mux
                                        └──▶ /navigate_to_pose/_action/cancel_goal
```

The coordinator maintains a set of ECUs with an update in progress and calls
`/emergency_stop/lock` when that set becomes non-empty, `/emergency_stop/unlock` when it
empties again. `emergency_stop_server` (in `ackermann_bringup`) owns the actual lock: it
publishes the `Bool` on `emergency_lock` that `twist_mux` watches at priority 255 (§2.3),
and it also cancels any in-flight Nav2 goal so the navigator does not resume on release.

Services exposed for the Jetson's own updates:

| Service | Type | Effect |
|---|---|---|
| `/update_coordinator/self_start` | `std_srvs/Trigger` | Adds `jetson` to the active set; locks the vehicle |
| `/update_coordinator/self_done` | `std_srvs/Trigger` | Removes it; releases the lock when the set empties |

## 5.2 CAN protocol

| CAN ID | Direction | Meaning |
|---|---|---|
| `0x300` | cluster → Jetson | Update request / running notification |
| `0x301` | Jetson → cluster | Approve / deny verdict |
| `0x310` | ESP32 → Jetson | Update request / running notification |
| `0x311` | Jetson → ESP32 | Approve / deny verdict |

Requests carry magic `0xA5`, running notifications `0x5A`; verdicts are 1 for approve and 0
for deny.

## 5.3 SecOC

Every frame in both directions is authenticated with **SecOC**: a truncated AES-128-CMAC
over the payload concatenated with a freshness counter. A receiver accepts a frame only if
the MAC verifies **and** the freshness value is strictly greater than the last it accepted —
which is what makes a captured frame useless to replay.

Counters are kept per data identifier:

| DID | Meaning | Direction |
|---|---|---|
| 1 | Cluster REQUEST | inbound |
| 2 | Cluster RUNNING | inbound |
| 3 | Cluster APPROVE | outbound |
| 4 | ESP32 REQUEST | inbound |
| 5 | ESP32 RUNNING | inbound |
| 6 | ESP32 APPROVE | outbound |

`secoc_utils.py` holds the key loading, the freshness store, and the verify/build helpers.
Counters must survive a reflash — the peers keep counting across the Jetson's updates — so
the deployed configuration roots the store on the persistent partition rather than the
default `/var/lib` path in the package config.

## 5.4 Driver approval

A verified request is no longer approved automatically. It is put in front of the driver
first, through a file spool the IVI head unit watches — files rather than a topic, because
the head unit is a separate image that does not run ROS. Approving locks the vehicle;
denying puts a 0 on the bus and the car keeps driving. Both ECUs already fail closed on a
deny, so no firmware change was needed on either side.

**The timing asymmetry between the two requesters is the constraint the design is built
around.** The cluster's bridge waits 60 s for a verdict. The ESP32 waits **10 s, once** — it
asks at the moment it is about to act, immediately before rebooting into new firmware or
dropping another MCU into its bootloader, and if no verdict arrives it abandons the update
outright. There is no retry, which makes that 10 s wall the number every other timeout in
the node is sized against.

Configuration (`config/update_coordinator.yaml`) exposes `require_approval`, `approval_dir`,
`ui_alive_max_age_s` and `on_no_verdict`. The default for the last is `approve`, keeping it
consistent with the head unit's own auto-accept — and the file states the consequence
plainly: approval is advisory, and anyone who can stop the IVI application gets automatic
updates back.

---

# 6. Interfaces

## 6.1 Principal topics

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Livox driver → detector, `pointcloud_to_laserscan` |
| `/object_detections_3d` | `object_detection_msgs/Object3dArray` | Detector → IVI head unit, visualiser |
| `/yolos_detector/detections` | `vision_msgs/Detection2DArray` | YOLO detector → CAN publisher |
| `/yolos_detector/debug_image` | `sensor_msgs/Image` | YOLO detector → RViz (debug) |
| `/image_raw` | `sensor_msgs/Image` | `v4l2_camera` → YOLO detector (intra-process) |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` → `robot_state_publisher` |
| `/cmd_vel_teleop`, `/cmd_vel_recovery`, `/cmd_vel_nav_smoothed` | `geometry_msgs/Twist` | Sources → `twist_mux` |
| `emergency_lock` | `std_msgs/Bool` | `emergency_stop_server` → `twist_mux` |
| `/odom` | `nav_msgs/Odometry` | Controller and EKF |

## 6.2 CAN identifiers originated by this workspace

| ID | Direction | Package |
|---|---|---|
| `0x215` (533) | Jetson → bus | `camera_sign_detect_bringup` — detected sign class |
| `0x300` / `0x301` | cluster ↔ Jetson | `update_coordinator` |
| `0x310` / `0x311` | ESP32 ↔ Jetson | `update_coordinator` |

Vehicle actuator and sensor traffic is defined by `v_pace_db.c` in `ackermann_hardware`.

Other identifiers on the bus — node liveness, WiFi credential provisioning — originate from
services in `meta-vpace` rather than from this workspace.

---

# 7. Recurring design themes

Four patterns show up across otherwise unrelated packages, and they are worth stating
because they explain a lot of the configuration.

**Failures on this platform are usually silent.** An unknown ROS parameter is discarded
without warning. An unadvertised V4L2 mode is silently substituted. A composable node that
throws leaves its container running. A detector fed the wrong geometry emits confident
nonsense. `compressed_image_transport` turns a non-colour encoding into grayscale with no
log line. None of these produce an error, which is why so many configuration files here
carry long comments — the comment is the only thing that makes the failure findable.

**Measurement precedes configuration.** The controller's `reference_timeout`, the camera's
frame rate, the twist priorities and the detector thresholds were each set from an observed
number, and the number is recorded next to the value.

**Contracts are versioned with their producers.** The 3D message definitions live in the
same repository as the detector and tracker that emit them, and the Yocto recipes for all
five packages share a single source revision, so a mismatch between a node and its own
message definitions cannot be built.

**A trained model is half a contract, and the runtime is the other half.** Class order,
anchor geometry, voxel grid and pillar capacity must agree between the training
configuration and the deployed decoder. They are not tuning parameters — disagreement
produces confident, plausible, wrong output rather than an error, which is why the detector
now cross-checks its engine against its compiled parameters at startup and refuses to run on
a mismatch (§3.3).

**Claims about a model are backed by measurement against a reference.** The TensorRT port was
not accepted because it looked right; it was accepted at 0.9999 tensor correlation against
the PyTorch original and 94% detection reproduction within 0.5 m end to end (§3.5).

**State that must outlive an update is kept off the root filesystem.** SecOC counters and the
TensorRT engine cache both live on the persistent partition, because the vehicle's A/B
update mechanism replaces the entire root filesystem.

---

# 8. Building

## 8.1 On a workstation

```bash
cd ~/ros2_ws_gp
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The perception packages need a CUDA architecture appropriate to the build machine —
`-DGPU_SMS=86` for an RTX 30-series workstation, `-DGPU_SMS=87` for Orin. The perception
repository can also be built on its own:

```bash
cd ~/ros2_ws_gp/src/ros2-lidar-perception
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DGPU_SMS=86
```

A packaged workstation ROS installation may lack `ros2 launch`, `ros2 run`, `ros2 bag` and
`ros2 topic`; `docs/RUNNING.md` in the perception repository lists what to install and
provides a demo script that needs none of them.

## 8.2 For the vehicle

The vehicle image does not build this workspace with colcon. `meta-vpace` has one BitBake
recipe per package, each pinning a source revision, and the packages are installed into
`/opt/ros/humble` in the image. Consequences:

- **Pinned revisions.** Changes here reach the vehicle only when the corresponding `SRCREV`
  is bumped in `meta-vpace`. The five perception packages share one revision.
- **Absolute paths in parameter files do not survive.** `kitti.yaml`'s `model_path` points
  into a developer's home directory; the deployed launch supplies `model_path:` explicitly.
- **Some packages need patches to build cross-compiled**, notably the `v4l2_camera`
  frame-rate support and the steering track-width split.

See `meta-vpace/docs/ARCHITECTURE.md` §8 for the packaging detail.

---

# 9. Related documents

In this workspace:

| Document | Contents |
|---|---|
| `src/camera_sign_detect_bringup/README.md` | Pipeline architecture, full per-class validation metrics, model details |
| `src/ros2-lidar-perception/README.md` | Repository origin and the IVI-facing contract |
| `src/ros2-lidar-perception/docs/RUNNING.md` | Running the pipeline, the KITTI demo script, expected output |
| `src/ros2-lidar-perception/docs/OPTIMIZATIONS.md` | Performance work on the detector |
| `src/ros2-lidar-perception/docs/Jetson_benchmark.md` | Measured performance on Orin |
| `src/ros2-lidar-perception/docs/BAG_VALIDATION_STATUS.md` | Validation against the PyTorch reference |
| `src/ros2-lidar-perception/docs/TRAINING_HANDOFF.md` | Training a replacement model |
| `src/ros2-lidar-perception/docs/COMPARISON.md` | Comparison against the earlier pipeline |
| `src/ros2_yolos_cpp_trt/README.md` | The inference wrapper's own documentation |
| `src/ros2_yolos_cpp_trt/JETSON_DEPLOYMENT.md` | TensorRT deployment notes for Jetson |

In `meta-vpace`:

| Document | Contents |
|---|---|
| `docs/ARCHITECTURE.md` | The full image: layer, distro, packaging, OTA, security, build and flash |
| `docs/deep-dives/camera-pipeline-optimization.md` | The camera throughput investigation |
