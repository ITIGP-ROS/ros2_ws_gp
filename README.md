# ros2_ws_gp

The ROS 2 Humble workspace for the **Jetson Orin node** of V-PACE, an ITI graduation project.
V-PACE is a four-node system; this workspace holds the perception, control and update
software that runs on the Orin.

The vehicle image is built by the Yocto layer **meta-vpace**, which packages a subset of
these packages from pinned revisions — changes here reach the vehicle only when that layer's
`SRCREV` is bumped.

## What is here

| Package | Role |
|---|---|
| `ackermann_description` | URDF/xacro vehicle model |
| `ackermann_hardware` | `ros2_control` hardware interface over CAN |
| `ackermann_bringup` | Launch files, controller, Nav2 and `twist_mux` configuration |
| `ros2-lidar-perception` | TensorRT PointPillars 3D detector + AB3DMOT tracking |
| `camera_sign_detect_bringup` | YOLO traffic-sign detection, publishing to CAN |
| `ros2_yolos_cpp_trt` | TensorRT-backed YOLO inference nodes |
| `livox_ros_driver2` | Livox LiDAR driver |
| `update_coordinator` | SecOC-authenticated OTA coordination over CAN |

`ros2-lidar-object-detection`, `lidar_object_detect_bringup` and `CUDA-PointPillars-ROS2`
are earlier iterations, superseded by `ros2-lidar-perception` and not built into the image.

## Build

```bash
git clone --recurse-submodules <this repo> ~/ros2_ws_gp
cd ~/ros2_ws_gp
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DGPU_SMS=86
source install/setup.bash
```

Set `-DGPU_SMS` to your GPU's compute capability — `86` for an RTX 30-series workstation,
`87` for the Jetson Orin.

## Documentation

**[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — the technologies used and how the
subsystems fit together, including how the perception models were trained, compiled for the
target GPU, and validated.

Per-package notes live with their packages; `docs/ARCHITECTURE.md` §9 indexes them.
