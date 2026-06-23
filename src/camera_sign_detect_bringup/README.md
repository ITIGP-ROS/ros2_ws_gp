
# V-PACE AI — Real-Time Traffic Sign Detection with ROS 2 & YOLO

A high-performance, production-ready object detection pipeline for autonomous driving applications, built on ROS 2 Humble with zero-copy intra-process communication and C++ YOLO inference.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│              component_container (single process)           │
│  ┌─────────────┐    shared_ptr    ┌─────────────────────┐   │
│  │   Camera    │ ──zero-copy────► │   YOLO Detector     │   │
│  │  (v4l2)     │   (no serial)    │  (ros2_yolos_cpp)   │   │
│  └─────────────┘                  └─────────────────────┘   │
│                                          │                  │
│                                          ▼                  │
│                              /detections (Detection2DArray) │
│                              /debug_image (annotated)       │
└─────────────────────────────────────────────────────────────┘
```

---

## Why This Pipeline Is Efficient

| Design Choice | Benefit |
|-------------|---------|
| **Composable Nodes** | Camera + detector live in **one OS process** — no inter-process serialization overhead |
| **Intra-Process Communication (IPC)** | Image messages passed as `std::shared_ptr<const Image>` — **zero-copy**, no DDS serialization |
| **C++ YOLO Inference (ONNX Runtime)** | Raw speed via `ros2_yolos_cpp` — no Python GIL, no PyTorch overhead |
| **Lifecycle Management** | Clean configure → activate → deactivate → shutdown transitions |

---


## Key Components

| Package | Role |
|---------|------|
| `v4l2_camera` | V4L2 camera driver (composable node) |
| `ros2_yolos_cpp` | C++ YOLO inference engine with ONNX Runtime |
| `camera_sign_detect_bringup` | Custom launch file + params + model assets |

---

## Quick Start

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select camera_sign_detect_bringup
source install/setup.bash

# Launch (auto-activates lifecycle)
ros2 launch camera_sign_detect_bringup yolo_auto.launch.py

# View detections
ros2 topic echo /yolos_detector/detections

# View annotated image in RViz
# Topic: /yolos_detector/debug_image
```

---

## Project Structure

```
camera_sign_detect_bringup/
├── launch/
│   └── yolo_auto.launch.py      # Composable container + lifecycle auto-activate
├── params/
│   ├── camera_params.yaml       # V4L2 camera configuration
│   └── detector_params.yaml   # YOLO inference parameters
├── models/
│   ├── best.onnx               # Trained yolo26n model
│   └── classes.names           # Class labels (15 traffic sign classes)
├── CMakeLists.txt
└── package.xml
```

---

## Model Details

| Property | Value |
|----------|-------|
| Architecture | YOLO26n (Ultralytics) |
| Parameters | 2,377,761 |
| Input size | 640×640 |
| Classes | 15 (Green Light, Red Light, Speed Limit 10-120, Stop) |
| Export format | ONNX (opset 12, static shapes) ***required by ros2_yolos_cpp*** |
| Inference backend | ONNX Runtime 1.20.1 |

### Validation Metrics (mAP)

| Class | mAP@50 | mAP@50-95 |
|-------|--------|-----------|
| **All** | **0.958** | **0.829** |
| Green Light | 0.883 | 0.503 |
| Red Light | 0.847 | 0.489 |
| Speed Limit 100 | 0.989 | 0.908 |
| Speed Limit 110 | 0.919 | 0.880 |
| Speed Limit 120 | 0.980 | 0.929 |
| Speed Limit 20 | 0.985 | 0.857 |
| Speed Limit 30 | 0.983 | 0.917 |
| Speed Limit 40 | 0.977 | 0.862 |
| Speed Limit 50 | 0.959 | 0.858 |
| Speed Limit 60 | 0.973 | 0.892 |
| Speed Limit 70 | 0.993 | 0.911 |
| Speed Limit 80 | 0.974 | 0.863 |
| Speed Limit 90 | 0.958 | 0.810 |
| Stop | 0.993 | 0.922 |

### Inference Speed  (RTX 3050 6GB mobile GPU)

| Stage | Time |
|-------|------|
| Preprocess | 0.2 ms |
| Inference | **3.1 ms** |
| Postprocess | 0.1 ms |
| **Total** | **~3.4 ms** |

---