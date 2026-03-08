# Robust 2.5D Human Tracking & Following in Low-Light Environments

[![ROS2](https://img.shields.io/badge/ROS2-Supported-090909?logo=ros)](https://docs.ros.org/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Fine--tuned-blue?logo=yolo)](https://github.com/ultralytics/ultralytics)
[![Platform](https://img.shields.io/badge/Platform-Unitree_Go2-orange)](https://www.unitree.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## 📝 Overview

This repository contains the official source code and supplementary materials for our paper on **autonomous robotic following in illumination-degraded environments**. 

We developed a robust ROS2-based 2.5D visual tracking and control system deployed on the **Unitree Go2 quadruped robot**. By integrating **CLAHE image enhancement** with a fine-tuned **YOLOv8n** detector, and coupling it with an **FSM-driven kinematic controller**, the system achieves real-time (30 FPS), highly reliable target locking and smooth following, even under severe low-light and occlusion conditions.

## 📂 Repository Structure

```text
human_tracker/
├── launch/
│   └── main_depth_ir.launch.py   # Main ROS2 launch file to start all nodes
├── models/
│   ├── best.pt                   # Our fine-tuned YOLOv8n weights for low-light IR
│   ├── yolov8n.pt                # Baseline YOLOv8n weights
│   ├── depth_tracker.py          # 2.5D visual perception and filtering node
│   └── go2_follower.py           # FSM-driven kinematic control node
├── package.xml                   # ROS2 package manifest
├── setup.py                      # Build script
└── README.md
```

## ✨ Key Features

* **Low-Light Robustness**: Utilizes Infrared (IR) streams from the Intel RealSense D435i, enhanced via CLAHE, to maintain high detection accuracy in near-pitch-black environments.
* **2.5D Deep Filtering Pipeline**: Rejects background noise by extracting the 30th percentile depth value within the target bounding box and geometrically cross-checking it against real-world human scale constraints.
* **Intelligent Target Recovery (FSM)**: Incorporates a Last-Known-Position (LKP) cache. If the target is occluded or lost, the robot initiates an active directional search rotation based on the disappearance vector.
* **Adaptive Kinematic Control**: Features piecewise linear velocity mapping (v_x) and proportional yaw correction (ω_z), combined with a deceleration-during-cornering mechanism to guarantee smooth motion and prevent chassis rollover.

## ⚙️ Prerequisites

### Hardware
* **Robot**: Unitree Go2 Quadruped Robot (or equivalent ROS2-compatible mobile base)
* **Sensor**: Intel RealSense D435i Camera

### Software & Dependencies
* **OS**: Ubuntu 20.04 / 22.04
* **Middleware**: ROS2 (Foxy / Humble)
* **Python Packages**:
  ```bash
  pip install ultralytics opencv-python numpy pandas
  ```
* **ROS2 Packages**:
  ```bash
  sudo apt install ros-<your_ros2_distro>-realsense2-camera
  # Ensure the official Unitree ROS2 SDK is also configured in your workspace
  ```

## 🚀 Quick Start

**1. Clone and Build the Workspace**
```bash
# Create a ROS2 workspace if you don't have one
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/Q-QIQI/Human-tracker-IR.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select human_tracker --symlink-install
source install/setup.bash
```

**2. Launch the Hardware Nodes (Camera & Robot Base)**
*(Note: Skip the camera launch if it is already included in your main launch file.)*
```bash
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_depth:=true
```

**3. Launch the Complete Tracking System**
We provide a unified launch file to start both the perception (YOLO+Depth) and control (FSM+Kinematics) nodes simultaneously:
```bash
ros2 launch human_tracker main_depth_ir.launch.py
```

## 📊 Performance & Results

The system was extensively tested on a hybrid dataset comprising the SYSU public dataset and self-collected images across various manual exposure times (100 µs, 500 µs, 1000 µs, 2500 µs).

| Model | mAP@0.5 | Recall | Precision | F1-Score | Inference Speed |
| :--- | :---: | :---: | :---: | :---: | :---: |
| Baseline YOLOv8n | 16.5% | 16.0% | 36.1% | 0.22 | - |
| **Our Fusion Model** | **92.0%** | **86.0%** | **88.5%** | **0.87** | **~30 FPS** |

## 📜 Citation

If you find this project or the proposed filtering pipeline helpful for your research, please consider citing our paper:

```bibtex
@article{your_paper_id_2024,
  title={Your Paper Title Here},
  author={Your Name and Co-authors},
  journal={Name of Conference/Journal},
  year={2024}
}
```

## 📞 Contact

For any questions, issues, or collaboration inquiries, please open an issue in this repository or contact: **[Your Email Address]**
