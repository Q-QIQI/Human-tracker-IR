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
│   └── __init__.py
├── depth_tracker.py              # 2.5D visual perception and filtering node
├── go2_follower.py               # FSM-driven kinematic control node
├── package.xml                   # ROS2 package manifest
├── setup.py                      # Build script
└── README.md
