<img width="732" height="280" alt="image" src="https://github.com/user-attachments/assets/346c6e19-922c-4dea-ac3e-23b7f6065419" /># diagnosis_pkg_acu_pkg

Diagnose the currently running subsystems in your vehicle and software stack using ROS 2.  
This package provides a simple, configurable diagnosis window that you can tailor to show only the components you care about (e.g., LiDAR, CAN, Camera, or your own vehicle-side publishers).

---

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [1) Install dependencies](#1-install-dependencies)
  - [2) Clone this package](#2-clone-this-package)
  - [3) Add platform configuration repo](#3-add-platform-configuration-repo)
  - [4) Verify directory layout](#4-verify-directory-layout)
- [Configuration](#configuration)
  - [Enable/disable subsystems](#enabledisable-subsystems)
  - [Customizing the config path](#customizing-the-config-path)
- [Build](#build)
- [Run](#run)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Prerequisites

- **ROS 2 Humble** (recommended)
- **colcon** build tools
- **diagnostic_msgs** package

---

## Installation

### 1) Install dependencies

Make sure your ROS environment is sourced, then install `diagnostic_msgs`:

```bash
source ~/.bashrc
sudo apt install ros-humble-diagnostic-msgs
<img width="732" height="280" alt="image" src="https://github.com/user-attachments/assets/62397ba2-67e9-41f8-a256-d4bde3f14495" />


### 2) Clone this package

Clone into your workspace’s src/ directory:
<img width="1003" height="280" alt="image" src="https://github.com/user-attachments/assets/51ce2efa-9f26-4fc3-99f0-82316977c96c" />

cd <your_workspace>/src
git clone https://github.com/himanshukumar-bullwork/diagnosis_pkg_acu_pkg.git
 <img width="295" height="250" alt="image" src="https://github.com/user-attachments/assets/4d46b346-bdfc-4028-9bff-004b21d09f13" />
After cloning, confirm the package location is correct. It should be inside your workspace and aligned with your project’s layout

### 3) Add platform configuration repo]

The diagnosis package and lifecycle nodes share a common platform configuration. Clone it into your core directory:
If you don't have core package 
mkdir -p <your_workspace>/<your_main_package>/core
cd <your_workspace>/<your_main_package>/core
git clone https://github.com/himanshukumar-bullwork/platform_configs.git
else in your core package
cd <your_workspace>/<your_main_package>/core
git clone https://github.com/himanshukumar-bullwork/platform_configs.git

<img width="1031" height="172" alt="image" src="https://github.com/user-attachments/assets/6b8a0088-ed36-4995-b9cc-82dc893232b1" />

You may place platform_configs elsewhere if you prefer—just make sure to update the path in the launch file
<img width="270" height="219" alt="image" src="https://github.com/user-attachments/assets/26918ce6-4f21-47bc-8c84-a24af5d4871e" />

### 4) Verify Directory Layout
Below is an ideal layout
<your_workspace>/
├─ src/
│  └─ diagnosis_pkg_acu_pkg/
│     ├─ launch/
│     ├─ src/
│     ├─ package.xml
│     └─ ...
├─ <your_main_package>/
│  └─ core/
│     └─ platform_configs/
│        ├─ platform.yaml
│        └─ ...
├─ build/
├─ install/
└─ log/

## Configuration

### 1) Enable/disable subsystems
Control which subsystems appear in the diagnosis window via the shared platform.yaml.
Set each component to true (enable) or false (disable) based on your needs.

diagnostics:
  lidar: true
  camera: true
  can: true
  gps: false
  imu: false
  # Add your own vehicle-side publishers here
  # my_custom_topic: true
<img width="1256" height="899" alt="image" src="https://github.com/user-attachments/assets/3e491110-7255-46d3-b5c8-a389d1fdfed0" />


### 2) Customizing the config path

If you moved platform_configs, update the path reference in the launch file:
File: diagnosis_pkg_acu_pkg/launch/diagnostic.launch.py
Look for the parameter or variable pointing to platform.yaml and change it to your new location, e.g.:
# Example only: adjust to match the actual code in diagnostic.launch.py
config_path = '/absolute/or/relative/path/to/platform_configs/platform.yaml'

## Build
From the workspace root:
cd <your_workspace>
colcon build --packages-select diagnosis_pkg_acu_pkg

<img width="771" height="216" alt="image" src="https://github.com/user-attachments/assets/8f50d77c-c387-4e3a-8053-63c7a1c32e82" />


If you prefer, you can also build the entire workspace without --packages-select.

Don’t forget to source after building:
source install/setup.bash

##RUN
Launch the diagnosis window:
ros2 launch diagnosis_pkg_acu_pkg diagnostic.launch.py
<img width="1256" height="41" alt="image" src="https://github.com/user-attachments/assets/e5aaa7a5-fd08-438f-ae34-3103da04695a" />

