# diagnosis_pkg_acu_pkg

ROS 2 (Humble) diagnostic dashboard for vehicle/robot subsystems. This package reads a shared platform configuration and launches a diagnosis window with per‑subsystem status.

> **Tested on:** Ubuntu 22.04, ROS 2 Humble

---

## Installation

### 1) Install dependencies
Make sure your ROS environment is sourced, then install `diagnostic_msgs`:

```bash
# Source your ROS setup (adjust the path if needed)
source /opt/ros/humble/setup.bash
# Also source your shell rc to keep environment consistent
source ~/.bashrc

# Install dependency
sudo apt install -y ros-humble-diagnostic-msgs
```

<details>
<summary>Screenshots</summary>

![apt-install](https://github.com/user-attachments/assets/62397ba2-67e9-41f8-a256-d4bde3f14495)

</details>

---

### 2) Clone this package
Clone into your workspace’s `src/` directory:

```bash
cd <your_workspace>/src
git clone https://github.com/himanshukumar-bullwork/diagnosis_pkg_acu_pkg.git
```

After cloning, confirm the package location is correct. It should be inside your workspace and aligned with your project’s layout.

<details>
<summary>Screenshots</summary>

![clone-1](https://github.com/user-attachments/assets/51ce2efa-9f26-4fc3-99f0-82316977c96c)

![clone-2](https://github.com/user-attachments/assets/4d46b346-bdfc-4028-9bff-004b21d09f13)

</details>

---

### 3) Add platform configuration repo
The diagnosis package and lifecycle nodes share a common platform configuration. Clone it into your core directory.

If you **don’t** have a `core` directory yet:

```bash
mkdir -p <your_workspace>/<your_main_package>/core
cd <your_workspace>/<your_main_package>/core
git clone https://github.com/himanshukumar-bullwork/platform_configs.git
```

If you **already** have a `core` directory:

```bash
cd <your_workspace>/<your_main_package>/core
git clone https://github.com/himanshukumar-bullwork/platform_configs.git
```

You may place `platform_configs` elsewhere if you prefer—just make sure to update the path in the launch file (see **Configuration → Customizing the config path** below).

<details>
<summary>Screenshots</summary>

![core-layout](https://github.com/user-attachments/assets/6b8a0088-ed36-4995-b9cc-82dc893232b1)

![config-path](https://github.com/user-attachments/assets/26918ce6-4f21-47bc-8c84-a24af5d4871e)

</details>

---

### 4) Verify directory layout
An ideal layout looks like this:

```
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
```

---

## Configuration

### 1) Enable/disable subsystems
Control which subsystems appear in the diagnosis window via the shared `platform.yaml`. Set each component to `true` (enable) or `false` (disable) based on your needs.

```yaml
diagnostics:
  lidar: true
  camera: true
  can: true
  gps: false
  imu: false
  # Add your own vehicle-side publishers here
  # my_custom_topic: true
```

### 2) Customizing the config path
If you moved `platform_configs`, update the path reference in the launch file:

- **File:** `diagnosis_pkg_acu_pkg/launch/diagnostic.launch.py`
- Look for the parameter or variable that points to `platform.yaml` and change it to your new location, for example:

```python
# Example only: adjust to match the actual code in diagnostic.launch.py
config_path = "/absolute/or/relative/path/to/platform_configs/platform.yaml"
```

---

## Build
From the workspace root:

```bash
cd <your_workspace>
# Option A: build just this package
colcon build --packages-select diagnosis_pkg_acu_pkg

# Option B: build the entire workspace
colcon build

# Don’t forget to source after building
source install/setup.bash
```

<details>
<summary>Screenshots</summary>

![build](https://github.com/user-attachments/assets/8f50d77c-c387-4e3a-8053-63c7a1c32e82)

</details>

---

## Run
Launch the diagnosis window:

```bash
ros2 launch diagnosis_pkg_acu_pkg diagnostic.launch.py
```

![run](https://github.com/user-attachments/assets/e5aaa7a5-fd08-438f-ae34-3103da04695a)

---

## Troubleshooting
- **Package not found / build errors**: Ensure the package lives under `<your_workspace>/src/` and you ran `colcon build` from the workspace root.
- **Command not found**: Make sure you sourced both `/opt/ros/humble/setup.bash` and your workspace `install/setup.bash`.
- **Config not applied**: Double‑check the `config_path` in `diagnostic.launch.py` if you moved `platform_configs/`.

---

## License
Add your license here (e.g., Apache-2.0, MIT).
