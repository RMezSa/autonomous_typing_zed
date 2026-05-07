# Setup — Ubuntu 22.04 + ROS 2 Humble

This is a one-time setup guide for collaborators who already have **ROS 2 Humble** installed on Ubuntu 22.04. It covers project-specific dependencies, fetching the workspace, and building.

For the actual run-time bring-up after setup, see [`SIM_CHECKPOINT.md`](./SIM_CHECKPOINT.md).

---

## 1. System dependencies

```bash
# Gazebo Fortress + ROS-Gazebo bridge (the rover sim runs on Fortress, not Classic)
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  python3-colcon-common-extensions \
  python3-vcstool \
  build-essential \
  git
```

Python deps used by the vision/coordinator nodes:

```bash
pip3 install --user opencv-python opencv-contrib-python numpy
```

---

## 2. Workspace + clone

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repo
git clone https://github.com/RMezSa/autonomous_typing_zed.git

# Use vcstool to fetch the rover description sibling repo automatically
vcs import . < autonomous_typing_zed/repos.yaml
```

After this, `~/ros2_ws/src/` should contain both `autonomous_typing_zed/` and `mi_rover_description/`.

---

## 3. Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  typing_interfaces arm_ik zed_aruco mi_rover_description
source install/setup.bash
```

> **If `typing_interfaces` build fails** with `existing path cannot be removed: Is a directory`, clear the cache: `rm -rf build/typing_interfaces` and rebuild.

> **You only need to build once** unless you pull new code. After that, just `source install/setup.bash` in each new terminal.

---

## 4. Verify the build

```bash
ros2 pkg list | grep -E "arm_ik|zed_aruco|typing_interfaces|mi_rover_description"
```

All four should appear.

```bash
ros2 launch mi_rover_description gazebo.launch.py
```

If Gazebo Fortress opens with the rover and keyboard panel visible, you're set.

---

## 5. Run the simulation

Follow [`SIM_CHECKPOINT.md`](./SIM_CHECKPOINT.md) section "Bring-up — 6 terminals" for the full run-time sequence with all calibrated parameters.

---

## Updating later

When new code is pushed to either repo:

```bash
cd ~/ros2_ws/src/autonomous_typing_zed && git pull
cd ~/ros2_ws/src/mi_rover_description && git pull
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  typing_interfaces arm_ik zed_aruco mi_rover_description
source install/setup.bash
```

Or in one shot from the workspace root:

```bash
cd ~/ros2_ws
vcs pull src
colcon build --symlink-install
source install/setup.bash
```

---

## Troubleshooting

- **`Package 'ros-humble-ros-gz' has no installation candidate`** — your apt sources don't include the `packages.osrfoundation.org` repo. ROS 2 Humble's Gazebo Fortress packages are mirrored in the standard ROS 2 repo, so make sure you actually finished the official ROS 2 Humble install (`https://docs.ros.org/en/humble/Installation.html`).
- **Gazebo opens but rover doesn't appear** — wait 5–10 s after the window opens; the rover spawns asynchronously. If still missing, check the Terminal 1 log for spawn errors.
- **Arm doesn't move when goals are sent** — confirm `arm_bridge.py` is running (Terminal 3) and the `brazo_controller` is loaded (you should see `[brazo_controller]: Accepted new action goal` in the Gazebo terminal when goals fire).
- **`Target out of workspace` warnings** — you forgot the `-p workspace_x_max:=1.0` parameter on the coordinator. The default 0.55 is too small for the sim setup.
