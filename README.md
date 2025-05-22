# ROS2 Robot Control

This repository contains a ROS2 robot control package with Mujoco simulation integration.

## Setup


### WSL2 on Ubuntu 22.04 (Windows Subsystem for Linux)

In administrator level powershell:
1. Install WSL running Ubuntu 22.04
```bash
wsl --install -d Ubuntu-22.04
```

2. Set WSL2 as default (if not already)
```bash
wsl --set-default-version 2
```

3. Open WSL terminal and update packages
```bash
sudo apt update && sudo apt upgrade -y
```

4. Install basic dev tools
``` bash
sudo apt install -y git curl wget build-essential
```


### ROS2 Humble
1. Setup locale
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

2. Add ROS2 GPG keys
```bash
sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

3. Add ROS2 apt repo
```bash
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

4. Install ROS2
``` bash
sudo apt update
sudo apt install ros-humble-desktop
```

5. Source ROS2 in shell Add to ~/.bashrc:
``` bash
source /opt/ros/humble/setup.bash
```

6. Install ROS2 build tools
``` bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```


### Mujoco ROS2 Control
1. Install MuJoCo 3.3.2
``` bash
mkdir -p ~/.mujoco
wget https://mujoco.org/download/mujoco-3.3.2-linux-x86_64.tar.gz
tar -xvzf mujoco-3.3.2-linux-x86_64.tar.gz -C ~/.mujoco
echo 'export MUJOCO_PY_MUJOCO_PATH=$HOME/.mujoco/mujoco-3.3.2' >> ~/.bashrc
echo 'export MUJOCO_DIR=$HOME/.mujoco/mujoco-3.3.2' >> ~/.bashrc
```

2. Install dependencies
``` bash
sudo apt install -y libosmesa6-dev libgl1-mesa-glx libglfw3
```


## Installation

1. In ~, clone the repository:
```bash
git clone https://github.com/UW-RoboSoccer/ROS2_control.git
git submodule init
git submodule update --remote --merge
```

2. Install dependencies
``` bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
colcon build --merge-install
source install/setup.bash
```


## Running the Simulation

### Terminal 1: Launch the Simulation
```bash
ros2 launch my_robot_description mujoco.launch.py
```


#### IF SIM IS BLACK SCREEN
Happens because of something with GPU? Do software rendering instead

1. Install dependencies
``` bash
sudo apt update
sudo apt install libosmesa6-dev libegl1-mesa-dev mesa-utils
```

2. Change environment variables to use software render instead of OpenGL
``` bash
export LIBGL_ALWAYS_SOFTWARE=1
export MUJOCO_GL=osmesa
glxinfo | grep "OpenGL renderer"
```

Should be able to launch as normal



### Terminal 2: Load and Control the Robot

1. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

2. Load the controllers:
```bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active position_controller
```

3. Verify the controllers are loaded:
```bash
ros2 control list_controllers
```
Expected output:
```
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
position_controller     joint_trajectory_controller/JointTrajectoryController  active
```

4. Control the robot by publishing joint trajectories:
```bash
ros2 topic pub --once /position_controller/joint_trajectory trajectory_msgs/JointTrajectory "header:
  stamp: {sec: 0, nanosec: 0}
joint_names: ['right_shoulder_pitch', 'right_shoulder_roll', 'right_elbow',
              'left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow',
              'head_yaw', 'head_pitch',
              'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw',
              'right_knee', 'right_ankle_pitch',
              'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw',
              'left_knee', 'left_ankle_pitch']
points:
- positions: [0.2, 0.3, -0.1,
              0.2, -0.3, -0.1,
              0.5, 0.2,
              0.3, -0.3, 0.4,
              0.2, -0.2,
              0.3, 0.3, 0.3,
              0.2, -0.2]
  time_from_start: {sec: 2, nanosec: 0}"
```

## Troubleshooting

If you encounter any issues:
1. Make sure all prerequisites are installed
2. Verify the workspace is built correctly
3. Check that the controllers are loaded properly
4. Ensure the joint names in the trajectory message match your robot's configuration
