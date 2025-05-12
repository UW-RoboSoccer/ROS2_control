# ROS2 Robot Control

This repository contains a ROS2 robot control package with Mujoco simulation integration.

## Prerequisites

- WSL (Windows Subsystem for Linux)
- ROS2 Humble
- Mujoco ROS2 Control

## Installation

1. Clone the repository:
```bash
git clone https://github.com/UW-RoboSoccer/ROS2_control.git
cd ROS2_control
```

2. Build the workspace:
```bash
colcon build --packages-select my_robot_description
source install/setup.bash
```

## Running the Simulation

### Terminal 1: Launch the Simulation
```bash
ros2 launch my_robot_description mujoco.launch.py
```

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
