import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    mjcf_file = os.path.join(pkg_path, 'mjcf', 'robot.xml')
    controller_yaml = os.path.join(pkg_path, 'config', 'controllers.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = {'robot_description': f.read()}

    return LaunchDescription([
        Node(
            package='mujoco_ros2_control',
            executable='mujoco_ros2_control',
            output='screen',
            parameters=[
                robot_description,
                controller_yaml,
                {'mujoco_model_path': mjcf_file}
            ]
        ),
        # Start the controllers
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'position_controller'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'imu_sensor_broadcaster'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'torso_test_fts_broadcaster'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'force_torque_sensor_broadcaster'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'left_fts_broadcaster'],
            output='screen'
        )
    ])
