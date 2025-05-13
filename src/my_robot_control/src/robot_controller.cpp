#include <memory>
#include <vector>
#include <string>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        // Create publisher for joint commands
        joint_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/position_controller/joint_trajectory", 10);

        // Create subscriber for joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RobotController::joint_state_callback, this, std::placeholders::_1));

        // Initialize joint positions map with all joints
        joint_positions_ = {
            {"right_shoulder_pitch", 0.0},
            {"right_shoulder_roll", 0.0},
            {"right_elbow", 0.0},
            {"left_shoulder_pitch", 0.0},
            {"left_shoulder_roll", 0.0},
            {"left_elbow", 0.0},
            {"head_yaw", 0.0},
            {"head_pitch", 0.0},
            {"right_hip_pitch", 0.0},
            {"right_hip_roll", 0.0},
            {"right_hip_yaw", 0.0},
            {"right_knee", 0.0},
            {"right_ankle_pitch", 0.0},
            {"left_hip_pitch", 0.0},
            {"left_hip_roll", 0.0},
            {"left_hip_yaw", 0.0},
            {"left_knee", 0.0},
            {"left_ankle_pitch", 0.0}
        };

        // Create a timer to move the robot periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),  // Move every 3 seconds
            std::bind(&RobotController::timer_callback, this));

        // Initialize movement sequence
        current_pose_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Robot controller node initialized");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update joint positions
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (joint_positions_.find(msg->name[i]) != joint_positions_.end()) {
                joint_positions_[msg->name[i]] = msg->position[i];
            }
        }
    }

    void timer_callback()
    {
        // Define a sequence of poses
        std::vector<std::map<std::string, double>> poses = {
            // Pose 1: Ready position
            {
                {"right_shoulder_pitch", 0.2},
                {"right_shoulder_roll", 0.3},
                {"right_elbow", -0.1},
                {"left_shoulder_pitch", 0.2},
                {"left_shoulder_roll", -0.3},
                {"left_elbow", -0.1},
                {"head_yaw", 0.5},
                {"head_pitch", 0.2},
                {"right_hip_pitch", 0.3},
                {"right_hip_roll", -0.3},
                {"right_hip_yaw", 0.4},
                {"right_knee", 0.2},
                {"right_ankle_pitch", -0.2},
                {"left_hip_pitch", 0.3},
                {"left_hip_roll", 0.3},
                {"left_hip_yaw", 0.3},
                {"left_knee", 0.2},
                {"left_ankle_pitch", -0.2}
            },
            // Pose 2: Wave right arm
            {
                {"right_shoulder_pitch", 0.5},
                {"right_shoulder_roll", 0.0},
                {"right_elbow", 0.3},
                {"left_shoulder_pitch", 0.2},
                {"left_shoulder_roll", -0.3},
                {"left_elbow", -0.1},
                {"head_yaw", 0.0},
                {"head_pitch", 0.0},
                {"right_hip_pitch", 0.0},
                {"right_hip_roll", 0.0},
                {"right_hip_yaw", 0.0},
                {"right_knee", 0.0},
                {"right_ankle_pitch", 0.0},
                {"left_hip_pitch", 0.0},
                {"left_hip_roll", 0.0},
                {"left_hip_yaw", 0.0},
                {"left_knee", 0.0},
                {"left_ankle_pitch", 0.0}
            },
            // Pose 3: Wave left arm
            {
                {"right_shoulder_pitch", 0.2},
                {"right_shoulder_roll", 0.3},
                {"right_elbow", -0.1},
                {"left_shoulder_pitch", 0.5},
                {"left_shoulder_roll", 0.0},
                {"left_elbow", 0.3},
                {"head_yaw", 0.0},
                {"head_pitch", 0.0},
                {"right_hip_pitch", 0.0},
                {"right_hip_roll", 0.0},
                {"right_hip_yaw", 0.0},
                {"right_knee", 0.0},
                {"right_ankle_pitch", 0.0},
                {"left_hip_pitch", 0.0},
                {"left_hip_roll", 0.0},
                {"left_hip_yaw", 0.0},
                {"left_knee", 0.0},
                {"left_ankle_pitch", 0.0}
            },
            // Pose 4: Zero position
            {
                {"right_shoulder_pitch", 0.0},
                {"right_shoulder_roll", 0.0},
                {"right_elbow", 0.0},
                {"left_shoulder_pitch", 0.0},
                {"left_shoulder_roll", 0.0},
                {"left_elbow", 0.0},
                {"head_yaw", 0.0},
                {"head_pitch", 0.0},
                {"right_hip_pitch", 0.0},
                {"right_hip_roll", 0.0},
                {"right_hip_yaw", 0.0},
                {"right_knee", 0.0},
                {"right_ankle_pitch", 0.0},
                {"left_hip_pitch", 0.0},
                {"left_hip_roll", 0.0},
                {"left_hip_yaw", 0.0},
                {"left_knee", 0.0},
                {"left_ankle_pitch", 0.0}
            }
        };

        // Move to the current pose
        move_all_joints(poses[current_pose_]);

        // Move to the next pose in the sequence
        current_pose_ = (current_pose_ + 1) % poses.size();
    }

    void move_all_joints(const std::map<std::string, double>& new_positions, double duration = 2.0)
    {
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        
        // Add all joint names in the correct order
        for (const auto& [joint_name, _] : joint_positions_) {
            trajectory_msg.joint_names.push_back(joint_name);
        }

        // Create trajectory point with positions for all joints
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        for (const auto& joint_name : trajectory_msg.joint_names) {
            // Use new position if provided, otherwise use current position
            double position = new_positions.count(joint_name) ? 
                            new_positions.at(joint_name) : 
                            joint_positions_[joint_name];
            point.positions.push_back(position);
        }

        // Set the time for the movement
        point.time_from_start.sec = static_cast<int32_t>(duration);
        point.time_from_start.nanosec = static_cast<uint32_t>((duration - static_cast<int32_t>(duration)) * 1e9);

        trajectory_msg.points.push_back(point);
        joint_command_pub_->publish(trajectory_msg);
        RCLCPP_INFO(this->get_logger(), "Moving all joints to new positions");
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, double> joint_positions_;
    size_t current_pose_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 