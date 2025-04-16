#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class MoveRobotNode : public rclcpp::Node
{
public:
  MoveRobotNode()
  : Node("move_robot_node")
  {
    // Create publisher for joint states
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

    // Joint names for the UR3 robot
    joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };

    // Define waypoints for a pick and place motion
    waypoints_ = {
      // Home position
      {0.0, -1.57, 0.0, -1.57, 0.0, 0.0},
      
      // Move to pre-grasp position
      {-0.5, -1.0, 0.5, -1.0, -0.5, 0.0},
      
      // Lower to grasp position
      {-0.5, -0.8, 0.8, -1.5, -0.5, 0.0},
      
      // Close gripper (simulated by wrist rotation)
      {-0.5, -0.8, 0.8, -1.5, -0.5, 0.5},
      
      // Lift object
      {-0.5, -1.0, 0.5, -1.0, -0.5, 0.5},
      
      // Move to place position
      {0.5, -1.0, 0.5, -1.0, 0.5, 0.5},
      
      // Lower to place position
      {0.5, -0.8, 0.8, -1.5, 0.5, 0.5},
      
      // Open gripper (simulated by wrist rotation)
      {0.5, -0.8, 0.8, -1.5, 0.5, 0.0},
      
      // Lift after placing
      {0.5, -1.0, 0.5, -1.0, 0.5, 0.0},
      
      // Return to home position
      {0.0, -1.57, 0.0, -1.57, 0.0, 0.0}
    };

    // Initialize current positions to home position
    current_positions_ = waypoints_[0];
    
    // Create timer to publish joint states (100 Hz)
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MoveRobotNode::publish_joint_states, this));
    
    // Create timer to update waypoints (slower rate)
    waypoint_timer_ = this->create_wall_timer(
      3000ms, std::bind(&MoveRobotNode::update_waypoint, this));
    
    RCLCPP_INFO(this->get_logger(), "Move robot node initialized");
  }

private:
  void publish_joint_states()
  {
    auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = joint_names_;
    joint_state_msg->position = current_positions_;
    
    joint_state_publisher_->publish(std::move(joint_state_msg));
  }

  void update_waypoint()
  {
    // Move to the next waypoint
    current_waypoint_ = (current_waypoint_ + 1) % waypoints_.size();
    current_positions_ = waypoints_[current_waypoint_];
    
    RCLCPP_INFO(this->get_logger(), "Moving to waypoint %d", current_waypoint_);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr waypoint_timer_;
  
  std::vector<std::string> joint_names_;
  std::vector<std::vector<double>> waypoints_;
  std::vector<double> current_positions_;
  size_t current_waypoint_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveRobotNode>());
  rclcpp::shutdown();
  return 0;
}
