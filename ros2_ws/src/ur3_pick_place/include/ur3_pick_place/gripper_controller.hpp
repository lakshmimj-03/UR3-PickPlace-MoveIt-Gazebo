#ifndef GRIPPER_CONTROLLER_HPP_
#define GRIPPER_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/gripper_command.hpp>

class GripperController {
public:
    GripperController(rclcpp::Node::SharedPtr node)
    : node_(node) {
        publisher_ = node_->create_publisher<control_msgs::msg::GripperCommand>(
            "/gripper_controller/command", 10);
    }

    void open() {
        auto command = control_msgs::msg::GripperCommand();
        command.position = 0.085;  // Maximum opening
        command.max_effort = 10.0;
        publisher_->publish(command);
    }

    void close() {
        auto command = control_msgs::msg::GripperCommand();
        command.position = 0.0;    // Fully closed
        command.max_effort = 10.0;
        publisher_->publish(command);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr publisher_;
};

#endif  // GRIPPER_CONTROLLER_HPP_