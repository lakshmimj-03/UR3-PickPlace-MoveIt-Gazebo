#ifndef PICK_PLACE_NODE_HPP
#define PICK_PLACE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>

class PickPlaceNode : public rclcpp::Node
{
public:
    PickPlaceNode();
    // Make move_group_node_ public so it can be accessed by the main function
    rclcpp::Node::SharedPtr move_group_node_;

private:
    void executePipeline();
    bool executePick(const std::string& object_id);
    bool executePlace(const std::string& object_id);
    void setupEnvironment();
    void controlGripper(double position);
    void detectObjects();
    std::vector<std::string> getAvailableObjects();
    bool getObjectPose(const std::string& object_id, geometry_msgs::msg::Pose& pose);
    void publishPoseMarker(const geometry_msgs::msg::Pose& pose,
                          const std::string& ns, int id,
                          float r, float g, float b);
    void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg);

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr gripper_command_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool use_sim_time_;
    bool use_perception_;
};

#endif // PICK_PLACE_NODE_HPP
