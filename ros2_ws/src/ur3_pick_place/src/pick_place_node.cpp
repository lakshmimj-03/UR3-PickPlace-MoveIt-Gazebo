#include "ur3_pick_place/pick_place_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

PickPlaceNode::PickPlaceNode()
    : Node("pick_place_node")
{
    // Initialize parameters
    try {
        this->declare_parameter("use_sim_time", true);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        // Parameter already declared, ignore
    }

    try {
        this->declare_parameter("use_perception", false);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        // Parameter already declared, ignore
    }

    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
    use_perception_ = this->get_parameter("use_perception").as_bool();

    RCLCPP_INFO(this->get_logger(), "Initializing pick and place node");
    RCLCPP_INFO(this->get_logger(), "Simulation mode: %s", use_sim_time_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Using perception: %s", use_perception_ ? "true" : "false");

    // Initialize MoveGroup
    move_group_node_ = std::make_shared<rclcpp::Node>(
        "move_group_interface_tutorial",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Using planning group "ur_manipulator" which is defined in the SRDF
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        move_group_node_, "ur_manipulator"
    );

    // Initialize planning scene interface for collision object management
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Set motion planning parameters
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    move_group_->setGoalJointTolerance(0.001);
    move_group_->setGoalPositionTolerance(0.001);
    move_group_->setGoalOrientationTolerance(0.01);

    // Create gripper command publisher
    gripper_command_publisher_ = this->create_publisher<control_msgs::msg::GripperCommand>(
        "/gripper_controller/gripper_cmd", 10
    );

    // Create visualization marker publisher
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization_marker_array", 10
    );

    // Initialize TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Wait for the planning scene to be ready
    RCLCPP_INFO(this->get_logger(), "Waiting for planning scene...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Move to the ready position first
    move_group_->setNamedTarget("home");
    bool move_success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    if (move_success) {
        RCLCPP_INFO(this->get_logger(), "Moved to home position");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to home position");
    }

    // Setup the environment (tables, objects, etc.)
    setupEnvironment();

    // Create a subscription to the trigger topic
    trigger_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/trigger_pick_place", 10,
        std::bind(&PickPlaceNode::triggerCallback, this, std::placeholders::_1)
    );

    // Create a timer to start the pick and place sequence after a delay
    timer_ = this->create_wall_timer(
        10s,
        std::bind(&PickPlaceNode::executePipeline, this)
    );

    RCLCPP_INFO(this->get_logger(), "Pick and place node initialized");
    RCLCPP_INFO(this->get_logger(), "Waiting for trigger message on /trigger_pick_place or timer expiration");
}

void PickPlaceNode::executePipeline()
{
    // Cancel the timer to prevent multiple executions
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Starting pick and place sequence...");

    // Move to a pre-defined starting position (home)
    move_group_->setNamedTarget("ready");
    bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to ready position");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Moved to ready position");

    // If we're using perception, detect objects
    if (use_perception_) {
        detectObjects();
    }

    // Get available objects
    std::vector<std::string> object_ids = getAvailableObjects();

    if (object_ids.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No objects available to pick");
        return;
    }

    // Pick and place each object
    for (const auto& object_id : object_ids) {
        RCLCPP_INFO(this->get_logger(), "Attempting to pick object: %s", object_id.c_str());

        if (!executePick(object_id)) {
            RCLCPP_ERROR(this->get_logger(), "Pick operation failed for object: %s", object_id.c_str());
            continue;
        }

        if (!executePlace(object_id)) {
            RCLCPP_ERROR(this->get_logger(), "Place operation failed for object: %s", object_id.c_str());
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully picked and placed object: %s", object_id.c_str());
    }

    // Return to home position
    move_group_->setNamedTarget("home");
    success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "Returned to home position");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to return to home position");
    }

    RCLCPP_INFO(this->get_logger(), "Pick and place sequence completed");

    // Restart the timer to run the pipeline again after a delay
    timer_ = this->create_wall_timer(
        30s,
        std::bind(&PickPlaceNode::executePipeline, this)
    );

    RCLCPP_INFO(this->get_logger(), "Waiting for trigger message on /trigger_pick_place or timer expiration");
}

void PickPlaceNode::detectObjects()
{
    // In a real system, this would use computer vision to detect objects
    // For simulation, we rely on known object positions from Gazebo
    RCLCPP_INFO(this->get_logger(), "Detecting objects...");

    // For a real perception system, this would subscribe to camera or point cloud topics
    // and use vision algorithms to detect and localize objects

    // For now, we'll use the known positions from our Gazebo world
}

std::vector<std::string> PickPlaceNode::getAvailableObjects()
{
    // In a real system, this would return detected objects from perception
    // For simulation, return known object IDs
    std::vector<std::string> objects;

    if (use_sim_time_) {
        objects.push_back("red_cube");
        objects.push_back("blue_cylinder");
    } else {
        // In a real scenario, would return IDs of detected objects
        auto collision_objects = planning_scene_interface_->getObjects();

        for (const auto& entry : collision_objects) {
            if (entry.first != "table" && entry.first != "destination_box" &&
                entry.first.find("table_leg") == std::string::npos) {
                objects.push_back(entry.first);
            }
        }
    }

    return objects;
}

bool PickPlaceNode::executePick(const std::string& object_id)
{
    RCLCPP_INFO(this->get_logger(), "Executing pick operation for object: %s", object_id.c_str());

    // Get the object's pose
    geometry_msgs::msg::Pose object_pose;
    if (!getObjectPose(object_id, object_pose)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pose for object: %s", object_id.c_str());
        return false;
    }

    // Create pre-grasp pose slightly above the object
    geometry_msgs::msg::Pose pre_grasp_pose = object_pose;
    pre_grasp_pose.position.z += 0.1;  // 10cm above the object

    // Create grasp pose with a small offset to account for gripper dimensions
    geometry_msgs::msg::Pose grasp_pose = object_pose;
    grasp_pose.position.z += 0.01;  // Slight offset to grasp object properly

    // Set the orientation for vertical approach (gripper facing down)
    tf2::Quaternion q;
    q.setRPY(0, M_PI/2, 0);  // Roll, Pitch, Yaw
    grasp_pose.orientation = tf2::toMsg(q);
    pre_grasp_pose.orientation = grasp_pose.orientation;

    // Visualize the approach and grasp poses
    publishPoseMarker(pre_grasp_pose, "pre_grasp_pose", 0, 1.0, 0.0, 0.0);
    publishPoseMarker(grasp_pose, "grasp_pose", 1, 0.0, 1.0, 0.0);

    // Open the gripper
    controlGripper(1.0);  // 1.0 = fully open
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Move to pre-grasp pose
    move_group_->setPoseTarget(pre_grasp_pose);
    bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to pre-grasp pose");
        return false;
    }

    // Approach the object using Cartesian path for better control
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pre_grasp_pose);
    waypoints.push_back(grasp_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction < 0.9) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute Cartesian path for approach: %f", fraction);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    approach_plan.trajectory = trajectory;

    success = (move_group_->execute(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to approach object");
        return false;
    }

    // Close the gripper to grasp the object
    controlGripper(0.0);  // 0.0 = fully closed
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Attach the object to the end effector
    move_group_->attachObject(object_id);

    // Retreat from the grasp position (reverse of the approach)
    waypoints.clear();
    waypoints.push_back(grasp_pose);
    waypoints.push_back(pre_grasp_pose);

    fraction = move_group_->computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction < 0.9) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute Cartesian path for retreat: %f", fraction);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    retreat_plan.trajectory = trajectory;

    success = (move_group_->execute(retreat_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to retreat with object");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully picked object: %s", object_id.c_str());
    return true;
}

bool PickPlaceNode::executePlace(const std::string& object_id)
{
    RCLCPP_INFO(this->get_logger(), "Executing place operation for object: %s", object_id.c_str());

    // Define the place location (destination_box position)
    geometry_msgs::msg::Pose place_pose;
    place_pose.position.x = 0.7;
    place_pose.position.y = 0.2;
    place_pose.position.z = 0.45;  // Slightly above the destination box

    // Set orientation for vertical approach
    tf2::Quaternion q;
    q.setRPY(0, M_PI/2, 0);  // Roll, Pitch, Yaw
    place_pose.orientation = tf2::toMsg(q);

    // Create pre-place pose slightly above the place location
    geometry_msgs::msg::Pose pre_place_pose = place_pose;
    pre_place_pose.position.z += 0.1;  // 10cm above

    // Visualize the approach and place poses
    publishPoseMarker(pre_place_pose, "pre_place_pose", 2, 0.0, 0.0, 1.0);
    publishPoseMarker(place_pose, "place_pose", 3, 1.0, 1.0, 0.0);

    // Move to pre-place pose
    move_group_->setPoseTarget(pre_place_pose);
    bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to pre-place pose");
        return false;
    }

    // Approach the place location using Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pre_place_pose);
    waypoints.push_back(place_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction < 0.9) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute Cartesian path for place approach: %f", fraction);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    approach_plan.trajectory = trajectory;

    success = (move_group_->execute(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to approach place location");
        return false;
    }

    // Open the gripper to release the object
    controlGripper(1.0);  // 1.0 = fully open
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Detach the object
    move_group_->detachObject(object_id);

    // Retreat from the place position
    waypoints.clear();
    waypoints.push_back(place_pose);
    waypoints.push_back(pre_place_pose);

    fraction = move_group_->computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction < 0.9) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute Cartesian path for place retreat: %f", fraction);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    retreat_plan.trajectory = trajectory;

    success = (move_group_->execute(retreat_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to retreat from place location");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully placed object: %s", object_id.c_str());
    return true;
}

void PickPlaceNode::controlGripper(double position)
{
    RCLCPP_INFO(this->get_logger(), "Setting gripper position to %f", position);

    auto command = std::make_unique<control_msgs::msg::GripperCommand>();
    command->position = position;  // 0.0 = closed, 1.0 = open
    command->max_effort = 50.0;   // Adjust based on gripper
    gripper_command_publisher_->publish(std::move(command));

    // Wait for the gripper to move
    rclcpp::sleep_for(std::chrono::seconds(1));
}

void PickPlaceNode::setupEnvironment()
{
    RCLCPP_INFO(this->get_logger(), "Setting up environment for collision checking");

    if (!use_sim_time_) {
        // In real mode, set up collision objects for the environment

        // Add a table
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = move_group_->getPlanningFrame();
        table.id = "table";

        shape_msgs::msg::SolidPrimitive table_primitive;
        table_primitive.type = table_primitive.BOX;
        table_primitive.dimensions = {0.8, 0.8, 0.03};  // Length, width, height

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.x = 0.5;
        table_pose.position.y = 0.0;
        table_pose.position.z = 0.4;

        table.primitives.push_back(table_primitive);
        table.primitive_poses.push_back(table_pose);
        table.operation = table.ADD;

        // Add destination box
        moveit_msgs::msg::CollisionObject destination_box;
        destination_box.header.frame_id = move_group_->getPlanningFrame();
        destination_box.id = "destination_box";

        shape_msgs::msg::SolidPrimitive box_primitive;
        box_primitive.type = box_primitive.BOX;
        box_primitive.dimensions = {0.1, 0.1, 0.02};

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.7;
        box_pose.position.y = 0.2;
        box_pose.position.z = 0.42;

        destination_box.primitives.push_back(box_primitive);
        destination_box.primitive_poses.push_back(box_pose);
        destination_box.operation = destination_box.ADD;

        // Add a cube to pick
        moveit_msgs::msg::CollisionObject red_cube;
        red_cube.header.frame_id = move_group_->getPlanningFrame();
        red_cube.id = "red_cube";

        shape_msgs::msg::SolidPrimitive cube_primitive;
        cube_primitive.type = cube_primitive.BOX;
        cube_primitive.dimensions = {0.035, 0.035, 0.035};

        geometry_msgs::msg::Pose cube_pose;
        cube_pose.orientation.w = 1.0;
        cube_pose.position.x = 0.5;
        cube_pose.position.y = 0.1;
        cube_pose.position.z = 0.435;

        red_cube.primitives.push_back(cube_primitive);
        red_cube.primitive_poses.push_back(cube_pose);
        red_cube.operation = red_cube.ADD;

        // Add a cylinder to pick
        moveit_msgs::msg::CollisionObject blue_cylinder;
        blue_cylinder.header.frame_id = move_group_->getPlanningFrame();
        blue_cylinder.id = "blue_cylinder";

        shape_msgs::msg::SolidPrimitive cylinder_primitive;
        cylinder_primitive.type = cylinder_primitive.CYLINDER;
        cylinder_primitive.dimensions = {0.035, 0.0175};  // height, radius

        geometry_msgs::msg::Pose cylinder_pose;
        cylinder_pose.orientation.w = 1.0;
        cylinder_pose.position.x = 0.5;
        cylinder_pose.position.y = -0.1;
        cylinder_pose.position.z = 0.435;

        blue_cylinder.primitives.push_back(cylinder_primitive);
        blue_cylinder.primitive_poses.push_back(cylinder_pose);
        blue_cylinder.operation = blue_cylinder.ADD;

        // Add the collision objects to the planning scene
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(table);
        collision_objects.push_back(destination_box);
        collision_objects.push_back(red_cube);
        collision_objects.push_back(blue_cylinder);

        planning_scene_interface_->applyCollisionObjects(collision_objects);

        RCLCPP_INFO(this->get_logger(), "Added table, destination box, and objects to the world");
    } else {
        // In simulation mode, Gazebo handles the physical objects
        // We still need to add them to the planning scene for MoveIt to plan around

        // Give MoveIt time to discover the objects via the scene monitor
        RCLCPP_INFO(this->get_logger(), "Using simulation objects from Gazebo");
    }
}

bool PickPlaceNode::getObjectPose(const std::string& object_id, geometry_msgs::msg::Pose& pose)
{
    if (use_sim_time_) {
        // For simulation, use hard-coded poses from our Gazebo world
        if (object_id == "red_cube") {
            pose.position.x = 0.5;
            pose.position.y = 0.1;
            pose.position.z = 0.435;
            pose.orientation.w = 1.0;
            return true;
        } else if (object_id == "blue_cylinder") {
            pose.position.x = 0.5;
            pose.position.y = -0.1;
            pose.position.z = 0.435;
            pose.orientation.w = 1.0;
            return true;
        }
        return false;
    } else {
        // In real mode, get object pose from planning scene
        auto objects = planning_scene_interface_->getObjects({object_id});

        if (objects.find(object_id) != objects.end()) {
            pose = objects[object_id].primitive_poses[0];
            return true;
        }
        return false;
    }
}

void PickPlaceNode::triggerCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Received trigger message, starting pick and place sequence");
        // Cancel any existing timer
        if (timer_) {
            timer_->cancel();
        }
        // Execute the pipeline
        executePipeline();
    }
}

void PickPlaceNode::publishPoseMarker(const geometry_msgs::msg::Pose& pose,
                                      const std::string& ns, int id,
                                      float r, float g, float b)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Position marker (small sphere)
    visualization_msgs::msg::Marker position_marker;
    position_marker.header.frame_id = move_group_->getPlanningFrame();
    position_marker.header.stamp = this->now();
    position_marker.ns = ns;
    position_marker.id = id;
    position_marker.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker.action = visualization_msgs::msg::Marker::ADD;
    position_marker.pose = pose;
    position_marker.scale.x = 0.02;
    position_marker.scale.y = 0.02;
    position_marker.scale.z = 0.02;
    position_marker.color.r = r;
    position_marker.color.g = g;
    position_marker.color.b = b;
    position_marker.color.a = 1.0;
    position_marker.lifetime = rclcpp::Duration(5s);

    // Orientation marker (arrow showing Z axis)
    visualization_msgs::msg::Marker orientation_marker;
    orientation_marker.header.frame_id = move_group_->getPlanningFrame();
    orientation_marker.header.stamp = this->now();
    orientation_marker.ns = ns + "_orientation";
    orientation_marker.id = id;
    orientation_marker.type = visualization_msgs::msg::Marker::ARROW;
    orientation_marker.action = visualization_msgs::msg::Marker::ADD;
    orientation_marker.pose = pose;
    orientation_marker.scale.x = 0.1;  // Arrow length
    orientation_marker.scale.y = 0.01;  // Arrow width
    orientation_marker.scale.z = 0.01;  // Arrow height
    orientation_marker.color.r = r;
    orientation_marker.color.g = g;
    orientation_marker.color.b = b;
    orientation_marker.color.a = 1.0;
    orientation_marker.lifetime = rclcpp::Duration(5s);

    marker_array.markers.push_back(position_marker);
    marker_array.markers.push_back(orientation_marker);

    marker_publisher_->publish(marker_array);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Enable logging
    auto logger = rclcpp::get_logger("pick_place_node");
    logger.set_level(rclcpp::Logger::Level::Info);

    auto node = std::make_shared<PickPlaceNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(node->move_group_node_);

    RCLCPP_INFO(node->get_logger(), "Starting executor for pick and place node");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
