#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from control_msgs.msg import GripperCommand
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.action import MoveGroup
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
import moveit_commander
import time
import math
import numpy as np
from tf_transformations import quaternion_from_euler

MOVEIT_ERROR_DICT = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.PREEMPTED: "PREEMPTED",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.INVALID_OBJECT_NAME: "INVALID_OBJECT_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "COLLISION_CHECKING_UNAVAILABLE",
    MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
    MoveItErrorCodes.SENSOR_INFO_STALE: "SENSOR_INFO_STALE",
    MoveItErrorCodes.COMMUNICATION_FAILURE: "COMMUNICATION_FAILURE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        # Initialize MoveIt
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander('ur_manipulator')
        self.gripper_group = moveit_commander.MoveGroupCommander('gripper')

        # Set planning parameters
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)

        # Publishers
        self.gripper_pub = self.create_publisher(
            GripperCommand,
            '/gripper_controller/command',
            10)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10)

        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a timer to start the demo
        self.timer = self.create_timer(5.0, self.execute_pick_place)
        self.get_logger().info('Pick and Place Node started')

        # Object positions
        self.red_cube_pose = Pose(
            position=Point(x=0.5, y=0.1, z=0.435),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.blue_cylinder_pose = Pose(
            position=Point(x=0.5, y=-0.1, z=0.435),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.place_pose = Pose(
            position=Point(x=0.7, y=0.2, z=0.435),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

    def execute_pick_place(self):
        self.timer.cancel()  # Run only once

        # Visualize objects
        self.visualize_objects()

        # Move to home position
        self.get_logger().info('Moving to home position')
        self.arm_group.set_named_target('home')
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()

        if not success:
            self.get_logger().error('Failed to move to home position')
            return

        # Open gripper
        self.control_gripper(1.0)
        time.sleep(1.0)

        # Pick red cube
        self.get_logger().info('Picking red cube')
        success = self.pick_object('red_cube', self.red_cube_pose)

        if not success:
            self.get_logger().error('Failed to pick red cube')
            return

        # Place red cube
        self.get_logger().info('Placing red cube')
        success = self.place_object('red_cube', self.place_pose)

        if not success:
            self.get_logger().error('Failed to place red cube')
            return

        # Return to home position
        self.get_logger().info('Moving to home position')
        self.arm_group.set_named_target('home')
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()

        # Pick blue cylinder
        self.get_logger().info('Picking blue cylinder')
        success = self.pick_object('blue_cylinder', self.blue_cylinder_pose)

        if not success:
            self.get_logger().error('Failed to pick blue cylinder')
            return

        # Place blue cylinder
        place_pose_cylinder = Pose()
        place_pose_cylinder.position.x = self.place_pose.position.x - 0.05
        place_pose_cylinder.position.y = self.place_pose.position.y
        place_pose_cylinder.position.z = self.place_pose.position.z + 0.04  # Place on top of the cube
        place_pose_cylinder.orientation = self.place_pose.orientation

        self.get_logger().info('Placing blue cylinder')
        success = self.place_object('blue_cylinder', place_pose_cylinder)

        if not success:
            self.get_logger().error('Failed to place blue cylinder')
            return

        # Return to home position
        self.get_logger().info('Moving to home position')
        self.arm_group.set_named_target('home')
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()

        self.get_logger().info('Pick and place demo completed')

    def pick_object(self, object_name, object_pose):
        # Pre-grasp position (10cm above object)
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = object_pose.position.x
        pre_grasp_pose.position.y = object_pose.position.y
        pre_grasp_pose.position.z = object_pose.position.z + 0.1

        # Set orientation for vertical grasp
        q = quaternion_from_euler(-math.pi/2, 0, 0)  # Rotate gripper to face down
        pre_grasp_pose.orientation.x = q[0]
        pre_grasp_pose.orientation.y = q[1]
        pre_grasp_pose.orientation.z = q[2]
        pre_grasp_pose.orientation.w = q[3]

        # Move to pre-grasp position
        self.get_logger().info(f'Moving to pre-grasp position for {object_name}')
        self.arm_group.set_pose_target(pre_grasp_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if not success:
            self.get_logger().error(f'Failed to move to pre-grasp position for {object_name}')
            return False

        # Open gripper
        self.control_gripper(1.0)
        time.sleep(1.0)

        # Move to grasp position
        grasp_pose = Pose()
        grasp_pose.position.x = object_pose.position.x
        grasp_pose.position.y = object_pose.position.y
        grasp_pose.position.z = object_pose.position.z + 0.02  # Slightly above object for better grasp
        grasp_pose.orientation = pre_grasp_pose.orientation

        # Plan and execute Cartesian path to grasp position
        self.get_logger().info(f'Moving to grasp position for {object_name}')
        waypoints = [grasp_pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step
            0.0)   # jump_threshold

        if fraction < 0.9:
            self.get_logger().error(f'Failed to compute Cartesian path to grasp position: {fraction}')
            return False

        success = self.arm_group.execute(plan, wait=True)

        if not success:
            self.get_logger().error(f'Failed to execute Cartesian path to grasp position')
            return False

        # Close gripper
        self.control_gripper(0.0)
        time.sleep(1.0)

        # Attach object to gripper in MoveIt scene
        self.scene.attach_box('tool0', object_name, touch_links=['gripper_finger_link', 'gripper_finger2_link'])

        # Lift object
        lift_pose = Pose()
        lift_pose.position.x = grasp_pose.position.x
        lift_pose.position.y = grasp_pose.position.y
        lift_pose.position.z = grasp_pose.position.z + 0.1
        lift_pose.orientation = grasp_pose.orientation

        self.get_logger().info(f'Lifting {object_name}')
        waypoints = [lift_pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step
            0.0)   # jump_threshold

        if fraction < 0.9:
            self.get_logger().error(f'Failed to compute Cartesian path for lifting: {fraction}')
            return False

        success = self.arm_group.execute(plan, wait=True)

        if not success:
            self.get_logger().error(f'Failed to execute Cartesian path for lifting')
            return False

        return True

    def place_object(self, object_name, place_pose):
        # Pre-place position (10cm above place location)
        pre_place_pose = Pose()
        pre_place_pose.position.x = place_pose.position.x
        pre_place_pose.position.y = place_pose.position.y
        pre_place_pose.position.z = place_pose.position.z + 0.1

        # Set orientation for vertical placement
        q = quaternion_from_euler(-math.pi/2, 0, 0)  # Rotate gripper to face down
        pre_place_pose.orientation.x = q[0]
        pre_place_pose.orientation.y = q[1]
        pre_place_pose.orientation.z = q[2]
        pre_place_pose.orientation.w = q[3]

        # Move to pre-place position
        self.get_logger().info(f'Moving to pre-place position for {object_name}')
        self.arm_group.set_pose_target(pre_place_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if not success:
            self.get_logger().error(f'Failed to move to pre-place position for {object_name}')
            return False

        # Move to place position
        place_pose_with_orientation = Pose()
        place_pose_with_orientation.position = place_pose.position
        place_pose_with_orientation.orientation = pre_place_pose.orientation

        # Plan and execute Cartesian path to place position
        self.get_logger().info(f'Moving to place position for {object_name}')
        waypoints = [place_pose_with_orientation]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step
            0.0)   # jump_threshold

        if fraction < 0.9:
            self.get_logger().error(f'Failed to compute Cartesian path to place position: {fraction}')
            return False

        success = self.arm_group.execute(plan, wait=True)

        if not success:
            self.get_logger().error(f'Failed to execute Cartesian path to place position')
            return False

        # Open gripper
        self.control_gripper(1.0)
        time.sleep(1.0)

        # Detach object from gripper in MoveIt scene
        self.scene.remove_attached_object('tool0', object_name)

        # Retreat from place position
        retreat_pose = Pose()
        retreat_pose.position.x = place_pose_with_orientation.position.x
        retreat_pose.position.y = place_pose_with_orientation.position.y
        retreat_pose.position.z = place_pose_with_orientation.position.z + 0.1
        retreat_pose.orientation = place_pose_with_orientation.orientation

        self.get_logger().info(f'Retreating from {object_name}')
        waypoints = [retreat_pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step
            0.0)   # jump_threshold

        if fraction < 0.9:
            self.get_logger().error(f'Failed to compute Cartesian path for retreating: {fraction}')
            return False

        success = self.arm_group.execute(plan, wait=True)

        if not success:
            self.get_logger().error(f'Failed to execute Cartesian path for retreating')
            return False

        return True

    def control_gripper(self, position):
        msg = GripperCommand()
        msg.position = position  # 0.0 (closed) to 1.0 (open)
        msg.max_effort = 50.0

        self.gripper_pub.publish(msg)

    def visualize_objects(self):
        marker_array = MarkerArray()

        # Red cube marker
        red_cube_marker = Marker()
        red_cube_marker.header.frame_id = 'world'
        red_cube_marker.header.stamp = self.get_clock().now().to_msg()
        red_cube_marker.ns = 'objects'
        red_cube_marker.id = 1
        red_cube_marker.type = Marker.CUBE
        red_cube_marker.action = Marker.ADD
        red_cube_marker.pose = self.red_cube_pose
        red_cube_marker.scale.x = 0.035
        red_cube_marker.scale.y = 0.035
        red_cube_marker.scale.z = 0.035
        red_cube_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)
        marker_array.markers.append(red_cube_marker)

        # Blue cylinder marker
        blue_cylinder_marker = Marker()
        blue_cylinder_marker.header.frame_id = 'world'
        blue_cylinder_marker.header.stamp = self.get_clock().now().to_msg()
        blue_cylinder_marker.ns = 'objects'
        blue_cylinder_marker.id = 2
        blue_cylinder_marker.type = Marker.CYLINDER
        blue_cylinder_marker.action = Marker.ADD
        blue_cylinder_marker.pose = self.blue_cylinder_pose
        blue_cylinder_marker.scale.x = 0.035
        blue_cylinder_marker.scale.y = 0.035
        blue_cylinder_marker.scale.z = 0.035
        blue_cylinder_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)
        marker_array.markers.append(blue_cylinder_marker)

        # Place location marker
        place_marker = Marker()
        place_marker.header.frame_id = 'world'
        place_marker.header.stamp = self.get_clock().now().to_msg()
        place_marker.ns = 'objects'
        place_marker.id = 3
        place_marker.type = Marker.CUBE
        place_marker.action = Marker.ADD
        place_marker.pose = self.place_pose
        place_marker.scale.x = 0.1
        place_marker.scale.y = 0.1
        place_marker.scale.z = 0.01
        place_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
        marker_array.markers.append(place_marker)

        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = PickPlaceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()