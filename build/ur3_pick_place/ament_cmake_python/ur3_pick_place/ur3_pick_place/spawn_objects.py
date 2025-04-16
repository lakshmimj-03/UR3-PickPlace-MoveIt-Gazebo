#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import xacro
from ament_index_python.packages import get_package_share_directory

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        
        # Create a client to spawn entities in Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Wait for the service to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_entity service...')
        
        # Spawn objects
        self.spawn_objects()
        
    def spawn_objects(self):
        # Spawn red cube
        self.spawn_cube(
            name='red_cube',
            position=Point(x=0.5, y=0.1, z=0.435),
            color=[1.0, 0.0, 0.0, 1.0],
            size=[0.035, 0.035, 0.035]
        )
        
        # Spawn blue cylinder
        self.spawn_cylinder(
            name='blue_cylinder',
            position=Point(x=0.5, y=-0.1, z=0.435),
            color=[0.0, 0.0, 1.0, 1.0],
            radius=0.0175,
            length=0.035
        )
        
        # Spawn destination box
        self.spawn_cube(
            name='destination_box',
            position=Point(x=0.7, y=0.2, z=0.42),
            color=[0.0, 1.0, 0.0, 0.5],
            size=[0.1, 0.1, 0.02],
            static=True
        )
        
    def spawn_cube(self, name, position, color, size, static=False):
        # Create SDF string for a cube
        sdf = f"""
        <sdf version='1.6'>
            <model name='{name}'>
                <static>{str(static).lower()}</static>
                <link name='link'>
                    <inertial>
                        <mass>0.05</mass>
                        <inertia>
                            <ixx>{0.083 * 0.05 * (size[1]**2 + size[2]**2)}</ixx>
                            <ixy>0</ixy>
                            <ixz>0</ixz>
                            <iyy>{0.083 * 0.05 * (size[0]**2 + size[2]**2)}</iyy>
                            <iyz>0</iyz>
                            <izz>{0.083 * 0.05 * (size[0]**2 + size[1]**2)}</izz>
                        </inertia>
                    </inertial>
                    <collision name='collision'>
                        <geometry>
                            <box>
                                <size>{size[0]} {size[1]} {size[2]}</size>
                            </box>
                        </geometry>
                        <surface>
                            <friction>
                                <ode>
                                    <mu>1.0</mu>
                                    <mu2>1.0</mu2>
                                </ode>
                            </friction>
                            <contact>
                                <ode>
                                    <kp>1000000.0</kp>
                                    <kd>100.0</kd>
                                    <max_vel>1.0</max_vel>
                                    <min_depth>0.001</min_depth>
                                </ode>
                            </contact>
                        </surface>
                    </collision>
                    <visual name='visual'>
                        <geometry>
                            <box>
                                <size>{size[0]} {size[1]} {size[2]}</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient>
                            <diffuse>{color[0]} {color[1]} {color[2]} {color[3]}</diffuse>
                            <specular>0.1 0.1 0.1 1</specular>
                            <emissive>0 0 0 0</emissive>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        """
        
        # Create pose
        pose = Pose()
        pose.position = position
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Create request
        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf
        request.initial_pose = pose
        request.reference_frame = 'world'
        
        # Call service
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Spawned {name} successfully')
        else:
            self.get_logger().error(f'Failed to spawn {name}')
    
    def spawn_cylinder(self, name, position, color, radius, length, static=False):
        # Create SDF string for a cylinder
        sdf = f"""
        <sdf version='1.6'>
            <model name='{name}'>
                <static>{str(static).lower()}</static>
                <link name='link'>
                    <inertial>
                        <mass>0.05</mass>
                        <inertia>
                            <ixx>{0.0833 * 0.05 * (3 * radius**2 + length**2)}</ixx>
                            <ixy>0</ixy>
                            <ixz>0</ixz>
                            <iyy>{0.0833 * 0.05 * (3 * radius**2 + length**2)}</iyy>
                            <iyz>0</iyz>
                            <izz>{0.5 * 0.05 * radius**2}</izz>
                        </inertia>
                    </inertial>
                    <collision name='collision'>
                        <geometry>
                            <cylinder>
                                <radius>{radius}</radius>
                                <length>{length}</length>
                            </cylinder>
                        </geometry>
                        <surface>
                            <friction>
                                <ode>
                                    <mu>1.0</mu>
                                    <mu2>1.0</mu2>
                                </ode>
                            </friction>
                            <contact>
                                <ode>
                                    <kp>1000000.0</kp>
                                    <kd>100.0</kd>
                                    <max_vel>1.0</max_vel>
                                    <min_depth>0.001</min_depth>
                                </ode>
                            </contact>
                        </surface>
                    </collision>
                    <visual name='visual'>
                        <geometry>
                            <cylinder>
                                <radius>{radius}</radius>
                                <length>{length}</length>
                            </cylinder>
                        </geometry>
                        <material>
                            <ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient>
                            <diffuse>{color[0]} {color[1]} {color[2]} {color[3]}</diffuse>
                            <specular>0.1 0.1 0.1 1</specular>
                            <emissive>0 0 0 0</emissive>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        """
        
        # Create pose
        pose = Pose()
        pose.position = position
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Create request
        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf
        request.initial_pose = pose
        request.reference_frame = 'world'
        
        # Call service
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Spawned {name} successfully')
        else:
            self.get_logger().error(f'Failed to spawn {name}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSpawner()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
