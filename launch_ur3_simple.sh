#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Kill any existing processes
pkill -f "rviz\|robot_state_publisher\|move_robot.py" || true

# Start the robot state publisher with the simple URDF
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ur3_simple.urdf.xacro)" &
ROBOT_STATE_PUBLISHER_PID=$!

# Wait for the robot state publisher to start
sleep 2

# Start the robot mover
python3.12 move_robot.py &
ROBOT_MOVER_PID=$!

# Wait for the robot mover to start
sleep 2

# Start RViz2
rviz2 -d simple_config.rviz &
RVIZ_PID=$!

# Wait for RViz2 to start
sleep 2

echo "All components started successfully!"
echo "Robot State Publisher PID: $ROBOT_STATE_PUBLISHER_PID"
echo "Robot Mover PID: $ROBOT_MOVER_PID"
echo "RViz2 PID: $RVIZ_PID"

# Wait for user to press Ctrl+C
echo "Press Ctrl+C to stop all components"
wait $RVIZ_PID

# Kill all components
kill $ROBOT_STATE_PUBLISHER_PID $ROBOT_MOVER_PID $RVIZ_PID || true
