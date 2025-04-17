#!/bin/bash

# Script to run the UR3 pick and place simulation
# This script launches RViz, robot state publisher, and the robot mover script

# Set the workspace path
WORKSPACE_PATH="/home/lachu/ros2_workspaces/ros2_ws"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to display colored text
print_color() {
    COLOR=$1
    TEXT=$2
    echo -e "\033[${COLOR}m${TEXT}\033[0m"
}

# Print header
print_color "1;34" "=== UR3 Pick and Place Simulation ==="
print_color "1;34" "=== Starting all components... ==="
echo ""

# Source ROS setup
print_color "1;33" "Sourcing ROS setup..."
source /opt/ros/jazzy/setup.bash

# Check if we're in the right directory, if not, change to it
if [ "$(pwd)" != "$WORKSPACE_PATH" ]; then
    print_color "1;33" "Changing to workspace directory: $WORKSPACE_PATH"
    cd $WORKSPACE_PATH
fi

# Check if Python 3.12 is available
if command_exists python3.12; then
    PYTHON_CMD="python3.12"
else
    print_color "1;31" "Python 3.12 not found, trying python3..."
    PYTHON_CMD="python3"
fi

# Launch RViz in the background
print_color "1;32" "Starting RViz..."
ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/view_robot.rviz &
RVIZ_PID=$!
sleep 2  # Give RViz time to start

# Launch robot state publisher in the background
print_color "1;32" "Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=ur3_robot ur_type:=ur3 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)" &
RSP_PID=$!
sleep 2  # Give robot state publisher time to start

# Launch the robot mover script
print_color "1;32" "Starting robot mover script..."
$PYTHON_CMD src/ur3_pick_place/scripts/move_robot.py &
MOVER_PID=$!

# Print information about the running processes
echo ""
print_color "1;36" "All components started successfully!"
print_color "1;36" "RViz PID: $RVIZ_PID"
print_color "1;36" "Robot State Publisher PID: $RSP_PID"
print_color "1;36" "Robot Mover PID: $MOVER_PID"
echo ""
print_color "1;33" "Press Ctrl+C to stop all components"

# Wait for user to press Ctrl+C
trap 'print_color "1;31" "Stopping all components..."; kill $RVIZ_PID $RSP_PID $MOVER_PID 2>/dev/null; exit' INT
wait $MOVER_PID

# If the robot mover script exits, kill the other processes
print_color "1;31" "Robot mover script exited, stopping other components..."
kill $RVIZ_PID $RSP_PID 2>/dev/null

print_color "1;34" "=== Simulation ended ==="
