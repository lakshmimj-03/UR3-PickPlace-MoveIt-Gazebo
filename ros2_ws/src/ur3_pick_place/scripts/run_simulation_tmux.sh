#!/bin/bash

# Script to run the UR3 pick and place simulation using tmux
# This script launches RViz, robot state publisher, and the robot mover script in a tmux session

# Set the workspace path
WORKSPACE_PATH="/home/lachu/ros2_workspaces/ros2_ws"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if tmux is installed
if ! command_exists tmux; then
    echo "tmux is not installed. Please install it with: sudo apt-get install tmux"
    echo "Alternatively, use the regular script: ./run_simulation.sh"
    exit 1
fi

# Check if we're in the right directory, if not, change to it
if [ "$(pwd)" != "$WORKSPACE_PATH" ]; then
    echo "Changing to workspace directory: $WORKSPACE_PATH"
    cd $WORKSPACE_PATH
fi

# Check if Python 3.12 is available
if command_exists python3.12; then
    PYTHON_CMD="python3.12"
else
    echo "Python 3.12 not found, trying python3..."
    PYTHON_CMD="python3"
fi

# Source ROS setup
source /opt/ros/jazzy/setup.bash

# Create a new tmux session
SESSION_NAME="ur3_simulation"

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create a new session
tmux new-session -d -s $SESSION_NAME -n "UR3 Simulation"

# Split the window into three panes
tmux split-window -h -t $SESSION_NAME
tmux split-window -v -t $SESSION_NAME:0.0

# Run RViz in the first pane
tmux send-keys -t $SESSION_NAME:0.0 "echo 'Starting RViz...'; ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/view_robot.rviz" C-m

# Run robot state publisher in the second pane
tmux send-keys -t $SESSION_NAME:0.1 "echo 'Starting robot state publisher...'; sleep 2; ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(xacro \$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=ur3_robot ur_type:=ur3 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)\"" C-m

# Run the robot mover script in the third pane
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Starting robot mover script...'; sleep 4; $PYTHON_CMD src/ur3_pick_place/scripts/move_robot.py" C-m

# Attach to the session
tmux attach-session -t $SESSION_NAME

# When the user detaches from the session (Ctrl+B, D), kill the session
tmux kill-session -t $SESSION_NAME 2>/dev/null
