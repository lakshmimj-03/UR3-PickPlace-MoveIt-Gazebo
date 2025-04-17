# Detailed Explanation of Shell Scripts

This document provides a comprehensive explanation of the shell scripts used to launch the UR3 robot pick and place simulation.

## Overview

Two shell scripts are provided to simplify the launching of the simulation:

1. `run_simulation.sh`: Launches all components in separate terminals
2. `run_simulation_tmux.sh`: Launches all components in a single terminal using tmux

## `run_simulation.sh`

The `run_simulation.sh` script launches all necessary components for the simulation in separate processes.

### Script Content

```bash
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
```

### Key Components

1. **Workspace Path**: Sets the path to the ROS 2 workspace
2. **Helper Functions**:
   - `command_exists`: Checks if a command is available
   - `print_color`: Prints colored text for better readability
3. **Environment Setup**:
   - Sources the ROS 2 setup script
   - Changes to the workspace directory if needed
   - Determines the Python command to use
4. **Component Launch**:
   - Launches RViz with the appropriate configuration
   - Launches the robot state publisher with the UR3 URDF
   - Launches the robot mover script
5. **Process Management**:
   - Stores the process IDs of each component
   - Sets up a trap to handle Ctrl+C
   - Waits for the robot mover script to exit
   - Kills all processes when the script exits

### Usage

To use the script:

```bash
cd /home/lachu/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation.sh
```

Press Ctrl+C to stop all components.

## `run_simulation_tmux.sh`

The `run_simulation_tmux.sh` script launches all necessary components for the simulation in a tmux session.

### Script Content

```bash
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
```

### Key Components

1. **Workspace Path**: Sets the path to the ROS 2 workspace
2. **Helper Functions**:
   - `command_exists`: Checks if a command is available
3. **Environment Setup**:
   - Checks if tmux is installed
   - Changes to the workspace directory if needed
   - Determines the Python command to use
   - Sources the ROS 2 setup script
4. **Tmux Session Setup**:
   - Creates a new tmux session
   - Splits the window into three panes
5. **Component Launch**:
   - Runs RViz in the first pane
   - Runs the robot state publisher in the second pane
   - Runs the robot mover script in the third pane
6. **Session Management**:
   - Attaches to the tmux session
   - Kills the session when the user detaches

### Usage

To use the script:

```bash
cd /home/lachu/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation_tmux.sh
```

Press Ctrl+B, then D to detach from the tmux session (this will also stop all components).

## Comparison

### `run_simulation.sh`

**Advantages**:
- Does not require tmux to be installed
- Simpler to understand for users not familiar with tmux
- Provides colored output for better readability

**Disadvantages**:
- Launches components in separate terminals
- May be harder to manage multiple terminal windows

### `run_simulation_tmux.sh`

**Advantages**:
- Launches all components in a single terminal window
- Provides a clean, organized view of all component outputs
- Easier to manage a single terminal window

**Disadvantages**:
- Requires tmux to be installed
- May be harder to use for users not familiar with tmux

## Customization

### Modifying the Workspace Path

If your workspace is in a different location, modify the `WORKSPACE_PATH` variable in both scripts:

```bash
# Set the workspace path
WORKSPACE_PATH="/path/to/your/workspace"
```

### Using a Different Python Version

The scripts automatically check for Python 3.12 and fall back to Python 3 if not found. If you want to use a specific Python version, modify the Python command check:

```bash
# Check if Python 3.12 is available
if command_exists python3.10; then  # Change to your preferred version
    PYTHON_CMD="python3.10"
else
    print_color "1;31" "Python 3.10 not found, trying python3..."
    PYTHON_CMD="python3"
fi
```

### Changing the RViz Configuration

To use a different RViz configuration, modify the RViz command:

```bash
# Launch RViz in the background
print_color "1;32" "Starting RViz..."
ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/your_custom_config.rviz &
```

### Modifying the Robot Description

To use a different robot description or parameters, modify the robot state publisher command:

```bash
# Launch robot state publisher in the background
print_color "1;32" "Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=your_robot ur_type:=ur5 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)" &
```

## Conclusion

These shell scripts provide convenient ways to launch the UR3 robot pick and place simulation. The `run_simulation.sh` script is simpler and does not require additional software, while the `run_simulation_tmux.sh` script provides a more organized view of all component outputs in a single terminal window.

Both scripts handle the necessary setup and launch all required components for the simulation, making it easy to start and stop the simulation with a single command.
