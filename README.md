# Crazyflie ROS2 Experiments

ROS2 workspace for autonomous drone experiments using Crazyflie 2.x with Vicon motion capture system.

## Features

- **Keyboard Teleoperation**: Manual control via keyboard
- **Autonomous Takeoff/Landing**: Scripted flight operations
- **Circle Flight**: Circular trajectory following
- **Jackal Tracking**: Hover above ground robot
- **Circular Tracking**: Follow Jackal during circular motion

## Hardware Requirements

- Crazyflie 2.x with Crazyradio PA
- Vicon motion capture system
- Jackal UGV (for tracking experiments)
- Ubuntu 22.04 with ROS2 Humble

## Installation

See [SETUP.md](docs/SETUP.md) for detailed installation instructions.

### Quick Start
```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/crazyflie_ros2_experiments.git
cd crazyflie_ros2_experiments

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash

# Launch Crazyswarm2
ros2 launch crazyflie launch.py motion_capture:=vicon
```

## Usage

### Run Individual Experiments
```bash
# Takeoff
ros2 run experiments takeoff_node --ros-args -p height:=1.0

# Land
ros2 run experiments land_node

# Circle flight
ros2 run experiments circle_flight_node

# Hover above Jackal
ros2 run experiments hover_above_jackal

# Circular tracking
ros2 run experiments circular_tracking_node
```

### Run All Experiments
```bash
cd src/experiments/scripts
./run_all_experiments.sh
```

## Project Structure
