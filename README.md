# Custom DWA Local Planner (ROS2 Humble)

A simple custom Dynamic Window Approach (DWA) local planner for TurtleBot3 running in ROS2 Humble. This node reads odometry, laser scan, and a goal from RViz, then publishes `/cmd_vel` commands based on sampled trajectories.

## Prerequisites

- ROS2 Humble
- TurtleBot3 packages
- Gazebo
- RViz2

## Setup

### 1. Create or use a ROS2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the repository

```bash
git clone https://github.com/saumyaezhil/Custom-DWA-Local-Planner.git
```

### 3. Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select dwa_local_planner --symlink-install
```

### 4. Source the workspace

```bash
source install/setup.bash
```

## Running

### 1. Launch TurtleBot3 in Gazebo

Open a terminal and run:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Start the DWA planner

Open a new terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run dwa_local_planner simple_dwa
```

### 3. Open RViz

Open another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2
```

In RViz, configure the following:

1. **Set Fixed Frame** → `odom`
2. **Add displays:**
   - LaserScan
   - Odometry
   - TF
   - MarkerArray (topic: `/dwa_trajectories`)
3. **Use 2D Nav Goal** tool to send a goal to the robot

The robot should begin to move toward the selected goal while avoiding obstacles.

## Features

- Dynamic Window Approach for local path planning
- Real-time obstacle avoidance using laser scan data
- Trajectory visualization in RViz
- Interactive goal setting via RViz

