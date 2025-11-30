# Custom DWA Local Planner (ROS2 Humble)

A custom Dynamic Window Approach (DWA) local planner implemented from scratch for TurtleBot3 in ROS2 Humble. This planner navigates the robot to goals while avoiding obstacles using velocity sampling, trajectory prediction, and cost-based evaluation.

## Implementation Details

- **Velocity Sampling**: Samples velocity commands within the robot's dynamic constraints
- **Trajectory Prediction**: Simulates forward trajectories for each velocity sample
- **Cost Function**: Evaluates trajectories based on distance to goal, obstacle avoidance, and path smoothness
- **ROS2 Integration**: Subscribes to `/odom` and `/scan`, publishes `/cmd_vel`, visualizes trajectories in RViz

## Setup

### 1. Create a ROS2 workspace

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
source install/setup.bash
```

## Running

### Terminal 1: Launch TurtleBot3 in Gazebo

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2: Start the DWA planner

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run dwa_local_planner simple_dwa
```

### Terminal 3: Open RViz

```bash
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2
```

**RViz Configuration:**
- Set Fixed Frame to `odom`
- Add: LaserScan, Odometry, TF, MarkerArray (`/dwa_trajectories`)
- Use 2D Nav Goal tool to send navigation goals
