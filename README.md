# Custom DWA Local Planner (ROS2 Humble)

This project contains a custom Dynamic Window Approach (DWA) local planner for TurtleBot3 running in ROS2 Humble.  
The planner uses odometry, laser scan data, and a clicked goal pose to generate safe velocity commands.  
Everything is implemented manually without Nav2 or the default DWB controller.

---

## ## Setup Instructions

### ### 1. Workspace Setup
If you don’t already have a ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

### 2. Clone the Repository

git clone https://github.com/saumyaezhil/Custom-DWA-Local-Planner.git

### 3. Build the Package

cd ~/ros2_ws
colcon build --packages-select dwa_local_planner --symlink-install

### 4. Source the Workspace

source install/setup.bash

## Running the Project
### 1. Start Gazebo with TurtleBot3

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### 2. Run the DWA Planner Node

Open a new terminal:

cd ~/ros2_ws
source install/setup.bash
ros2 run dwa_local_planner simple_dwa

### 3. Open RViz

source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2

In RViz:

    Set Fixed Frame → odom

    Add the following:

        LaserScan → /scan

        Odometry → /odom

        TF

        MarkerArray → /dwa_trajectories

    Use 2D Nav Goal to click a goal on the map
    The robot should start moving toward it.

## Troubleshooting
### Robot not moving

Check if the planner is publishing:

ros2 topic echo /cmd_vel

If only angular.z changes and linear.x stays at 0.0:

    Increase min_vel_x

    Lower obstacle cost

    Raise goal tolerance

### No ROS2 topics showing

ros2 topic list --no-daemon
ros2 daemon stop
ros2 daemon start

### RViz showing nothing

Make sure:

    Fixed Frame = odom

    Topics match:

        /scan

        /odom

        /dwa_trajectories

    TF is enabled

### Robot moving by itself

Kill leftover processes:

pkill -9 gzserver
pkill -9 gzclient
pkill -9 ros2

## How the DWA Planner Works (Simple Overview)
### 1. Velocity Sampling

The planner samples several:

    Linear velocities (v)

    Angular velocities (ω)

within the robot’s allowed ranges.
### 2. Predicting Trajectories

Each (v, ω) pair is simulated over a short time window.
This produces many possible future paths.
### 3. Scoring Each Trajectory

Each path is evaluated based on:

    Distance to goal

    Distance from obstacles

    How fast it is

    Smoothness of change

### 4. Choosing the Best Path

The trajectory with the lowest cost is selected,
and its (v, ω) is published to /cmd_vel.
### 5. Visualizing in RViz

All candidate trajectories are shown as red lines.
The best one is shown in green.
## Folder Structure

dwa_local_planner/
  ├── dwa_local_planner/
  │     └── dwa_local_planner_node.py
  ├── package.xml
  ├── setup.py
  ├── setup.cfg
  └── resource/

