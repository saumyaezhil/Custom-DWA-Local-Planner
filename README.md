# Custom DWA Local Planner (ROS2 Humble)

This project contains a simple, custom implementation of the Dynamic Window Approach (DWA) local planner for TurtleBot3 in ROS2 Humble.  
The planner reads odometry, laser scan data, and a goal pose, then generates a safe velocity command based on a sampled set of trajectories.

The goal of this project is to show how a basic local planner can be built from scratch without using Nav2 or the default DWB controller.

---

## Setup Instructions

### 1. Create a ROS2 workspace (if you don’t already have one)
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

2. Clone this repository

git clone https://github.com/saumyaezhil/Custom-DWA-Local-Planner.git

3. Build the package

cd ~/ros2_ws
colcon build --packages-select dwa_local_planner --symlink-install

4. Source the workspace

source install/setup.bash

Running the Project
1. Launch Gazebo with TurtleBot3

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2. Run the DWA local planner

Open a new terminal:

cd ~/ros2_ws
source install/setup.bash
ros2 run dwa_local_planner simple_dwa

You should see:

DWA planner node started
Received new goal

3. Open RViz

source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2

In RViz:

    Set Fixed Frame → odom

    Add the following displays:

        LaserScan (/scan)

        Odometry (/odom)

        TF

        MarkerArray (/dwa_trajectories)

    Use 2D Nav Goal to click on a point in the world

The robot should begin moving toward the goal.
Troubleshooting
Nothing is moving

Check if the planner is publishing velocity commands:

ros2 topic echo /cmd_vel

You should see values like:

linear:
  x: 0.12
angular:
  z: -0.20

If angular.z changes but linear.x stays at 0.0:

    The planner thinks obstacles are too close → increase obstacle tolerance or reduce obstacle cost

    Increase min_vel_x slightly (e.g., 0.05)

No topics visible in ros2 topic list

Run:

ros2 topic list --no-daemon

Restart ROS daemon if needed:

ros2 daemon stop
ros2 daemon start

RViz shows nothing

Make sure you added:

    LaserScan → /scan

    MarkerArray → /dwa_trajectories

    Fixed Frame = odom

Gazebo robot moves by itself

Kill leftover processes:

pkill -9 gzserver
pkill -9 gzclient
pkill -9 ros2

How the DWA Planner Works (Simple Explanation)

The Dynamic Window Approach is a local planning method used to generate safe, real-time velocity commands for mobile robots.

This implementation follows these steps:
1. Sample Velocities

The planner tests a range of:

    Linear velocities (v)

    Angular velocities (ω)

within the robot’s acceleration and velocity limits.
2. Predict Trajectories

For each (v, ω) pair, the robot’s motion is simulated for a short time window.
This creates many possible future paths.
3. Score Each Trajectory

Each predicted path is evaluated based on:

    Distance to goal

    Obstacle clearance

    How fast it is

    Smoothness (difference from current velocity)

Each factor contributes to a final cost.
4. Pick the Best One

The trajectory with the lowest cost wins.
Its velocity command is published to /cmd_vel.
5. Visualize in RViz

All sampled trajectories are published as markers.
The best one is highlighted.

This gives a simple but effective obstacle-avoiding local planner.
Folder Structure

dwa_local_planner/
  ├── dwa_local_planner/
  │     └── dwa_local_planner_node.py
  ├── package.xml
  ├── setup.py
  ├── setup.cfg
  └── resource/
