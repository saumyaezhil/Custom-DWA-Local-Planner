Custom DWA Local Planner (ROS2 Humble)

This project contains a basic implementation of the Dynamic Window Approach (DWA) local planner for TurtleBot3 in ROS2 Humble.
The planner runs independently (without Nav2) and computes /cmd_vel based on odometry, laser scan data, and a goal clicked in RViz.

Setup Instructions
1. Create or use a ROS2 workspace

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

2. Clone the repository

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

2. Run the DWA planner

(open a new terminal)
cd ~/ros2_ws
source install/setup.bash
ros2 run dwa_local_planner simple_dwa

3. Open RViz

source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2

In RViz:

Set Fixed Frame → odom

Add:

LaserScan → /scan

Odometry → /odom

TF

MarkerArray → /dwa_trajectories

Use 2D Nav Goal to send a goal

The robot should start moving toward the clicked point.

Troubleshooting
Robot does not move

Check if velocity commands are being published:
ros2 topic echo /cmd_vel

If only angular.z changes:

Increase min_vel_x

Reduce obstacle cost

Increase goal tolerance

No ROS2 topics appear

ros2 topic list --no-daemon

If still empty:
ros2 daemon stop
ros2 daemon start

RViz shows nothing

Fixed Frame must be odom

Ensure /scan, /odom, /dwa_trajectories are active

Robot moves by itself

Kill leftover processes:
pkill -9 gzserver
pkill -9 gzclient
pkill -9 ros2

How the DWA Planner Works (Short Overview)
1. Velocity Sampling

The node samples several forward speeds (v) and rotation speeds (w).

2. Predicting Motion

Each (v, w) pair is simulated over a short time window.

3. Scoring Paths

Each path is evaluated based on:

Distance to goal

Obstacle clearance

Speed

Smoothness

4. Choosing a Command

The best-cost trajectory is selected and published on /cmd_vel.

5. Visualizing

All sampled trajectories appear as red lines in RViz.
The chosen trajectory is green.

Folder Structure

dwa_local_planner/
├── dwa_local_planner/
│ └── dwa_local_planner_node.py
├── package.xml
├── setup.py
├── setup.cfg
└── resource/
