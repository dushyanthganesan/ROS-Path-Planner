# ROS Path Planner (in Progress)
This is a ROS-based control system and simulator for a point-following robot. This architecture has a vast number of use cases in autonomous systems and robotics. The base path planning algorithm is the A-star algorithm used in conjunction with a trajectory optimizer. Visualization is done in RViz. Developed on Ubuntu 18.04.

Requirements:
ROS Melodic

install dependencies:
```
sudo apt install ros-melodic-navigation
```

To install project:
```
cd {WORKSPACE}
git clone https://github.com/dushyanthganesan/ROS-Path-Planner.git
catkin init
catkin build
```

Running the project:
```
roslaunch point_follower follower.launch
rosrun rviz rviz -d rviz/visualization.rviz
```