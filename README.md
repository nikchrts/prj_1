# Robotics Cource - Politecnico di Milano

## Goals
- Two different odometry methods: 
  1. Differential Drive Kinematics
  2. Ackerman model
- Publish them in two ways:
  1. tf
  2. odom topic
- Use dynamic configuration for:
  1. switch between different published odometry (Differential vs Ackerman)
  2. reset odometry to (0,0) or set to a specific (x,y) starting point
- Publish a custom message with odometry value and type of source

## Files
- **bag_class.cpp**:
  Includes a class which retrieves the data from the bag in a synchronous way using message_filters' ApproximateTime policy. Using a dynamic configurable parameter, the odometry method is selected and the corresponding parameters (ω, v and R) are computed. Finally, these parameters are published to a topic via custom message.
- **pub_odom.cpp**:
Using Euler integration, the pose (x, y, θ) is computed for every time instant. A tf transformation is then used between the frames "world" and "car".

## How to use it
- place all the files in the relative direcotry (catki workspace)/src/prj_bag
- open the terminal and run in different windows/tabs:
  - `roscore`
  - `rosbag play -l (path to bag_1.bag/bag_2.bag)`
  - `rosrun prj_bag bag_class`
  - `rosrun prj_bag pub_odom`
  - to view the tf: `rviz` and select 'add' to add a TF and change the 'fixed frame' (eg to 'world')
- parameters can be changed dynamically using:
  `rosrun dynamic_reconfigure dynparam set /(node name) (value)`
  and if you want to change many simultaneously
  `rosrun dynamic_reconfigure dynparam set /(node name) {(name1):value1, (name2):value2}`
