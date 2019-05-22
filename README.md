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
- **sub_bag_pol.cpp**:
Initial version of the aforementioned implementation without a class. DEPRECATED, used only as reference! 
