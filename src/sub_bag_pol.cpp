#include "ros/ros.h"
#include "prj_bag/floatStamped.h"
#include "prj_bag/bagData.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <prj_bag/odom_typeConfig.h>
#include "std_msgs/String.h"
#include <stdlib.h>

void callback(const prj_bag::floatStamped::ConstPtr& vL, const prj_bag::floatStamped::ConstPtr& vR, const prj_bag::floatStamped::ConstPtr& theta){
    float omega, speed, radius;

    omega = (vR->data-vL->data)/1.3;
    speed = (vR->data+vL->data)/2;
    radius = 0.65*(vR->data+vL->data)/(vR->data-vL->data);
    // There is a steering factor of 18 -> msg3.data contains the steeering wheel angle, whereas we need the wheels angle
    // ROS_INFO ("Wheels Data   ---   V_l:  %f,   V_r:  %f,   Angle:  %f", vL->data, vR->data, theta->data/18);
    // ROS_INFO ("Wheels Data   ---   Omega:  %f,   Speed:  %f,   Radius:  %f", omega, speed, radius);
}

void callback1(prj_bag::odom_typeConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d", config.type);
    // ROS_INFO ("%d",level);
    if (config.type == 0){
        ROS_INFO("Differential is selected");
    }
    else{
        ROS_INFO("Ackerman is selected");
    }
}


int main(int argc, char** argv)
{
    // Initialize the node
    ros::init(argc, argv, "subscriber_sync");

    // Define variables to use
    ros::NodeHandle n;

    message_filters::Subscriber<prj_bag::floatStamped> sub1(n, "speedL_stamped", 1);
    message_filters::Subscriber<prj_bag::floatStamped> sub2(n, "speedR_stamped", 1);
    message_filters::Subscriber<prj_bag::floatStamped> sub3(n, "steer_stamped", 1);

    dynamic_reconfigure::Server<prj_bag::odom_typeConfig> server;
    dynamic_reconfigure::Server<prj_bag::odom_typeConfig>::CallbackType f;
    
    typedef message_filters::sync_policies::ApproximateTime<prj_bag::floatStamped, prj_bag::floatStamped, prj_bag::floatStamped> MySyncPolicy;
    
    // Create the synchronizer for the bag data (+ callback)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    // Create the callback for the dynamic configuration
    f = boost::bind(&callback1, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}