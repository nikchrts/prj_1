#include "ros/ros.h"
#include "prj_bag/floatStamped.h"
#include "prj_bag/odomParam.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <prj_bag/odom_typeConfig.h>
#include "std_msgs/Float64.h"

class Node{
    public:
        Node(){
            // subscribe to bag topics and synchronize them
            sub1.subscribe(node, "speedL_stamped", 1);
            sub2.subscribe(node, "speedR_stamped", 1);
            sub3.subscribe(node, "steer_stamped", 1);
            sync.reset(new Sync(MySyncPolicy(10), sub1, sub2, sub3));      
            sync->registerCallback(boost::bind(&Node::callback, this, _1, _2, _3));
            // set up dynamic configuration
            f = boost::bind(&Node::callback1, this, _1, _2);
            server.setCallback(f);
            // publish back
            pub = node.advertise<prj_bag::odomParam>("rechatter", 1);
            // timer1 = node.createTimer(ros::Duration(1), &Node::callback2, this);
        }

        void callback(const prj_bag::floatStamped::ConstPtr& vL, const prj_bag::floatStamped::ConstPtr& vR, const prj_bag::floatStamped::ConstPtr& theta){
            // ROS_INFO("Data:   %f  %f  %f", vL->data, vR->data, theta->data/18);
            if (type == 0){
                omega = (vR->data-vL->data)/1.3;
                speed = (vR->data+vL->data)/2;
                radius = 0.65*(vR->data+vL->data)/(vR->data-vL->data);
                // ROS_INFO("%s", typeid(vR->data).name());
                ROS_INFO("Differential:  %f  |  %f  |  %f", omega, speed, radius);
            }
            else{
                omega = 0;
                speed = 0;
                radius = 0;
                ROS_INFO("Ackerman:   --  |  --  |  --");
            }  
            // the .data of a std_msgs::Float64 is a double number
            msg.type = 0;
            msg.omega = omega;
            msg.velocity = speed;
            msg.radius = radius;
            pub.publish(msg);          
        }   

        void callback1(prj_bag::odom_typeConfig &config, uint32_t level) {
            ROS_INFO("Reconfigure Request: %d", config.type);
            type = config.type;
            if (config.type == 0){
                ROS_INFO("Differential is selected");
            }
            else{
                ROS_INFO("Ackerman is selected");
            }
        }

    private:
        // node handler
        ros::NodeHandle node;
        // synchronized bag data retrieval
        message_filters::Subscriber<prj_bag::floatStamped> sub1, sub2, sub3;
        typedef message_filters::sync_policies::ApproximateTime<prj_bag::floatStamped, prj_bag::floatStamped, prj_bag::floatStamped> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync;
        // publish back data
        ros::Publisher pub;
        ros::Timer timer1;
        // dynamic configuration
        dynamic_reconfigure::Server<prj_bag::odom_typeConfig> server;
        dynamic_reconfigure::Server<prj_bag::odom_typeConfig>::CallbackType f;
        // odometry parameters
        int type;
        double omega, speed, radius;
        prj_bag::odomParam msg;
    };

int main(int argc, char **argv){
    ros::init(argc, argv, "synchronizer");
    Node synchronizer;
    ros::spin();
}