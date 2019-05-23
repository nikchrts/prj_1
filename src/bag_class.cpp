#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "prj_1/floatStamped.h"
#include "prj_1/odomParam.h"
#include <prj_1/odom_typeConfig.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

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
            f = boost::bind(&Node::typeCallback, this, _1, _2);
            server.setCallback(f);
            
            // publish back
            pub = node.advertise<prj_1::odomParam>("odomParam", 1);
        }

    void callback(const prj_1::floatStamped::ConstPtr& vL, const prj_1::floatStamped::ConstPtr& vR, const prj_1::floatStamped::ConstPtr& theta){
        msg.theta = theta->data/18*3.14159/180;  // from degrees to rad
        if (type == 0){
            omega = (vR->data-vL->data)/1.3;
            velocity = (vR->data+vL->data)/2;
        }
        else{
            radius = 1.765/tan(msg.theta);
            omega = (vR->data+vL->data)/2/radius;
            velocity = omega*radius;
        }

        msg.omega = omega;
        msg.velocity = velocity;
        pub.publish(msg);          
    }   

    void typeCallback(prj_1::odom_typeConfig &config, uint32_t level) {
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
        message_filters::Subscriber<prj_1::floatStamped> sub1, sub2, sub3;
        typedef message_filters::sync_policies::ApproximateTime<prj_1::floatStamped, prj_1::floatStamped, prj_1::floatStamped> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync;

        // publish back data
        ros::Publisher pub;
        ros::Timer timer1;

        // dynamic configuration
        dynamic_reconfigure::Server<prj_1::odom_typeConfig> server;
        dynamic_reconfigure::Server<prj_1::odom_typeConfig>::CallbackType f;
        
        // odometry parameters
        int type;
        double omega, velocity, radius;
        prj_1::odomParam msg;
    };

int main(int argc, char **argv){
    ros::init(argc, argv, "bag_retrieval");
    Node synchronizer;
    ros::spin();
}