#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "prj_1/floatStamped.h"
#include "prj_1/odomCustom.h"
#include <prj_1/odom_paramConfig.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class Node{
    public:
        Node(){
            x = 0;
            y = 0;
            th = 0;
            type_id = 0;
            // subscribe to bag topics and synchronize them
            sub1.subscribe(node, "speedL_stamped", 1);
            sub2.subscribe(node, "speedR_stamped", 1);
            sub3.subscribe(node, "steer_stamped", 1);
            sync.reset(new Sync(MySyncPolicy(10), sub1, sub2, sub3));      
            sync->registerCallback(boost::bind(&Node::SyncCallback, this, _1, _2, _3));
            
            // set up dynamic configuration
            f_dyn = boost::bind(&Node::dynamicCallback, this, _1, _2);
            server.setCallback(f_dyn);

            // publish back
            pub_odom = node.advertise<nav_msgs::Odometry>("nav_odom", 1);
            pub_custom = node.advertise<prj_1::odomCustom>("custom_odom", 1);

            current_time = ros::Time::now();
            last_time = ros::Time::now();
        }

    void SyncCallback(const prj_1::floatStamped::ConstPtr& vL, const prj_1::floatStamped::ConstPtr& vR, const prj_1::floatStamped::ConstPtr& theta){
        th_rad = theta->data/18*3.14159/180;  // from degrees to rad
        if (type_id == 0){
            omega = (vR->data-vL->data)/1.3;
            velocity = (vR->data+vL->data)/2;
            ROS_INFO("Diff");
        }
        else{
            radius = 1.765/tan(th_rad);
            omega = (vR->data+vL->data)/2/radius;
            velocity = omega*radius;
            ROS_INFO("Acker");
        }

        Node::odometry(velocity, omega);       
    }   

    void odometry(double velocity, double omega){
        // compute pose parameters
        vx = velocity*cos(th);
        vy = velocity*sin(th);

        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();
        x += vx*dt;
        y += vy*dt;
        th += omega*dt;

        // create the tf transformation
        geom_trans.header.stamp = current_time;
        geom_trans.header.frame_id = "world";
        geom_trans.child_frame_id = "car";

        geom_q = tf::createQuaternionMsgFromYaw(th);
        geom_trans.transform.translation.x = x;
        geom_trans.transform.translation.y = y;
        geom_trans.transform.translation.z = 0.0;
        geom_trans.transform.rotation = geom_q;

        //send the transform
        br.sendTransform(geom_trans);

        // publish nav_msgs/Odometry topic
        odom.header.stamp = current_time;
        odom.header.frame_id = "world";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = geom_q;

        odom.child_frame_id = "car";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = omega;

        pub_odom.publish(odom);

        // publish custom topic
        custom_msg.header.stamp = current_time;
        custom_msg.header.frame_id = "car";
        custom_msg.x = x;
        custom_msg.y = y;
        custom_msg.yaw = th;
        if (type_id == 0){
            custom_msg.type = "Differential Drive Model";
        }
        else if (type_id == 1){
            custom_msg.type = "Ackerman Model";
        }

        pub_custom.publish(custom_msg);

        last_time = current_time; 
    }

    void dynamicCallback(prj_1::odom_paramConfig &config, uint32_t level) {
        if (level == 0){
            if (config.reset == 1){
                ROS_INFO("Reset car to position (0,0)");
                x = 0;
                y = 0;
                config.reset = 0;
            }
        }
        else if (level == 1){
            if (config.type == 0){
                type_id = 0;
                ROS_INFO("Differential Drive Model is selected");
            }
            else{
                type_id = 1;
                ROS_INFO("Ackerman Model is selected");
            }
        }
        else if (level == 2){
            ROS_INFO("The car was placed at the position (%f,%f)", config.x, config.y);
            x = config.x;
            y = config.y;
            config.x = 0;
            config.y = 0;
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

        // dynamic configuration
        dynamic_reconfigure::Server<prj_1::odom_paramConfig> server;
        dynamic_reconfigure::Server<prj_1::odom_paramConfig>::CallbackType f_dyn;

        // compute and publish odometry
        tf::TransformBroadcaster br;
        nav_msgs::Odometry odom;
        geometry_msgs::Quaternion geom_q;
        geometry_msgs::TransformStamped geom_trans;
        ros::Publisher pub_odom;
        ros::Time current_time, last_time;

        // custom  message type
        ros::Publisher pub_custom;
        prj_1::odomCustom custom_msg;
        
        // odometry parameters
        double omega, velocity, radius, dt, th_rad, vx, vy;
        int type_id;
        double x, y, th;
    };

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_car");
    Node synchronizer;
    ros::spin();
}