#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "prj_bag/odomParam.h"
// #include <prj_bag/odom_typeConfig.h>
#include <dynamic_reconfigure/server.h>

class odom_sub_pub{
    public:
        odom_sub_pub(){
            current_time = ros::Time::now();
            last_time = ros::Time::now();
            sub = n.subscribe("odomParam", 1000, &odom_sub_pub::odomCallback, this);

            // set up dynamic configuration
            // f = boost::bind(&odom_sub_pub::resetCallback, this, _1, _2);
            // server.setCallback(f);
        }

    void odomCallback(const prj_bag::odomParam::ConstPtr& msg){
        ome = msg->omega;
        vel = msg->velocity;
        th = msg->theta;

        current_time = ros::Time::now();

        dt = (current_time - last_time).toSec();
        x += vel*dt*cos(th);
        y += vel*dt*sin(th);
        th += ome*dt;

        //ROS_INFO("%f   %f   %f", x, y, th);

        transform.setOrigin(tf::Vector3(x, y, 0));
        q.setRPY(0, 0, th);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "car"));

        last_time = current_time;
    }

    // void resetCallback(prj_bag::odom_typeConfig &config, uint32_t level) {
    //     // reset = config.reset;
    //     ROS_INFO("%d", config.reset);
    //     if (config.reset == 0){
    //         ROS_INFO("Got in");
    //         config.reset = 2;
    //     }
    // }

    private:
        ros::NodeHandle n;
        ros::Subscriber sub;

        tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;

        ros::Time current_time, last_time;

        // dynamic_reconfigure::Server<prj_bag::odom_typeConfig> server;
        // dynamic_reconfigure::Server<prj_bag::odom_typeConfig>::CallbackType f;

        // initial conditions
        double x = 0;
        double y = 0;
        double th = 0;

        double ome, vel;
        double dt, d_x, d_y, d_th;
};


int main(int argc, char **argv){
    ros::init(argc, argv, "pub_odom");
    odom_sub_pub pub_odom;
    ros::spin();
    return 0;
}