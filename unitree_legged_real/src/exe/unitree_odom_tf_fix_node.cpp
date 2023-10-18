#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

static ros::Publisher pub;
void callback(const nav_msgs::Odometry &odom)
{
    nav_msgs::Odometry odom_out = odom;
    odom_out.child_frame_id = "base_link";

    pub.publish(odom_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_odom_tf_fix_node");
    ros::NodeHandle node;

    pub = node.advertise<nav_msgs::Odometry>("odom_fixed", 10);
    ros::Subscriber fix_sub = node.subscribe("odom", 10, callback);

    ros::spin();
}
