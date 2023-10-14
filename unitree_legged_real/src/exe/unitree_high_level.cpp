#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

#define UDP_HIGH_LEVEL_IP "192.168.123.161"
#define UDP_HIGH_LEVEL_PORT_LOCAL 8090
#define UDP_HIGH_LEVEL_PORT_TARGET 8082

using namespace UNITREE_LEGGED_SDK;
class UnitreeHighLevel
{
public:
    UDP udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

public:
    UnitreeHighLevel()
        :
        udp(UDP_HIGH_LEVEL_PORT_LOCAL, 
        UDP_HIGH_LEVEL_IP, 
        UDP_HIGH_LEVEL_PORT_TARGET, 
        sizeof(HighCmd), 
        sizeof(HighState))
    {
        udp.InitCmdData(high_cmd);
    }

    void highUdpSend()
    {
        udp.SetSend(high_cmd);
        udp.Send();
    }

    void highUdpRecv()
    {
        udp.Recv();
        udp.GetRecv(high_state);
    }
};

UnitreeHighLevel unitree;

ros::Subscriber sub_cmd_vel;
ros::Publisher pub_high_state;
ros::Publisher pub_imu;

void publishIMU()
{
    sensor_msgs::Imu imu;

    imu.header.frame_id = "base_link";
    imu.header.stamp = ros::Time::now();
    imu.orientation.x = unitree.high_state.imu.quaternion[0];
    imu.orientation.y = unitree.high_state.imu.quaternion[1];
    imu.orientation.z = unitree.high_state.imu.quaternion[2];
    imu.orientation.w = unitree.high_state.imu.quaternion[3];

    imu.angular_velocity.x = unitree.high_state.imu.rpy[0];
    imu.angular_velocity.y = unitree.high_state.imu.rpy[1];
    imu.angular_velocity.z = unitree.high_state.imu.rpy[2];

    imu.linear_acceleration.x = unitree.high_state.imu.accelerometer[0];
    imu.linear_acceleration.y = unitree.high_state.imu.accelerometer[1];
    imu.linear_acceleration.z = unitree.high_state.imu.accelerometer[2];

    imu.orientation_covariance.fill(0);
    imu.angular_velocity_covariance.fill(0);
    imu.linear_acceleration_covariance.fill(0);

    pub_imu.publish(imu);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    unitree.high_cmd = rosMsg2Cmd(msg);

    unitree_legged_msgs::HighState high_state_ros;
    high_state_ros = state2rosMsg(unitree.high_state);

    pub_high_state.publish(high_state_ros);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_sub");
    ros::NodeHandle nh;

    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    pub_high_state = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
    pub_imu = nh.advertise<sensor_msgs::Imu>("imu/data", 1);

    LoopFunc loop_imuPub("imu", 0.002, 3, publishIMU);
    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&UnitreeHighLevel::highUdpSend, &unitree));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&UnitreeHighLevel::highUdpRecv, &unitree));

    loop_imuPub.start();
    loop_udpSend.start();
    loop_udpRecv.start();

    ros::spin();

    return 0;
}
