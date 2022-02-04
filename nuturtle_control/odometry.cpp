#include "ros/ros.h"
#include <turtlelib/diff_drive.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

static nav_msgs::Odometry odom;

void wheel_cmd_callback(const sensor_msgs::JointState &msg) // odometry callback function
{
    odom.header.frame_id = odom_id;
    odom.pose.pose.position
    odom.child_frame_id = body_id;
}

bool set_pose_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{

    return true;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    nhp.getParam("body_id", body_id);
    nhp.getParam("odom_id", odom_id);
    nhp.getParam("wheel_left", wheel_left);
    nhp.getParam("wheel_right", wheel_right);

    ros::Subscriber joint_sub = nh.subscribe("JointState", 1000, joints_callback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", rate);

    ros::ServiceServer set_pose = nhp.advertiseService("set_pose", set_pose_callback);

}