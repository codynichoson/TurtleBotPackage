#include "ros/ros.h"
#include "turtlelib/diff_drive.hpp"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    nhp.getParam("body_id", body_id);
    nhp.getParam("odom_id", odom_id);
    nhp.getParam("wheel_left", wheel_left);
    nhp.getParam("wheel_right", wheel_right);

    ros::Subscriber joint_sub = nh.subscribe("JointStates", 1000, cmd_vel_callback);

}