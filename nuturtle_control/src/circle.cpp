#include "ros/ros.h"
#include <turtlelib/diff_drive.hpp>
#include <geometry_msgs/Twist.h>

int main(int argc, char * argv[])
{
    // initialize node
    ros::init(argc, argv, "circle");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    nhp.getParam("rate", rate);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", rate);

}