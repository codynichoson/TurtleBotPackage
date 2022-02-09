#include "ros/ros.h"
#include "nuturtle_control/control.h"
#include <turtlelib/diff_drive.hpp>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

static int publishing = 1;
static geometry_msgs::Twist twist;

bool control_callback(nuturtle_control::control::Request &req, nuturtle_control::control::Response &res)
{
    publishing = 1;
    twist.linear.x = req.radius * req.velocity;
    twist.angular.z = req.velocity;

    return true;
}

bool reverse_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &)
{
    publishing = 1;
    twist.linear.x = -twist.linear.x;
    twist.angular.z = -twist.angular.z;

    return true;
}

bool stop_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &)
{
    publishing = 0;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;

    return true;
}

int main(int argc, char * argv[])
{
    // initialize node
    ros::init(argc, argv, "circle");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    int rate;
    nh.getParam("/nusim/rate", rate);
    ros::Rate r(rate);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::ServiceServer control = nh.advertiseService("control", control_callback);
    ros::ServiceServer reverse = nh.advertiseService("reverse", reverse_callback);
    ros::ServiceServer stop = nh.advertiseService("stop", stop_callback);

    while(ros::ok())
    {
        if (publishing == 1)
        {
            cmd_vel_pub.publish(twist);
        }
        else if (publishing == 0)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            cmd_vel_pub.publish(twist);
            publishing = 2;
        }

        r.sleep();
        ros::spinOnce();
    }

}