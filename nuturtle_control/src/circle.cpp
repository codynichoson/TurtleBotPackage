#include "ros/ros.h"
#include "nuturtle_control/control.h"
#include <turtlelib/diff_drive.hpp>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

static int publishing = 1;
static geometry_msgs::Twist twist;

/// \brief Service that starts robot in circular motion
/// \param circlevals - radius and angular velocity for circular motion
/// \return True
bool control_callback(nuturtle_control::control::Request &circlevals, nuturtle_control::control::Response &res)
{
    publishing = 1;
    twist.linear.x = circlevals.radius * std::abs(circlevals.velocity);
    twist.angular.z = circlevals.velocity;
    ROS_WARN("Yay! Circle time!");
    return true;
}

/// \brief Service that reverses a current circular motion
/// \param req - Empty
/// \return True
bool reverse_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &)
{
    publishing = 1;
    twist.linear.x = -twist.linear.x;
    twist.angular.z = -twist.angular.z;
    ROS_WARN("Beep! Beep! Back it up!");
    return true;
}

/// \brief Service that stops current motion of robot
/// \param req - Empty
/// \return True
bool stop_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &)
{
    publishing = 0;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    ROS_WARN("HALT!");
    return true;
}

/// \brief circle node main function
int main(int argc, char * argv[])
{
    // initialize node
    ros::init(argc, argv, "circle");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // int rate;
    // nh.getParam("rate", rate);
    // ros::Rate r(rate);
    ros::Rate r(100);

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