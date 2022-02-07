#include "ros/ros.h"
#include "ros/console.h"
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/rigid2d.hpp>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

static nuturtlebot_msgs::WheelCommands wheel_cmd;
static sensor_msgs::JointState jointstates;
// ROS_INFO(jointstates);
static double encoder_ticks_to_rad, motor_cmd_to_radsec;
static int rate;
static turtlelib::WheelVel wheel_vel;

void cmd_vel_callback(const geometry_msgs::Twist &msg) // cmd_vel callback function
{
    turtlelib::DiffDrive ddrive;
    turtlelib::Twist2D twist;
    twist.xdot = msg.linear.x;
    twist.ydot = msg.linear.y;
    twist.thetadot = msg.angular.z;
    wheel_vel = ddrive.invKin(twist);

    wheel_cmd.left_velocity = wheel_vel.left/motor_cmd_to_radsec;
    wheel_cmd.right_velocity = wheel_vel.right/motor_cmd_to_radsec;

    if (wheel_cmd.left_velocity > 256){
        wheel_cmd.left_velocity = 256; 
    }
    else if (wheel_cmd.left_velocity < -256){
        wheel_cmd.left_velocity = -256;
    }
    else{
        wheel_cmd.left_velocity = wheel_cmd.left_velocity; 
    }

    if (wheel_cmd.right_velocity > 256){
        wheel_cmd.right_velocity = 256; 
    }
    else if (wheel_cmd.right_velocity < -256){
        wheel_cmd.right_velocity = -256;
    }
    else{
        wheel_cmd.right_velocity = wheel_cmd.right_velocity;
    }
}

void sensor_callback(const nuturtlebot_msgs::SensorData &msg) // sensor_data callback function
{
    jointstates.header.stamp = ros::Time::now();
    jointstates.name = {"red_wheel_left_joint", "red_wheel_right_joint"};
    jointstates.position = {msg.left_encoder*encoder_ticks_to_rad, msg.right_encoder*encoder_ticks_to_rad};
    jointstates.velocity = {wheel_vel.left, wheel_vel.right};
}

int main(int argc, char * argv[])
{
    // initialize node
    ros::init(argc, argv, "turtle_interface");
    // ros::NodeHandle nh("~");
    ros::NodeHandle nh;

    // load params from server
    nh.getParam("nusim/motor_cmd_to_radsec", motor_cmd_to_radsec);
    nh.getParam("/nusim/encoder_ticks_to_rad", encoder_ticks_to_rad);
    nh.getParam("/nusim/rate", rate);
    ros::Rate r(rate);

    // initialize jointstates to avoid empty message
    jointstates.position = {0.0, 0.0};

    // create publishers
    ros::Publisher wheel_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", rate);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", rate);

    // create subscribers
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1000, cmd_vel_callback);
    ros::Subscriber sensor_sub = nh.subscribe("sensor_data", 1000, sensor_callback);

    while(ros::ok())
    {
        wheel_pub.publish(wheel_cmd);
        joint_pub.publish(jointstates);
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}