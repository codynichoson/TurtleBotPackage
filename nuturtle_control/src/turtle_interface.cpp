#include "ros/ros.h"
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/rigid2d.hpp>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

static nuturtlebot_msgs::WheelCommands wheel_cmd;
static sensor_msgs::JointState jointstates;
static double encoder_ticks_to_rad, motor_cmd_to_radsec;
static int rate;

void cmd_vel_callback(const geometry_msgs::Twist &msg) // cmd_vel callback function
{
    turtlelib::DiffDrive ddrive;
    turtlelib::Twist2D twist;
    twist.xdot = msg.linear.x;
    twist.ydot = msg.linear.y;
    twist.thetadot = msg.angular.z;
    turtlelib::WheelVel wheel_vel = ddrive.invKin(twist);
    wheel_cmd.left_velocity = wheel_vel.left; 
    wheel_cmd.right_velocity = wheel_vel.right;
}

void sensor_callback(const nuturtlebot_msgs::SensorData &msg) // sensor_data callback function
{
    
    jointstates.name = {"wheel_left_joint", "wheel_right_joint"};
    jointstates.position = {msg.left_encoder*encoder_ticks_to_rad, msg.right_encoder*encoder_ticks_to_rad};
    jointstates.velocity = {msg.left_encoder*motor_cmd_to_radsec, msg.right_encoder*motor_cmd_to_radsec}; // this might be wrong
}

int main(int argc, char * argv[])
{
    // initialize node
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    // load params from server
    nhp.getParam("motor_cmd_to_radsec", motor_cmd_to_radsec);
    nhp.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);
    nhp.getParam("rate", rate);
    ros::Rate r(rate);

    // create publishers
    ros::Publisher wheel_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", rate);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", rate);

    // create subscribers
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1000, cmd_vel_callback);
    ros::Subscriber sensor_sub = nh.subscribe("sensor_data", 1000, sensor_callback);

    wheel_pub.publish(wheel_cmd);
    joint_pub.publish(jointstates);
}