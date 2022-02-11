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
static double encoder_ticks_to_rad, motor_cmd_to_radsec;
static turtlelib::WheelVel wheel_vel;


/// \brief Subscribes to cmd_vel and populates wheel_cmd message
/// \param cmdvel - cmd_vel values from user input
/// \return None
void cmd_vel_callback(const geometry_msgs::Twist &cmdvel) // cmd_vel callback function
{
    turtlelib::DiffDrive ddrive;
    turtlelib::Twist2D twist;
    twist.xdot = cmdvel.linear.x;
    twist.ydot = 0.0;
    twist.thetadot = cmdvel.angular.z;
    wheel_vel = ddrive.invKin(twist);

    wheel_cmd.left_velocity = (int)(wheel_vel.left/motor_cmd_to_radsec);
    wheel_cmd.right_velocity = (int)(wheel_vel.right/motor_cmd_to_radsec);

    if (wheel_cmd.left_velocity > 256){
        wheel_cmd.left_velocity = 256; 
    }
    else if (wheel_cmd.left_velocity < -256){
        wheel_cmd.left_velocity = -256;
    }

    if (wheel_cmd.right_velocity > 256){
        wheel_cmd.right_velocity = 256; 
    }
    else if (wheel_cmd.right_velocity < -256){
        wheel_cmd.right_velocity = -256;
    }
}

/// \brief Subscribes to sensor_data and populates joint_states message
/// \param sensordata - sensor_data message with encoder values
/// \return None
void sensor_callback(const nuturtlebot_msgs::SensorData &sensordata) // sensor_data callback function
{
    jointstates.header.stamp = ros::Time::now();
    jointstates.name = {"red_wheel_left_joint", "red_wheel_right_joint"};
    jointstates.position = {sensordata.left_encoder*encoder_ticks_to_rad, sensordata.right_encoder*encoder_ticks_to_rad};
    jointstates.velocity = {wheel_vel.left, wheel_vel.right};
}

/// \brief turtle_interface node main function
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh;

    int rate;

    // load params from server
    nh.getParam("/nusim/motor_cmd_to_radsec", motor_cmd_to_radsec);
    nh.getParam("/nusim/encoder_ticks_to_rad", encoder_ticks_to_rad);
    nh.getParam("rate", rate);
    ros::Rate r(rate);

    wheel_cmd.left_velocity = 0;
    wheel_cmd.left_velocity = 0;

    // initialize jointstates to avoid empty message
    jointstates.name = {"red_wheel_left_joint", "red_wheel_right_joint"};
    jointstates.position = {0.0, 0.0};
    jointstates.velocity = {0.0, 0.0};

    // create publishers
    ros::Publisher wheel_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", 100);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);

    // create subscribers
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 100, cmd_vel_callback);
    ros::Subscriber sensor_sub = nh.subscribe("sensor_data", 100, sensor_callback);

    while(ros::ok())
    {
        wheel_pub.publish(wheel_cmd);
        joint_pub.publish(jointstates);
        
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}