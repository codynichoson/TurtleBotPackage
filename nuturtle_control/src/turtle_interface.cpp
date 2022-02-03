#include "ros/ros.h"
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include "turtlelib/diff_drive.hpp"

void cmd_vel_callback(const geometry_msgs::Twists &msg)
{
    
}

void sensor_callback(const nuturtlebot_msgs::SensorData &msg)
{
    
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    // create publishers
    ros::Publisher wheel_pub = nh.advertise<nuturtlebot_msgs/WheelCommands>("wheel_cmd", rate);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs/JointState>("joint_states", rate);

    // create subscribers
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1000, cmd_vel_callback);
    ros::Subscriber sensor_sub = nh.subscribe("sensor_data", 1000, sensor_callback);


}