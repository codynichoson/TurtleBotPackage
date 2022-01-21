// Static variables used by callbacks here
#include "ros/ros.h"
#include "std_msgs/UInt64.h"
#include "sensor_msgs/JointState.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sstream>
#include <iostream>
#include <std_srvs/Trigger.h>

static int rate;
static std_msgs::UInt64 timestep;

enum class State {STOP, GO, END};
static State state = State::STOP;

bool reset_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    timestep.data = 0;
    std::cout << "Service Successful\n";
    return true;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    // set private parameters
    nhp.setParam("rate", 500);
    nhp.getParam("rate", rate);
    ros::Rate r(rate);

    // create transform broadcaster and broadcast message
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // create joint states publisher
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("red/joint_states", rate);

    // create reset service
    ros::ServiceServer reset = nhp.advertiseService("reset", reset_callback);
    
    while(ros::ok())
    {
        // increment time step and publish
        timestep.data = timestep.data + 1;
        ros::Publisher timestep_pub = nhp.advertise<std_msgs::UInt64>("timestep", rate);

        // populate transform and publish
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red_base_footprint";
        transformStamped.transform.translation.x = 0.5;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);

        // populate joint states and publish
        sensor_msgs::JointState joint_states;
        joint_states.name = {"red_wheel_left_joint", "red_wheel_right_joint"};
        joint_states.position = {0.0, 0.0};
        joint_pub.publish(joint_states);

        ros::spinOnce();
        r.sleep();
    }
    return 0;   
}