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
#include "nusim/teleport.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

static int rate;
static std_msgs::UInt64 timestep;
static double x=-0.6, y=0.8, theta=1.57;

enum class State {STOP, GO, END};
static State state = State::STOP;

// std::vector<double> radius, x, y ...

bool reset_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    timestep.data = 0;
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    return true;
}

bool teleport_callback(nusim::teleport::Request &req, nusim::teleport::Response &res)
{
    x = req.x;
    y = req.y;
    theta = req.theta;
    
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

    // create timestep publisher
    ros::Publisher timestep_pub = nhp.advertise<std_msgs::UInt64>("timestep", rate);

    // create obstacle publisher
    ros::Publisher marker_pub = nhp.advertise<visualization_msgs::Marker>("/obstacles", 1000, true);
    // ros::Publisher array_pub;

    // create reset service
    ros::ServiceServer reset = nhp.advertiseService("reset", reset_callback);

    // create teleport service
    ros::ServiceServer teleport = nhp.advertiseService("teleport", teleport_callback);
    
    while(ros::ok())
    {
        // increment time step and publish
        timestep.data = timestep.data + 1;
        timestep_pub.publish(timestep);
        
        // populate transform and publish
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red_base_footprint";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
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

        // create obstacles and publish
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        // marker.ns = "marker_test_" + type_name;
        marker.id = 1;
        marker.ns = "obstacles";
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);

        ros::spinOnce();
        r.sleep();
    }
    return 0;   
}