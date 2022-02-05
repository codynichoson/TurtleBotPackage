#include "ros/ros.h"
#include "nuturtle_control/set_pose.h"
#include <turtlelib/diff_drive.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <string>

static nav_msgs::Odometry odom;
static turtlelib::Config config;
static std::string odom_id, body_id, wheel_left, wheel_right;
static int rate; 

void joints_callback(const sensor_msgs::JointState &msg) // odometry callback function
{
    turtlelib::WheelAngles wheelangles = {msg.position[0], msg.position[1]};
    turtlelib::DiffDrive ddrive;
    
    config = ddrive.fKin(wheelangles);

    odom.header.frame_id = odom_id;
    odom.pose.pose.position.x = config.x;
    odom.pose.pose.position.y = config.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, config.theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
    turtlelib::Twist2D twist = ddrive.Ang2Twist(wheelangles);
    odom.child_frame_id = body_id;
    odom.twist.twist.linear.x = twist.xdot;
    odom.twist.twist.angular.z = twist.thetadot;
}

bool set_pose_callback(nuturtle_control::set_pose::Request &req, nuturtle_control::set_pose::Response &res)
{
    odom.header.frame_id = odom_id;
    odom.pose.pose.position.x = req.x;
    odom.pose.pose.position.y = req.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, req.theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    return true;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    nhp.getParam("body_id", body_id);
    nhp.getParam("odom_id", odom_id);
    nhp.getParam("wheel_left", wheel_left);
    nhp.getParam("wheel_right", wheel_right);
    nhp.getParam("rate", rate);

    ros::Subscriber joint_sub = nh.subscribe("JointState", 1000, joints_callback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", rate);

    ros::ServiceServer set_pose = nhp.advertiseService("set_pose", set_pose_callback);

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // populate transform and publish
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odom_id;
    transformStamped.child_frame_id = body_id;
    transformStamped.transform.translation.x = config.x;
    transformStamped.transform.translation.y = config.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, config.theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
    
    odom_pub.publish(odom);


}