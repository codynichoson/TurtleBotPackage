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

nav_msgs::Odometry odom;
static turtlelib::Config config;
std::string odom_id, body_id, wheel_left, wheel_right;
static sensor_msgs::JointState js;
turtlelib::WheelAngles wheel_angles{.left = 0.0, .right = 0.0};
turtlelib::DiffDrive ddrive;

/// \brief Subscribes to joint_states and calculates new red robot configuration
/// \param js - Updated joint_states message
/// \return None
void joints_callback(const sensor_msgs::JointState &js) // odometry callback function
{   
    wheel_angles.left = js.position[0];
    wheel_angles.right = js.position[1];

    config = ddrive.fKin(wheel_angles, config);
}

/// \brief Teleports blue robot (odometry) to desired pose in world frame
/// \param q - Desired coniguration in world frame
/// \return True
bool set_pose_callback(nuturtle_control::set_pose::Request &q, nuturtle_control::set_pose::Response &res)
{
    config = {q.x, q.y, q.theta};
    ROS_WARN("Odometry has been reset!");
    return true;
}

/// \brief odometry node main function
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;

    int rate;
    double robot_start_x;
    double robot_start_y;
    double robot_start_theta;
    
    if (nh.hasParam("body_id")){
        nh.getParam("body_id", body_id);
    }
    else{
        ROS_ERROR_STREAM("body_id parameter not found.");
    }
    if (nh.hasParam("odom_id")){
        nh.getParam("odom_id", odom_id);
    }
    else{
        ROS_ERROR_STREAM("odom_id parameter not found.");
    }
    if (nh.hasParam("rate")){
        nh.getParam("rate", rate);
    }
    else{
        ROS_ERROR_STREAM("rate parameter not found.");
    }
    if (nh.hasParam("robot_start_x")){
        nh.getParam("robot_start_x", robot_start_x);
    }
    else{
        ROS_ERROR_STREAM("robot_start_x parameter not found.");
    }
    if (nh.hasParam("robot_start_y")){
        nh.getParam("robot_start_y", robot_start_y);
    }
    else{
        ROS_ERROR_STREAM("robot_start_y parameter not found.");
    }
    if (nh.hasParam("robot_start_theta")){
        nh.getParam("robot_start_theta", robot_start_theta);
    }
    else{
        ROS_ERROR_STREAM("robot_start_theta parameter not found.");
    }

    // nh.getParam("wheel_left", wheel_left);
    // nh.getParam("wheel_right", wheel_right);

    config = {.x = robot_start_x, .y = robot_start_y, .theta = robot_start_theta};

    ros::Rate r(rate);

    ros::Subscriber joint_sub = nh.subscribe("joint_states", 100, joints_callback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", rate);

    ros::ServiceServer set_pose = nh.advertiseService("set_pose", set_pose_callback);

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    while(ros::ok())
    {
        odom.header.frame_id = odom_id;
        odom.pose.pose.position.x = config.x;
        odom.pose.pose.position.y = config.y;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, config.theta);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        
        turtlelib::Twist2D twist = ddrive.Ang2Twist(wheel_angles);
        odom.child_frame_id = body_id;
        odom.twist.twist.linear.x = twist.xdot;
        odom.twist.twist.angular.z = twist.thetadot;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = odom_id;
        transformStamped.child_frame_id = body_id;
        transformStamped.transform.translation.x = config.x;
        transformStamped.transform.translation.y = config.y;
        transformStamped.transform.translation.z = 0.0;
        q.setRPY(0, 0, config.theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
        
        odom_pub.publish(odom);

        r.sleep();
        ros::spinOnce();
    }


}