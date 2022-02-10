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


void joints_callback(const sensor_msgs::JointState &msg) // odometry callback function
{   
    // ROS_WARN("msg.position[0]: %f, msg.position[1]: %f", msg.position[0], msg.position[1]);
    wheel_angles.left = msg.position[0];
    wheel_angles.right = msg.position[1];

    config = ddrive.fKin(wheel_angles, config);
    // ROS_WARN("theta: %f", config.theta);

    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = config.x;
    odom.pose.pose.position.y = config.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, config.theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
    turtlelib::Twist2D twist = ddrive.Ang2Twist(wheel_angles);
    odom.child_frame_id = "blue_base_footprint";
    odom.twist.twist.linear.x = twist.xdot;
    odom.twist.twist.angular.z = twist.thetadot;
}

bool set_pose_callback(nuturtle_control::set_pose::Request &req, nuturtle_control::set_pose::Response &res)
{
    ROS_WARN("set_pose_callback");
    // odom.header.frame_id = "odom";
    // odom.pose.pose.position.x = req.x;
    // odom.pose.pose.position.y = req.y;
    // tf2::Quaternion q;
    // q.setRPY(0.0, 0.0, req.theta);
    // odom.pose.pose.orientation.x = q.x();
    // odom.pose.pose.orientation.y = q.y();
    // odom.pose.pose.orientation.z = q.z();
    // odom.pose.pose.orientation.w = q.w();
    config = {req.x, req.y, req.theta};
    // turtlelib::DiffDrive ddrive(config);

    return true;
}

int main(int argc, char * argv[])
{
    // js.name = {"red_wheel_left_joint", "red_wheel_right_joint"};
    // js.position = {0.0, 0.0};
    // js.velocity = {0.0, 0.0};


    // ROS_WARN("start of main");
    ros::init(argc, argv, "turtle_interface");
    // ros::NodeHandle nh("~");
    ros::NodeHandle nh;

    nh.getParam("body_id", body_id);
    nh.getParam("odom_id", odom_id);
    nh.getParam("wheel_left", wheel_left);
    nh.getParam("wheel_right", wheel_right);

    int rate;
    nh.getParam("/nusim/rate", rate);
    ros::Rate r(rate);

    ros::Subscriber joint_sub = nh.subscribe("joint_states", 100, joints_callback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", rate);

    ros::ServiceServer set_pose = nh.advertiseService("set_pose", set_pose_callback);

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    while(ros::ok())
    {
        // populate transform and publish
        transformStamped.header.stamp = ros::Time::now();
        // transformStamped.header.frame_id = odom_id;
        // transformStamped.child_frame_id = body_id;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "blue_base_footprint";
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

        r.sleep();
        ros::spinOnce();
    }


}