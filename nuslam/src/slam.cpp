/*******************************************************************************
 * ODOMETRY NODE
 * 
 * Node Description:
 * This node publishes odometry messages and the odometry transform of the robot.
 * It also offers a service to reset the pose of the blue robot (representing robot
 * pose calculated via odometry) to a desired configuration. 
 * 
 * Publishers:
 * odom - The pose of the robot based on odometry 
 * 
 * Subscribers:
 * joint_states - The positions and velocities of the robot's joints (wheels)
 * 
 * Services:
 * /set_pose - Takes in a robot configuration and moves the blue robot (representing 
 * the robot's configuration based on odometry) to the inputted configuration.
 ******************************************************************************/

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
#include "nuslam/nuslam.hpp"
#include <visualization_msgs/MarkerArray.h>

int rate;
nav_msgs::Odometry odom;
static turtlelib::Config config;
std::string odom_id, body_id, wheel_left, wheel_right;
static sensor_msgs::JointState js;
static turtlelib::WheelAngles wheel_angles{.left = 0.0, .right = 0.0};
static turtlelib::WheelVel wheel_vels{.left = 0.0, .right = 0.0};
turtlelib::DiffDrive ddrive;
static turtlelib::Twist2D twist = {0.0, 0.0, 0.0};
arma::mat z;
int flag = 0;
static arma::mat state(9, 1, arma::fill::zeros);
nuslam::SLAM Slammy(3);

/// \brief Subscribes to joint_states and calculates new red robot configuration
/// \param js - Updated joint_states message
/// \return None
void joints_callback(const sensor_msgs::JointState &js) // odometry callback function
{   
    wheel_angles.left = js.position[0];
    wheel_angles.right = js.position[1];
    wheel_vels.left = js.velocity[0];
    wheel_vels.right = js.velocity[1];
    
    twist = ddrive.Vel2Twist(wheel_vels);

    config = ddrive.fKin(wheel_angles, config);
}

/// \brief Subscribes to fake_sensor and updates estimated marker coords
/// \param fake_sensor - Updated fake_sensor message
/// \return None
void laser_callback(const visualization_msgs::MarkerArray &fake_sensor) // odometry callback function
{   
    int num_markers = fake_sensor.markers.size();

    z = arma::mat(2*num_markers, 1);

    for (int i = 0; i < num_markers; i++){
        double xi = fake_sensor.markers[i].pose.position.x;
        double yi = fake_sensor.markers[i].pose.position.y;

        nuslam::RangeBearing rb;
        rb.range = std::sqrt(std::pow(xi, 2) + std::pow(yi, 2));
        rb.bearing = std::atan2(yi, xi);

        z(2*i, 0) = rb.range;
        z((2*i)+1, 0) = rb.bearing;
    }

    if (flag == 0){
        Slammy.init_landmarks(num_markers, z);
    }

    flag = 1;

    Slammy.predict(twist, 10.0);

    state = Slammy.update(num_markers, z);
    ROS_INFO_STREAM(state);

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

    config = {.x = robot_start_x, .y = robot_start_y, .theta = robot_start_theta};

    state(0,0) = robot_start_theta;
    state(1,0) = robot_start_x;
    state(2,0) = robot_start_y;

    ROS_WARN("state: %f, %f, %f", state(1,0), state(2,0), state(0,0));

    ros::Rate r(rate);

    ros::Subscriber joint_sub = nh.subscribe("joint_states", 100, joints_callback);
    ros::Subscriber fake_sensor_sub = nh.subscribe("/nusim/fake_sensor", 100, laser_callback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", rate);

    ros::ServiceServer set_pose = nh.advertiseService("set_pose", set_pose_callback);

    tf2_ros::TransformBroadcaster Twb_br, Tmg_br, Tmo_br, Tog_br;
    geometry_msgs::TransformStamped Twb_msg, Tmg_msg, Tmo_msg, Tog_msg;

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
        
        
        odom.child_frame_id = body_id;
        odom.twist.twist.linear.x = twist.xdot;
        odom.twist.twist.angular.z = twist.thetadot;

        Twb_msg.header.stamp = ros::Time::now();
        // Twb_msg.header.frame_id = odom_id;
        // Twb_msg.child_frame_id = body_id;
        Twb_msg.header.frame_id = "world";
        Twb_msg.child_frame_id = "blue_base_footprint";
        Twb_msg.transform.translation.x = config.x;
        Twb_msg.transform.translation.y = config.y;
        Twb_msg.transform.translation.z = 0.0;
        q.setRPY(0, 0, config.theta);
        Twb_msg.transform.rotation.x = q.x();
        Twb_msg.transform.rotation.y = q.y();
        Twb_msg.transform.rotation.z = q.z();
        Twb_msg.transform.rotation.w = q.w();
        Twb_br.sendTransform(Twb_msg);

        // Transforms
        turtlelib::Vector2D Vmg;
        Vmg.x = state(1,0);
        Vmg.y = state(2,0);
        turtlelib::Transform2D Tmg(Vmg, state(0,0));

        turtlelib::Vector2D Vog;
        Vog.x = config.x;
        Vog.y = config.y;
        turtlelib::Transform2D Tog(Vog, config.theta);

        turtlelib::Transform2D Tmo = Tmg*Tog.inv();
        turtlelib::Vector2D Vmo = Tmo.translation();
        double theta_mo = Tmo.rotation();

        Tmo_msg.header.stamp = ros::Time::now();
        Tmo_msg.header.frame_id = "map";
        Tmo_msg.child_frame_id = "odom";
        Tmo_msg.transform.translation.x = Vmo.x;
        Tmo_msg.transform.translation.y = Vmo.y;
        Tmo_msg.transform.translation.z = 0.0;
        q.setRPY(0, 0, theta_mo);
        Tmo_msg.transform.rotation.x = q.x();
        Tmo_msg.transform.rotation.y = q.y();
        Tmo_msg.transform.rotation.z = q.z();
        Tmo_msg.transform.rotation.w = q.w();
        Tmo_br.sendTransform(Tmo_msg);

        Tog_msg.header.stamp = ros::Time::now();
        Tog_msg.header.frame_id = "odom";
        Tog_msg.child_frame_id = "green_base_footprint";
        Tog_msg.transform.translation.x = config.x;
        Tog_msg.transform.translation.y = config.y;
        Tog_msg.transform.translation.z = 0.0;
        q.setRPY(0, 0, config.theta);
        Tog_msg.transform.rotation.x = q.x();
        Tog_msg.transform.rotation.y = q.y();
        Tog_msg.transform.rotation.z = q.z();
        Tog_msg.transform.rotation.w = q.w();
        Tog_br.sendTransform(Tog_msg);
        
        odom_pub.publish(odom);

        r.sleep();
        ros::spinOnce();
    }
}