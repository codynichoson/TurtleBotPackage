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

int rate;
nav_msgs::Odometry odom;
static turtlelib::Config config;
std::string odom_id, body_id, wheel_left, wheel_right;
static sensor_msgs::JointState js;
turtlelib::WheelAngles wheel_angles{.left = 0.0, .right = 0.0};
turtlelib::DiffDrive ddrive;
turtlelib::Twist2D twist = {0.0, 0.0, 0.0};
int flag == 0;

/// \brief Subscribes to joint_states and calculates new red robot configuration
/// \param js - Updated joint_states message
/// \return None
void joints_callback(const sensor_msgs::JointState &js) // odometry callback function
{   
    wheel_angles.left = js.position[0];
    wheel_angles.right = js.position[1];

    config = ddrive.fKin(wheel_angles, config);
}

/// \brief Subscribes to fake_sensor and updates estimated marker coords
/// \param fake_sensor - Updated fake_sensor message
/// \return None
void laser_callback(const sensor_msgs::JointState &fake_sensor) // odometry callback function
{   
    int num_markers = fake_sensor.markers.size();

    nuslam::SLAM Slammy(num_markers);

    for (i = 0; i < num_markers; i++){
        double xi = fake_sensor.markers[i].pose.position.x;
        double yi = fake_sensor.markers[i].pose.position.y;

        nuslam::RangeBearing rb;
        rb.range = std::sqrt(std::pow(xi, 2) + std::pow(yi, 2));
        rb.bearing = std::atan2(yi, xi);

        Slammy.z(2*i, 0) = rb.range;
        Slammy.z((2*i)+1, 0) = rb.bearing;

        if (flag == 0){
            Slammy.init_landmarks(Slammy.n, Slammy.z);
        }
    }

    flag = 1;

    Slammy.predict(twist, rate);

    arma::mat state = Slammy.update(n, Slammy.z)
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

    ros::Rate r(rate);

    ros::Subscriber joint_sub = nh.subscribe("joint_states", 100, joints_callback);
    ros::Subscriber fake_sensor_sub = nh.subscribe("fake_sensor", 100, laser_callback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", rate);

    ros::ServiceServer set_pose = nh.advertiseService("set_pose", set_pose_callback);

    tf2_ros::TransformBroadcaster blue_br, green_br;
    geometry_msgs::TransformStamped blue_tf, green_tf;

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
        
        twist = ddrive.Ang2Twist(wheel_angles);
        odom.child_frame_id = body_id;
        odom.twist.twist.linear.x = twist.xdot;
        odom.twist.twist.angular.z = twist.thetadot;



        blue_tf.header.stamp = ros::Time::now();
        blue_tf.header.frame_id = odom_id;
        blue_tf.child_frame_id = body_id;
        blue_tf.transform.translation.x = config.x;
        blue_tf.transform.translation.y = config.y;
        blue_tf.transform.translation.z = 0.0;
        q.setRPY(0, 0, config.theta);
        blue_tf.transform.rotation.x = q.x();
        blue_tf.transform.rotation.y = q.y();
        blue_tf.transform.rotation.z = q.z();
        blue_tf.transform.rotation.w = q.w();
        blue_br.sendTransform(blue_tf);

        green_tf.header.stamp = ros::Time::now();
        green_tf.header.frame_id = "map";
        green_tf.child_frame_id = "green_base_footprint";
        green_tf.transform.translation.x = state(1,0);
        green_tf.transform.translation.y = state(2,0);
        green_tf.transform.translation.z = 0.0;
        q.setRPY(0, 0, state(0,0));
        green_tf.transform.rotation.x = q.x();
        green_tf.transform.rotation.y = q.y();
        green_tf.transform.rotation.z = q.z();
        green_tf.transform.rotation.w = q.w();
        green_br.sendTransform(green_tf);
        
        odom_pub.publish(odom);

        r.sleep();
        ros::spinOnce();
    }
}