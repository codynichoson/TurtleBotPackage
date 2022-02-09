#include "ros/ros.h"
#include "std_msgs/UInt64.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/TransformStamped.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sstream>
#include <iostream>
#include <std_srvs/Trigger.h>
#include "nusim/teleport.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <turtlelib/diff_drive.hpp>

static double rate;
static std_msgs::UInt64 timestep;
static std::vector<double> obs_x, obs_y;
static double radius;
static double height;
static double robot_start_x, robot_start_y, robot_start_theta;
static double x_length;
static double y_length;
static double wall_height;
static double thickness;
static double motor_cmd_to_radsec;
static double encoder_ticks_to_rad;
static int num_walls = 4;

static turtlelib::WheelAngles new_wheelangles = {.left = 0.0, .right = 0.0};
static turtlelib::WheelAngles old_wheelangles = {.left = 0.0, .right = 0.0};

static turtlelib::WheelVel wheelvel;

turtlelib::Config new_config = {.x = -0.6, .y = 0.8, .theta = 1.57};
turtlelib::Config old_config = {.x = 0.0, .y = 0.0, .theta = 0.0};

int isResetting = 0;
int isTeleporting = 0;

// turtlelib::Config new_config;
// turtlelib::Config old_config;

static nuturtlebot_msgs::SensorData sensor_data;

static turtlelib::DiffDrive ddrive;

bool reset_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    timestep.data = 0;
    new_config.x = 0.0;
    new_config.y = 0.0;
    new_config.theta = 0.0;

    ddrive = turtlelib::DiffDrive(new_config.x, new_config.y, new_config.theta);

    isResetting = 1;
    ROS_WARN("isResetting: %d", isResetting);

    return true;
}

bool teleport_callback(nusim::teleport::Request &req, nusim::teleport::Response &res)
{
    new_config.x = req.x;
    new_config.y = req.y;
    new_config.theta = req.theta;

    ddrive = turtlelib::DiffDrive(new_config.x, new_config.y, new_config.theta);

    isTeleporting = 1;
    ROS_WARN("isTeleporting: %d", isTeleporting);

    return true;
}

void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands &msg)
{
    wheelvel.left = msg.left_velocity * motor_cmd_to_radsec;
    wheelvel.right = msg.right_velocity * motor_cmd_to_radsec;
    
    // calculate encoder stuff to populate sensor_data
    int encoder_left = (int)(((wheelvel.left/rate) + new_wheelangles.left)/encoder_ticks_to_rad);
    int encoder_right = (int)(((wheelvel.right/rate) + new_wheelangles.right)/encoder_ticks_to_rad);

    sensor_data.left_encoder = encoder_left;
    sensor_data.right_encoder = encoder_right;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    nhp.getParam("radius", radius);
    nhp.getParam("height", height);
    nhp.getParam("obs_x", obs_x);
    nhp.getParam("obs_y", obs_y);
    nhp.getParam("robot_start_x", robot_start_x);
    nhp.getParam("robot_start_y", robot_start_y);
    nhp.getParam("robot_start_theta", robot_start_theta);
    nhp.getParam("x_length", x_length);
    nhp.getParam("y_length", y_length);
    nhp.getParam("wall_height", wall_height);
    nhp.getParam("thickness", thickness);
    nh.getParam("/nusim/motor_cmd_to_radsec", motor_cmd_to_radsec);
    nhp.getParam("/nusim/encoder_ticks_to_rad", encoder_ticks_to_rad);
    nhp.getParam("rate", rate);
    ros::Rate r(rate);

    // turtlelib::Config new_config = {.x = robot_start_x, .y = robot_start_y, .theta = robot_start_theta};
    // turtlelib::Config old_config = {.x = robot_start_x, .y = robot_start_y, .theta = robot_start_theta};

    // create transform broadcaster and broadcast message
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // create publishers
    ros::Publisher sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", rate);
    ros::Publisher timestep_pub = nhp.advertise<std_msgs::UInt64>("timestep", rate);
    ros::Publisher obstacles_pub = nhp.advertise<visualization_msgs::MarkerArray>("obstacles", 1, true);
    ros::Publisher walls_pub = nhp.advertise<visualization_msgs::MarkerArray>("walls", 1, true);

    // create subscribers
    ros::Subscriber sub = nh.subscribe("wheel_cmd", 1000, wheel_cmd_callback);
    
    // create services
    ros::ServiceServer reset = nh.advertiseService("reset", reset_callback);
    ros::ServiceServer teleport = nh.advertiseService("teleport", teleport_callback);

    visualization_msgs::MarkerArray obs_arr;
    obs_arr.markers.resize(obs_x.size());

    visualization_msgs::MarkerArray wall_arr;
    wall_arr.markers.resize(num_walls);
    
    while(ros::ok())
    {
        // increment time step and publish
        timestep.data = timestep.data + 1;
        timestep_pub.publish(timestep);

        for(int i = 0; i < obs_x.size(); i++){
            obs_arr.markers[i].header.frame_id = "world";
            obs_arr.markers[i].header.stamp = ros::Time::now();
            obs_arr.markers[i].id = i;
            obs_arr.markers[i].type = visualization_msgs::Marker::CYLINDER;
            obs_arr.markers[i].action = visualization_msgs::Marker::ADD;
            obs_arr.markers[i].pose.position.x = obs_x[i];
            obs_arr.markers[i].pose.position.y = obs_y[i];
            obs_arr.markers[i].pose.position.z = height/2;
            obs_arr.markers[i].pose.orientation.x = 0.0;
            obs_arr.markers[i].pose.orientation.y = 0.0;
            obs_arr.markers[i].pose.orientation.z = 0.0;
            obs_arr.markers[i].pose.orientation.w = 1.0;
            obs_arr.markers[i].scale.x = 2*radius;
            obs_arr.markers[i].scale.y = 2*radius;
            obs_arr.markers[i].scale.z = height;
            obs_arr.markers[i].color.r = 1.0;
            obs_arr.markers[i].color.g = 0.0;
            obs_arr.markers[i].color.b = 0.0;
            obs_arr.markers[i].color.a = 1.0;
        }
        obstacles_pub.publish(obs_arr);

        for(int i = 0; i < num_walls; i++){
            wall_arr.markers[i].header.frame_id = "world";
            wall_arr.markers[i].header.stamp = ros::Time::now();
            wall_arr.markers[i].id = i;
            wall_arr.markers[i].type = visualization_msgs::Marker::CUBE;
            wall_arr.markers[i].action = visualization_msgs::Marker::ADD;
            if (i == 0){
                wall_arr.markers[i].pose.position.x = -x_length/2 - thickness/2;
                wall_arr.markers[i].pose.position.y = 0;
                wall_arr.markers[i].scale.x = thickness;
                wall_arr.markers[i].scale.y = y_length;
            }
            if (i == 1){
                wall_arr.markers[i].pose.position.x = 0;
                wall_arr.markers[i].pose.position.y = y_length/2 + thickness/2;
                wall_arr.markers[i].scale.x = x_length + 2*thickness;
                wall_arr.markers[i].scale.y = thickness;
            }
            if (i == 2){
                wall_arr.markers[i].pose.position.x = x_length/2 + thickness/2;
                wall_arr.markers[i].pose.position.y = 0;
                wall_arr.markers[i].scale.x = thickness;
                wall_arr.markers[i].scale.y = y_length;
            }
            if (i == 3){
                wall_arr.markers[i].pose.position.x = 0;
                wall_arr.markers[i].pose.position.y = -y_length/2 - thickness/2;
                wall_arr.markers[i].scale.x = x_length + 2*thickness;
                wall_arr.markers[i].scale.y = thickness;
            }
            wall_arr.markers[i].pose.position.z = wall_height/2;
            wall_arr.markers[i].pose.orientation.x = 0.0;
            wall_arr.markers[i].pose.orientation.y = 0.0;
            wall_arr.markers[i].pose.orientation.z = 0.0;
            wall_arr.markers[i].pose.orientation.w = 1.0;
            wall_arr.markers[i].scale.z = wall_height;
            wall_arr.markers[i].color.r = 0.3;  // Go Cats
            wall_arr.markers[i].color.g = 0.16;
            wall_arr.markers[i].color.b = 0.52;
            wall_arr.markers[i].color.a = 1.0;
        }
        walls_pub.publish(wall_arr);

        // // update configuration with forward kinematics
        // if (isTeleporting == 0 && isResetting == 0){
            
        new_wheelangles = {.left = ((wheelvel.left/rate)+old_wheelangles.left), .right = ((wheelvel.right/rate)+old_wheelangles.right)};
        new_config = ddrive.fKin(new_wheelangles);
        old_wheelangles = new_wheelangles;
        // }

        // ROS_WARN("new_config.x: %f", new_config.x);
        // ROS_WARN("new_config.y: %f", new_config.y);

        // populate transform and publish
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red_base_footprint";
        transformStamped.transform.translation.x = new_config.x;
        transformStamped.transform.translation.y = new_config.y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, new_config.theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);

        old_config = new_config;

        sensor_pub.publish(sensor_data);

        isTeleporting = 0;
        isResetting = 0;
        
        r.sleep();
        ros::spinOnce();
    }
    return 0;   
}