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
static std::vector<double> obs_x, obs_y;
static double radius, height, robot_start_x, robot_start_y, robot_start_theta, x_length, y_length, wall_height, thickness;
static int num_walls = 4;

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

visualization_msgs::MarkerArray make_walls(){

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
    nhp.getParam("rate", rate);
    ros::Rate r(rate);

    // create transform broadcaster and broadcast message
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // create publishers
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("red/joint_states", rate);
    ros::Publisher timestep_pub = nhp.advertise<std_msgs::UInt64>("timestep", rate);
    ros::Publisher obstacles_pub = nhp.advertise<visualization_msgs::MarkerArray>("obstacles", 1, true);
    ros::Publisher walls_pub = nhp.advertise<visualization_msgs::MarkerArray>("walls", 1, true);
    
    // create services
    ros::ServiceServer reset = nhp.advertiseService("reset", reset_callback);
    ros::ServiceServer teleport = nhp.advertiseService("teleport", teleport_callback);

    visualization_msgs::MarkerArray obs_arr;
    obs_arr.markers.resize(obs_x.size());

    visualization_msgs::MarkerArray wall_arr;
    wall_arr.markers.resize(num_walls);
    
    while(ros::ok())
    {
        // increment time step and publish
        timestep.data = timestep.data + 1;
        timestep_pub.publish(timestep);

        // populate joint states and publish
        sensor_msgs::JointState joint_states;
        joint_states.name = {"red_wheel_left_joint", "red_wheel_right_joint"};
        joint_states.position = {0.0, 0.0};
        joint_pub.publish(joint_states);

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

        // populate transform and publish
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red_base_footprint";
        transformStamped.transform.translation.x = robot_start_x;
        transformStamped.transform.translation.y = robot_start_y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, robot_start_theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
        
        ros::spinOnce();
        r.sleep();
    }
    return 0;   
}