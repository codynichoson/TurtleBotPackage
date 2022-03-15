/*******************************************************************************
 * LANDMARKS NODE
 * 
 * Node Description:
 * This node serves to create a simulated environment for our robot to navigate.
 * 
 * Publishers:
 * sensor_data - Encoder tick values from the robot
 * timestep - Timestep of the simulation
 * obstacles - The attributes of the column obstacles in the simulated environment
 * walls - The attributes of the walls of our simulated environment
 * 
 * Subscribers:
 * wheel_cmd - Wheel commands for the robot calculated from cmd_vel values.
 * may know?
 * Services:
 * /reset - Moves red robot back to a (0,0,0) configuration
 * /teleport - Moves red robot to inputted configuration relative to world frame
 ******************************************************************************/

#include "ros/ros.h"
#include <turtlelib/diff_drive.hpp>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159

int num_degrees = 360;
int num_clusters = 0;
std::vector<std::vector<turtlelib::Vector2D>> cluster_list;
std::vector<turtlelib::Vector2D> cluster;


/// \brief Subscribes to laser_scan
/// \param laser_scan - laser scan data from turtlebot
/// \return None
void laser_scan_callback(const sensor_msgs::LaserScan &laser_scan)
{
    double prev_range;
    double prev_angle;
    double curr_range;
    double curr_angle;
    double curr_dist;
    double prev_dist = 0.0;
    double threshold = 0.1;

    num_degrees = laser_scan.ranges.size();

    for (int angle; angle < num_degrees; angle++)
    {
        if (angle == 0)
        {   
            prev_range = laser_scan.ranges[num_degrees];
            prev_angle = num_degrees;

            curr_range = laser_scan.ranges[angle];
            curr_angle = angle;

            curr_dist = curr_range - prev_range;
        }
        else if (angle != 0 && angle != num_degrees)
        {
            prev_range = laser_scan.ranges[angle - 1];
            prev_angle = angle - 1;

            curr_range = laser_scan.ranges[angle];
            curr_angle = angle;

            curr_dist = curr_range - prev_range;
        }

        if (curr_dist < threshold && curr_dist != 0.0)      // if a cluster point is found
        {
            if (prev_dist == 0.0)                           // if the previous point was not part of cluster
            {
                num_clusters += 1;
            }

            turtlelib::Vector2D point;
            point.x = curr_range*std::cos(angle*(PI/180));
            point.y = curr_range*std::sin(angle*(PI/180));

            cluster.push_back(point);
        }

        if (curr_dist > threshold || curr_dist == 0.0)
        {
            cluster_list.push_back(cluster);
            cluster.clear();
        }
    }
}


/// \brief nusim node main function
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;
    ros::Rate r(100);

    // create publishers
    ros::Publisher landmark_pub = nh.advertise<visualization_msgs::MarkerArray>("landmarks", 1, true);

    // create subscribers
    ros::Subscriber sub = nh.subscribe("laser_scan", 1000, laser_scan_callback);

    int cluster_count = 0;
    int id = 0;

    for (int i = 0; i < cluster_list.size(); i++)
    {
        for (int j = 0; j < cluster_list.at(i).size(); j++)
        {
            cluster_count++;
        }
    }

    while(ros::ok())
    {
        visualization_msgs::MarkerArray landmark_arr;
        landmark_arr.markers.resize(cluster_count);

        for (int a = 0; a < cluster_list.size() - 1; a++)
        {
            for (int b = 0; b < cluster_list.at(a).size(); b++)
            {
                landmark_arr.markers[id].header.frame_id = "red_base_footprint";
                landmark_arr.markers[id].header.stamp = ros::Time::now();
                landmark_arr.markers[id].id = id;
                landmark_arr.markers[id].type = visualization_msgs::Marker::SPHERE;
                landmark_arr.markers[id].action = visualization_msgs::Marker::ADD;
                landmark_arr.markers[id].pose.position.x = cluster_list.at(a).at(b).x;
                landmark_arr.markers[id].pose.position.y = cluster_list.at(a).at(b).y;
                landmark_arr.markers[id].pose.position.z = 0.5;
                landmark_arr.markers[id].pose.orientation.x = 0.0;
                landmark_arr.markers[id].pose.orientation.y = 0.0;
                landmark_arr.markers[id].pose.orientation.z = 0.0;
                landmark_arr.markers[id].pose.orientation.w = 1.0;
                landmark_arr.markers[id].scale.x = 0.05;
                landmark_arr.markers[id].scale.y = 0.05;
                landmark_arr.markers[id].scale.z = 0.05;
                landmark_arr.markers[id].color.r = 0.0;
                landmark_arr.markers[id].color.g = 1.0;
                landmark_arr.markers[id].color.b = 0.0;
                landmark_arr.markers[id].color.a = 1.0;
                id++;
            }
        }

        landmark_pub.publish(landmark_arr);

        r.sleep();
        ros::spinOnce();
    }
    return 0; 
}