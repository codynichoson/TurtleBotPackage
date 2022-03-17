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
#include <ros/console.h>
#include <turtlelib/diff_drive.hpp>
#include "nuslam/circlelib.hpp"
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159

class Landmarks
{
    public:
        Landmarks()
        {
            cluster_pub = nh.advertise<visualization_msgs::MarkerArray>("clusters", 1, true);
            landmark_pub = nh.advertise<visualization_msgs::MarkerArray>("landmarks", 1, true);
            laser_sub = nh.subscribe("/nusim/laser_scan", 10, &Landmarks::laser_scan_callback, this);
            num_clusters = 0;
            num_degrees = 360;
            // nuslam::CircleFit CircleBoy();
        }

        /// \brief Subscribes to laser_scan
        /// \param laser_scan - laser scan data from turtlebot
        /// \return None
        void laser_scan_callback(const sensor_msgs::LaserScan & laser_scan)
        {
            double prev_range;
            double prev_angle;
            double curr_range;
            double curr_angle;
            double curr_dist;
            double prev_dist = 0.0;
            double threshold = 0.05;

            num_degrees = laser_scan.ranges.size();

            for (int angle = 0; angle < num_degrees; angle++)
            {
                if (angle == 0)
                {   
                    prev_range = laser_scan.ranges[num_degrees-1];
                    prev_angle = num_degrees-1;

                    curr_range = laser_scan.ranges[angle];
                    curr_angle = angle;

                    curr_dist = abs(curr_range - prev_range);
                }
                else if (angle != 0)
                {
                    prev_range = laser_scan.ranges[angle - 1];
                    prev_angle = angle - 1;

                    curr_range = laser_scan.ranges[angle];
                    curr_angle = angle;

                    curr_dist = abs(curr_range - prev_range);
                }

                if (curr_dist < threshold && curr_dist != 0.0)      // if a cluster point is found
                {
                    if (prev_dist > 0.0)                           // if the previous point was not part of cluster
                    {
                        num_clusters += 1;
                        turtlelib::Vector2D lastpoint;
                        lastpoint.x = prev_range*std::cos(prev_angle*(PI/180));
                        lastpoint.y = prev_range*std::sin(prev_angle*(PI/180));

                        cluster.push_back(lastpoint);
                    }

                    turtlelib::Vector2D point;
                    point.x = curr_range*std::cos(angle*(PI/180));
                    point.y = curr_range*std::sin(angle*(PI/180));

                    cluster.push_back(point);

                    prev_dist = curr_dist;
                }

                // if (curr_dist > threshold || curr_dist == 0.0)
                if (curr_dist > threshold && prev_dist < threshold)
                {
                    if (cluster.size() > 3)
                    {
                        cluster_list.push_back(cluster);
                        cluster.clear();
                    }
                }
            }

            int point_count = 0;
            int id = 0;

            std::vector<nuslam::Circle> est_landmarks;
            est_landmarks.resize(cluster_list.size());
            
            // init class
            nuslam::CircleFit CircleBoy;

            for (int i = 0; i < cluster_list.size(); i++)
            {
                cluster = cluster_list.at(i);
                for (int j = 0; j < cluster.size(); j++)
                {
                    point_count++;
                }

                est_landmarks.at(i) = CircleBoy.detect_circle(cluster); 
                
            }

            visualization_msgs::MarkerArray cluster_arr;
            cluster_arr.markers.resize(point_count);

            for (int a = 0; a < cluster_list.size(); a++)
            {
                for (int b = 0; b < cluster_list.at(a).size(); b++)
                {
                    cluster_arr.markers[id].header.frame_id = "red_base_footprint";
                    cluster_arr.markers[id].header.stamp = ros::Time::now();
                    cluster_arr.markers[id].id = id;
                    cluster_arr.markers[id].type = visualization_msgs::Marker::SPHERE;
                    cluster_arr.markers[id].action = visualization_msgs::Marker::ADD;
                    cluster_arr.markers[id].pose.position.x = cluster_list.at(a).at(b).x;
                    cluster_arr.markers[id].pose.position.y = cluster_list.at(a).at(b).y;
                    cluster_arr.markers[id].pose.position.z = 0.3;
                    cluster_arr.markers[id].pose.orientation.x = 0.0;
                    cluster_arr.markers[id].pose.orientation.y = 0.0;
                    cluster_arr.markers[id].pose.orientation.z = 0.0;
                    cluster_arr.markers[id].pose.orientation.w = 1.0;
                    cluster_arr.markers[id].scale.x = 0.02;
                    cluster_arr.markers[id].scale.y = 0.02;
                    cluster_arr.markers[id].scale.z = 0.02;
                    cluster_arr.markers[id].color.r = 0.0;
                    cluster_arr.markers[id].color.g = 1.0;
                    cluster_arr.markers[id].color.b = 0.0;
                    cluster_arr.markers[id].color.a = 1.0;
                    
                    id++;
                }
            }

            cluster_pub.publish(cluster_arr);

            // displaying estimated landmark locations
            visualization_msgs::MarkerArray landmark_arr;
            landmark_arr.markers.resize(cluster_list.size());

            int lm_id = 0;

            for (int a = 0; a < cluster_list.size(); a++)
            {
                landmark_arr.markers[lm_id].header.frame_id = "red_base_footprint";
                landmark_arr.markers[lm_id].header.stamp = ros::Time::now();
                landmark_arr.markers[lm_id].id = lm_id;
                landmark_arr.markers[lm_id].type = visualization_msgs::Marker::CYLINDER;
                landmark_arr.markers[lm_id].action = visualization_msgs::Marker::ADD;
                landmark_arr.markers[lm_id].pose.position.x = est_landmarks.at(a).x;
                landmark_arr.markers[lm_id].pose.position.y = est_landmarks.at(a).y;
                landmark_arr.markers[lm_id].pose.position.z = 0.4;
                landmark_arr.markers[lm_id].pose.orientation.x = 0.0;
                landmark_arr.markers[lm_id].pose.orientation.y = 0.0;
                landmark_arr.markers[lm_id].pose.orientation.z = 0.0;
                landmark_arr.markers[lm_id].pose.orientation.w = 1.0;
                landmark_arr.markers[lm_id].scale.x = 2*est_landmarks.at(a).radius;
                landmark_arr.markers[lm_id].scale.y = 2*est_landmarks.at(a).radius;
                landmark_arr.markers[lm_id].scale.z = 0.05;
                landmark_arr.markers[lm_id].color.r = 0.0;
                landmark_arr.markers[lm_id].color.g = 1.0;
                landmark_arr.markers[lm_id].color.b = 0.0;
                landmark_arr.markers[lm_id].color.a = 1.0;
                
                lm_id++;
            }

            landmark_pub.publish(landmark_arr);

            cluster.clear();
            cluster_list.clear();
        }

        void main_loop(const ros::TimerEvent &) const
        {
            //implement the state machine here
            
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher cluster_pub;
        ros::Publisher landmark_pub;
        ros::Subscriber laser_sub;
        ros::Timer timer;
        int num_degrees;
        int num_clusters;
        std::vector<std::vector<turtlelib::Vector2D>> cluster_list;
        std::vector<turtlelib::Vector2D> cluster;
        // nuslam::CircleFit CircleBoy;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "landmarks");
    Landmarks node;
    ros::spin();
    return 0;
}