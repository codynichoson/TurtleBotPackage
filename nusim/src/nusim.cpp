/*******************************************************************************
 * NUSIM NODE
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
#include "std_msgs/UInt64.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sstream>
#include <iostream>
#include <std_srvs/Trigger.h>
#include "nusim/teleport.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <turtlelib/diff_drive.hpp>
#include <random>

static int rate = 100;
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

static turtlelib::WheelAngles wheelangles = {.left = 0.0, .right = 0.0};
static turtlelib::WheelAngles slip_wheelangles = {.left = 0.0, .right = 0.0};
static turtlelib::WheelVel wheelvel;
static turtlelib::Config new_config, hold_config;

int isResetting = 0;
int isTeleporting = 0;
static nuturtlebot_msgs::SensorData sensor_data;
static turtlelib::DiffDrive ddrive;

/// \brief Random number generator
/// \return mt
std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object eV2ry time get_random is called
     return mt;
 }

/// \brief Resets red robot to origin of world frame
/// \param req - Trigger
/// \return True
bool reset_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    timestep.data = 0;
    new_config.x = 0.0;
    new_config.y = 0.0;
    new_config.theta = 0.0;

    isResetting = 1;
    ROS_WARN("Let's try that again...");
    return true;
}

/// \brief Teleports red robot to specified configuration in world frame
/// \param q - Desired configuration in world frame
/// \return True
bool teleport_callback(nusim::teleport::Request &q, nusim::teleport::Response &res)
{
    new_config.x = q.x;
    new_config.y = q.y;
    new_config.theta = q.theta;

    isTeleporting = 1;
    ROS_WARN("Now I'm in a new place!");

    return true;
}

/// \brief Subscribes to wheel_cmd and populates sensor_data message
/// \param wheelcmd - wheel_cmd values in range (-256, 256)
/// \return None
void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands &wheelcmd)
{
    // Generate a gaussian variable:
    std::normal_distribution<> wheel_cmd_noise(1, 0.2); // (mean, variance)

    // adding noise
    wheelvel.left = wheel_cmd_noise(get_random()) * wheelcmd.left_velocity * motor_cmd_to_radsec;
    wheelvel.right = wheel_cmd_noise(get_random()) * wheelcmd.right_velocity * motor_cmd_to_radsec;
    
    // calculate encoder stuff to populate sensor_data
    int encoder_left = (int)(((wheelvel.left/rate) + slip_wheelangles.left)/encoder_ticks_to_rad);
    int encoder_right = (int)(((wheelvel.right/rate) + slip_wheelangles.right)/encoder_ticks_to_rad);

    sensor_data.left_encoder = encoder_left;
    sensor_data.right_encoder = encoder_right;
}

double distance(double x1, double y1, double x2, double y2){
    double distance = (std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2)));
    return distance;
}

/// \brief nusim node main function
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;

    int collision_flag = 0;

    nh.getParam("radius", radius);
    nh.getParam("height", height);
    nh.getParam("obs_x", obs_x);
    nh.getParam("obs_y", obs_y);
    nh.getParam("robot_start_x", robot_start_x);
    nh.getParam("robot_start_y", robot_start_y);
    nh.getParam("robot_start_theta", robot_start_theta);
    nh.getParam("x_length", x_length);
    nh.getParam("y_length", y_length);
    nh.getParam("wall_height", wall_height);
    nh.getParam("thickness", thickness);
    nh.getParam("/nusim/motor_cmd_to_radsec", motor_cmd_to_radsec);
    nhp.getParam("/nusim/encoder_ticks_to_rad", encoder_ticks_to_rad);
    nh.getParam("rate", rate);
    double basic_sensor_variance = 0.05; // add param
    double max_range = 2; // add param
    double collision_radius = 0.11; // add param

    new_config = {.x = robot_start_x, .y = robot_start_y, .theta = robot_start_theta};
    hold_config = {.x = robot_start_x, .y = robot_start_y, .theta = robot_start_theta};
    
    ros::Rate r(rate);

    // create transform broadcaster and broadcast message
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // create publishers
    ros::Publisher sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", rate);
    ros::Publisher timestep_pub = nhp.advertise<std_msgs::UInt64>("timestep", rate);
    ros::Publisher obstacles_pub = nhp.advertise<visualization_msgs::MarkerArray>("obstacles", 1, true);
    ros::Publisher walls_pub = nhp.advertise<visualization_msgs::MarkerArray>("walls", 1, true);
    ros::Publisher fake_sensor_pub = nhp.advertise<visualization_msgs::MarkerArray>("fake_sensor", 1, true);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
    ros::Publisher laser_pub = nhp.advertise<sensor_msgs::LaserScan>("laser_scan", 50);

    // create subscribers
    ros::Subscriber sub = nh.subscribe("wheel_cmd", 1000, wheel_cmd_callback);
    
    // create services
    ros::ServiceServer reset = nh.advertiseService("reset", reset_callback);
    ros::ServiceServer teleport = nh.advertiseService("teleport", teleport_callback);

    visualization_msgs::MarkerArray obs_arr;
    obs_arr.markers.resize(obs_x.size());

    visualization_msgs::MarkerArray fake_sensor_arr;
    fake_sensor_arr.markers.resize(obs_x.size());

    visualization_msgs::MarkerArray wall_arr;
    wall_arr.markers.resize(num_walls);

    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;

    int count = 0;

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

        // publish fake sensor markers
        for(int i = 0; i < obs_x.size(); i++){
            turtlelib::Vector2D new_config_vec = {new_config.x, new_config.y};
            turtlelib::Vector2D obs_vec = {obs_x[i], obs_y[i]};
            turtlelib::Transform2D Twr(new_config_vec, new_config.theta);
            turtlelib::Transform2D Trw = Twr.inv();
            turtlelib::Transform2D Twm(obs_vec);
            turtlelib::Transform2D Trm = Trw*Twm;
            turtlelib::Vector2D Trm_vec = Trm.translation();

            // Generate a gaussian variable:
            double basic_sensor_variance = 0.001; // make a param
            std::normal_distribution<> fake_sensor_noise(0, basic_sensor_variance); // (mean, variance)

            fake_sensor_arr.markers[i].header.frame_id = "red_base_footprint";
            fake_sensor_arr.markers[i].header.stamp = ros::Time::now();
            fake_sensor_arr.markers[i].id = i;
            fake_sensor_arr.markers[i].type = visualization_msgs::Marker::CYLINDER;
            fake_sensor_arr.markers[i].action = visualization_msgs::Marker::ADD;
            fake_sensor_arr.markers[i].pose.position.x = Trm_vec.x + fake_sensor_noise(get_random());
            fake_sensor_arr.markers[i].pose.position.y = Trm_vec.y + fake_sensor_noise(get_random());;
            fake_sensor_arr.markers[i].pose.position.z = height/2;
            fake_sensor_arr.markers[i].pose.orientation.x = 0.0;
            fake_sensor_arr.markers[i].pose.orientation.y = 0.0;
            fake_sensor_arr.markers[i].pose.orientation.z = 0.0;
            fake_sensor_arr.markers[i].pose.orientation.w = 1.0;
            fake_sensor_arr.markers[i].scale.x = 2*radius;
            fake_sensor_arr.markers[i].scale.y = 2*radius;
            fake_sensor_arr.markers[i].scale.z = height;
            fake_sensor_arr.markers[i].color.r = 1.0;
            fake_sensor_arr.markers[i].color.g = 1.0;
            fake_sensor_arr.markers[i].color.b = 0.0;
            fake_sensor_arr.markers[i].color.a = 1.0;
        }

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
            wall_arr.markers[i].color.r = 1.0;
            wall_arr.markers[i].color.g = 0.0;
            wall_arr.markers[i].color.b = 0.0;
            wall_arr.markers[i].color.a = 1.0;
        }
        walls_pub.publish(wall_arr);

        // Generate a gaussian variable:
        std::uniform_real_distribution<> left_noise(1, 1.05); // (mean, variance)
        std::uniform_real_distribution<> right_noise(1, 1.05); // (mean, variance)

        double left_slip = left_noise(get_random())*wheelvel.left/rate;
        double right_slip = right_noise(get_random())*wheelvel.right/rate;
            
        wheelangles = {.left = ((wheelvel.left/rate)+wheelangles.left), .right = ((wheelvel.right/rate)+wheelangles.right)};
        slip_wheelangles = {.left = slip_wheelangles.left + left_slip, .right = slip_wheelangles.right + right_slip};
        
        // get new config
        new_config = ddrive.fKin(wheelangles, new_config);

        // check if new config is in a collision state
        for(int i = 0; i < 3; i++){
            double distance = std::sqrt(std::pow(obs_x[i] - new_config.x, 2) + std::pow(obs_y[i] - new_config.y, 2));
            if (distance < (collision_radius + radius)){
                collision_flag+=1;
            }
        }

        // if not, hold_config gets updated
        // if so, new_config becomes hold_config
        if (collision_flag == 0){
            hold_config = new_config;
        }
        else{
            new_config = hold_config;
        }

        collision_flag = 0;

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

        // create path and publish
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = new_config.x;
        pose.pose.position.y = new_config.y;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "world";
        path.poses.push_back(pose);
        path_pub.publish(path);

        int num_readings = 360;
        double laser_frequency = 5;
        double ranges[num_readings];
        double scan_time = 1/5;
        double angle_increment = 2*turtlelib::PI / 360;

        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = "red_base_scan";
        scan.angle_min = 0.0;
        scan.angle_max = 2*turtlelib::PI;
        scan.angle_increment = angle_increment;
        scan.time_increment = 1/1800;
        scan.range_min = 0.120;
        scan.range_max = 3.5;
        scan.ranges.resize(num_readings);

        turtlelib::Vector2D Vwr = {.x = new_config.x, .y = new_config.y};
        turtlelib::Transform2D Twr(Vwr, new_config.theta); // world to laser

        double x1, y1;
        double x2, y2;
        double laser_min = 0.12;
        double laser_max = 3.5;

        // iterate through obstacles
        for (int obs = 0; obs < 3; obs++){
            turtlelib::Vector2D Vwm = {.x = obs_x[obs], .y =obs_y[obs]};
            turtlelib::Transform2D Twm(Vwm); // world to obstacle

            // iterate through each laser angle
            for (int i = 0; i < num_readings; i++){

                // find laser (robot) coordinates in marker frame
                turtlelib::Transform2D Tmr = Twm.inv()*Twr;
                turtlelib::Vector2D Vmr = Tmr.translation();

                turtlelib::Transform2D Trm = Tmr.inv();
                turtlelib::Vector2D Vrm = Trm.translation();

                // find slope of laser scan line
                double angle = i*angle_increment;
                double m = std::tan(angle);
                
                // find first point at beginning of laser range
                x1 = laser_min*std::cos(angle);
                y1 = laser_min*std::sin(angle);

                // find second point at end of laser range
                x2 = laser_max*std::cos(angle);
                y2 = laser_max*std::sin(angle);

                // conV2rt points to marker frame
                turtlelib::Vector2D V1r, V2r;
                V1r.x = x1; V1r.y = y1;
                V2r.x = x2; V2r.y = y2;
                turtlelib::Transform2D T1r(V1r), T2r(V2r);
                turtlelib::Transform2D Tm1 = Tmr*T1r.inv();
                turtlelib::Transform2D Tm2 = Tmr*T2r.inv();
                turtlelib::Vector2D Vm1 = Tm1.translation();
                turtlelib::Vector2D Vm2 = Tm2.translation();
                x1 = Vm1.x; y1 = Vm1.y;
                x2 = Vm2.x; y2 = Vm2.y;

                // check if line from laser at certain angle will intersect obstacle between first and second points
                double dx = x2 - x1;
                double dy = y2 - y1;
                double dr = std::sqrt(dx*dx + dy*dy);
                double D = x1*y2 - x2*y1;
                double discriminant = collision_radius*collision_radius*dr*dr - D*D;
                double sgn;
                if (dy < 0){
                    sgn = -1.0;
                }
                else{
                    sgn = 1.0;
                }

                // if so, calculate the two intersection points with the circle
                turtlelib::Vector2D Vmi1, Vmi2, Vmi;
                Vmi1.x = (D*dy + sgn*dx*std::sqrt(collision_radius*collision_radius*dr*dr - D*D)) / (dr*dr);
                Vmi1.y = (-D*dx + std::abs(dy)*std::sqrt(collision_radius*collision_radius*dr*dr - D*D)) / (dr*dr);
                Vmi2.x = (D*dy - sgn*dx*std::sqrt(collision_radius*collision_radius*dr*dr - D*D)) / (dr*dr);
                Vmi2.y = (-D*dx - std::abs(dy)*std::sqrt(collision_radius*collision_radius*dr*dr - D*D)) / (dr*dr);

                turtlelib::Transform2D Tmi1(Vmi1), Tmi2(Vmi2), Tri1, Tri2;
                Tri1 = Tmr.inv()*Tmi1;
                Tri2 = Tmr.inv()*Tmi2;

                turtlelib::Vector2D Vri1, Vri2;
                Vri1 = Tri1.translation();
                Vri2 = Tri2.translation();
                
                // only use intersection point closest to robot
                if (distance(laser_min, 0.0, Vri1.x, Vri1.y) < distance(laser_min, 0.0, Vri2.x, Vri2.y)){
                    Vmi = Vmi1;
                }
                else{
                    Vmi = Vmi2;
                }

                turtlelib::Transform2D Tmi(Vmi);

                // intersection relative to robot
                turtlelib::Transform2D Tri = Tmr.inv()*Tmi;
                turtlelib::Vector2D Vri = Tri.translation();

                // calculate plotted location of intersection (some may be "reflections")
                double px = distance(0.0, 0.0, Vri.x, Vri.y)*std::cos(angle);
                double py = distance(0.0, 0.0, Vri.x, Vri.y)*std::sin(angle);

                if (discriminant > 0 && distance(px, py, Vrm.x, Vrm.y) < distance(0.0, 0.0, Vrm.x, Vrm.y)){
                    scan.ranges[i] = distance(Vri.x, Vri.y, 0, 0);
                }
            }
        }

        
    
        if (std::fmod(count, 20) == 0){
            fake_sensor_pub.publish(fake_sensor_arr);
            laser_pub.publish(scan);
        }

        count+=1;

        sensor_pub.publish(sensor_data);

        isTeleporting = 0;
        isResetting = 0;

        count+=1;
        
        r.sleep();
        ros::spinOnce();
    }
    return 0;   
}