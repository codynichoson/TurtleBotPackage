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
static turtlelib::Config config;

int isResetting = 0;
int isTeleporting = 0;
static nuturtlebot_msgs::SensorData sensor_data;
static turtlelib::DiffDrive ddrive;

static double wheel_cmd_mean = 0.0;
static double wheel_cmd_variance = 0.01;

static double fake_sensor_mean = 0.0;
static double fake_sensor_variance = 0.01;

static double left_slip_mean = -0.01;
static double left_slip_variance = 0.01;

static double right_slip_mean = -0.01;
static double right_slip_variance = 0.01;

static double laser_noise_mean = 0.0;
static double laser_noise_variance = 0.01;

/// \brief Random number generator
/// \return mt
std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number generator object. This is always the
     // same object eV2ry time get_random is called
     return mt;
 }

/// \brief Resets red robot to origin of world frame
/// \param req - Trigger
/// \return True
bool reset_callback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &)
{
    timestep.data = 0;
    config.x = 0.0;
    config.y = 0.0;
    config.theta = 0.0;

    isResetting = 1;
    ROS_WARN("Let's try that again...");
    return true;
}

/// \brief Teleports red robot to specified configuration in world frame
/// \param q - Desired configuration in world frame
/// \return True
bool teleport_callback(nusim::teleport::Request &q, nusim::teleport::Response &)
{
    config.x = q.x;
    config.y = q.y;
    config.theta = q.theta;

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
    // std::normal_distribution<> wheel_cmd_noise(0, 0.01); // (mean, variance)
    std::normal_distribution<> wheel_cmd_noise(wheel_cmd_mean, wheel_cmd_variance); // (mean, variance)

    // adding noise
    wheelvel.left = (wheelcmd.left_velocity * motor_cmd_to_radsec) + wheel_cmd_noise(get_random());
    wheelvel.right = (wheelcmd.right_velocity * motor_cmd_to_radsec) + wheel_cmd_noise(get_random());
    
    // calculate encoder stuff to populate sensor_data
    int encoder_left = (int)(((wheelvel.left/rate) + slip_wheelangles.left)/encoder_ticks_to_rad);
    int encoder_right = (int)(((wheelvel.right/rate) + slip_wheelangles.right)/encoder_ticks_to_rad);

    sensor_data.left_encoder = encoder_left;
    sensor_data.right_encoder = encoder_right;
}

/// \brief Calculates distance between two points
/// \param x1 - first x coordinate
/// \param y1 - first y coordinate
/// \param x2 - second x coordinate
/// \param y2 - second y coordinate
/// \return distance between two points
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

    // int collision_flag = 4;
    // double fake_sensor_variance;
    double max_laser_range;
    double collision_radius_robot;

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
    // nh.getParam("fake_sensor_variance", fake_sensor_variance);
    nh.getParam("max_laser_range", max_laser_range);
    nh.getParam("collision_radius_robot", collision_radius_robot);
    nh.getParam("wheel_cmd_mean", wheel_cmd_mean);
    nh.getParam("wheel_cmd_variance", wheel_cmd_variance);

    config = {.x = robot_start_x, .y = robot_start_y, .theta = robot_start_theta};
    
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
    // ros::Publisher laser_pub = nhp.advertise<sensor_msgs::LaserScan>("scan", 50);

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

        int obsx_size = obs_x.size();

        for(int i = 0; i < obsx_size; i++){
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
        for(int i = 0; i < obsx_size; i++){
            turtlelib::Vector2D config_vec = {config.x, config.y};
            turtlelib::Vector2D Vwm = {obs_x[i], obs_y[i]};
            turtlelib::Transform2D Twr(config_vec, config.theta);
            turtlelib::Transform2D Trw = Twr.inv();
            turtlelib::Transform2D Twm(Vwm);
            turtlelib::Transform2D Trm = Trw*Twm;
            turtlelib::Vector2D Vrm = Trm.translation();

            // Generate a gaussian variable:
            std::normal_distribution<> fake_sensor_noise(fake_sensor_mean, fake_sensor_variance);

            fake_sensor_arr.markers[i].header.frame_id = "red_base_footprint";
            fake_sensor_arr.markers[i].header.stamp = ros::Time::now();
            fake_sensor_arr.markers[i].id = i;
            fake_sensor_arr.markers[i].type = visualization_msgs::Marker::CYLINDER;

            if (distance(0.0, 0.0, Vrm.x, Vrm.y) < max_laser_range){
                fake_sensor_arr.markers[i].action = visualization_msgs::Marker::ADD;
            }
            else{
                fake_sensor_arr.markers[i].action = visualization_msgs::Marker::DELETE;
            }
            
            fake_sensor_arr.markers[i].pose.position.x = Vrm.x + fake_sensor_noise(get_random());
            fake_sensor_arr.markers[i].pose.position.y = Vrm.y + fake_sensor_noise(get_random());;
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
        std::uniform_real_distribution<> left_noise(left_slip_mean, left_slip_variance);
        std::uniform_real_distribution<> right_noise(right_slip_mean, right_slip_variance);

        double left_slip = wheelvel.left/rate + left_noise(get_random());
        double right_slip = wheelvel.right/rate + left_noise(get_random());
            
        wheelangles = {.left = ((wheelvel.left/rate)+wheelangles.left), .right = ((wheelvel.right/rate)+wheelangles.right)};
        slip_wheelangles = {.left = slip_wheelangles.left + left_slip, .right = slip_wheelangles.right + right_slip};
        
        // get new config
        config = ddrive.fKin(wheelangles, config);

        // check if new config is in a collision state
        for(int i = 0; i < 3; i++){
            double distance = std::sqrt(std::pow(obs_x[i] - config.x, 2) + std::pow(obs_y[i] - config.y, 2));
            if (distance < (collision_radius_robot + radius)){
                double ang = std::atan2((config.y - obs_y[i]),(config.x - obs_x[i]));
                config.x = obs_x[i] + (collision_radius_robot + radius)*std::cos(ang);
                config.y = obs_y[i] + (collision_radius_robot + radius)*std::sin(ang);
            }
        }

        // populate transform and publish
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red_base_footprint";
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

        // create path and publish
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = config.x;
        pose.pose.position.y = config.y;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "world";
        path.poses.push_back(pose);
        path_pub.publish(path);

        int num_readings = 360;
        // double laser_frequency = 5;
        // double ranges[num_readings];
        // double scan_time = 1/5;
        double angle_increment = 2*turtlelib::PI / 360;
        double laser_min = 0.12;
        double laser_max = 3.5;
        // double laser_max = 1.0;

        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = "red_base_footprint";
        scan.angle_min = 0.0;
        scan.angle_max = 2*turtlelib::PI;
        scan.angle_increment = angle_increment;
        scan.time_increment = 1/1800;
        scan.range_min = laser_min;
        scan.range_max = laser_max;
        scan.ranges.resize(num_readings);

        turtlelib::Vector2D Vwr = {.x = config.x, .y = config.y};
        turtlelib::Transform2D Twr(Vwr, config.theta); // world to laser

        double x1, y1;
        double x2, y2;

        // iterate through obstacles
        for (int obs = 0; obs < 3; obs++){
            turtlelib::Vector2D Vwm = {.x = obs_x[obs], .y =obs_y[obs]};
            turtlelib::Transform2D Twm(Vwm); // world to obstacle

            // iterate through each laser angle
            for (int i = 0; i < num_readings; i++){

                // find laser (robot) coordinates in marker frame
                turtlelib::Transform2D Tmr = Twm.inv()*Twr;
                // turtlelib::Vector2D Vmr = Tmr.translation();

                turtlelib::Transform2D Trm = Tmr.inv();
                turtlelib::Vector2D Vrm = Trm.translation();

                // find slope of laser scan line
                double angle = i*angle_increment;
                // double m = std::tan(angle);
                
                // find first point at beginning of laser range
                x1 = laser_min*std::cos(angle);
                y1 = laser_min*std::sin(angle);

                // find second point at end of laser range
                x2 = laser_max*std::cos(angle);
                y2 = laser_max*std::sin(angle);

                // convert points to marker frame
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
                double discriminant = collision_radius_robot*collision_radius_robot*dr*dr - D*D;
                double sgn;
                if (dy < 0){
                    sgn = -1.0;
                }
                else{
                    sgn = 1.0;
                }

                // calculate both intersection points of obstacle, in marker frame
                turtlelib::Vector2D Vmi1, Vmi2, Vmi;
                Vmi1.x = (D*dy + sgn*dx*std::sqrt(collision_radius_robot*collision_radius_robot*dr*dr - D*D)) / (dr*dr);
                Vmi1.y = (-D*dx + std::abs(dy)*std::sqrt(collision_radius_robot*collision_radius_robot*dr*dr - D*D)) / (dr*dr);
                Vmi2.x = (D*dy - sgn*dx*std::sqrt(collision_radius_robot*collision_radius_robot*dr*dr - D*D)) / (dr*dr);
                Vmi2.y = (-D*dx - std::abs(dy)*std::sqrt(collision_radius_robot*collision_radius_robot*dr*dr - D*D)) / (dr*dr);

                // convert intersections to robot frame
                turtlelib::Transform2D Tmi1(Vmi1), Tmi2(Vmi2), Tri1, Tri2;
                Tri1 = Tmr.inv()*Tmi1;
                Tri2 = Tmr.inv()*Tmi2;
                turtlelib::Vector2D Vri1, Vri2;
                Vri1 = Tri1.translation();
                Vri2 = Tri2.translation();
                
                // only use intersection point closest to robot, convert to marker frame
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

                std::normal_distribution<> laser_scanner_noise(laser_noise_mean, laser_noise_variance);

                if (discriminant > 0 && distance(px, py, Vrm.x, Vrm.y) < distance(0.0, 0.0, Vrm.x, Vrm.y)){
                    // scan.ranges[i] = distance(Vri.x, Vri.y, 0, 0) + laser_scanner_noise(get_random());
                    scan.ranges[i] = distance(Vri.x, Vri.y, 0, 0);
                    if (scan.ranges[i] >= laser_max){
                        scan.ranges[i] = 0.0;
                    }
                }
            }
        }

        // calculate wall-laser intersections
        double a[] = {0.0, 1.0, 0.0, 1.0};
        double b[] = {1.0, 0.0, 1.0, 0.0};
        double c[] = {-(y_length/2), -(x_length/2), -(-y_length/2), -(-x_length/2)};

        for (int j = 0; j < 4; j++){
            for (int i = 0; i < num_readings; i++){
                double angle = i*angle_increment + config.theta;
                // double angle = i*angle_increment;
                double m = std::tan(angle);

                // in world frame (?)
                double a1, b1, c1, a2, b2, c2;
                a1 = -m;
                b1 = 1;
                c1 = -(config.y - m*config.x);
                a2 = a[j];
                b2 = b[j];
                c2 = c[j];

                // in world frame
                turtlelib::Vector2D Vwwall;
                Vwwall.x = (b1*c2 - b2*c1)/(a1*b2 - a2*b1);
                Vwwall.y = (c1*a2 - c2*a1)/(a1*b2 - a2*b1);

                turtlelib::Transform2D Twwall(Vwwall);
                turtlelib::Transform2D Trwall = Twr.inv()*Twwall;
                turtlelib::Vector2D Vrwall = Trwall.translation();

                // calculate plotted location of intersection (some may be "reflections")
                double wx = distance(0.0, 0.0, Vrwall.x, Vrwall.y)*std::cos(i*angle_increment);
                double wy = distance(0.0, 0.0, Vrwall.x, Vrwall.y)*std::sin(i*angle_increment);

                std::normal_distribution<> laser_scanner_noise(laser_noise_mean, laser_noise_variance);

                if (distance(wx, wy, Vrwall.x, Vrwall.y) < distance(0.0, 0.0, Vrwall.x, Vrwall.y)){
                    double check = distance(0.0, 0.0, wx, wy);
                    if (scan.ranges[i] == 0 || check < scan.ranges[i]){
                        scan.ranges[i] = distance(Vrwall.x, Vrwall.y, 0.0, 0.0) + laser_scanner_noise(get_random());
                        if (scan.ranges[i] >= 3.5){
                            scan.ranges[i] = 0.0;
                        }
                    }
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