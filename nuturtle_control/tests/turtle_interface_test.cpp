#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <ros/console.h>
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/rigid2d.hpp>
#include <sensor_msgs/JointState.h>

constexpr double PI=3.14159265358979323846;

int wheel_cmd_left;
int wheel_cmd_right;
double js_position_left;
double js_position_right;
double js_velocity_left;
double js_velocity_right;

void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands &msg)
{
  wheel_cmd_left = msg.left_velocity;
  wheel_cmd_right = msg.right_velocity;
}

void joint_states_callback(const sensor_msgs::JointState &msg)
{
  js_position_left = msg.position.at(0);
  js_position_right = msg.position.at(1);
  // js_velocity_left = msg.velocity.at(0);
  // js_velocity_right = msg.velocity.at(1);
}

TEST_CASE("turtle_interface test") {
  
  ros::NodeHandle nh;
  ros::Rate r(100);

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Publisher sensor_data_pub = nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 100);

  ros::Subscriber wheel_cmd_sub = nh.subscribe("wheel_cmd", 100, wheel_cmd_callback);
  ros::Subscriber joint_states_sub = nh.subscribe("joint_states", 100, joint_states_callback);

  SECTION("cmd_vel to wheel_cmd, pure translation"){
    geometry_msgs::Twist twist;
    twist.linear.x = 0.11;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    for (int i = 0; i < 100; i++){
      cmd_vel_pub.publish(twist);
      r.sleep();
      ros::spinOnce();
    }

    CHECK(wheel_cmd_left == Approx(138));
    CHECK(wheel_cmd_right == Approx(138));
  }

  SECTION("cmd_vel to wheel_cmd, pure rotation"){
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.2;

    for (int i = 0; i < 100; i++){
      cmd_vel_pub.publish(twist);
      r.sleep();
      ros::spinOnce();
    }
    
    CHECK(wheel_cmd_left == Approx(-20));
    CHECK(wheel_cmd_right == Approx(20));
  }

  SECTION("encoder data to joint_states"){

    nuturtlebot_msgs::SensorData sensor_data;
    sensor_data.left_encoder = 1024;
    sensor_data.right_encoder = 2048;

    for (int i = 0; i < 100; i++){
      sensor_data_pub.publish(sensor_data);
      r.sleep();
      ros::spinOnce();
    }
    
    CHECK(js_position_left == Approx(PI/2));
    CHECK(js_position_right == Approx(PI));
  }
  
}