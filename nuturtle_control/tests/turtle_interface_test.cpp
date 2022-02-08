#include <catch_ros/catch.hpp>
#include <ros/ros.h>


TEST_CASE("cmd_vel to wheel_cmd, pure translation", "[wheel_cmd]") {
  
  ros::NodeHandle nh("~"); // this initializes time

  ros::Time time_now = ros::Time::now();

  ros::Time time_future = time_now + ros::Duration(5.0);

  REQUIRE(time_future > time_now);

}