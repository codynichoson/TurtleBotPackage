#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include "nuturtle_control/set_pose.h"
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

double pose_x;
double pose_y;

void odom_callback(nav_msgs::Odometry msg)
{
  pose_x = msg.pose.pose.position.x;
  pose_y = msg.pose.pose.position.y;
}

TEST_CASE("Odometry Test") {
  
  ros::NodeHandle nh;
  ros::Rate r(100);

  ros::ServiceClient set_pose_client = nh.serviceClient<nuturtle_control::set_pose>("/set_pose");

  ros::Subscriber odom_sub = nh.subscribe("/odom", 100, odom_callback);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  SECTION("Test /set_pose service"){

    // set_pose_client.waitForExistence();
    
    nuturtle_control::set_pose pose;
    pose.request.x = 1.0;
    pose.request.y = 2.0;
    pose.request.theta = 1.57;

    set_pose_client.call(pose);

    for (int i = 0; i < 100; i++){
        r.sleep();
        ros::spinOnce();
    }

    CHECK(pose_x == Approx(1.0));
    CHECK(pose_y == Approx(2.0));
  }

  SECTION("Test transform from odom to base_print"){

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("odom", "blue_base_footprint", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
    for (int i = 0; i < 100; i++){
        r.sleep();
        ros::spinOnce();
    }

    CHECK(transformStamped.transform.translation.x == Approx(0.0));
    CHECK(transformStamped.transform.translation.y == Approx(0.0));
    CHECK(transformStamped.transform.rotation.z == Approx(0.0));
    
  }
}