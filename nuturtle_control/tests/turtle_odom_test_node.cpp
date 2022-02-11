

//set service client
// ros::ServiceClient setPoseClient = n.serviceClient<nutrutle_control::SetPose>("/set_pose");

// SECTION()
//     create service input

//     setPoseClient.waitForExistence();

//     if(setPoseCloent.call(variable)){

//     }
//     else{
//         ROS_ERROR_STREAM("no bueno");
//     }

//     r.sleep();
//     ros::spinOnce();

//     CHECK

TEST_CASE("Odometry Test") {
  
  ros::NodeHandle nh;
  ros::Rate r(100);

  SECTION("Test /set_pose service"){
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
}