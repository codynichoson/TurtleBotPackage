#include <iostream>
#include <cmath>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

/// \brief PI.  Not in C++ standard until C++20.
constexpr double PI=3.14159265358979323846;

/// \brief Distance between wheels on TurtleBot.
constexpr double D = 0.08;

/// \brief Wheel radius on TurtleBot.
constexpr double r = 0.033;

namespace turtlelib{
    DiffDrive::DiffDrive(){
        config.x = 0.0;
        config.y = 0.0;
        config.theta = 0.0;
        old_angles.pos_l = 0.0;
        old_angles.po_r = 0.0;
    }

    Config DiffDrive::fKin(WheelPos new_wheel_pos){
        Twist2D twist;
        twist.xdot = (r*(new_wheel_pos.pos_l + new_wheel_pos.pos_r))/2.0;
        twist.thetadot = (r/2.0*D)*(new_wheel_pos.pos_r - new_wheel_pos.pos_l);
        Transform2D tf = integrate_twist(twist);
        Config new_config;
        new_config.x = tf.x; 
        new_config.y = 0.0, 
        new_config.theta = tf.theta;

        return config+=new_config;
    }

    WheelVel DiffDrive::invKin(Twist2D twist){
        double theta1dot = (-D/r)*twist.theta + (1/r)*twist.x;
        double theta2dot = (D/r)*twist.theta + (1/r)*twist.x;
        
        return {theta1dot, theta2dot};
    }
}