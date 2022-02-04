#include <iostream>
#include <cmath>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
using std::logic_error;

/// \brief PI.  Not in C++ standard until C++20.
constexpr double PI=3.14159265358979323846;

/// \brief Distance between wheels on TurtleBot.
constexpr double D = 0.08;

/// \brief Wheel radius on TurtleBot.
constexpr double r = 0.033;

namespace turtlelib{

    Config DiffDrive::fKin(WheelAngles new_wheel_angles){
        Twist2D twist;
        twist.xdot = (r*(new_wheel_angles.left + new_wheel_angles.right))/2.0;
        twist.thetadot = (r/2.0*D)*(new_wheel_angles.right - new_wheel_angles.left);
        Transform2D tf = integrate_twist(twist);
        Vector2D vec = tf.translation();
        double theta = tf.rotation();
        Config config;
        config.x = vec.x; 
        config.y = 0.0, 
        config.theta = theta;

        return config;
    };

    WheelVel DiffDrive::invKin(Twist2D twist){
        double theta1dot = (-D/r)*twist.thetadot + (1/r)*twist.xdot;
        double theta2dot = (D/r)*twist.thetadot + (1/r)*twist.xdot;

        if (twist.ydot != 0){
            throw logic_error("Oh no! Robot is sliding sideways!");
        }
        
        return {theta1dot, theta2dot};
    };
}