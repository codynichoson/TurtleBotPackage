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
        Twist2D twist = Ang2Twist(new_wheel_angles);
        Transform2D tf = integrate_twist(twist);
        Vector2D vec = tf.translation();
        double theta = tf.rotation();
        Config config;
        config.x = vec.x; 
        config.y = vec.y; 
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

    Twist2D DiffDrive::Ang2Twist(WheelAngles wheel_angles){
        Twist2D twist;
        twist.xdot = (r*(wheel_angles.left + wheel_angles.right))/2.0;
        twist.ydot = 0.0;
        twist.thetadot = (r/2.0*D)*(wheel_angles.right - wheel_angles.left);

        return twist;
    };
}