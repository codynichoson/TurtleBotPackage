#include "ros/ros.h"
// #include <ros/console.h>
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
    DiffDrive::DiffDrive()
    {
        config.x = 0.0; config.y = 0.0; config.theta = 0.0;
        wheelangles.left = 0.0; wheelangles.right = 0.0;
        wheelvels.left = 0.0; wheelvels.right = 0.0;
    }

    DiffDrive::DiffDrive(double x, double y, double theta)
    {
        config.x = x; config.y = y; config.theta = theta;
        // wheelangles.left = 0.0; wheelangles.right = 0.0;
        // wheelvels.left = 0.0; wheelvels.right = 0.0;
    }

    Config DiffDrive::fKin(WheelAngles new_wheel_angles){
        wheelvels.left = (new_wheel_angles.left - wheelangles.left);
        wheelvels.right = (new_wheel_angles.right - wheelangles.right);

        wheelangles.left = new_wheel_angles.left;
        wheelangles.right = new_wheel_angles.right;

        Twist2D twist;
        twist.thetadot = (r/(2*D))*(-wheelvels.left+wheelvels.right);
        twist.xdot = (r/2)*(wheelvels.left+wheelvels.right);
        twist.ydot = 0.0;
        
        Vector2D trans; 
        trans.x = config.x; 
        trans.y = config.y;
        double rot = config.theta;
        
        Transform2D Twb, Tbbp, Twbp;
        Twb = Transform2D(trans, rot);
        Tbbp = integrate_twist(twist);
        Twbp = Twb*Tbbp;

        Vector2D new_trans = Twbp.translation();
        double new_theta = normalizeAngle(Twbp.rotation());

        config.x = new_trans.x; 
        config.y = new_trans.y; 
        config.theta = new_theta;

        Config config_return;
        config_return = config;

        return config_return;
    };

    WheelVel DiffDrive::invKin(Twist2D twist){
        double left_wheel_vel = (-D/r)*twist.thetadot + (1/r)*twist.xdot;
        double right_wheel_vel = (D/r)*twist.thetadot + (1/r)*twist.xdot;

        if (twist.ydot != 0){
            throw logic_error("Oh no! Robot is sliding sideways!");
        }
        
        return {left_wheel_vel, right_wheel_vel};
    };

    Twist2D DiffDrive::Ang2Twist(WheelAngles new_wheel_angles){
        wheelvels.left = new_wheel_angles.left - wheelangles.left;
        wheelvels.right = new_wheel_angles.right - wheelangles.right;

        Twist2D twist;
        twist.xdot = (r/2)*(wheelvels.left + wheelvels.right);
        twist.ydot = 0.0;
        twist.thetadot = (r/(2*D))*(wheelvels.right - wheelvels.left);

        return twist;
    }

    Twist2D DiffDrive::Vel2Twist(WheelVel wheel_vels){
        Twist2D twist;
        twist.xdot = (r*(wheel_vels.left + wheel_vels.right))/2.0;
        twist.ydot = 0.0;
        twist.thetadot = (r/2.0*D)*(wheel_vels.right - wheel_vels.left);

        return twist;
    };
}