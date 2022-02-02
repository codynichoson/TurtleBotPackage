#include <iostream>
#include <cmath>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

constexpr double PI=3.14159265358979323846;
constexpr double D = 0.08;
constexpr double r = 0.033;

namespace turtlelib{
    DiffDrive::DiffDrive(){
        config.x = 0.0;
        config.x = 0.0;
        config.theta = 0.0;
        old_angles.phi_l = 0.0;
        old_angles.phi_r = 0.0;
    }

    Config DiffDrive::fKin(WheelPos new_wheel_pos, Config old_config){
        
    }

    WheelVel DiffDrive::invKin(Twist2D twist){
        double theta1dot = (-D/r)*twist.theta + (1/r)*twist.x;
        double theta2dot = (D/r)*twist.theta + (1/r)*twist.x;
        
        return {theta1dot, theta2dot};
    }
    turtlelib::DiffDrive diff();
}