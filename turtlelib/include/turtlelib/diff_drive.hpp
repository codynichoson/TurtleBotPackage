#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Differential drive calculations.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include"turtlelib/rigid2d.hpp"

namespace turtlelib
{
    /// \brief Wheel positions
    struct WheelPos
    {
        /// \brief right wheel angle
        double phi_r = 0.0;

        /// \brief left wheel angle
        double phi_l = 0.0;
    };

    /// \brief Wheel velocities
    struct WheelVel
    {
        /// \brief right wheel velocity
        double v_r= 0.0;

        /// \brief left wheel velocity
        double v_l = 0.0;
    };

    /// \brief 2D robot configuration
    struct Config
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief the angle theta
        double theta = 0.0;
    };


    /// \brief a rigid body transformation in 2 dimensions
    class DiffDrive
    {
    public:
        /// \brief update robot configuration using forward kinematics
        /// \param wheel_pos - wheel angles
        /// \return updated configuration
        Config fKin(WheelPos new_wheel_pos, Config old_config);

        /// \brief update robot configuration using forward kinematics
        /// \param wheel_pos - wheel angles
        /// \return updated configuration
        WheelVel invKin(Twist2D twist);



    private:
        Config config;
        WheelPos old_angles;
    };

}