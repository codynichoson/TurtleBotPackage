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
    struct WheelAngles
    {
        /// \brief left wheel angle
        double left = 0.0;

        /// \brief right wheel angle
        double right = 0.0;
    };

    /// \brief Wheel velocities
    struct WheelVel
    {
        /// \brief left wheel velocity
        double left = 0.0;

        /// \brief right wheel velocity
        double right = 0.0;
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
        DiffDrive();

        DiffDrive(double x, double y, double theta);

        /// \brief update robot configuration using forward kinematics
        /// \param new_wheel_angles - wheel angles
        /// \return updated configuration
        Config fKin(WheelAngles new_wheel_angles);

        /// \brief update robot wheel velocities using inverse kinematics
        /// \param twist - inputted twist
        /// \return updated wheel velocities
        WheelVel invKin(Twist2D twist);

        /// \brief convert wheel angles to twist
        /// \param wheel_angles - left and right wheel angles
        /// \return a twisty treat!
        Twist2D Ang2Twist(WheelAngles wheel_angles);

        /// \brief convert wheel velocities to twist
        /// \param wheel_angles - left and right wheel velocities
        /// \return a twisty treat!
        Twist2D Vel2Twist(WheelVel wheel_vels);

    private:
        Config config;
        WheelAngles wheelangles;
        WheelVel wheelvels;
    };
}

#endif