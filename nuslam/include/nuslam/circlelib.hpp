#ifndef CIRCLELIB_INCLUDE_GUARD_HPP
#define CIRCLELIB_INCLUDE_GUARD_HPP

/// \file
/// \brief Circle detection Library

#include <turtlelib/diff_drive.hpp>
#include <armadillo>

namespace nuslam
{
    /// \brief Circle measurements
    struct Circle
    {
        /// \brief radius of circle
        double radius = 0.0;

        /// \brief x-coordinate of center
        double x = 0.0;

        /// \brief y-coordinate of center
        double y = 0.0;
    };

    class CircleFit
    {
        private:

            int n {};
            int size {};
            arma::mat I {};

        public:

            // /// \brief class constructor to initialize private parameters
            // /// \return none
            // CircleFit();

            /// \brief calculate centroid of points
            /// \param cluster - a cluster of points
            /// \return centroid - the centroid of the cluster 
            Circle detect_circle(std::vector<turtlelib::Vector2D>);
    };
}

#endif