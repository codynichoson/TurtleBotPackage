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
            /// \brief calculate centroid of points
            /// \param cluster - a cluster of points
            /// \return centroid - the centroid of the cluster 
            Circle detect_circle(std::vector<turtlelib::Vector2D>);

            /// \brief cleanse the cluster_list of any non-circle clusters
            /// \param cluster_list - a vector containing multiple clusters
            /// \return cluster_list_circles_only - the cluster list with any non-circles removed
            std::vector<std::vector<turtlelib::Vector2D>> circles_only(std::vector<std::vector<turtlelib::Vector2D>>);
    };
}

#endif