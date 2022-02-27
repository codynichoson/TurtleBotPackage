#ifndef SLAMLIB_INCLUDE_GUARD_HPP
#define SLAMLIB_INCLUDE_GUARD_HPP

/// \file
/// \brief SLAM Library

#include <turtlelib/diff_drive.hpp>
#include <armadillo>

namespace nuslam
{
    /// \brief Range bearing measurements
    struct RangeBearing
    {
        /// \brief range distance
        double range = 0.0;

        /// \brief bearing angle
        double bearing = 0.0;
    };

    class SLAM
    {
        public:

        SLAM(int);

        EKF();

        arma::mat find_h(int);                 // for each obstacle

        arma::mat find_H(int);                 // for each obstacle

        arma::mat find_A(turtlelib::Twist2D, int rate);   // Eq 9 & 10, state transition

        find_currentState(turtlelib::Twist2D); // Eq 5 & 6

        init_landmarks((int n, arma::mat z));            //

        predict(turtlelib::Twist2D twist, int rate);     // Eq 27 (20 + 5 or 7)

        update(int n, arma::mat z);                      //

        
        private:

        int n;
        arma::mat I;
        arma::mat state;
        arma::mat estimate;
        arma::mat covariance;
        arma::mat process_noise;
        arma::mat kalman_gain;
        arma::mat A;
        arma::mat z;
    }
}

#endif