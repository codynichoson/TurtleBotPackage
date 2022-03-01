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
        private:

        int n {};
        int size {};
        arma::mat I {};
        arma::mat state {};
        arma::mat estimate {};
        arma::mat covariance {};
        arma::mat process_noise {};
        arma::mat kalman_gain {};
        arma::mat R {};

        public:

        SLAM(int num_mark);

        // SLAM(int);

        arma::mat find_h(int);                 // for each obstacle

        arma::mat find_H(int);                 // for each obstacle

        arma::mat find_A(turtlelib::Twist2D, float);   // Eq 9 & 10, state transition

        void updateState(turtlelib::Twist2D, float); // Eq 5 & 6

        void init_landmarks(int, arma::mat);            //

        void predict(turtlelib::Twist2D, float);     // Eq 27 (20 + 5 or 7)

        arma::mat update(int, arma::mat);                     
    };
}

#endif