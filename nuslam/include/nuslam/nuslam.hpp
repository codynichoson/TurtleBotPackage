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
        arma::mat Q {};
        std::vector<turtlelib::Vector2D> known_landmarks {};
        std::vector<turtlelib::Vector2D> sus_landmarks {};

        public:

        /// \brief class constructor to initialize private parameters
        /// \param num_mark - the number of visible landmarks
        /// \return none
        SLAM(int num_mark);

        /// \brief calculate range and bearing to landmarks
        /// \param n - number of landmarks
        /// \return h - a matrix of all range and bearing values to landmarks
        arma::mat find_h(int);

        /// \brief derivative of h with respect to the state
        /// \param n - number of landmarks
        /// \return H - derivate of of h matrix
        arma::mat find_H(int, int);

        /// \brief calculate transition
        /// \param twist - twist of robot
        /// \return A - transition
        arma::mat find_A(turtlelib::Twist2D);

        /// \brief update the state of the robot and landmarks
        /// \param twist - twist of robot
        /// \return none
        void updateState(turtlelib::Twist2D);

        /// \brief initialize landmark locations
        /// \param n - number of landmarks
        /// \param z - range and bearing to visible landmarks
        /// \return a 
        void init_landmarks(int, arma::mat);

        /// \brief update state estimate and propogate uncertainty
        /// \param twist - twist of robot
        /// \return none
        void predict(turtlelib::Twist2D);

        /// \brief compute Kalman gain and posterior state update
        /// \param n - number of landmarks
        /// \param z - range and bearing to visible landmarks
        /// \return robot and landmark state
        arma::mat update(int, int, arma::mat);

        double distance(turtlelib::Vector2D v1, turtlelib::Vector2D v2);

        std::vector<turtlelib::Vector2D> get_known_landmarks();

        bool check_landmarks(std::vector<turtlelib::Vector2D> temp_landmarks);

        
    };
}

#endif