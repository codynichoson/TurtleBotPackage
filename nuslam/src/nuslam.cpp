#include "nuslam/nuslam.hpp"

namespace nuslam
{
    SLAM::SLAM(int n){
        int size = 3+2*n;
        I = arma::eye(size, size);
        state = arma::mat(size, 1, arma::fill::zeros);
        estimate = arma::mat(size, 1, arma::fill::zeros);
        covariance = arma::mat(size, size, arma::fill::zeros);
        process_noise = arma::mat(size, size, arma::fill::zeros);
        kalman_gain = arma::mat(size, 2, arma::fill::zeros);
    }

    arma::mat SLAM::find_h(int n){
        arma::mat h(2*n,1, arma::fill::zeros);
        for (int j = 1; j < n+1; j++){
            h(2*(j-1), 0) = std::sqrt(std::pow(state(2*j + 1, 0) - state(1,0), 2) + std::pow(state(2*j + 2, 0) - state(2,0), 2));
            h(2*j, 0) = std::atan2(state(2*j + 2,0) - state(2,0), state(2*j + 1,0) - state(1,0)) - state(0,0);
        }
        return h;
    }

    arma::mat SLAM::find_H(int n){
        arma::mat H(2*n, 3+2*n, arma::fill::zeros);

        for (int j = 1; j < n+1; j++){
            double deltax = state(2*j + 1) - state(1,0);
            double deltay = state(2*j + 2) - state(2,0);
            double d = std::pow(deltax, 2) + std::pow(deltay, 2);

            // top row
            H(2*(j-1),0) = 0;
            H(2*(j-1),1) = -deltax/std::sqrt(d);
            H(2*(j-1),2) = -deltay/std::sqrt(d);
            H(2*(j-1),2*j+1) = deltax/std::sqrt(d);
            H(2*(j-1),2*j+2) = deltay/std::sqrt(d);

            // bottom row
            H(2*j,0) = -1;
            H(2*j,1) = deltay/d;
            H(2*j,2) = -deltax/d;
            H(2*j,2*j+1) = -deltay/d;
            H(2*j,2*j+2) = deltax/d;

            
        }
        return H;
    }

    arma::mat SLAM::find_A(turtlelib::Twist2D twist, int rate){
        arma::mat A(3+2*n, 3+2*n, arma::fill::zeros), temp(3+2*n, 3+2*n, arma::fill::zeros);

        double deltax = twist.xdot/rate;
        double deltay = twist.ydot/rate;
        double deltat = twist.thetadot/rate;

        if (twist.thetadot == 0){
            temp(1,0) = -deltax*std::sin(state(0,0));
            temp(2,0) = deltax*std::cos(state(0,0));
        }
        else{
            temp(1,0) = -(deltax/deltat)*std::cos(state(0,0)) + (deltax/deltat)*std::cos(state(0,0) + deltat);
            temp(2,0) = -(deltax/deltat)*std::sin(state(0,0)) + (deltax/deltat)*std::sin(state(0,0) + deltat);
        }

        A = I + temp;
    }

    void SLAM::updateState(turtlelib::Twist2D twist, int rate){
        arma::mat transition(3+2*n,1,arma::fill::zeros);

        double deltax = twist.xdot/rate;
        double deltay = twist.ydot/rate;
        double deltat = twist.thetadot/rate;

        if (twist.thetadot == 0){
            transition(1,0) = deltax*std::cos(state(0,0));
            transition(2,0) = deltax*std::sin(state(0,0));
        }
        else{
            transition(0,0) = deltat;
            transition(1,0) = -(deltax/deltat)*std::sin(state(0,0)) + (deltax/deltat)*std::sin(state(0,0) + deltat);
            transition(2,0) = (deltax/deltat)*std::cos(state(0,0)) - (deltax/deltat)*std::cos(state(0,0) + deltat);
        }

        state = state + transition; // g(Xi_t, ut)  // Eq 5,7
    }

    void SLAM::init_landmarks(int n, arma::mat z){
        for (int j = 1; j < n+1; j++){
            state(2*j+1,0) = state(1,0) + z(2*(j-1), 0)*std::cos(z(2*j, 0) + state(0,0));
            state(2*j+2,0) = state(2,0) + z(2*(j-1), 0)*std::sin(z(2*j, 0) + state(0,0));
        }
    }

    void SLAM::predict(turtlelib::Twist2D twist, int rate){
        SLAM::updateState(twist, rate); // Eq 20

        SLAM::find_A(twist, rate);

        covariance = A*covariance*arma::trans(A) + process_noise;   // Eq 21
    }

    arma::mat SLAM::update(int n, arma::mat z){
        arma::mat h = SLAM::find_h(n);

        // theoretical measurement
        arma::mat z_hat = h;    // Eq 25

        arma::mat H = SLAM::find_H(n);

        // Kalman gain
        kalman_gain = covariance*arma::trans(H)*arma::inv((H*covariance*arma::trans(H)));  // Eq 26

        // posterior statue update
        state = state + kalman_gain*(z - z_hat);  // Eq 27

        // posterior covariance
        covariance = (I - kalman_gain*H)*covariance;    // Eq 28

        return state;
    }
}

