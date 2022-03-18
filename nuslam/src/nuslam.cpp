#include "nuslam/nuslam.hpp"
#include <ros/ros.h>

namespace nuslam
{
    // initalizer list for constructor
    SLAM::SLAM(int num_mark) : n(num_mark), 
                               size(3 + 2*n),
                               I(arma::eye(size, size)),
                               state(arma::mat(size, 1, arma::fill::zeros)),
                               estimate(arma::mat(size, 1, arma::fill::zeros)),
                               covariance(arma::mat(size, size, arma::fill::zeros)),
                               process_noise(arma::mat(size, size, arma::fill::zeros)),
                               kalman_gain(arma::mat(size, 2*n, arma::fill::zeros)),
                               R((arma::eye(2*n, 2*n))*0.001),
                               Q(arma::mat(size, size, arma::fill::zeros)),
                               known_landmarks(),
                               sus_landmarks()
                               {
                                    // arma::mat sub_covar(size-3, size-3, arma::fill::eye);
                                    // sub_covar = sub_covar*1000000;
                                    // covariance.submat(3, 3, size-2, size-2);
                                    covariance(3,3) = 1000000;
                                    covariance(4,4) = 1000000;
                                    covariance(5,5) = 1000000;
                                    covariance(6,6) = 1000000;
                                    covariance(7,7) = 1000000;
                                    covariance(8,8) = 1000000;

                                    Q(0,0) = 1;
                                    Q(1,1) = 1;
                                    Q(2,2) = 1;
                               }

    arma::mat SLAM::find_h(int n){
        arma::mat h(2*n,1, arma::fill::zeros);
        for (int j = 1; j < n+1; j++){
            h(2*(j-1), 0) = std::sqrt(std::pow(state(2*j + 1, 0) - state(1,0), 2) + std::pow(state(2*j + 2, 0) - state(2,0), 2));
            h((2*j)-1, 0) = std::atan2(state(2*j + 2, 0) - state(2, 0), state(2*j + 1, 0) - state(1, 0)) - state(0, 0);
        }
        return h;
    }

    arma::mat SLAM::find_H(int n, int real_n){
        arma::mat H(2*n, 3+2*n, arma::fill::zeros);

        // for (int j = 1; j < n+1; j++){
        for (int j = 1; j < real_n+1; j++){
            double deltax = state(2*j + 1, 0) - state(1, 0);
            double deltay = state(2*j + 2, 0) - state(2, 0);
            double d = std::pow(deltax, 2) + std::pow(deltay, 2);

            if (d != 0.0){
                // top row
                H(2*(j-1),0) = 0;
                H(2*(j-1),1) = -deltax/std::sqrt(d);
                H(2*(j-1),2) = -deltay/std::sqrt(d);
                H(2*(j-1),2*j+1) = deltax/std::sqrt(d);
                H(2*(j-1),2*j+2) = deltay/std::sqrt(d);

                // bottom row
                H((2*j)-1,0) = -1;
                H((2*j)-1,1) = deltay/d;
                H((2*j)-1,2) = -deltax/d;
                H((2*j)-1,2*j+1) = -deltay/d;
                H((2*j)-1,2*j+2) = deltax/d;
            }
        }
        return H;
    }

    arma::mat SLAM::find_A(turtlelib::Twist2D twist){
        arma::mat A(3+2*n, 3+2*n, arma::fill::zeros), temp(3+2*n, 3+2*n, arma::fill::zeros);

        double deltax = twist.xdot;
        double deltay = twist.ydot;
        double deltat = twist.thetadot;

        if (twist.thetadot == 0){
            temp(1,0) = -deltax*std::sin(state(0,0));
            temp(2,0) = deltax*std::cos(state(0,0));
        }
        else{
            temp(1,0) = -(deltax/deltat)*std::cos(state(0,0)) + (deltax/deltat)*std::cos(state(0,0) + deltat);
            temp(2,0) = -(deltax/deltat)*std::sin(state(0,0)) + (deltax/deltat)*std::sin(state(0,0) + deltat);
        }

        A = I + temp;

        return A;
    }

    void SLAM::updateState(turtlelib::Twist2D twist){
        arma::mat transition(3+2*n,1,arma::fill::zeros);

        double deltax = twist.xdot;
        double deltay = twist.ydot;
        double deltat = twist.thetadot;

        if (twist.thetadot == 0){
            transition(1,0) = deltax*std::cos(state(0,0));
            transition(2,0) = deltax*std::sin(state(0,0));
        }
        else{
            transition(0,0) = deltat;
            transition(1,0) = -(deltax/deltat)*std::sin(state(0,0)) + (deltax/deltat)*std::sin(state(0,0) + deltat);
            transition(2,0) = (deltax/deltat)*std::cos(state(0,0)) - (deltax/deltat)*std::cos(state(0,0) + deltat);
        }

        state(0,0) = state(0,0) + transition(0,0);
        state(1,0) = state(1,0) + transition(1,0);
        state(2,0) = state(2,0) + transition(2,0);
    }

    void SLAM::init_landmarks(int n, arma::mat z){
        for (int j = 1; j < n+1; j++){
            state((2*j)+1,0) = state(1,0) + z(2*(j-1), 0)*std::cos(z((2*j)-1, 0) + state(0,0));
            state((2*j)+2,0) = state(2,0) + z(2*(j-1), 0)*std::sin(z((2*j)-1, 0) + state(0,0));

            // initial known landmarks for data assocation
            turtlelib::Vector2D landmark;
            landmark.x = state((2*j)+1,0);
            landmark.y = state((2*j)+2,0);
            known_landmarks.push_back(landmark);
        }
    }

    void SLAM::predict(turtlelib::Twist2D twist){
        SLAM::updateState(twist);
        arma::mat A = SLAM::find_A(twist);
        covariance = A*covariance*arma::trans(A) + Q;
    }

    arma::mat SLAM::update(int n, int real_n, arma::mat z){
        arma::mat h = SLAM::find_h(n);

        // theoretical measurement
        arma::mat z_hat = h;
        arma::mat H = SLAM::find_H(n, real_n);
        arma::mat Ht = arma::trans(H);

        // Kalman gain
        kalman_gain = covariance*arma::trans(H)*arma::inv((H*covariance*arma::trans(H)) + R);  // Eq 26

        arma::mat deltaz = z - z_hat;
        for (int i = 1; i < deltaz.size(); i = i+2){
            deltaz(i,0) = turtlelib::normalizeAngle(deltaz(i,0));
        }
        
        // posterior statue update
        state = state + kalman_gain*deltaz;  // Eq 27

        // posterior covariance
        covariance = (I - kalman_gain*H)*covariance;    // Eq 28

        return state;
    }

    double SLAM::distance(turtlelib::Vector2D v1, turtlelib::Vector2D v2){
        double distance = (std::sqrt(std::pow(v2.x - v1.x, 2) + std::pow(v2.y - v1.y, 2)));
        return distance;
    }



    void SLAM::check_landmarks(std::vector<turtlelib::Vector2D> temp_landmarks){
        double threshold = 0.5;

        // compare currently known landmarks to new ones
        for (int i = 0; i < known_landmarks.size(); i++){
            for (int j = 0; j < temp_landmarks.size(); j++){

                // if distance between any new and known landmarks is low enough
                if (SLAM::distance(known_landmarks.at(i), temp_landmarks.at(j)) > threshold){

                    // add to list of known landmarks
                    known_landmarks.push_back(temp_landmarks.at(j));
                }
            }
        }
    }
}

