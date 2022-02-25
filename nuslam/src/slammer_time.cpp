// #include "include/nuslam.hpp"
#include <armadillo>

int main(){
    // initalize matrices
    // arma::mat matrix = arma::mat(4,4,arma::fill::zeros);
    // arma::mat matrix(4,4,arma::fill::zeros);
    // arma::mat matrix = { {},{},{},{} }

    // state of robot at time t
    arma::mat state_robot(3,1,arma::fill::zeros);

    // state of map (length = num_landmarks*2)
    arma::mat state_map(3*2,1,arma::fill::zeros);

    // combining robot and map states
    arma::mat state_combined = std::move(arma::join_cols(state_robot, state_map));


    
}