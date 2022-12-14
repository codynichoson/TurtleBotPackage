/// \file
/// \brief Circle Library

#include "nuslam/circlelib.hpp"
#include <turtlelib/diff_drive.hpp>
#include <ros/ros.h>

namespace nuslam
{
    /// \brief Detects circle within a cluster
    /// \param cluster - 2D vector of cluster points
    /// \return Detected circle
    Circle CircleFit::detect_circle(std::vector<turtlelib::Vector2D> cluster)
    {
        int n = cluster.size();
        float xsum = 0.0;
        float ysum = 0.0;

        for (int i = 0; i < n; i++)
        {
            xsum += cluster.at(i).x;
            ysum += cluster.at(i).y;
        }

        // find the mean of the x and y coordinates
        float xmean = xsum/n;
        float ymean = ysum/n;

        // shift the coordinates so that the centroid is at the origin
        for (int i = 0; i < n; i++)
        {
            cluster.at(i).x = cluster.at(i).x - xmean;
            cluster.at(i).y = cluster.at(i).y - ymean;
        }

        // compute z_i
        std::vector<double> z_i;
        z_i.resize(n);
        float zsum = 0.0;

        for (int i = 0; i < n; i++)
        {
            z_i.at(i) = std::pow(cluster.at(i).x, 2) + std::pow(cluster.at(i).y, 2);
            zsum += z_i.at(i);
        }

        // compute the mean of z
        float zmean = zsum/n;

        // form the data matrix from the n data points
        arma::mat Z(n, 4, arma::fill::zeros);
        for (int i = 0; i < n; i++)
        {
            Z(i, 0) = z_i.at(i);
            Z(i, 1) = cluster.at(i).x;
            Z(i, 2) = cluster.at(i).y;
            Z(i, 3) = 1;
        }

        // form the moment matrix
        arma::mat M(n, 4, arma::fill::zeros);
        M = (trans(Z)*Z)/n;

        // form the constraint matrix for the "Hyperaccurate algabraic fit"
        arma::mat H(4, 4, arma::fill::zeros);

        H(0,0) = 8*zmean;
        H(0,3) = 2;
        H(1,1) = 1;
        H(2,2) = 1;
        H(3,0) = 2;

        // compute H inverse
        arma::mat H_inv = inv(H);

        // compute Singular Value Decomposition of Z
        arma::mat U;
        arma::vec s;
        arma::mat V;
        svd(U,s,V,Z);

        // turn s into diag matrix with column values
        arma::mat S = diagmat(s);

        // if smallest singular value s_4 is lass than 10^-12, let A be the 4th column of V matrix
        arma::vec A;
        if (s(3) < std::pow(10, -12))
        {
            A = V.col(3);
        }
        else
        {
            arma::mat Y = V*S*V.t();

            arma::mat Q = Y*H_inv*Y;

            arma::vec eigval;
            arma::mat eigvec;

            eig_sym(eigval, eigvec, Q);

            int Astar_index;

            int eigval_size = eigval.size(); 
            for (int i = 0; i < eigval_size; i++)
            {
                if (eigval(i) > 0)
                {
                    Astar_index = i;
                    break;
                }
            }

            arma::vec Astar;
            
            Astar = eigvec.col(Astar_index);

            solve(A,Y,Astar);
        }
        
        double a = -A(1)/(2*A(0));
        double b = -A(2)/(2*A(0));
        double R = std::sqrt((std::pow(A(1),2) + std::pow(A(2),2) - 4*A(0)*A(3))/(4*std::pow(A(0), 2)));

        Circle circle_return;

        circle_return.radius = R;
        circle_return.x = a + xmean;
        circle_return.y = b + ymean;

        return circle_return;
    }

    /// \brief Removes any non-circular clusters
    /// \param cluster_list - 2D vector of different clusters
    /// \return 2D vector of only circular clusters
    std::vector<std::vector<turtlelib::Vector2D>> CircleFit::circles_only(std::vector<std::vector<turtlelib::Vector2D>> cluster_list)
    {
        std::vector<std::vector<turtlelib::Vector2D>> new_cluster_list;

        int cluster_list_size = cluster_list.size();
        for (int i = 0; i < cluster_list_size; i++)
        {
            std::vector<turtlelib::Vector2D> cluster = cluster_list.at(i);
            std::vector<double> angle_list; 
            double angle;

            turtlelib::Vector2D P1 = {.x = cluster.at(0).x, .y = cluster.at(0).y};
            turtlelib::Vector2D P2 = {.x = cluster.back().x, .y = cluster.back().y};

            double sum = 0.0;

            int cluster_size = cluster.size();
            for (int j = 1; j < cluster_size - 1; j++)
            {
                turtlelib::Vector2D P1_P{.x = P1.x - cluster.at(j).x, .y = P1.y - cluster.at(j).y}; // vector between P1 and P
                turtlelib::Vector2D P2_P{.x = P2.x - cluster.at(j).x, .y = P2.y - cluster.at(j).y}; // vector between P2 and P

                angle = turtlelib::angle(P1_P, P2_P);
                angle_list.push_back(angle);

                sum += angle;
            }

            double mean = sum / angle_list.size();

            double val_sum = 0.0;

            int angle_list_size = angle_list.size();
            for (int k = 0; k < angle_list_size; k++)
            {
                val_sum += pow((angle_list.at(k) - mean), 2);
            }

            double stdev = sqrt(val_sum / (angle_list.size() - 1.0));

            if (turtlelib::rad2deg(mean) > 100.0 && turtlelib::rad2deg(mean) < 150.0 && stdev < 0.15)    // if cluster meets circle characteristics
            {
                new_cluster_list.push_back(cluster_list.at(i));   // keep it
            }
        }

        return new_cluster_list;
    }
}