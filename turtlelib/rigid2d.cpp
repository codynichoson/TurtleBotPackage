#include <iostream>
#include <cmath>
#include "rigid2d.hpp"

constexpr double PI=3.14159265358979323846;

namespace turtlelib{
    Transform2D::Transform2D(){                               // identity transformation
        T[0][0] = 1; T[0][1] = 0; T[0][2] = 0;
        T[1][0] = 0; T[1][1] = 1; T[1][2] = 0;
        T[2][0] = 0; T[2][1] = 0; T[2][2] = 1;
    }

    Transform2D::Transform2D(Vector2D trans){                 // purely translation
        T[0][0] = 1; T[0][1] = 0; T[0][2] = trans.x;
        T[1][0] = 0; T[1][1] = 1; T[1][2] = trans.y;
        T[2][0] = 0; T[2][1] = 0; T[2][2] = 1;
    }

    Transform2D::Transform2D(double radians){                 // purely rotation
        T[0][0] = cos(radians); T[0][1] =-sin(radians); T[0][2] = 0;
        T[1][0] = sin(radians); T[1][1] = cos(radians); T[1][2] = 0;
        T[2][0] = 0;            T[2][1] = 0;            T[2][2] = 1;
    }

    Transform2D::Transform2D(Vector2D trans, double radians){ // translation and rotation
        T[0][0] = cos(radians); T[0][1] =-sin(radians); T[0][2] = trans.x;
        T[1][0] = sin(radians); T[1][1] = cos(radians); T[1][2] = trans.y;
        T[2][0] = 0;            T[2][1] = 0;            T[2][2] = 1;
    }

    Twist2D Transform2D::convertTwist(Twist2D twist){
        Twist2D new_twist;
        new_twist.thetadot = twist.thetadot + 0 + 0;
        new_twist.xdot = twist.xdot*
    }
}

// Main function
int main(void){
    using namespace std;
    double x, y;
    cout << "Enter degrees: ";
    cin >> x;
    y = deg2rad(x);
    cout << y;

    return(0);
}




