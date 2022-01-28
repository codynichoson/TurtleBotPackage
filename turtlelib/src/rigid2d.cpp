#include <iostream>
#include <cmath>
#include "turtlelib/rigid2d.hpp"

constexpr double PI=3.14159265358979323846;

namespace turtlelib{
    Transform2D::Transform2D(){                               // identity transformation
        theta = 0;
        x = 0;
        y = 0;
    }

    Transform2D::Transform2D(Vector2D trans){                 // purely translation
        theta = 0;
        x = trans.x;
        y = trans.y;
    }

    Transform2D::Transform2D(double radians){                 // purely rotation
        theta = radians;
        x = 0;
        y = 0;
    }

    Transform2D::Transform2D(Vector2D trans, double radians){ // translation and rotation
        theta = radians;
        x = trans.x;
        y = trans.y;
    }

    Transform2D Transform2D::inv() const{
        Transform2D inverse;
        inverse.x = -x*cos(theta) - y*sin(theta);
        inverse.y = -y*cos(theta) + x*sin(theta);
        inverse.theta = -theta;

        return inverse;
    }

    Vector2D Transform2D::convertVec(Transform2D tf, Vector2D vec){
        Vector2D new_vec;
        new_vec.x = cos(tf.theta)*vec.x - sin(tf.theta)*vec.y + tf.x;
        new_vec.y = sin(tf.theta)*vec.x + cos(tf.theta)*vec.y + tf.y;

        return new_vec;
    }

    Twist2D Transform2D::convertTwist(Transform2D tf, Twist2D twist){
        Twist2D new_twist;
        new_twist.thetadot = twist.thetadot + 0 + 0;
        new_twist.xdot = tf.y*twist.thetadot + cos(tf.theta)*twist.xdot - sin(tf.theta)*twist.ydot;
        new_twist.ydot = -tf.x*twist.thetadot + sin(tf.theta)*twist.xdot + cos(tf.theta)*twist.ydot;

        return new_twist;
    }

    Vector2D Transform2D::translation() const{
        Vector2D vec;
        vec.x = x;
        vec.y = y;

        return vec;
    }

    double Transform2D::rotation() const{
        double angle = theta;

        return angle;
    }

    Vector2D normalize(Vector2D vec){
        Vector2D normalized;
        double magnitude = std::abs(std::sqrt(vec.x*vec.x + vec.y*vec.y));

        normalized.x = vec.x/magnitude;
        normalized.y = vec.y/magnitude;

        return normalized;
    }

    double normalize_angle(double rad){
        while (rad > PI){
            rad = rad - 2.0*PI;
        }
        while (rad < -PI){
            rad = rad + 2.0*PI;
        }
        return rad;
    }

    std::ostream & operator << (std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]" << std::endl;
        return os;
    }

    std::istream & operator >> (std::istream & is, Vector2D & v){
        char c1;
        c1 = is.peek();
        if (c1 == '['){
            is.get();
            is >> v.x;
            is.get();
            is >> v.y;
        }
        else {
            is >> v.x;
            is.get();
            is >> v.y;
        }
        return is;
    }

    std::ostream & operator << (std::ostream & os, const Twist2D & v){
        os << "[" << v.thetadot << " " << v.xdot << " " << v.ydot << "]" << std::endl;
        return os;
    }
    
    std::istream & operator >> (std::istream & is, Twist2D & v){
        char c1;
        char trash[5];
        c1 = is.peek();
        if (c1 == '['){
            is.get();
            is >> v.thetadot;
            is.get();
            is >> v.xdot;
            is.get();
            is >> v.ydot;
        }
        else if (c1 == 'd'){
            is.get(trash,5);
            is >> v.thetadot;
            is.get(trash,4);
            is >> v.xdot;
            is.get(trash,4);
            is >> v.ydot;
        }
        else {
            is >> v.thetadot;
            is.get();
            is >> v.xdot;
            is.get();
            is >> v.ydot;
        }
        return is;
    }

    std::istream & operator >> (std::istream & is, Transform2D & tf){
        Vector2D vector;
        double theta;
        char c1;
        char trash[5];
        
        c1 = is.peek();
        if (c1 == '['){
            is.get();
            is >> theta;
            is.get();
            is >> vector.x;
            is.get();
            is >> vector.y;
            is.get();
        }
        else if (c1 == 'd'){
            is.get(trash,5);
            is >> theta;
            is.get(trash,4);
            is >> vector.x;
            is.get(trash,4);
            is >> vector.y;
            is.get();
        }
        else {
            is >> theta;
            is.get();
            is >> vector.x;
            is.get();
            is >> vector.y;
            is.get();
        }

        Transform2D transform(vector, deg2rad(theta));
        tf = transform;
        return is;
    }

    std::ostream & operator << (std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.theta) << " x: " << tf.x << " y: " << tf.y << std::endl;
        return os;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        x = cos(theta)*rhs.x - sin(theta)*rhs.y + x*1;
        y = sin(theta)*rhs.x + cos(theta)*rhs.y +y*1; 
        theta = acos(cos(theta)*cos(rhs.theta) - sin(theta)*sin(rhs.theta) + x*0);

        return *this;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        lhs*=rhs;
        return lhs;
    }
}

