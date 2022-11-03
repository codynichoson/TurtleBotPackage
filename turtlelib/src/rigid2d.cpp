#include <iostream>
#include <cmath>
#include "turtlelib/rigid2d.hpp"

constexpr double PI=3.14159265358979323846;

namespace turtlelib{
    /// \brief Identity transformation
    Transform2D::Transform2D(){
        theta = 0;
        x = 0;
        y = 0;
    }

    /// \brief Purely translational transformation
    Transform2D::Transform2D(Vector2D trans){
        theta = 0;
        x = trans.x;
        y = trans.y;
    }

    /// \brief Purely rotational transformation
    Transform2D::Transform2D(double radians){
        theta = radians;
        x = 0;
        y = 0;
    }

    /// \brief Translational and rotational transformation
    Transform2D::Transform2D(Vector2D trans, double radians){
        theta = radians;
        x = trans.x;
        y = trans.y;
    }

    /// \brief Invert a transformation matrix
    /// \return Inverted transformation matrix
    Transform2D Transform2D::inv() const{
        Transform2D inverse;
        inverse.x = -x*std::cos(theta) - y*std::sin(theta);
        inverse.y = -y*std::cos(theta) + x*std::sin(theta);
        inverse.theta = -theta;

        if (almost_equal(inverse.x, 0.0)){
            inverse.x = 0.0;
        }
        if (almost_equal(inverse.y, 0.0)){
            inverse.y = 0.0;
        }
        if (almost_equal(inverse.theta, 0.0)){
            inverse.theta = 0.0;
        }

        return inverse;
    }

    /// \brief Transform a 2D coordinate using a specified transformation matrix
    /// \param tf - Transformation matrix
    /// \param vec - 2D coordinate
    /// \return Transformed coordinate
    Vector2D Transform2D::convertVec(Transform2D tf, Vector2D vec){
        Vector2D new_vec;
        new_vec.x = cos(tf.theta)*vec.x - sin(tf.theta)*vec.y + tf.x;
        new_vec.y = sin(tf.theta)*vec.x + cos(tf.theta)*vec.y + tf.y;
        return new_vec;
    }

    /// \brief Transform a twist using a specified transformation matrix
    /// \param tf - Transformation matrix
    /// \param twist - Accelerations applied to robot
    /// \return Transformed twist
    Twist2D Transform2D::convertTwist(Transform2D tf, Twist2D twist){
        Twist2D new_twist;
        new_twist.thetadot = twist.thetadot + 0 + 0;
        new_twist.xdot = tf.y*twist.thetadot + cos(tf.theta)*twist.xdot - sin(tf.theta)*twist.ydot;
        new_twist.ydot = -tf.x*twist.thetadot + sin(tf.theta)*twist.xdot + cos(tf.theta)*twist.ydot;
        return new_twist;
    }

    /// \brief Get the translational component of a transformation matrix
    /// \return 2D vector of x and y translational components
    Vector2D Transform2D::translation() const{
        Vector2D vec;
        vec.x = x;
        vec.y = y;
        return vec;
    }

    /// \brief Get the rotational component of a transformation matrix
    /// \return angle of rotational component
    double Transform2D::rotation() const{
        double angle = theta;
        return angle;
    }

    /// \brief Integrate a twist to get a transformation matrix
    /// \param twist - twist applied to robot
    /// \return Transformation matrix
    Transform2D integrate_twist(const Twist2D &twist){
        turtlelib::Transform2D Tbbp, Tsb, Tbs, Tssp, Tspbp;
        turtlelib::Vector2D Tbbp_vec;

        if (almost_equal(twist.thetadot, 0.0)){ // pure translation
            Tbbp_vec.x = twist.xdot;
            Tbbp_vec.y = twist.ydot;

            Tbbp = Transform2D(Tbbp_vec);

            return Tbbp;
        }
        else { // translation and rotation
            Vector2D Tsb_vec; 
            
            Tsb_vec.x = (twist.ydot/twist.thetadot); 
            Tsb_vec.y = (-twist.xdot/twist.thetadot);

            if ((almost_equal(Tsb_vec.x, 0.0))){
                Tsb_vec.x = 0.0;
            }
            if ((almost_equal(Tsb_vec.y, 0.0))){
                Tsb_vec.y = 0.0;
            }

            Tsb = Transform2D(Tsb_vec);
            double theta = twist.thetadot;
            Tssp = Transform2D(theta);
            Tbs = Tsb.inv();
            Tspbp = Tsb;

            Tbbp = Tbs*Tssp*Tspbp;

            return Tbbp;
        }
    }

    /// \brief Normalize a 2D vector
    /// \param vec - 2D vector
    /// \return The normalized 2D vector
    Vector2D normalize(Vector2D vec){
        Vector2D normalized;
        double magnitude = std::abs(std::sqrt(vec.x*vec.x + vec.y*vec.y));
        normalized.x = vec.x/magnitude;
        normalized.y = vec.y/magnitude;
        return normalized;
    }

    /// \brief Normalize an angle
    /// \param angle - An angle in radians
    /// \return The normalized angle in radians
    double normalizeAngle(double angle){
        double normalized = fmod(angle - PI, (2*PI));
        if (normalized > 0.0){
            normalized = normalized - (2*PI);
        }

        return normalized + PI;
    }

    /// \brief Operator overload for adding a value to a 2D vector
    Vector2D & Vector2D::operator+=(const Vector2D &rhs){
        x = x + rhs.x;
        y = y + rhs.y;
        return *this;
    }

    /// \brief Operator overload for subtracting a value from a 2D vector
    Vector2D & Vector2D::operator-=(const Vector2D &rhs){
        x = x - rhs.x;
        y = y - rhs.y;
        return *this;
    }

    /// \brief Operator overload for multiplying 2D vector by a value
    Vector2D & Vector2D::operator*=(const double &rhs){
        x = x*rhs;
        y = y*rhs;
        return *this;
    }

    /// \brief Operator overload for adding two 2D vectors
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs){
        lhs+=rhs;
        return lhs;
    }

    /// \brief Operator overload for subtracting two 2D vectors
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs){
        lhs-=rhs;
        return lhs;
    }

    /// \brief Operator overload for multiplying two 2D vectors
    Vector2D operator*(Vector2D lhs, const double & rhs){
        lhs*=rhs;
        return lhs;
    }

    /// \brief Calculate the dot product between two 2D vectors
    /// \param vec1 - A 2D vector
    /// \param vec2 - A 2D vector
    /// \return A double that is the dot product of the two vectors
    double dot(Vector2D vec1, Vector2D vec2){
        double dotprod = vec1.x*vec2.x + vec1.y*vec2.y;
        return dotprod;
    }

    /// \brief Calculate the magnitude of a 2D vector
    /// \param vec - A 2D vector
    /// \return The magnitude of the 2D vector
    double magnitude(Vector2D vec){
        double mag = std::sqrt(vec.x*vec.x + vec.y*vec.y);
        return mag;
    }

    /// \brief Calculate the angle between two 2D vectors
    /// \param vec1 - A 2D vector
    /// \param vec2 - A 2D vector
    /// \return The angle between the two vectors
    double angle(Vector2D vec1, Vector2D vec2){
        double theta = std::acos((turtlelib::dot(vec1, vec2))/(turtlelib::magnitude(vec1) * turtlelib::magnitude(vec2)));
        return theta;
    }

    /// \brief Normalize an angle
    /// \param rad - An angle in radians
    /// \return The angle normalized between -PI and PI
    double normalize_angle(double rad){
        while (rad > PI){
            rad = rad - 2.0*PI;
        }
        while (rad < -PI){
            rad = rad + 2.0*PI;
        }
        return rad;
    }

    /// \brief Operator overload for output of a 2D vector
    /// \return A 2D vector displayed in brackets
    std::ostream & operator << (std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]" << std::endl;
        return os;
    }

    /// \brief Operator overload for input of a 2D vector
    /// \return An inputted 2D vector
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

    /// \brief Operator overload for output of a 2D twist
    /// \return A 2D twist displayed in brackets
    std::ostream & operator << (std::ostream & os, const Twist2D & v){
        os << "[" << v.thetadot << " " << v.xdot << " " << v.ydot << "]" << std::endl;
        return os;
    }
    
    /// \brief Operator overload for input of a 2D twist
    /// \return An inputted 2D twist
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

    /// \brief Operator overload for input of a 2D transformation matrix
    /// \return An inputted 2D transformation matrix
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
    
    /// \brief Operator overload for output of a 2D transformation matrix
    /// \return A 2D transformation displayed via angle, x, and y values
    std::ostream & operator << (std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.theta) << " x: " << tf.x << " y: " << tf.y << std::endl;
        return os;
    }

    /// \brief Operator overload for multiplying transformation matrix (the object) by another
    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        x = std::cos(theta)*rhs.x - std::sin(theta)*rhs.y + x;
        y = std::sin(theta)*rhs.x + std::cos(theta)*rhs.y + y; 
        // theta = acos(cos(theta)*cos(rhs.theta) - sin(theta)*sin(rhs.theta));
        theta = theta + rhs.theta;

        return *this;
    }

    /// \brief Operator overload for multiplying two transformation matrices by one another
    /// \return The resulting 2D transformation matrix
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        lhs*=rhs;
        return lhs;
    }

    
}

