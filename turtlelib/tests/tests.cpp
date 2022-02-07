#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch_ros/catch.hpp>
#include <sstream>

constexpr double PI=3.14159265358979323846;
constexpr double eps=0.0000000001;

TEST_CASE("normalize"){ // Cody, Nichoson
    turtlelib::Vector2D v_in, v_out;
    v_in.x = 1; v_in.y = 1;
    v_out = normalize(v_in);
    CHECK(v_out.x == Approx(0.7071067812));
    CHECK(v_out.y == Approx(0.7071067812));
}

TEST_CASE("inverse"){ // Cody, Nichoson
    turtlelib::Transform2D tf1, tf1inv;
    tf1inv = tf1.inv();
    double angle1 = tf1inv.rotation();
    turtlelib::Vector2D vec1 = tf1inv.translation();
    CHECK(angle1 == Approx(0));
    CHECK(vec1.x == Approx(0));
    CHECK(vec1.y == Approx(0));

    turtlelib::Vector2D vec; vec.x = 2; vec.y = 4;
    double angle = PI/2;
    turtlelib::Transform2D tf2(vec, angle), tf2inv;
    tf2inv = tf2.inv();
    double angle2 = tf2inv.rotation();
    turtlelib::Vector2D vec2 = tf2inv.translation();
    CHECK(angle2 == Approx(-PI/2));
    CHECK(vec2.x == Approx(-4));
    CHECK(vec2.y == Approx(2));
}

TEST_CASE("translation"){ // Cody, Nichoson
    turtlelib::Transform2D tf;
    turtlelib::Vector2D vec = tf.translation();
    CHECK(vec.x == Approx(0));
    CHECK(vec.y == Approx(0));
}

TEST_CASE("rotation"){ // Cody, Nichoson
    turtlelib::Transform2D tf;
    double angle = tf.rotation();
    CHECK(angle == Approx(0));
}

TEST_CASE("normalizeAngle"){ // Cody, Nichoson
    double theta1 = PI;
    double theta2 = -PI;
    double theta3 = 0;
    double theta4 = -PI/4.0;
    double theta5 = (3.0*PI)/2.0;
    double theta6 = (-5.0*PI)/2.0;

    double res1 = turtlelib::normalizeAngle(theta1);
    double res2 = turtlelib::normalizeAngle(theta2);
    double res3 = turtlelib::normalizeAngle(theta3);
    double res4 = turtlelib::normalizeAngle(theta4);
    double res5 = turtlelib::normalizeAngle(theta5);
    double res6 = turtlelib::normalizeAngle(theta6);

    CHECK(res1 == Approx(PI).margin(0.0001));
    CHECK(res2 == Approx(PI).margin(0.0001));
    CHECK(res3 == Approx(0).margin(0.0001));
    CHECK(res4 == Approx(-PI/4.0).margin(0.0001));
    CHECK(res5 == Approx(-PI/2.0).margin(0.0001));
    CHECK(res6 == Approx(-PI/2.0).margin(0.0001));
}

TEST_CASE("adding vectors"){ // Cody, Nichoson
    turtlelib::Vector2D vec1, vec2;

    vec1.x = 2.0; vec1.y = 4.0;
    vec2.x = 3.0; vec2.y = -2.0;

    turtlelib::Vector2D res = vec1 + vec2;

    CHECK(res.x == Approx(5.0).margin(eps));
    CHECK(res.y == Approx(2.0).margin(eps));
}

TEST_CASE("angle"){ // Cody, Nichoson
    turtlelib::Vector2D vec1, vec2;

    vec1.x = 4.0; vec1.y = 2.0;
    vec2.x = -3.0; vec2.y = 3.0;

    double theta = angle(vec1, vec2);

    CHECK(theta == Approx(1.8925).margin(0.0001));
}

TEST_CASE("inverse_kinematics"){ // Cody, Nichoson
    turtlelib::DiffDrive ddrive;
    turtlelib::Twist2D twist1, twist2, twist3;
    twist1.xdot = 0.0; twist1.thetadot = 0.2; // pure rotation
    twist2.xdot = 0.5; twist2.thetadot = 0.0;  // pure translation
    twist3.xdot = 0.5; twist3.thetadot = 0.2; // rotation and translation

    turtlelib::WheelVel vel1 = ddrive.invKin(twist1);
    turtlelib::WheelVel vel2 = ddrive.invKin(twist2);
    turtlelib::WheelVel vel3 = ddrive.invKin(twist3);
    
    CHECK(vel1.left == Approx(-0.484848).margin(0.000001));
    CHECK(vel1.right == Approx(0.484848).margin(0.000001));
    CHECK(vel2.left == Approx(15.151515).margin(0.000001));
    CHECK(vel2.right == Approx(15.151515).margin(0.000001));
    CHECK(vel3.left == Approx(14.666666).margin(0.000001));
    CHECK(vel3.right == Approx(15.636363).margin(0.000001));
}

TEST_CASE("forward_kinematics"){ // Cody, Nichoson
    turtlelib::DiffDrive ddrive;
    turtlelib::WheelAngles ang1, ang2, ang3;
    // ang1.left = 2*PI; ang1.right = 2*PI; // pure translation

    ang2.left = -PI/4; ang2.right = PI/4; // pure rotation
    ang3.left = 19.992; ang3.right = 27.6079; // from Marco

    // turtlelib::Config q1 = ddrive.fKin(ang1);
    turtlelib::Config q2 = ddrive.fKin(ang2);
    // turtlelib::Config q3 = ddrive.fKin(ang3);

    // CHECK(q1.x == Approx(2*PI*0.033).margin(0.001));
    // CHECK(q1.y == Approx(0.0).margin(0.000001));
    // CHECK(q1.theta == Approx(0.0).margin(0.000001));

    CHECK(q2.x == Approx(0.0));
    CHECK(q2.y == Approx(0));
    CHECK(q2.theta == Approx(0.32397).margin(0.00001));

    // CHECK(q3.x == Approx(0.5));
    // CHECK(q3.y == Approx(0.5));
    // CHECK(q3.theta == Approx(PI/2));
}

TEST_CASE("integrate_twist"){ // Cody, Nichoson
    turtlelib::Twist2D twist1, twist2, twist3;
    twist1.xdot = 0.0; twist1.ydot = 0.0; twist1.thetadot = PI/2; // pure rotation
    twist2.xdot = 3.0; twist2.ydot = 5.0; twist2.thetadot = 0.0;  // pure translation
    twist3.xdot = 1.0; twist3.ydot = 1.0; twist3.thetadot = PI/2; // rotation and translation

    turtlelib::Transform2D T1 = integrate_twist(twist1);
    turtlelib::Transform2D T2 = integrate_twist(twist2);
    turtlelib::Transform2D T3 = integrate_twist(twist3);

    turtlelib::Vector2D T1vec = T1.translation();
    double T1rot = T1.rotation();
    turtlelib::Vector2D T2vec = T2.translation();
    double T2rot = T2.rotation();
    turtlelib::Vector2D T3vec = T3.translation();
    double T3rot = T3.rotation();

    CHECK(T1vec.x == Approx(0.0).margin(0.0001));
    CHECK(T1vec.y == Approx(0.0).margin(0.0001));
    CHECK(T1rot == Approx(PI/2).margin(0.01));

    CHECK(T2vec.x == Approx(3.0).margin(0.0001));
    CHECK(T2vec.y == Approx(5.0).margin(0.0001));
    CHECK(T2rot == Approx(0.0).margin(0.01));

    CHECK(T3vec.x == Approx(0.0).margin(0.01));
    CHECK(T3vec.y == Approx(4/PI).margin(0.01));
    CHECK(T3rot == Approx(PI/2).margin(0.01));
}

TEST_CASE("istream Vector input","[Vector2D]"){ // James Avtges
    std::stringstream bracket;
    turtlelib::Vector2D bracketV;
    bracket.str("[1 1]");
    std::stringstream number;
    turtlelib::Vector2D numberV;
    number.str("1 1");
    bracket >> bracketV;
    number >> numberV;
    CHECK(bracketV.x == 1);
    CHECK(numberV.x == 1);
}

TEST_CASE("ostream Vector output","[Vector2D]"){ // James Avtges
    std::stringstream vectorOut;
    turtlelib::Vector2D vector;
    vector.x = 9;
    vector.y = 1;

    vectorOut << vector;

    CHECK(vectorOut.str() == "[9 1]\n");
}

TEST_CASE("istream Twist input","[Twist2D]"){ // James Avtges
    std::stringstream bracket;
    turtlelib::Twist2D bracketT;
    bracket.str("[1 2 3]");
    std::stringstream number;
    turtlelib::Twist2D numberT;
    number.str("1 2 3");
    bracket >> bracketT;
    number >> numberT;
    CHECK(bracketT.xdot == 2);
    CHECK(numberT.xdot == 2);
}

TEST_CASE("ostream Twist output","[Twist2D]"){ // James Avtges
    std::stringstream twistOut;
    turtlelib::Twist2D twist;
    twist.thetadot = 10;
    twist.xdot = 5;
    twist.ydot = 4;

    twistOut << twist;

    CHECK(twistOut.str() == "[10 5 4]\n");
}

TEST_CASE("istream Transform input","[Transform2D]"){ // James Avtges
    std::stringstream transform;
    turtlelib::Transform2D T(0);
    transform.str("deg: 80 x: 2 y: 4\n");
    transform >> T;

    turtlelib::Vector2D translation = T.translation();
    double rotation = turtlelib::rad2deg(T.rotation());
    CHECK(translation.x == 2);
    CHECK(translation.y == 4);
    CHECK(rotation == 80);
}

TEST_CASE("ostream Transform output","[Transform2D]"){ // James Avtges
    std::stringstream transformOut;
    turtlelib::Vector2D trans;
    trans.x = 3.2;
    trans.y = 4;
    double rot = turtlelib::deg2rad(6.1);
    turtlelib::Transform2D T(trans, rot);

    transformOut << T;

    CHECK(transformOut.str() == "deg: 6.1 x: 3.2 y: 4\n");
}

TEST_CASE("constructor_all", "[transform]") { // Anna Garverick
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;

    double r = turtlelib::deg2rad(90);

    turtlelib::Transform2D T(v, r);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    CHECK(t_out.x == 1);
    CHECK(t_out.y == 2);
    CHECK(d == 90);
} 

TEST_CASE("constructor_trans", "[transform]") { // Anna Garverick
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;

    turtlelib::Transform2D T(v);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    CHECK(t_out.x == 1);
    CHECK(t_out.y == 2);
    CHECK(d == 0);
}

TEST_CASE("constructor_rot", "[transform]") { // Anna Garverick
    double r = turtlelib::deg2rad(90);

    turtlelib::Transform2D T(r);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    CHECK(t_out.x == 0);
    CHECK(t_out.y == 0);
    CHECK(d == 90);
}

TEST_CASE("Multiplying transforms", "[transform]") { // Cody Nichoson
    turtlelib::Vector2D vec1, vec2;
    vec1.x = 2; vec1.y = 6;
    vec2.x = -5; vec2.y = -2;
    double theta1 = PI/8;
    double theta2 = -PI;

    turtlelib::Transform2D tfab(vec1, theta1);
    turtlelib::Transform2D tfbc(vec2, theta2);

    turtlelib::Transform2D tfac = tfab*tfbc;
    turtlelib::Vector2D vec_ac = tfac.translation();
    double theta_ac = tfac.rotation();

    CHECK(vec_ac.x == Approx(-1.85).margin(0.01));
    CHECK(vec_ac.y == Approx(2.24).margin(0.01));
    CHECK(theta_ac == Approx(2.75).margin(0.01));

    turtlelib::Vector2D vec3, vec4;
    vec3.x = -4; vec3.y = 2;
    vec4.x = 3; vec4.y = 1;
    double theta3 = -PI;
    double theta4 = -PI/2;

    turtlelib::Transform2D tfcd(vec3, theta3);
    turtlelib::Transform2D tfde(vec4, theta4);

    turtlelib::Transform2D tfce = tfcd*tfde;
    turtlelib::Vector2D vec_ce = tfce.translation();
    double theta_ce = tfce.rotation();

    CHECK(vec_ce.x == Approx(-7.0));
    CHECK(vec_ce.y == Approx(1.0));
    CHECK(theta_ce == Approx(PI/2).margin(0.01));
}