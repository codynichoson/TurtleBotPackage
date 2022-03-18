#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuslam/circlelib.hpp"
#include <catch_ros/catch.hpp>
#include <sstream>

constexpr double PI=3.14159265358979323846;
constexpr double eps=0.0001;

TEST_CASE("circle_detect_1"){ // Cody, Nichoson
    nuslam::CircleFit CircleBoy;
    std::vector<turtlelib::Vector2D> cluster;
    turtlelib::Vector2D a, b, c, d, e, f;
    a.x = 1.0; a.y = 7.0;
    b.x = 2.0; b.y = 6.0;
    c.x = 5.0; c.y = 8.0;
    d.x = 7.0; d.y = 7.0;
    e.x = 9.0; e.y = 5.0;
    f.x = 3.0; f.y = 7.0;

    cluster = {a, b, c, d, e, f};

    nuslam::Circle circle = CircleBoy.detect_circle(cluster);

    CHECK(circle.x == Approx(4.615482).margin(eps));
    CHECK(circle.y == Approx(2.807354).margin(eps));
    CHECK(circle.radius == Approx(4.8275).margin(eps));
}

TEST_CASE("circle_detect_2"){ // Cody, Nichoson
    nuslam::CircleFit CircleBoy;
    std::vector<turtlelib::Vector2D> cluster;
    turtlelib::Vector2D a, b, c, d;
    a.x = -1.0; a.y = 0.0;
    b.x = -0.3; b.y = -0.06;
    c.x = 0.3;  c.y = 0.1;
    d.x = 1.0;  d.y = 0.0;

    cluster = {a, b, c, d};

    nuslam::Circle circle = CircleBoy.detect_circle(cluster);

    CHECK(circle.x == Approx(0.4908357).margin(eps));
    CHECK(circle.y == Approx(-22.15212).margin(eps));
    CHECK(circle.radius == Approx(22.17979).margin(eps));
}