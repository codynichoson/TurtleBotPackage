#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch_ros/catch.hpp>
#include <sstream>

constexpr double PI=3.14159265358979323846;
constexpr double eps=0.0000000001;

// TEST_CASE("normalize"){ // Cody, Nichoson
//     turtlelib::Vector2D v_in, v_out;
//     v_in.x = 1; v_in.y = 1;
//     v_out = normalize(v_in);
//     CHECK(v_out.x == Approx(0.7071067812));
//     CHECK(v_out.y == Approx(0.7071067812));
// }