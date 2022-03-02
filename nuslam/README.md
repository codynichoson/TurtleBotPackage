# NUSLAM Package
## Creator
Cody Nichoson  
Winter 2022  
Northwestern University  
MS in Robotics

### Package Description
This package contains a C++ library and node that implement Feature-Based Extended Kalman Filter (EKF) SLAM on the simulated TurtleBot.

### Launchfiles
#### `slam.launch`
Launch this package using `roslaunch nuslam slam.launch` to launch the TurtleBot simulation with EKF SLAM and odometry estimates shown. Launch with arguments `cmd_src:=teleop` or `cmd_src:=circle` to drive the turtle using WAXD keys or `nusim` services, respectively. 

### Libraries
#### `nusim`
This library contains functions used for implementing EKF SLAM. The math and logic behind them is referenced from (https://nu-msr.github.io/navigation_site/lectures/slam.pdf).

### Nodes
#### `slam.cpp`
This node implements EKF SLAM using the `nuslam` library. 
