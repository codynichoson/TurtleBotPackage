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
Launch the SLAM visualization using `roslaunch nuslam slam.launch` to launch the TurtleBot simulation with EKF SLAM and odometry estimates shown. Launch with arguments `cmd_src:=teleop` or `cmd_src:=circle` to drive the turtle using WAXD keys or `nusim` services, respectively. 

#### `landmark_detect.launch`
Launch this SLAM with landmark detection using `roslaunch nuslam landmark_detect.launch` to launch a simulation similar to `slam.launch`, but with the ability to estimate landmark locations using laser scan data from the TurtleBot.

### Libraries
#### `nuslam`
This library contains functions used for implementing EKF SLAM. The math and logic behind them is referenced from (https://nu-msr.github.io/navigation_site/lectures/slam.pdf).

#### `circlelib`
This library contains a function used to find the location and size of an associated circle given a cluster of points (presumably an arc), as well as a function that classifies a list of clusters and returns only those clusters

### Nodes
#### `slam.cpp`
This node implements EKF SLAM using the `nuslam` library. 

#### `landmarks.cpp`
This node subscribes to laser scan data and uses it to estimate landmark locations.

### Media
##### EKF
<img src="https://user-images.githubusercontent.com/62906322/156726797-4f0eb09c-e53c-4b65-96d2-f23736f57f34.png" alt="drawing" width="600"/>

##### Landmark Detection
<img src="https://user-images.githubusercontent.com/62906322/159108910-3e3e918f-b864-4d57-8940-1e3da3c81813.jpg" alt="drawing" width="600"/>

##### Landmark Detection Real TurtleBot
https://youtu.be/11i9O1-8XTM





