# ME495 Sensing, Navigation and Machine Learning For Robotics
## Creator
Cody Nichoson  
Winter 2022  
Northwestern University  
MS in Robotics

## Demonstration Videos
### Straight Line Test
#### Real
https://youtu.be/wf5oIyrZR1A
#### Odometry
https://youtu.be/ooLIbjrE8DY

### Spin Test
#### Real
https://youtu.be/wwuZlec7XPU
#### Odometry
https://youtu.be/oY9r61NI1cs

### Circle Test
#### Real
https://youtu.be/1D3B_T9j9QQ
#### Odometry
https://youtu.be/DMlfGwZ1oho

## Packages
This repository consists of several ROS packages:

`nuturtle_description`  
An adaptation of the TurtleBot model suitable to this project's needs.  

`turtlelib`  
A C++ library used for performing 2D rigid body transformations and other functionalities.

`nusim`  
A simluator and visualizer that provides a simulated robot environment for our TurtleBot.  

`nuturtle_control`  
A package containing several nodes used for controlling the robot in both simulation and in the real world environment.

## Launchfiles
### nusim
`nusim.launch`  
Launches a single TurtleBot into a simulated environment.

### nuturtle_control
`start_robot.launch`  
Launches TurtleBot into simulated environment and allows user to call services to publish `cmd_vel` values to the TurtleBot (real or simulated, remote or local), receive odometry, and visualize everything in Rviz.

## Nodes

### nusim
`nusim`  
Provides a simulated robot environment and configuration updates.

### nuturtle_control
`circle`  
Publishes `cmd_vel` commands that cause robot to drive in a circle.  
`turtle_interface`  
Enables control of the TurtleBot via `cmd_vel` commands.  
`odometry`  
Publishes odometry messages and the odometry transform of the robot.  



