# NUTURTLE_CONTROL Package
## Creator
Cody Nichoson  
Winter 2022  
Northwestern University  
MS in Robotics

## Package Description
A package containing several nodes used for controlling the robot in both simulation and in the real world environment.

## How to Launch
### Run the following commands depending on desired launch state:
#### In Simulation (with services available)
`roslaunch nuturtle_control start_robot.launch robot:=nusim cmd_src:=circle`  

#### In Simulation (with teleop control)
`roslaunch nuturtle_control start_robot.launch robot:=nusim cmd_src:=teleop`  

#### In the Real World (with services available)
`roslaunch nuturtle_control start_robot.launch robot:=<robot_name> cmd_src:=teleop`  

#### In the Real World (with teleop control)
`roslaunch nuturtle_control start_robot.launch robot:=<robot_name> cmd_src:=circle` 

## Launchfiles
#### `start_robot.launch`  
Launches TurtleBot into simulated environment and allows user to call services to publish `cmd_vel` values to the TurtleBot (real or simulated, remote or local), receive odometry, and visualize everything in Rviz.

## Nodes
#### `circle`  
Publishes `cmd_vel` commands that cause robot to drive in a circle.  
#### `turtle_interface`  
Enables control of the TurtleBot via `cmd_vel` commands.   
#### `odometry`  
Publishes odometry messages and the odometry transform of the robot. 

## Services
#### `/control`  
Takes in an angular velocity and radius and moves TurtleBot in a corresponding circular motion.
#### `/reverse`  
Takes in no parameters; reverses the TurtleBot based on the initial /control service called.
#### `/set_pose`  
Takes in a robot configuration and moves the blue robot (representing the robot's configuration based on odometry) to the inputted configuration.
#### `/stop`  
Takes in no parameters; publishes `cmd)vel` values of zero and stops robot.

## Demonstration Videos
### Straight Line Test
Starting Coordinates: (0,0,0)  
Final Coordinates:    (-0.00387, 0.00102, -0.00016)
#### Real
https://youtu.be/TiDCJBVdU1g
#### Odometry
https://youtu.be/ooLIbjrE8DY

### Spin Test
Starting Coordinates: (0,0,0)  
Final Coordinates:    (-0.00040, 0.000005, -0.01946)
#### Real
https://youtu.be/n_YxbGPr-Z4
#### Odometry
https://youtu.be/oY9r61NI1cs

### Circle Test
Starting Coordinates: (0,0,0)  
Final Coordinates:    (-0.00122, -0.00083, -0.00316)
#### Real
https://youtu.be/NHRz6_yNdhE
#### Odometry
https://youtu.be/DMlfGwZ1oho

### Slip Test
Starting Coordinates: (0,0,0)  
Final Coordinates:    (0.09458, -0.00329, 0.02499)
#### Real
https://youtu.be/hayYHKsN6F8
#### Odometry
https://youtu.be/DMlfGwZ1oho
