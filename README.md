# ROS-assignment-2
Assignment 2 for the ROS-course at Fontys Eindhoven.

The assignment consists of a simple pick and place task, where the object and it's position for picking and placing on the working surface is your choice. The assigbnment is design for using the UR3 robot manipulator, but you are free to use a UR5 or the Sawyer from Rethink Robotics.

These are examples of how it could look like in [rviz](https://www.youtube.com/watch?v=7vrwx3QfseM)  and in [real life](https://www.youtube.com/watch?v=ayQRK4hnlew&feature=youtu.be)

# Requirments :
You Pick an object from a preprogrammed position and place it at a diffrent preprogrammed position.
For this assignment we recommend using Moveit! It is a path planner/kinemetics solver for robotic arms (manipulators) that has a really easy interface for Python and C++

To install Moveit! simply run:
```
sudo apt-get install ros-kinetic-moveit
sudo apt-get install ros-kinetic-ur3-moveit-config 
```
To make sure you will not get an error with pyasimp install pip and pyasimp:
```
sudo apt-get install python-pip
pip install pyassimp
```

In case it does not install look [here](http://moveit.ros.org/install/) for more information.
You will also have to install the ur_modern_driver form ThomasTimm. You can download it from his [github](https://github.com/ThomasTimm/ur_modern_driver) or run:
```
cd catkin_ws/src
git clone https://github.com/ThomasTimm/ur_modern_driver.git
sudo apt-get install ros-kinetic-ur-*
```
This will give you the ability to run your program on a real Universal robot (UR3, UR5, UR10).

**Beaware that you need to change the ur_modern_driver/src/ur_hardware_interface.cpp to the ur_hardware_interface.cpp that is uploaded to this github. Copy it and paste it in your catkin_ws/src/ur_modern_driver/src. Don't forget to delete the old one.**

# Installing the ROS-assignment-2 package
This package contains the UR3, Gripper and working environment descriptions for planning and executing multi-joint movements with MoveIt!.:
## Installing in your catkin work space:
```
cd catkin_ws/src
git clone https://github.com/RoboHubEindhoven/ur3_ros_assignment_2
catkin_make
cd catkin_ws/src/ur3_ros_assignment_2
sudo chmod +x script/gripper_service_client.py 
sudo chmod +x script/gripper_service_server.py 
```
## Test your installed package:

After installing the package run:
```
roslaunch ur3_ros_assignment demo.launch
```
# Using the real UR3
After connecting to the ethernet cable of the robot.
Run this line to start the connection between ROS and the UR3:
```
roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=192.168.2.134
```
To now see the visualization, run this in another terminal:
```
roslaunch ur3_ros_assignment_2 real.launch
```
**Test if the robot can plan and execute without colliding into its surroundings.**

After this run your own program and you can test it. 

**REMINDER: ALWAYS KEEP EMERGENCY STOP IN REACH IF THE ROBOT IS MOVING!!!**

# Using the gripper
To use the Gripper run (remember to connect the gripper to your laptop with the usb cable):
``` 
rosrun ur3_ros_assignment_2 gripper_service_server.py
```
and to test the gripper you can run this in an other terminal:
```
rosrun ur3_ros_assignment_2 gripper_service_client.py
```
The client part needs to be added in you statemachine so you can control it from your statemachine. **YOU CAN NOT SEE THE GRIPPER WORKING IN RVIZ.**

# The assignment:
this assignment can be performed in **groups of 2 or 3 students**, so, not individual assignments or groups of more than 3 will be accepted.

## Things you need to do:

- A statemachine to preform different steps
- Add the gripper service to your statemachine. 
- A controller for Moveit! ([tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#getting-started)) 
The tutorial doesn't install the robot model for the panda so dont forget to run:
```
sudo apt-get install ros-kinetic-franka-description
```
This can be inside of one single program. You can also devide them into two diffrent programs and connect them with a service. To do that look [here](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

## Step by step:

- Try to get your robot arm in rviz with Moveit! so you can drag the end-effector around. Also make sure you can plan and execute properly. 
- Make the controller for Moveit! in a python program. It should be able to move the robot to a specific position in space.
- Make a statemachine that can send a position to your controller program.
- Make sure the robot makes no complicated moves or collides with its surroundings (including the ground) in rviz.
- Add the gripper to your statemachine.
- Try your program on the robot.

## Tips:
1. 
You can copy the current position of the robot by printing group.get_current_pose()
If you are connected to the robot you can use Freedrive to position the robot exactly where you want it to be. Then copy the position with Moveit!. This way it is easier to get good positions and you know it wont colide with the base.

2. 
If the robot moves strange **look into the Constrains** that can be set in the yaml files of the Moveit! package.

## Deliverables
1. Report explaining the design of your state machine and your code.
2. Source code.
3. Video showing the movement of the robot in real life and in Rviz.

## Deadline
the assignment 2 (pick and place) must be delivered by mail to (p.negreterubio@fontys.nl) before 23:00 hours on Tuesday 12 of June 2018.






