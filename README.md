# ROS-assignment-2-
Assignment 2 for the ROS-course at Fontys Eindhoven

You will need to use a robot arm to preform a manipulation test.

You need to pick and place a object of choice. 

# Requirments :
You Pick an object form a preprogrammed position and place it at a diffrent preprogrammed position.
For this assignment we recommend using Moveit! It is a path planner/ kinemetics solver for robotic arms that has a really easy interface for Python and C++

To install Moveit! simply run:
```
sudo apt-get install ros-kinetic-moveit
```
If that doesn't work look [here](http://moveit.ros.org/install/).
You will also have to install the ur_modern_driver form ThomasTimm. You can download it from his [github](https://github.com/ThomasTimm/ur_modern_driver)
This will give you the ability to run you program on the real robot.

The github has only one problem you need to change the ur_modern_driver/src/ur_hardware_interface.cpp to the ur_hardware_interface.cpp that is uploaded to this github. Copy it and paste it in your catkin_ws/src/ur_modern_driver/src.

You can download the ur3_with_gripper_ur_moveit_config package from this github. This will give you a simulation of the arm.
When you installed the package you can run:
```
roslaunch ur3_with_gripper_ur_moveit_config demo.launch
```

To run your program on the real UR3, First:
```
roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=192.168.2.134
```
Then in another terminal:
```
roslaunch ur3_with_gripper_ur_moveit_config real.launch
```
Test if the robot can plan and execute without coliding into anything.

After this run your own program and you can test it. 

REMINDER: ALWAYS KEEP EMERGENCY STOP IN REACH IF THE ROBOT IS MOVING!!!


# Things you need to build:

- A statemachine to preform diffrent steps
- A controller for Moveit! ([tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#getting-started))

This can be inside of one single program. You can also devide them into two diffrent programs and connect them with a service. To do that look [here](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

# Step by step:

- Try to get your robot arm in rviz with Moveit! so you can drag the eef arround. Also make sure you can plan and execute properly. 
- Make the controller for Moveit! in a python program. It should be able to move the robot to a specific position in space.
- Make a statemachine that can send a position to your controller program.
- Make sure the robot makes no complicated moves or collisions with the ground in the simulation in rviz.
- Add the gripper to your statemachine.
- Try your program on the robot.

# Tips:
1. 
You can copy the current position of the robot by printing group.getCurrentPose()
If you are connected to the robot you can use Freedrive to position the robot exactly where you want him to be. Then copy the position with Moveit!. This way it is easier to get good positions and you know it wont colide with the base.

2. 
If the robot moves strange look into the Constrains that can be set in the yaml files of the Moveit! package.

3. 
If you don't want it to collide with the ground you can add scene objects. More information about that [here](http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/planning_scene_ros_api_tutorial.html)




