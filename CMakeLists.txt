cmake_minimum_required(VERSION 2.8.3)
project(ur3_ros_assignment_2)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
# Generate services in the 'srv' folder
 add_service_files(
   FILES
   #Service1.srv
   #Service2.srv
   GripperControl.srv
   LEDControl.srv
 )
# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
