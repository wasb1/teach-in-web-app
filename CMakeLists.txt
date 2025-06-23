cmake_minimum_required(VERSION 3.0.2)
project(franka_repl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   src/teach_in.py
   src/execute.py
   scripts/realtime_control.py
   scripts/emergency_stop_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 

catkin_python_setup()
