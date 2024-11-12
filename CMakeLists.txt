cmake_minimum_required(VERSION 3.0.2)
project(controlbox_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}

)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




