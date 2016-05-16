cmake_minimum_required(VERSION 2.8.3)
project(ddynamic_reconfigure_python)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)