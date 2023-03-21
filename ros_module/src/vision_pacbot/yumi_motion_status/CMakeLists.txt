cmake_minimum_required(VERSION 3.0.2)
project(yumi_motion_status)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    rospy
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
)

catkin_install_python(PROGRAMS
  src/${PROJECT_NAME}/yumi_motion_status.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_python_setup()