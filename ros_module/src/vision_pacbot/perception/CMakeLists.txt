cmake_minimum_required(VERSION 3.0.2)
project(perception)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    rospy
#    message_generation
#    std_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
#   LIBRARIES ${PROJECT_NAME}
)

catkin_install_python(PROGRAMS
  src/${PROJECT_NAME}/rgb_handler.py
  src/${PROJECT_NAME}/depth_handler.py
  src/${PROJECT_NAME}/vision_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_python_setup()
