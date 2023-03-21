cmake_minimum_required(VERSION 3.0.2)
project(vision_logic)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    rospy
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
#   LIBRARIES ${PROJECT_NAME}
)

catkin_install_python(PROGRAMS
  src/${PROJECT_NAME}/vision_logic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_python_setup()
#===========================================================
# Install
#===========================================================
install(
  DIRECTORY database launch
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)
#===========================================================