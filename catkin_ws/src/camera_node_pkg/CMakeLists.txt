cmake_minimum_required(VERSION 3.0.2)
project(camera_node_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS scripts/camera_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch files)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.launch"
        PATTERN ".svn" EXCLUDE)

