execute_process(COMMAND "/home/ahooker/catkin_ws/build/ROS-TCP-Endpoint/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ahooker/catkin_ws/build/ROS-TCP-Endpoint/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
