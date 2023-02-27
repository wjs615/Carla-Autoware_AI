execute_process(COMMAND "/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/build/ros-bridge/carla_fake_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/build/ros-bridge/carla_fake_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
