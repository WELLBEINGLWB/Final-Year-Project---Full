execute_process(COMMAND "/home/faisallab008/catkin_ws/build/python_natnet/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/faisallab008/catkin_ws/build/python_natnet/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
