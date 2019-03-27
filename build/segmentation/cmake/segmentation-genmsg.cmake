# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "segmentation: 0 messages, 4 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(segmentation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/seg.srv" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/faisallab008/catkin_ws/src/segmentation/srv/seg.srv" "sensor_msgs/PointField:std_msgs/MultiArrayLayout:sensor_msgs/PointCloud2:std_msgs/Header:std_msgs/Float32MultiArray:std_msgs/MultiArrayDimension"
)

get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/gazeOptimiser.srv" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/faisallab008/catkin_ws/src/segmentation/srv/gazeOptimiser.srv" "std_msgs/Float32MultiArray:std_msgs/MultiArrayDimension:geometry_msgs/Point:std_msgs/MultiArrayLayout"
)

get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/gazePoint.srv" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/faisallab008/catkin_ws/src/segmentation/srv/gazePoint.srv" ""
)

get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/faisallab008/catkin_ws/src/segmentation/srv/AddTwoInts.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/seg.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)
_generate_srv_cpp(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/gazeOptimiser.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)
_generate_srv_cpp(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/gazePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)
_generate_srv_cpp(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)

### Generating Module File
_generate_module_cpp(segmentation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(segmentation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(segmentation_generate_messages segmentation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/seg.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/gazeOptimiser.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/gazePoint.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmentation_gencpp)
add_dependencies(segmentation_gencpp segmentation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmentation_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/seg.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)
_generate_srv_lisp(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/gazeOptimiser.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)
_generate_srv_lisp(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/gazePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)
_generate_srv_lisp(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)

### Generating Module File
_generate_module_lisp(segmentation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(segmentation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(segmentation_generate_messages segmentation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/seg.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/gazeOptimiser.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/gazePoint.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmentation_genlisp)
add_dependencies(segmentation_genlisp segmentation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmentation_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/seg.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)
_generate_srv_py(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/gazeOptimiser.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)
_generate_srv_py(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/gazePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)
_generate_srv_py(segmentation
  "/home/faisallab008/catkin_ws/src/segmentation/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)

### Generating Module File
_generate_module_py(segmentation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(segmentation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(segmentation_generate_messages segmentation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/seg.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/gazeOptimiser.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/gazePoint.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/faisallab008/catkin_ws/src/segmentation/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmentation_genpy)
add_dependencies(segmentation_genpy segmentation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmentation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(segmentation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(segmentation_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(segmentation_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(segmentation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(segmentation_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(segmentation_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(segmentation_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(segmentation_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(segmentation_generate_messages_py geometry_msgs_generate_messages_py)
endif()
