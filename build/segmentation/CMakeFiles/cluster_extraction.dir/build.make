# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/faisallab008/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/faisallab008/catkin_ws/build

# Include any dependencies generated for this target.
include segmentation/CMakeFiles/cluster_extraction.dir/depend.make

# Include the progress variables for this target.
include segmentation/CMakeFiles/cluster_extraction.dir/progress.make

# Include the compile flags for this target's objects.
include segmentation/CMakeFiles/cluster_extraction.dir/flags.make

segmentation/CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.o: segmentation/CMakeFiles/cluster_extraction.dir/flags.make
segmentation/CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.o: /home/faisallab008/catkin_ws/src/segmentation/src/cluster_extraction.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/faisallab008/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object segmentation/CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.o"
	cd /home/faisallab008/catkin_ws/build/segmentation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.o -c /home/faisallab008/catkin_ws/src/segmentation/src/cluster_extraction.cpp

segmentation/CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.i"
	cd /home/faisallab008/catkin_ws/build/segmentation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/faisallab008/catkin_ws/src/segmentation/src/cluster_extraction.cpp > CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.i

segmentation/CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.s"
	cd /home/faisallab008/catkin_ws/build/segmentation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/faisallab008/catkin_ws/src/segmentation/src/cluster_extraction.cpp -o CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.s

# Object files for target cluster_extraction
cluster_extraction_OBJECTS = \
"CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.o"

# External object files for target cluster_extraction
cluster_extraction_EXTERNAL_OBJECTS =

/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: segmentation/CMakeFiles/cluster_extraction.dir/src/cluster_extraction.cpp.o
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: segmentation/CMakeFiles/cluster_extraction.dir/build.make
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libpcl_ros_filters.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libpcl_ros_io.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libpcl_ros_tf.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_common.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_octree.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_io.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_kdtree.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_search.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_sample_consensus.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_filters.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_features.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_keypoints.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_segmentation.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_visualization.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_outofcore.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_registration.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_recognition.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_surface.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_people.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_tracking.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libpcl_apps.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libqhull.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libOpenNI.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libvtkCommon.so.5.8.0
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libvtkRendering.so.5.8.0
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libvtkHybrid.so.5.8.0
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libvtkCharts.so.5.8.0
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libnodeletlib.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libbondcpp.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libclass_loader.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/libPocoFoundation.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libdl.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libroslib.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/librospack.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/librosbag.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/librosbag_storage.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libroslz4.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/liblz4.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libtopic_tools.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libtf.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libtf2_ros.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libactionlib.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libmessage_filters.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libtf2.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libroscpp.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/librosconsole.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/liblog4cxx.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/librostime.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /opt/ros/indigo/lib/libcpp_common.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction: segmentation/CMakeFiles/cluster_extraction.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/faisallab008/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction"
	cd /home/faisallab008/catkin_ws/build/segmentation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cluster_extraction.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
segmentation/CMakeFiles/cluster_extraction.dir/build: /home/faisallab008/catkin_ws/devel/lib/segmentation/cluster_extraction

.PHONY : segmentation/CMakeFiles/cluster_extraction.dir/build

segmentation/CMakeFiles/cluster_extraction.dir/clean:
	cd /home/faisallab008/catkin_ws/build/segmentation && $(CMAKE_COMMAND) -P CMakeFiles/cluster_extraction.dir/cmake_clean.cmake
.PHONY : segmentation/CMakeFiles/cluster_extraction.dir/clean

segmentation/CMakeFiles/cluster_extraction.dir/depend:
	cd /home/faisallab008/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/faisallab008/catkin_ws/src /home/faisallab008/catkin_ws/src/segmentation /home/faisallab008/catkin_ws/build /home/faisallab008/catkin_ws/build/segmentation /home/faisallab008/catkin_ws/build/segmentation/CMakeFiles/cluster_extraction.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : segmentation/CMakeFiles/cluster_extraction.dir/depend

