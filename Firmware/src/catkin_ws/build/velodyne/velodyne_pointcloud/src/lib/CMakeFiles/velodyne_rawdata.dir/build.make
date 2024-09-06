# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/eren/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eren/catkin_ws/build

# Include any dependencies generated for this target.
include velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/depend.make

# Include the progress variables for this target.
include velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/progress.make

# Include the compile flags for this target's objects.
include velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/flags.make

velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/rawdata.cc.o: velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/flags.make
velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/rawdata.cc.o: /home/eren/catkin_ws/src/velodyne/velodyne_pointcloud/src/lib/rawdata.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eren/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/rawdata.cc.o"
	cd /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/velodyne_rawdata.dir/rawdata.cc.o -c /home/eren/catkin_ws/src/velodyne/velodyne_pointcloud/src/lib/rawdata.cc

velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/rawdata.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/velodyne_rawdata.dir/rawdata.cc.i"
	cd /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eren/catkin_ws/src/velodyne/velodyne_pointcloud/src/lib/rawdata.cc > CMakeFiles/velodyne_rawdata.dir/rawdata.cc.i

velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/rawdata.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/velodyne_rawdata.dir/rawdata.cc.s"
	cd /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eren/catkin_ws/src/velodyne/velodyne_pointcloud/src/lib/rawdata.cc -o CMakeFiles/velodyne_rawdata.dir/rawdata.cc.s

velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/calibration.cc.o: velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/flags.make
velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/calibration.cc.o: /home/eren/catkin_ws/src/velodyne/velodyne_pointcloud/src/lib/calibration.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eren/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/calibration.cc.o"
	cd /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/velodyne_rawdata.dir/calibration.cc.o -c /home/eren/catkin_ws/src/velodyne/velodyne_pointcloud/src/lib/calibration.cc

velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/calibration.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/velodyne_rawdata.dir/calibration.cc.i"
	cd /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eren/catkin_ws/src/velodyne/velodyne_pointcloud/src/lib/calibration.cc > CMakeFiles/velodyne_rawdata.dir/calibration.cc.i

velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/calibration.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/velodyne_rawdata.dir/calibration.cc.s"
	cd /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eren/catkin_ws/src/velodyne/velodyne_pointcloud/src/lib/calibration.cc -o CMakeFiles/velodyne_rawdata.dir/calibration.cc.s

# Object files for target velodyne_rawdata
velodyne_rawdata_OBJECTS = \
"CMakeFiles/velodyne_rawdata.dir/rawdata.cc.o" \
"CMakeFiles/velodyne_rawdata.dir/calibration.cc.o"

# External object files for target velodyne_rawdata
velodyne_rawdata_EXTERNAL_OBJECTS =

/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/rawdata.cc.o
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/calibration.cc.o
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/build.make
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /home/eren/catkin_ws/devel/lib/libvelodyne_input.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libnodeletlib.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libbondcpp.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libclass_loader.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libroslib.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/librospack.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libtf.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libtf2_ros.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libactionlib.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libmessage_filters.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libtf2.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libdiagnostic_updater.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libroscpp.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/librosconsole.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/librostime.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /opt/ros/noetic/lib/libcpp_common.so
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so: velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eren/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library /home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so"
	cd /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/velodyne_rawdata.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/build: /home/eren/catkin_ws/devel/lib/libvelodyne_rawdata.so

.PHONY : velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/build

velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/clean:
	cd /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib && $(CMAKE_COMMAND) -P CMakeFiles/velodyne_rawdata.dir/cmake_clean.cmake
.PHONY : velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/clean

velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/depend:
	cd /home/eren/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eren/catkin_ws/src /home/eren/catkin_ws/src/velodyne/velodyne_pointcloud/src/lib /home/eren/catkin_ws/build /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib /home/eren/catkin_ws/build/velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : velodyne/velodyne_pointcloud/src/lib/CMakeFiles/velodyne_rawdata.dir/depend

