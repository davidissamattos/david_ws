# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/davidis/david_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/davidis/david_ws/src

# Include any dependencies generated for this target.
include tb_self_experimentation/CMakeFiles/apriltag_filter.dir/depend.make

# Include the progress variables for this target.
include tb_self_experimentation/CMakeFiles/apriltag_filter.dir/progress.make

# Include the compile flags for this target's objects.
include tb_self_experimentation/CMakeFiles/apriltag_filter.dir/flags.make

tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o: tb_self_experimentation/CMakeFiles/apriltag_filter.dir/flags.make
tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o: tb_self_experimentation/src/apriltag_filter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/davidis/david_ws/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o"
	cd /home/davidis/david_ws/src/tb_self_experimentation && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o -c /home/davidis/david_ws/src/tb_self_experimentation/src/apriltag_filter.cpp

tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.i"
	cd /home/davidis/david_ws/src/tb_self_experimentation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/davidis/david_ws/src/tb_self_experimentation/src/apriltag_filter.cpp > CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.i

tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.s"
	cd /home/davidis/david_ws/src/tb_self_experimentation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/davidis/david_ws/src/tb_self_experimentation/src/apriltag_filter.cpp -o CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.s

tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o.requires:
.PHONY : tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o.requires

tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o.provides: tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o.requires
	$(MAKE) -f tb_self_experimentation/CMakeFiles/apriltag_filter.dir/build.make tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o.provides.build
.PHONY : tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o.provides

tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o.provides.build: tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o

# Object files for target apriltag_filter
apriltag_filter_OBJECTS = \
"CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o"

# External object files for target apriltag_filter
apriltag_filter_EXTERNAL_OBJECTS =

/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: tb_self_experimentation/CMakeFiles/apriltag_filter.dir/build.make
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /home/davidis/david_ws/devel/lib/libapriltags.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libimage_transport.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libtf.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libtf2_ros.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libmessage_filters.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libtf2.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libcv_bridge.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libnodeletlib.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libbondcpp.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libclass_loader.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/libPocoFoundation.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libdl.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libroslib.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libactionlib.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libroscpp.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/librosconsole.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/liblog4cxx.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/librostime.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /opt/ros/indigo/lib/libcpp_common.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter: tb_self_experimentation/CMakeFiles/apriltag_filter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter"
	cd /home/davidis/david_ws/src/tb_self_experimentation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/apriltag_filter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tb_self_experimentation/CMakeFiles/apriltag_filter.dir/build: /home/davidis/david_ws/devel/lib/tb_self_experimentation/apriltag_filter
.PHONY : tb_self_experimentation/CMakeFiles/apriltag_filter.dir/build

tb_self_experimentation/CMakeFiles/apriltag_filter.dir/requires: tb_self_experimentation/CMakeFiles/apriltag_filter.dir/src/apriltag_filter.cpp.o.requires
.PHONY : tb_self_experimentation/CMakeFiles/apriltag_filter.dir/requires

tb_self_experimentation/CMakeFiles/apriltag_filter.dir/clean:
	cd /home/davidis/david_ws/src/tb_self_experimentation && $(CMAKE_COMMAND) -P CMakeFiles/apriltag_filter.dir/cmake_clean.cmake
.PHONY : tb_self_experimentation/CMakeFiles/apriltag_filter.dir/clean

tb_self_experimentation/CMakeFiles/apriltag_filter.dir/depend:
	cd /home/davidis/david_ws/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/davidis/david_ws/src /home/davidis/david_ws/src/tb_self_experimentation /home/davidis/david_ws/src /home/davidis/david_ws/src/tb_self_experimentation /home/davidis/david_ws/src/tb_self_experimentation/CMakeFiles/apriltag_filter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tb_self_experimentation/CMakeFiles/apriltag_filter.dir/depend

