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
CMAKE_COMMAND = /home/wxj/clion-2020.1.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/wxj/clion-2020.1.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wxj/CLionProjects/camera_client

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wxj/CLionProjects/camera_client/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/camera_client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera_client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera_client.dir/flags.make

CMakeFiles/camera_client.dir/main.cpp.o: CMakeFiles/camera_client.dir/flags.make
CMakeFiles/camera_client.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wxj/CLionProjects/camera_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/camera_client.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera_client.dir/main.cpp.o -c /home/wxj/CLionProjects/camera_client/main.cpp

CMakeFiles/camera_client.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_client.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wxj/CLionProjects/camera_client/main.cpp > CMakeFiles/camera_client.dir/main.cpp.i

CMakeFiles/camera_client.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_client.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wxj/CLionProjects/camera_client/main.cpp -o CMakeFiles/camera_client.dir/main.cpp.s

# Object files for target camera_client
camera_client_OBJECTS = \
"CMakeFiles/camera_client.dir/main.cpp.o"

# External object files for target camera_client
camera_client_EXTERNAL_OBJECTS =

camera_client: CMakeFiles/camera_client.dir/main.cpp.o
camera_client: CMakeFiles/camera_client.dir/build.make
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
camera_client: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
camera_client: CMakeFiles/camera_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wxj/CLionProjects/camera_client/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable camera_client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera_client.dir/build: camera_client

.PHONY : CMakeFiles/camera_client.dir/build

CMakeFiles/camera_client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera_client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera_client.dir/clean

CMakeFiles/camera_client.dir/depend:
	cd /home/wxj/CLionProjects/camera_client/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wxj/CLionProjects/camera_client /home/wxj/CLionProjects/camera_client /home/wxj/CLionProjects/camera_client/cmake-build-debug /home/wxj/CLionProjects/camera_client/cmake-build-debug /home/wxj/CLionProjects/camera_client/cmake-build-debug/CMakeFiles/camera_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera_client.dir/depend

