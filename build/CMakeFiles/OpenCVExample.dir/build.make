# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nicolas/Desktop/ATTACKS-SW/OpenCVDemo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nicolas/Desktop/ATTACKS-SW/build

# Include any dependencies generated for this target.
include CMakeFiles/OpenCVExample.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/OpenCVExample.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/OpenCVExample.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OpenCVExample.dir/flags.make

CMakeFiles/OpenCVExample.dir/main.cpp.o: CMakeFiles/OpenCVExample.dir/flags.make
CMakeFiles/OpenCVExample.dir/main.cpp.o: /home/nicolas/Desktop/ATTACKS-SW/OpenCVDemo/main.cpp
CMakeFiles/OpenCVExample.dir/main.cpp.o: CMakeFiles/OpenCVExample.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nicolas/Desktop/ATTACKS-SW/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OpenCVExample.dir/main.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/OpenCVExample.dir/main.cpp.o -MF CMakeFiles/OpenCVExample.dir/main.cpp.o.d -o CMakeFiles/OpenCVExample.dir/main.cpp.o -c /home/nicolas/Desktop/ATTACKS-SW/OpenCVDemo/main.cpp

CMakeFiles/OpenCVExample.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenCVExample.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nicolas/Desktop/ATTACKS-SW/OpenCVDemo/main.cpp > CMakeFiles/OpenCVExample.dir/main.cpp.i

CMakeFiles/OpenCVExample.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenCVExample.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nicolas/Desktop/ATTACKS-SW/OpenCVDemo/main.cpp -o CMakeFiles/OpenCVExample.dir/main.cpp.s

# Object files for target OpenCVExample
OpenCVExample_OBJECTS = \
"CMakeFiles/OpenCVExample.dir/main.cpp.o"

# External object files for target OpenCVExample
OpenCVExample_EXTERNAL_OBJECTS =

OpenCVExample: CMakeFiles/OpenCVExample.dir/main.cpp.o
OpenCVExample: CMakeFiles/OpenCVExample.dir/build.make
OpenCVExample: /usr/local/lib/libopencv_gapi.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_stitching.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_aruco.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_bgsegm.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_bioinspired.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_ccalib.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_dnn_objdetect.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_dnn_superres.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_dpm.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_face.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_freetype.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_fuzzy.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_hfs.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_img_hash.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_intensity_transform.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_line_descriptor.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_mcc.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_quality.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_rapid.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_reg.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_rgbd.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_saliency.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_stereo.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_structured_light.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_superres.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_surface_matching.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_tracking.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_videostab.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_wechat_qrcode.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_xfeatures2d.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_xobjdetect.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_xphoto.so.4.8.0
OpenCVExample: /home/nicolas/repos/libtorch/lib/libtorch.so
OpenCVExample: /home/nicolas/repos/libtorch/lib/libc10.so
OpenCVExample: /home/nicolas/repos/libtorch/lib/libkineto.a
OpenCVExample: /usr/local/lib/libopencv_shape.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_highgui.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_datasets.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_plot.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_text.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_ml.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_phase_unwrapping.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_optflow.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_ximgproc.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_video.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_videoio.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_imgcodecs.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_objdetect.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_calib3d.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_dnn.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_features2d.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_flann.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_photo.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_imgproc.so.4.8.0
OpenCVExample: /usr/local/lib/libopencv_core.so.4.8.0
OpenCVExample: /home/nicolas/repos/libtorch/lib/libc10.so
OpenCVExample: CMakeFiles/OpenCVExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nicolas/Desktop/ATTACKS-SW/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable OpenCVExample"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OpenCVExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OpenCVExample.dir/build: OpenCVExample
.PHONY : CMakeFiles/OpenCVExample.dir/build

CMakeFiles/OpenCVExample.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OpenCVExample.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OpenCVExample.dir/clean

CMakeFiles/OpenCVExample.dir/depend:
	cd /home/nicolas/Desktop/ATTACKS-SW/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nicolas/Desktop/ATTACKS-SW/OpenCVDemo /home/nicolas/Desktop/ATTACKS-SW/OpenCVDemo /home/nicolas/Desktop/ATTACKS-SW/build /home/nicolas/Desktop/ATTACKS-SW/build /home/nicolas/Desktop/ATTACKS-SW/build/CMakeFiles/OpenCVExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/OpenCVExample.dir/depend
