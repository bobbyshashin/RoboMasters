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
CMAKE_SOURCE_DIR = /home/ubuntu/Documents/RoboMasters/infantry/rune

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/Documents/RoboMasters/infantry/rune

# Include any dependencies generated for this target.
include CMakeFiles/function_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/function_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/function_test.dir/flags.make

CMakeFiles/function_test.dir/getImgs.cpp.o: CMakeFiles/function_test.dir/flags.make
CMakeFiles/function_test.dir/getImgs.cpp.o: getImgs.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/Documents/RoboMasters/infantry/rune/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/function_test.dir/getImgs.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/function_test.dir/getImgs.cpp.o -c /home/ubuntu/Documents/RoboMasters/infantry/rune/getImgs.cpp

CMakeFiles/function_test.dir/getImgs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/function_test.dir/getImgs.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/Documents/RoboMasters/infantry/rune/getImgs.cpp > CMakeFiles/function_test.dir/getImgs.cpp.i

CMakeFiles/function_test.dir/getImgs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/function_test.dir/getImgs.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/Documents/RoboMasters/infantry/rune/getImgs.cpp -o CMakeFiles/function_test.dir/getImgs.cpp.s

CMakeFiles/function_test.dir/getImgs.cpp.o.requires:
.PHONY : CMakeFiles/function_test.dir/getImgs.cpp.o.requires

CMakeFiles/function_test.dir/getImgs.cpp.o.provides: CMakeFiles/function_test.dir/getImgs.cpp.o.requires
	$(MAKE) -f CMakeFiles/function_test.dir/build.make CMakeFiles/function_test.dir/getImgs.cpp.o.provides.build
.PHONY : CMakeFiles/function_test.dir/getImgs.cpp.o.provides

CMakeFiles/function_test.dir/getImgs.cpp.o.provides.build: CMakeFiles/function_test.dir/getImgs.cpp.o

# Object files for target function_test
function_test_OBJECTS = \
"CMakeFiles/function_test.dir/getImgs.cpp.o"

# External object files for target function_test
function_test_EXTERNAL_OBJECTS =

function_test: CMakeFiles/function_test.dir/getImgs.cpp.o
function_test: CMakeFiles/function_test.dir/build.make
function_test: /usr/lib/libopencv_vstab.so.2.4.10
function_test: /usr/lib/libopencv_tegra.so.2.4.10
function_test: /usr/lib/libopencv_imuvstab.so.2.4.10
function_test: /usr/lib/libopencv_facedetect.so.2.4.10
function_test: /usr/lib/libopencv_esm_panorama.so.2.4.10
function_test: /usr/lib/libopencv_videostab.so.2.4.10
function_test: /usr/lib/libopencv_video.so.2.4.10
function_test: /usr/lib/libopencv_ts.a
function_test: /usr/lib/libopencv_superres.so.2.4.10
function_test: /usr/lib/libopencv_stitching.so.2.4.10
function_test: /usr/lib/libopencv_photo.so.2.4.10
function_test: /usr/lib/libopencv_objdetect.so.2.4.10
function_test: /usr/lib/libopencv_ml.so.2.4.10
function_test: /usr/lib/libopencv_legacy.so.2.4.10
function_test: /usr/lib/libopencv_imgproc.so.2.4.10
function_test: /usr/lib/libopencv_highgui.so.2.4.10
function_test: /usr/lib/libopencv_gpu.so.2.4.10
function_test: /usr/lib/libopencv_flann.so.2.4.10
function_test: /usr/lib/libopencv_features2d.so.2.4.10
function_test: /usr/lib/libopencv_core.so.2.4.10
function_test: /usr/lib/libopencv_contrib.so.2.4.10
function_test: /usr/lib/libopencv_calib3d.so.2.4.10
function_test: /usr/lib/libopencv_tegra.so.2.4.10
function_test: /usr/lib/libopencv_stitching.so.2.4.10
function_test: /usr/lib/libopencv_gpu.so.2.4.10
function_test: /usr/lib/libopencv_photo.so.2.4.10
function_test: /usr/lib/libopencv_objdetect.so.2.4.10
function_test: /usr/lib/libopencv_legacy.so.2.4.10
function_test: /usr/lib/libopencv_video.so.2.4.10
function_test: /usr/lib/libopencv_ml.so.2.4.10
function_test: /usr/lib/libopencv_calib3d.so.2.4.10
function_test: /usr/lib/libopencv_features2d.so.2.4.10
function_test: /usr/lib/libopencv_highgui.so.2.4.10
function_test: /usr/lib/libopencv_imgproc.so.2.4.10
function_test: /usr/lib/libopencv_flann.so.2.4.10
function_test: /usr/lib/libopencv_core.so.2.4.10
function_test: CMakeFiles/function_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable function_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/function_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/function_test.dir/build: function_test
.PHONY : CMakeFiles/function_test.dir/build

CMakeFiles/function_test.dir/requires: CMakeFiles/function_test.dir/getImgs.cpp.o.requires
.PHONY : CMakeFiles/function_test.dir/requires

CMakeFiles/function_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/function_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/function_test.dir/clean

CMakeFiles/function_test.dir/depend:
	cd /home/ubuntu/Documents/RoboMasters/infantry/rune && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Documents/RoboMasters/infantry/rune /home/ubuntu/Documents/RoboMasters/infantry/rune /home/ubuntu/Documents/RoboMasters/infantry/rune /home/ubuntu/Documents/RoboMasters/infantry/rune /home/ubuntu/Documents/RoboMasters/infantry/rune/CMakeFiles/function_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/function_test.dir/depend

