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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yixin/fuerte_workspace/sandbox/navigation/amcl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yixin/fuerte_workspace/sandbox/navigation/amcl/build

# Utility rule file for rostest_test_small_loop_prf.xml_result.

# Include the progress variables for this target.
include CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/progress.make

CMakeFiles/rostest_test_small_loop_prf.xml_result:
	cd /home/yixin/fuerte_workspace/sandbox/navigation/amcl && /opt/ros/fuerte/share/rosunit/bin/check_test_ran.py --rostest amcl test/small_loop_prf.xml

rostest_test_small_loop_prf.xml_result: CMakeFiles/rostest_test_small_loop_prf.xml_result
rostest_test_small_loop_prf.xml_result: CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/build.make
.PHONY : rostest_test_small_loop_prf.xml_result

# Rule to build all files generated by this target.
CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/build: rostest_test_small_loop_prf.xml_result
.PHONY : CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/build

CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/clean

CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/depend:
	cd /home/yixin/fuerte_workspace/sandbox/navigation/amcl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yixin/fuerte_workspace/sandbox/navigation/amcl /home/yixin/fuerte_workspace/sandbox/navigation/amcl /home/yixin/fuerte_workspace/sandbox/navigation/amcl/build /home/yixin/fuerte_workspace/sandbox/navigation/amcl/build /home/yixin/fuerte_workspace/sandbox/navigation/amcl/build/CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rostest_test_small_loop_prf.xml_result.dir/depend

