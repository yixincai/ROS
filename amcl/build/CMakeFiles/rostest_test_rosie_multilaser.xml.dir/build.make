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

# Utility rule file for rostest_test_rosie_multilaser.xml.

# Include the progress variables for this target.
include CMakeFiles/rostest_test_rosie_multilaser.xml.dir/progress.make

CMakeFiles/rostest_test_rosie_multilaser.xml: ../test/rosie_multilaser.xml
	cd /home/yixin/fuerte_workspace/sandbox/navigation/amcl && rostest test/rosie_multilaser.xml

rostest_test_rosie_multilaser.xml: CMakeFiles/rostest_test_rosie_multilaser.xml
rostest_test_rosie_multilaser.xml: CMakeFiles/rostest_test_rosie_multilaser.xml.dir/build.make
.PHONY : rostest_test_rosie_multilaser.xml

# Rule to build all files generated by this target.
CMakeFiles/rostest_test_rosie_multilaser.xml.dir/build: rostest_test_rosie_multilaser.xml
.PHONY : CMakeFiles/rostest_test_rosie_multilaser.xml.dir/build

CMakeFiles/rostest_test_rosie_multilaser.xml.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rostest_test_rosie_multilaser.xml.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rostest_test_rosie_multilaser.xml.dir/clean

CMakeFiles/rostest_test_rosie_multilaser.xml.dir/depend:
	cd /home/yixin/fuerte_workspace/sandbox/navigation/amcl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yixin/fuerte_workspace/sandbox/navigation/amcl /home/yixin/fuerte_workspace/sandbox/navigation/amcl /home/yixin/fuerte_workspace/sandbox/navigation/amcl/build /home/yixin/fuerte_workspace/sandbox/navigation/amcl/build /home/yixin/fuerte_workspace/sandbox/navigation/amcl/build/CMakeFiles/rostest_test_rosie_multilaser.xml.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rostest_test_rosie_multilaser.xml.dir/depend

