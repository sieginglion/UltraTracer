# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/jon/CLionProjects/UltraTracer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/jon/CLionProjects/UltraTracer/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/UltraTracer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/UltraTracer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/UltraTracer.dir/flags.make

CMakeFiles/UltraTracer.dir/main.cpp.o: CMakeFiles/UltraTracer.dir/flags.make
CMakeFiles/UltraTracer.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jon/CLionProjects/UltraTracer/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/UltraTracer.dir/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UltraTracer.dir/main.cpp.o -c /Users/jon/CLionProjects/UltraTracer/main.cpp

CMakeFiles/UltraTracer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UltraTracer.dir/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jon/CLionProjects/UltraTracer/main.cpp > CMakeFiles/UltraTracer.dir/main.cpp.i

CMakeFiles/UltraTracer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UltraTracer.dir/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jon/CLionProjects/UltraTracer/main.cpp -o CMakeFiles/UltraTracer.dir/main.cpp.s

# Object files for target UltraTracer
UltraTracer_OBJECTS = \
"CMakeFiles/UltraTracer.dir/main.cpp.o"

# External object files for target UltraTracer
UltraTracer_EXTERNAL_OBJECTS =

UltraTracer: CMakeFiles/UltraTracer.dir/main.cpp.o
UltraTracer: CMakeFiles/UltraTracer.dir/build.make
UltraTracer: CMakeFiles/UltraTracer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/jon/CLionProjects/UltraTracer/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable UltraTracer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/UltraTracer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/UltraTracer.dir/build: UltraTracer

.PHONY : CMakeFiles/UltraTracer.dir/build

CMakeFiles/UltraTracer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/UltraTracer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/UltraTracer.dir/clean

CMakeFiles/UltraTracer.dir/depend:
	cd /Users/jon/CLionProjects/UltraTracer/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/jon/CLionProjects/UltraTracer /Users/jon/CLionProjects/UltraTracer /Users/jon/CLionProjects/UltraTracer/cmake-build-debug /Users/jon/CLionProjects/UltraTracer/cmake-build-debug /Users/jon/CLionProjects/UltraTracer/cmake-build-debug/CMakeFiles/UltraTracer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/UltraTracer.dir/depend

