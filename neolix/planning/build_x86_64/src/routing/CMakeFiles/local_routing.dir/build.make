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


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_SOURCE_DIR = /home/caros/planning

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/caros/planning/build_x86_64

# Include any dependencies generated for this target.
include src/routing/CMakeFiles/local_routing.dir/depend.make

# Include the progress variables for this target.
include src/routing/CMakeFiles/local_routing.dir/progress.make

# Include the compile flags for this target's objects.
include src/routing/CMakeFiles/local_routing.dir/flags.make

src/routing/CMakeFiles/local_routing.dir/executables/local_routing.cpp.o: src/routing/CMakeFiles/local_routing.dir/flags.make
src/routing/CMakeFiles/local_routing.dir/executables/local_routing.cpp.o: ../src/routing/executables/local_routing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/caros/planning/build_x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/routing/CMakeFiles/local_routing.dir/executables/local_routing.cpp.o"
	cd /home/caros/planning/build_x86_64/src/routing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/local_routing.dir/executables/local_routing.cpp.o -c /home/caros/planning/src/routing/executables/local_routing.cpp

src/routing/CMakeFiles/local_routing.dir/executables/local_routing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/local_routing.dir/executables/local_routing.cpp.i"
	cd /home/caros/planning/build_x86_64/src/routing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/caros/planning/src/routing/executables/local_routing.cpp > CMakeFiles/local_routing.dir/executables/local_routing.cpp.i

src/routing/CMakeFiles/local_routing.dir/executables/local_routing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/local_routing.dir/executables/local_routing.cpp.s"
	cd /home/caros/planning/build_x86_64/src/routing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/caros/planning/src/routing/executables/local_routing.cpp -o CMakeFiles/local_routing.dir/executables/local_routing.cpp.s

src/routing/CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.o: src/routing/CMakeFiles/local_routing.dir/flags.make
src/routing/CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.o: ../src/routing/executables/sim_routing_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/caros/planning/build_x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/routing/CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.o"
	cd /home/caros/planning/build_x86_64/src/routing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.o -c /home/caros/planning/src/routing/executables/sim_routing_server.cpp

src/routing/CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.i"
	cd /home/caros/planning/build_x86_64/src/routing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/caros/planning/src/routing/executables/sim_routing_server.cpp > CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.i

src/routing/CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.s"
	cd /home/caros/planning/build_x86_64/src/routing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/caros/planning/src/routing/executables/sim_routing_server.cpp -o CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.s

# Object files for target local_routing
local_routing_OBJECTS = \
"CMakeFiles/local_routing.dir/executables/local_routing.cpp.o" \
"CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.o"

# External object files for target local_routing
local_routing_EXTERNAL_OBJECTS =

src/routing/local_routing: src/routing/CMakeFiles/local_routing.dir/executables/local_routing.cpp.o
src/routing/local_routing: src/routing/CMakeFiles/local_routing.dir/executables/sim_routing_server.cpp.o
src/routing/local_routing: src/routing/CMakeFiles/local_routing.dir/build.make
src/routing/local_routing: src/common/libplanning_common.so
src/routing/local_routing: src/routing/CMakeFiles/local_routing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/caros/planning/build_x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable local_routing"
	cd /home/caros/planning/build_x86_64/src/routing && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/local_routing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/routing/CMakeFiles/local_routing.dir/build: src/routing/local_routing

.PHONY : src/routing/CMakeFiles/local_routing.dir/build

src/routing/CMakeFiles/local_routing.dir/clean:
	cd /home/caros/planning/build_x86_64/src/routing && $(CMAKE_COMMAND) -P CMakeFiles/local_routing.dir/cmake_clean.cmake
.PHONY : src/routing/CMakeFiles/local_routing.dir/clean

src/routing/CMakeFiles/local_routing.dir/depend:
	cd /home/caros/planning/build_x86_64 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/caros/planning /home/caros/planning/src/routing /home/caros/planning/build_x86_64 /home/caros/planning/build_x86_64/src/routing /home/caros/planning/build_x86_64/src/routing/CMakeFiles/local_routing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/routing/CMakeFiles/local_routing.dir/depend

