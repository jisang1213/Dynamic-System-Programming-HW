# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /home/jakob/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/jakob/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/build

# Include any dependencies generated for this target.
include CMakeFiles/mymat.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/mymat.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/mymat.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mymat.dir/flags.make

CMakeFiles/mymat.dir/src/mymat.cpp.o: CMakeFiles/mymat.dir/flags.make
CMakeFiles/mymat.dir/src/mymat.cpp.o: /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/src/mymat.cpp
CMakeFiles/mymat.dir/src/mymat.cpp.o: CMakeFiles/mymat.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mymat.dir/src/mymat.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mymat.dir/src/mymat.cpp.o -MF CMakeFiles/mymat.dir/src/mymat.cpp.o.d -o CMakeFiles/mymat.dir/src/mymat.cpp.o -c /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/src/mymat.cpp

CMakeFiles/mymat.dir/src/mymat.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mymat.dir/src/mymat.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/src/mymat.cpp > CMakeFiles/mymat.dir/src/mymat.cpp.i

CMakeFiles/mymat.dir/src/mymat.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mymat.dir/src/mymat.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/src/mymat.cpp -o CMakeFiles/mymat.dir/src/mymat.cpp.s

# Object files for target mymat
mymat_OBJECTS = \
"CMakeFiles/mymat.dir/src/mymat.cpp.o"

# External object files for target mymat
mymat_EXTERNAL_OBJECTS =

libmymat.so: CMakeFiles/mymat.dir/src/mymat.cpp.o
libmymat.so: CMakeFiles/mymat.dir/build.make
libmymat.so: CMakeFiles/mymat.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libmymat.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mymat.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mymat.dir/build: libmymat.so
.PHONY : CMakeFiles/mymat.dir/build

CMakeFiles/mymat.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mymat.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mymat.dir/clean

CMakeFiles/mymat.dir/depend:
	cd /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/build /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/build /home/jakob/Desktop/Dynamic-System-Programming-HW/hw2_20190837/mymat/build/CMakeFiles/mymat.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mymat.dir/depend

