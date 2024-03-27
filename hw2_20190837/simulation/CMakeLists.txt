cmake_minimum_required (VERSION 3.16)
project(homework2)
set(CMAKE_CXX_STANDARD 17)

add_executable(simulate src/simulate.cpp)

# Find Python
find_package(Python3 COMPONENTS Development NumPy)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

# The following imports the shared library with:

# 1. path to the directory with the shared library file (.so)
target_link_directories(simulate PRIVATE ../mymat/build)

# 2. library to link
target_link_libraries(simulate PRIVATE mymat)

target_link_libraries(simulate PRIVATE Python3::Python)

# 3. path to the directory with the library header file (.hpp)
target_include_directories(simulate PRIVATE ../mymat/include)
target_include_directories(simulate PRIVATE include)