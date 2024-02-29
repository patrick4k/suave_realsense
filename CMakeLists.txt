cmake_minimum_required(VERSION 3.27)
project(suave_realsense)

set(CMAKE_CXX_STANDARD 17)

# Create executable
add_executable(suave_realsense main.cpp)

# Link librealsense
target_link_libraries(suave_realsense realsense2)

set(CMAKE_BUILD_TYPE Debug)