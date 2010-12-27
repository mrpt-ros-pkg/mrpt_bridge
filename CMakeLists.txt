cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# We need the MRPT libraries installed in the system
# and anywhere where CMake finds them (/usr, /usr/local, ...)
FIND_PACKAGE(MRPT REQUIRED base obs maps)

rosbuild_add_library(mrpt_bridge  src/point_cloud.cpp src/laser_scan.cpp)
target_link_libraries(mrpt_bridge ${MRPT_LIBS})

