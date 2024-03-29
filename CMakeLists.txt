cmake_minimum_required(VERSION 3.1)
project(mrpt_bridge)

## Find catkin macros and# libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	geometry_msgs
	marker_msgs
	message_generation
	mrpt_msgs
	nav_msgs
	pcl_conversions
	roscpp
	sensor_msgs
	std_msgs
	stereo_msgs
	tf
)

## System dependencies are found with CMake's conventions
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(MRPT REQUIRED obs graphs maps slam ros1bridge)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")

# Debian Stretch doesn't properly find the recursive Qt5Widgets dependency from
# PCL io.  We force find it here so that cmake is happy.
find_package(Qt5Widgets REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX)
	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
	geometry_msgs
	marker_msgs
	message_runtime
	mrpt_msgs
	nav_msgs
	sensor_msgs
	std_msgs
	stereo_msgs
#  DEPENDS pcl mrpt  # opencv
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare a cpp library
add_library(${PROJECT_NAME}
	# headers:
	include/${PROJECT_NAME}/network_of_poses.h
	include/${PROJECT_NAME}/utils.h
	include/${PROJECT_NAME}/landmark.h
	include/${PROJECT_NAME}/marker_msgs.h
	include/${PROJECT_NAME}/beacon.h
	include/${PROJECT_NAME}/image.h
	# implementations:
	src/beacon.cpp
	src/landmark.cpp
	src/marker_msgs.cpp
	src/network_of_poses.cpp
	src/image.cpp
)

# Declare a cpp executable

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}
   mrpt_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} PUBLIC
   ${catkin_LIBRARIES}
   ${MRPT_LIBRARIES}
)

add_executable(test_Bridge src/test/test_Bridge.cpp)
target_link_libraries(test_Bridge 
  ${catkin_LIBRARIES}
  ${MRPT_LIBRARIES}
  ${PROJECT_NAME}
)
add_dependencies(test_Bridge mrpt_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test
    src/test/test_main.cpp
    src/test/test_time.cpp
    src/test/test_pose.cpp
    src/test/test_pointcloud2.cpp
    src/test/test_map.cpp
  )
  if(TARGET ${PROJECT_NAME}-test)
     target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES} ${MRPT_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
  endif()
endif()
