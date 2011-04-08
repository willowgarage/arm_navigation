cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_definitions(-msse -msse2)
add_definitions(-DEIGEN_NO_DEBUG)
add_definitions(-DNDEBUG)

rosbuild_add_library(collision_proximity_planner_lib
	src/collision_proximity_planner_plugin.cpp
	src/collision_proximity_planner.cpp
	src/chomp_collision_point.cpp
	src/chomp_robot_model.cpp
	src/treefksolverjointposaxis.cpp
	src/treefksolverjointposaxis_partial.cpp
)	

rosbuild_add_executable(collision_proximity_planner src/main.cpp)
target_link_libraries(collision_proximity_planner collision_proximity_planner_lib)

rosbuild_add_executable(cp_client src/collision_proximity_planner_test.cpp)
target_link_libraries(cp_client collision_proximity_planner_lib)
