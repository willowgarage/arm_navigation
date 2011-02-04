cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(ROS_BUILD_TYPE Release)

rosbuild_add_boost_directories()

rosbuild_add_library(collision_space src/environment_objects.cpp
				    src/environment.cpp
				    src/environmentODE.cpp)

rosbuild_link_boost(collision_space thread)

rosbuild_add_gtest(test_collision_space test/test_collision_space.cpp)
target_link_libraries(test_collision_space collision_space)
rosbuild_link_boost(test_collision_space thread)
