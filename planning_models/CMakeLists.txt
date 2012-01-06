cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(ROS_BUILD_TYPE Release)

rosbuild_add_library(planning_models src/kinematic_model.cpp src/kinematic_state.cpp)

# Unit tests
rosbuild_add_gtest(test_kinematic test/test_kinematic.cpp)
target_link_libraries(test_kinematic planning_models)

rosbuild_add_executable(test_kinematic_complicated test/test_kinematic_complicated.cpp)
target_link_libraries(test_kinematic_complicated planning_models gtest)

