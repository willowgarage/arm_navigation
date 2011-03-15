cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()

#rosbuild_add_executable(move_arm_monitor src/move_arm_setup.cpp src/move_arm_monitor.cpp)
#rosbuild_link_boost(move_arm_monitor thread)

rosbuild_add_executable(move_arm_simple_action src/move_arm_simple_action.cpp)
rosbuild_link_boost(move_arm_simple_action thread)

rosbuild_add_executable(planning_scene_visualizer_keyboard test/planning_scene_visualizer_keyboard.cpp)
target_link_libraries(planning_scene_visualizer_keyboard planning_environment)

rosbuild_add_executable(test_simple_trajectory_pipeline test/test_simple_trajectory_pipeline.cpp) 
rosbuild_declare_test(test_simple_trajectory_pipeline)
rosbuild_add_gtest_build_flags(test_simple_trajectory_pipeline)
target_link_libraries(test_simple_trajectory_pipeline gtest)
rosbuild_add_rostest(test/test_simple_trajectory_pipeline.launch)
