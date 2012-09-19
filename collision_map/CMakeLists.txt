cmake_minimum_required(VERSION 2.4.6)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_add_executable(collision_map_self_occ_node src/collision_map_self_occ.cpp)
rosbuild_add_openmp_flags(collision_map_self_occ_node)
target_link_libraries(collision_map_self_occ_node assimp) 
rosbuild_link_boost(collision_map_self_occ_node signals thread)

rosbuild_add_executable(collision_map_ground_plane_test tests/collision_map_ground_plane_test.cpp)
target_link_libraries(collision_map_ground_plane_test gtest assimp) 
rosbuild_link_boost(collision_map_ground_plane_test signals thread)
