cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OMPL REQUIRED ompl)
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_library(ompl_ros_interface
  src/ompl_ros.cpp
  src/ompl_ros_projection_evaluator.cpp
  src/ompl_ros_planner_config.cpp
  src/ompl_ros_planning_group.cpp
  src/ompl_ros_state_validity_checker.cpp
  src/ompl_console.cpp
  src/helpers/ompl_ros_conversions.cpp
  src/ik/ompl_ros_ik_goal_sampleable_region.cpp
  src/ik/ompl_ros_ik_sampler.cpp
  src/state_transformers/ompl_ros_ik_state_transformer.cpp
  src/state_transformers/ompl_ros_rpy_ik_state_transformer.cpp
  src/state_validity_checkers/ompl_ros_joint_state_validity_checker.cpp
  src/state_validity_checkers/ompl_ros_task_space_validity_checker.cpp
  src/planners/ompl_ros_joint_planner.cpp
  src/planners/ompl_ros_task_space_planner.cpp
  src/planners/ompl_ros_rpy_ik_task_space_planner.cpp
)

rosbuild_link_boost(ompl_ros_interface thread)
target_link_libraries(ompl_ros_interface ${OMPL_LIBRARIES})

rosbuild_add_executable(ompl_ros src/main.cpp)
target_link_libraries(ompl_ros ompl_ros_interface)

rosbuild_add_executable(test_ompl_planning test/test_ompl_planning.cpp) 
rosbuild_declare_test(test_ompl_planning)
rosbuild_add_gtest_build_flags(test_ompl_planning)
target_link_libraries(test_ompl_planning planning_environment)
target_link_libraries(test_ompl_planning gtest)
rosbuild_add_rostest(test/test_ompl_planning.launch)