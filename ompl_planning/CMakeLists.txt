cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(ROS_BUILD_TYPE Debug)

rosbuild_add_boost_directories()

rosbuild_add_library(ompl_planning_helpers 
  src/helpers/ompl_planner/PlannerSetup.cpp
  src/helpers/ompl_planner/kinematicESTSetup.cpp
  src/helpers/ompl_planner/kinematicIKSBLSetup.cpp
  src/helpers/ompl_planner/kinematicLazyRRTSetup.cpp
  src/helpers/ompl_planner/kinematicRRTSetup.cpp
  src/helpers/ompl_planner/kinematicRRTConnectSetup.cpp
  src/helpers/ompl_planner/kinematicpRRTSetup.cpp
  src/helpers/ompl_planner/kinematicKPIECESetup.cpp
  src/helpers/ompl_planner/kinematicLBKPIECESetup.cpp
  src/helpers/ompl_planner/kinematicSBLSetup.cpp
  src/helpers/ompl_planner/kinematicpSBLSetup.cpp
  src/helpers/ompl_planner/dynamicRRTSetup.cpp
  src/helpers/ompl_planner/dynamicKPIECESetup.cpp
  src/helpers/PlannerConfig.cpp
  src/helpers/Model.cpp
  )
rosbuild_link_boost(ompl_planning_helpers thread)

rosbuild_add_executable(motion_planner src/motion_planner.cpp src/request_handler/RequestHandler.cpp src/debug/DisplayPlanner.cpp)
rosbuild_link_boost(motion_planner thread)
target_link_libraries(motion_planner ompl_planning_helpers)

rosbuild_add_executable(test_ompl_planning test/test_ompl_planning.cpp) 
rosbuild_declare_test(test_ompl_planning)
rosbuild_add_gtest_build_flags(test_ompl_planning)
target_link_libraries(test_ompl_planning planning_environment)
target_link_libraries(test_ompl_planning gtest)
rosbuild_add_rostest(test/test_ompl_planning.launch)