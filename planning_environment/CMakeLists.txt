cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(ROS_BUILD_TYPE Release)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
find_package(PkgConfig REQUIRED)
find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
    set(IS_ASSIMP3 1)
    add_definitions(-DIS_ASSIMP3)
    message(STATUS "Found assimp v3")
  else()
    message(STATUS "Found assimp v2")
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
  set(IS_ASSIMP3 0) # most likely not
endif()

rosbuild_add_library(planning_environment src/models/robot_models.cpp
					 src/models/collision_models.cpp
					 src/models/collision_models_interface.cpp
					 src/monitors/kinematic_model_state_monitor.cpp
					 src/monitors/collision_space_monitor.cpp
					 src/monitors/planning_monitor.cpp
					 src/util/kinematic_state_constraint_evaluator.cpp
					 src/util/construct_object.cpp
					 src/util/collision_operations_generator.cpp
					 src/models/model_utils.cpp
					 src/monitors/monitor_utils.cpp
				         src/monitors/joint_state_monitor.cpp)

rosbuild_add_openmp_flags(planning_environment)
rosbuild_link_boost(planning_environment thread)
target_link_libraries(planning_environment yaml-cpp)

rosbuild_add_executable(environment_server src/monitors/environment_server.cpp)
rosbuild_link_boost(environment_server thread)
target_link_libraries(environment_server planning_environment)

rosbuild_add_executable(planning_scene_validity_server src/monitors/planning_scene_validity_server.cpp)
target_link_libraries(planning_scene_validity_server planning_environment)

rosbuild_add_executable(visualize_all_collisions test/visualize_all_collisions.cpp)
rosbuild_link_boost(visualize_all_collisions thread)
target_link_libraries(visualize_all_collisions planning_environment)

rosbuild_add_executable(visualize_collision_models test/visualize_collision_models.cpp)
rosbuild_link_boost(visualize_collision_models thread)
target_link_libraries(visualize_collision_models planning_environment)

rosbuild_add_executable(visualize_planning_scene test/visualize_planning_scene.cpp)
rosbuild_link_boost(visualize_planning_scene thread)
target_link_libraries(visualize_planning_scene planning_environment)
 
# Utility apps
#rosbuild_add_executable(view_state_validity src/tools/view_state_validity.cpp) 
#target_link_libraries(view_state_validity planning_environment)
#rosbuild_link_boost(view_state_validity thread)
#rosbuild_add_openmp_flags(view_state_validity)

rosbuild_add_executable(filter_attached_objects src/tools/filter_attached_objects.cpp) 
target_link_libraries(filter_attached_objects planning_environment)
rosbuild_link_boost(filter_attached_objects thread signals)

#rosbuild_add_executable(collision_operations_generator src/tools/collision_operations_generator.cpp) 


set( sources
	src/tools/planning_description_configuration_wizard.cpp
)

set( headers
        include/planning_environment/tools/planning_description_configuration_wizard.h
)


find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
qt4_wrap_cpp(moc_sources ${headers})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})


rosbuild_add_executable(planning_description_configuration_wizard ${sources} ${headers} ${moc_sources})
target_link_libraries(planning_description_configuration_wizard planning_environment)
rosbuild_link_boost(planning_description_configuration_wizard thread)
target_link_libraries(planning_description_configuration_wizard ${QT_LIBRARIES})
target_link_libraries(planning_description_configuration_wizard tinyxml)
#rosbuild_add_executable(display_planner_collision_model src/tools/display_planner_collision_model.cpp) 
#target_link_libraries(display_planner_collision_model planning_environment)
#rosbuild_link_boost(display_planner_collision_model thread)
#rosbuild_add_openmp_flags(display_planner_collision_model)

# Examples
#rosbuild_add_executable(remove_object_example src/examples/remove_object_example.cpp) 
#target_link_libraries(remove_object_example planning_environment)
#rosbuild_link_boost(remove_object_example thread)
#rosbuild_add_openmp_flags(remove_object_example)

# Tests - these have moved into pr2_arm_navigation_tests

# Create a model of the PR2
#rosbuild_add_executable(allowable_contacts_test test/allowable_contacts_test.cpp)
#target_link_libraries(allowable_contacts_test gtest)
#rosbuild_add_executable(path_constraints_test test/path_constraints_test.cpp)
#target_link_libraries(path_constraints_test gtest)

#rosbuild_add_executable(joint_state_decumulator test/joint_state_decumulator.cpp)
#target_link_libraries(joint_state_decumulator planning_environment)

rosbuild_add_executable(default_joint_state_publisher test/default_joint_state_publisher.cpp)
target_link_libraries(default_joint_state_publisher planning_environment)

rosbuild_add_executable(test_robot_models test/test_robot_models.cpp) 
rosbuild_declare_test(test_robot_models)
rosbuild_add_gtest_build_flags(test_robot_models)
target_link_libraries(test_robot_models planning_environment)

rosbuild_add_executable(test_collision_models test/test_collision_models.cpp) 
rosbuild_declare_test(test_collision_models)
rosbuild_add_gtest_build_flags(test_collision_models)
target_link_libraries(test_collision_models planning_environment)
rosbuild_add_rostest(test/test_models.launch)

rosbuild_add_executable(test_planning_monitor test/test_planning_monitor.cpp) 
rosbuild_declare_test(test_planning_monitor)
rosbuild_add_gtest_build_flags(test_planning_monitor)
target_link_libraries(test_planning_monitor planning_environment)
target_link_libraries(test_planning_monitor gtest)
rosbuild_add_rostest(test/test_monitors.launch)
