cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.ompl
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of ompl failed")
endif(_make_failed)

set(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/build/ompl)

option(ROS "Build against ROS" TRUE)

add_subdirectory(build/ompl)
