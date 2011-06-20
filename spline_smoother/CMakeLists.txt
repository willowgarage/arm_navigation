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
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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

rosbuild_add_library(spline_smoother
	src/clamped_cubic_spline_smoother.cpp
	src/fritsch_butland_spline_smoother.cpp
	src/numerical_differentiation_spline_smoother.cpp
	src/spline_smoothers.cpp
  src/linear_spline_velocity_scaler.cpp
  src/cubic_spline_velocity_scaler.cpp
  src/cubic_trajectory.cpp
  src/linear_trajectory.cpp
  src/lspb_trajectory.cpp
  src/spline_smoother_utils.cpp
  src/cubic_parameterized_trajectory.cpp
  src/cubic_parameterized_spline_velocity_scaler.cpp
)

rosbuild_add_gtest(test/test_spline_smoother
	test/test_clamped_cubic_spline_smoother.cpp
	test/test_splines.cpp
	test/test_utils.cpp
)
target_link_libraries(test/test_spline_smoother spline_smoother)

rosbuild_add_executable(test/test_smoothers_as_filters
	test/test_smoothers_as_filters.cpp
)
rosbuild_declare_test(test/test_smoothers_as_filters)
rosbuild_add_gtest_build_flags(test/test_smoothers_as_filters)
rosbuild_add_rostest(test/test_smoothers_as_filters.launch)
target_link_libraries(test/test_smoothers_as_filters spline_smoother)

rosbuild_add_executable(test/test_linear_spline_velocity_scaler
	test/test_linear_spline_velocity_scaler.cpp
)
rosbuild_declare_test(test/test_linear_spline_velocity_scaler)
rosbuild_add_gtest_build_flags(test/test_linear_spline_velocity_scaler)
rosbuild_add_rostest(test/test_linear_spline_velocity_scaler.launch)
target_link_libraries(test/test_linear_spline_velocity_scaler spline_smoother)

rosbuild_add_gtest(test/regress_linear test/regress_linear.cpp)
target_link_libraries(test/regress_linear spline_smoother)
rosbuild_add_gtest(test/regress_cubic test/regress_cubic.cpp)
target_link_libraries(test/regress_cubic spline_smoother)
rosbuild_add_gtest(test/regress_lspb test/regress_lspb.cpp)
target_link_libraries(test/regress_lspb spline_smoother)


#rosbuild_add_gtest(test/regress_cubic_parameterized test/regress_cubic_parameterized.cpp)
#target_link_libraries(test/regress_cubic_parameterized spline_smoother)

rosbuild_add_executable(test/test_cubic_parameterized test/test_cubic_parameterized.cpp)
target_link_libraries(test/test_cubic_parameterized spline_smoother)
