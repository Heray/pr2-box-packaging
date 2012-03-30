cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

        #include_directories(${PCL_INCLUDE_DIRS})
        #add_definitions(${PCL_DEFINITIONS})

#common commands for building c++ executables and libraries
rosbuild_add_library(extract_planes src/extract_planes.cpp)
rosbuild_add_library(geometry_utils src/geometry_utils.cpp)
rosbuild_add_library(plane_to_rectangle src/plane_to_rectangle.cpp)
rosbuild_add_library(add_planar_obstacle src/add_planar_obstacle.cpp)
rosbuild_add_library(find_path src/find_path.cpp)

rosbuild_add_executable(close_flaps src/close_flaps.cpp)

target_link_libraries(close_flaps extract_planes)
target_link_libraries(close_flaps geometry_utils)
target_link_libraries(close_flaps plane_to_rectangle)
target_link_libraries(close_flaps add_planar_obstacle)
target_link_libraries(close_flaps find_path)

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#target_link_libraries(example ${PROJECT_NAME})
