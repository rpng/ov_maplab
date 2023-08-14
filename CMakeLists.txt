cmake_minimum_required(VERSION 3.12)

# Project name
project(ov_maplab)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge ov_core ov_msckf vi_map vio_common feature_tracking online_map_builders console_common)
add_definitions(-DROS_AVAILABLE=1)

# Include libraries (if we don't have opencv 4, then fallback to opencv 3)
# The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
endif ()
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
message(STATUS "OPENCV: " ${OpenCV_VERSION} " | BOOST: " ${Boost_VERSION} " | CERES: " ${Ceres_VERSION})

# Describe catkin project
catkin_package(
        CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge ov_core ov_msckf vi_map vio_common feature_tracking online_map_builders console_common
        INCLUDE_DIRS src
)

# We need c++14 for ROS2, thus just require it for everybody
# NOTE: To future self, hope this isn't an issue...
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES}
)

##################################################
# Make the core library
##################################################
add_library(ov_maplab_lib
        src/mapper/MapBuilder.cpp
)
target_link_libraries(ov_maplab_lib ${thirdparty_libraries})


##################################################
# Make binary files!
##################################################

add_executable(ros1_serial_msckf src/ros1_serial_msckf.cpp)
target_link_libraries(ros1_serial_msckf ov_maplab_lib ${thirdparty_libraries})


##################################################
# maplab console plugin
##################################################

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(-fPIC -shared)
cs_add_library(export_to_openvins src/export_to_openvins.cpp)
create_console_plugin(export_to_openvins)
cs_install()
cs_export()

