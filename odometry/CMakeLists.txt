#@file: CMakeLists.txt 
#@brife: odometry node
#@author: Tal Regev

cmake_minimum_required(VERSION 2.8.3)
project(odometry)

# Set the version number for the library
set (NODE_VERSION_MAJOR 0)
set (NODE_VERSION_MINOR 1)
set (NODE_VERSION_PATCH 0)
math (EXPR NODE_VERSION_NUMERIC "10000 * ${NODE_VERSION_MAJOR} + 100 * ${NODE_VERSION_MINOR} + ${NODE_VERSION_PATCH}")
set (NODE_VERSION_STRING "${NODE_VERSION_MAJOR}.${NODE_VERSION_MINOR}.${NODE_VERSION_PATCH}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Wno-deprecated-declarations)

set(PREFIX /usr/ANPLprefix/)

find_package(PkgConfig       QUIET)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    mrbsp_utils
    mrbsp_msgs
    rosbag
    tf
)
find_package(Boost 1.58 REQUIRED COMPONENTS
    system
    serialization
    graph
)
find_package(Doxygen QUIET)

set(BOOST_LIBRARIES ${BOOST_LIBRARIES} boost_graph boost_serialization)
set(GTSAM_LIBRARIES gtsam tbb tbbmalloc)
set(EIGEN_INCLUDE_DIRS ${PREFIX}/include/gtsam/3rdparty/Eigen)
set(ANPL_LIBRARIES planar_icp)


catkin_package(
	INCLUDE_DIRS include
)

include_directories(
        include
        ${PREFIX}/include
        ${PREFIX}/include/planar_icp
        ${EIGEN_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

link_directories(${PREFIX}/lib)

add_executable(odometry_external src/${PROJECT_NAME}/odometry_external.cpp)

add_executable(odometry_icp_laser src/${PROJECT_NAME}/odometry_icp_laser.cpp)

target_link_libraries(odometry_external
        ${catkin_LIBRARIES}
        ${BOOST_LIBRARIES}
        ${GTSAM_LIBRARIES}	
)

target_link_libraries(odometry_icp_laser
        ${catkin_LIBRARIES}
        ${BOOST_LIBRARIES}
        ${GTSAM_LIBRARIES}
        ${ANPL_LIBRARIES}	
)

# Doxygen documentation - enabling options in subfolder
if (DOXYGEN_FOUND)
    add_subdirectory(doc)
endif()

