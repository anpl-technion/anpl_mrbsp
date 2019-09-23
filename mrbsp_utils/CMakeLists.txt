#@file: CMakeLists.txt 
#@brife: mrbsp_utils node
#@author: Tal Regev

cmake_minimum_required(VERSION 2.8.3)
project(mrbsp_utils)

# Set the version number for the library
set (NODE_VERSION_MAJOR 0)
set (NODE_VERSION_MINOR 1)
set (NODE_VERSION_PATCH 0)
math (EXPR MRBSP_ROS_UTILS_PASSIVE_VERSION_NUMERIC "10000 * ${NODE_VERSION_MAJOR} + 100 * ${NODE_VERSION_MINOR} + ${NODE_VERSION_PATCH}")
set (NODE_VERSION_STRING "${NODE_VERSION_MAJOR}.${NODE_VERSION_MINOR}.${NODE_VERSION_PATCH}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Wno-deprecated-declarations)
add_compile_options(-Wno-format-security)

set(PREFIX /usr/ANPLprefix/)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PkgConfig 	QUIET)
find_package(catkin	REQUIRED COMPONENTS
	roscpp
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

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES    anpl_utils
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include/
	${PREFIX}/include/
	${EIGEN_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_library(anpl_utils src/${PROJECT_NAME}/mrbsp_utils.cpp src/${PROJECT_NAME}/function_logger.cpp)

install(TARGETS 
  ARCHIVE 	    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY 	    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME 	    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Doxygen documentation - enabling options in subfolder
#if (DOXYGEN_FOUND)
#    add_subdirectory(doc)
#endif()
