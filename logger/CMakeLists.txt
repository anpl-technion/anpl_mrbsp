#@file: CMakeLists.txt
#@brife: mrbsp_utils node
#@author: Tal Regev

cmake_minimum_required(VERSION 2.8.3)
project(logger)

# Set the version number for the library
set (NODE_VERSION_MAJOR 0)
set (NODE_VERSION_MINOR 1)
set (NODE_VERSION_PATCH 0)
math (EXPR NODE_PASSIVE_VERSION_NUMERIC "10000 * ${NODE_VERSION_MAJOR} + 100 * ${NODE_VERSION_MINOR} + ${NODE_VERSION_PATCH}")
set (NODE_VERSION_STRING "${NODE_VERSION_MAJOR}.${NODE_VERSION_MINOR}.${NODE_VERSION_PATCH}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Wno-deprecated-declarations)

set(PREFIX /usr/ANPLprefix/)

find_package(PkgConfig       QUIET)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	mrbsp_utils
)
find_package(Doxygen QUIET)

set(GTSAM_LIBRARIES gtsam tbb tbbmalloc)

catkin_package(
	INCLUDE_DIRS include
)


include_directories(
	include
	${PREFIX}/include/
	#${PREFIX}/include/gtsam/3rdparty/Eigen/
	/usr/include/eigen3/
	${catkin_INCLUDE_DIRS}
)

link_directories(${PREFIX}/lib)

add_executable(logger src/${PROJECT_NAME}/logger.cpp)

target_link_libraries(logger
	${catkin_LIBRARIES}
	${BOOST_LIBRARIES}
	${GTSAM_LIBRARIES}
)

# Doxygen documentation - enabling options in subfolder
#if (DOXYGEN_FOUND)
#    add_subdirectory(doc)
#endif()

