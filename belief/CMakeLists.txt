cmake_minimum_required(VERSION 2.8.3)
project(belief)

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
    mrbsp_msgs
    mrbsp_utils
)
find_package(Boost 1.58 REQUIRED COMPONENTS
    system
    serialization
    graph
)

set(BOOST_LIBRARIES ${BOOST_LIBRARIES} boost_graph boost_serialization)
set(GTSAM_LIBRARIES gtsam tbb tbbmalloc)
set(EIGEN_INCLUDE_DIRS ${PREFIX}/include/gtsam/3rdparty/Eigen)


catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${PREFIX}/include
    ${EIGEN_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

link_directories(${PREFIX}/lib)

add_executable(belief_isam2 src/${PROJECT_NAME}/belief_isam2.cpp)

target_link_libraries(belief_isam2
    ${catkin_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${GTSAM_LIBRARIES}
)

