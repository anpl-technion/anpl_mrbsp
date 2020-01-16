cmake_minimum_required(VERSION 2.8.3)
project(collision_detection)

# Set the version number for the library
set (COLLISION_DETECTION_VERSION_MAJOR 3)
set (COLLISION_DETECTION_VERSION_MINOR 0)
set (COLLISION_DETECTION_VERSION_PATCH 0)
math (EXPR COLLISION_DETECTION_VERSION_NUMERIC "10000 * ${COLLISION_DETECTION_VERSION_MAJOR} + 100 * ${COLLISION_DETECTION_VERSION_MINOR} + ${COLLISION_DETECTION_VERSION_PATCH}")
set (COLLISION_DETECTION_VERSION_STRING "${COLLISION_DETECTION_VERSION_MAJOR}.${COLLISION_DETECTION_VERSION_MINOR}.${COLLISION_DETECTION_VERSION_PATCH}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

##
#set(CMAKE_BUILD_TYPE Debug)

if(${ARCHITECTURE} MATCHES ${odroid} )
    set(PREFIX $ENV{HOME}/prefix)
else()
    set(PREFIX /usr/ANPLprefix/)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  laser_geometry
  octomap_msgs
  octomap_ros
  mrbsp_utils
)

## System dependencies are found with CMake's conventions
#set(BOOST_ROOT /usr/ANPL/boost_prefix)
#set(Boost_INCLUDE_DIR /usr/ANPL/boost_prefix/include)


find_package(Boost 1.54      REQUIRED COMPONENTS system serialization graph)
find_package(PkgConfig 	     QUIET)
set( ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${PREFIX}/lib/pkgconfig" )
find_package(OCTOMAP 1.8.0 REQUIRED)

find_package(FCL QUIET)
if(NOT FCL_FOUND)
    pkg_check_modules (FCL REQUIRED fcl)
    message("${FCL_LIBRARIES} library found in ${FCL_LIBDIR}. FCL headers in ${FCL_INCLUDEDIR}.")
    set(FCL_INCLUDE_DIRS ${FCL_INCLUDEDIR})
endif()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES collision_detection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(BOOST_LIBRARIES ${BOOST_LIBRARIES} boost_graph boost_serialization)
set(GTSAM_LIBRARIES gtsam tbb tbbmalloc)
set(ANPL_LIBRARIES planar_icp)
set(EIGEN_INCLUDE_DIRS "/usr/ANPLprefix/include/gtsam/3rdparty/Eigen")
#set(FCL_INCLUDE_DIRS "/usr/ANPLprefix/include/fcl")
#set(FCL_LIBRARIES fcl)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ./include/
        ${FCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${PREFIX}/include
        ${PREFIX}/include/planar_icp
        ${PREFIX}/include/gtsam/3rdparty/Eigen
        ${catkin_INCLUDE_DIRS})

link_directories(${PREFIX}/lib ${FCL_LIBDIR})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/collision_detection_node.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(collosion_detection_octomap
	src/${PROJECT_NAME}/collosion_detection_octomap_node.cpp
	src/${PROJECT_NAME}/collision_detection_octomap.cpp
	src/${PROJECT_NAME}/collision_detection_base.cpp
 )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(collosion_detection_octomap
   ${catkin_LIBRARIES}
   ${GTSAM_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${FCL_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_collision_detection_node.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
