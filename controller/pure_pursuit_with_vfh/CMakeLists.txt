cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit_with_vfh)
set(CMAKE_VERBOSE_MAKEFILE ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
nav_msgs
roscpp
sensor_msgs
std_msgs
)

add_definitions(
-DMODEL=Pure_Pursuit_With_VFH -DNUMST=1 -DNCSTATES=0 -DHAVESTDIO -DUNIX -DINTEGER_CODE=0 -DMT=0 -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DONESTEPFCN=1 -DTERMFCN=1 -DMULTI_INSTANCE_CODE=1 -DTID01EQ=0 -DROS_PROJECT -DMW_SCHED_OTHER -DSTACK_SIZE=64 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DRT=RT )

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs 
)

###########
## Build ##
###########
set(${PROJECT_NAME}_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME})


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/${PROJECT_NAME}
  #${PROJECT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Get list of .c files in project source directory
file(GLOB ${PROJECT_NAME}_C_SOURCES RELATIVE ${PROJECT_NAME}_SOURCE_DIR ${PROJECT_NAME}_SOURCE_DIR/*.c)

## Declare executable
add_executable(${PROJECT_NAME}_node
   ${${PROJECT_NAME}_SOURCE_DIR}/ert_main.cpp
   ${${PROJECT_NAME}_SOURCE_DIR}/Pure_Pursuit_With_VFH.cpp
   ${${PROJECT_NAME}_SOURCE_DIR}/rtGetInf.cpp
   ${${PROJECT_NAME}_SOURCE_DIR}/rtGetNaN.cpp
   ${${PROJECT_NAME}_SOURCE_DIR}/rt_nonfinite.cpp
   ${${PROJECT_NAME}_SOURCE_DIR}/linuxinitialize.cpp
   ${${PROJECT_NAME}_SOURCE_DIR}/slros_busmsg_conversion.cpp
   ${${PROJECT_NAME}_SOURCE_DIR}/slros_initialize.cpp
   ${${PROJECT_NAME}_SOURCE_DIR}/slros_generic_param.cpp
   ${${PROJECT_NAME}_C_SOURCES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   rt
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES COMPILE_FLAGS " ")
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive" )
SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include stdbool.h" )
