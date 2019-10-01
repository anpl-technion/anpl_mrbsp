#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles/Subscribe
extern SimulinkSubscriber<std_msgs::Bool, SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool> Sub_Pure_Pursuit_With_VFH_145;

// For Block Pure_Pursuit_With_VFH/Inputs/Subscribe
extern SimulinkSubscriber<sensor_msgs::LaserScan, SL_Bus_Pure_Pursuit_With_VFH_sensor_msgs_LaserScan> Sub_Pure_Pursuit_With_VFH_54;

// For Block Pure_Pursuit_With_VFH/Inputs/Subscribe1
extern SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_Pure_Pursuit_With_VFH_nav_msgs_Odometry> Sub_Pure_Pursuit_With_VFH_55;

// For Block Pure_Pursuit_With_VFH/Inputs/Subscribe2
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point> Sub_Pure_Pursuit_With_VFH_87;

// For Block Pure_Pursuit_With_VFH/Outputs/Subscribe
extern SimulinkSubscriber<std_msgs::Bool, SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool> Sub_Pure_Pursuit_With_VFH_119;

// For Block Pure_Pursuit_With_VFH/Outputs/Publish2
extern SimulinkPublisher<geometry_msgs::Twist, SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Twist> Pub_Pure_Pursuit_With_VFH_81;

void slros_node_init(int argc, char** argv);

#endif
