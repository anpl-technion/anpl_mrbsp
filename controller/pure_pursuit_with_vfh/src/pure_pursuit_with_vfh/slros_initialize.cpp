#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "Pure_Pursuit_With_VFH";

// For Block Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles/Subscribe
SimulinkSubscriber<std_msgs::Bool, SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool> Sub_Pure_Pursuit_With_VFH_145;

// For Block Pure_Pursuit_With_VFH/Inputs/Subscribe
SimulinkSubscriber<sensor_msgs::LaserScan, SL_Bus_Pure_Pursuit_With_VFH_sensor_msgs_LaserScan> Sub_Pure_Pursuit_With_VFH_54;

// For Block Pure_Pursuit_With_VFH/Inputs/Subscribe1
SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_Pure_Pursuit_With_VFH_nav_msgs_Odometry> Sub_Pure_Pursuit_With_VFH_55;

// For Block Pure_Pursuit_With_VFH/Inputs/Subscribe2
SimulinkSubscriber<geometry_msgs::Point, SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point> Sub_Pure_Pursuit_With_VFH_87;

// For Block Pure_Pursuit_With_VFH/Outputs/Subscribe
SimulinkSubscriber<std_msgs::Bool, SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool> Sub_Pure_Pursuit_With_VFH_119;

// For Block Pure_Pursuit_With_VFH/Outputs/Publish2
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Twist> Pub_Pure_Pursuit_With_VFH_81;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

