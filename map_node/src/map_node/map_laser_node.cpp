/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (ANPL),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */

/**
 * @file: map_laser_node.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include "map_node/robot_map_octoamp_laser.h"
#include <ros/ros.h>
#include <signal.h>

#include <mrbsp_utils/gtsam_serialization.h>

void mySigintHandler(int sig) {
    ROS_INFO("Shutting down Map node...");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotMapLaser");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);
    ros::NodeHandle pnh("~");

    ANPL::RobotMapOctomapLaser robot_map_laser(nh, pnh);

    ros::spin();
    return 0;
}