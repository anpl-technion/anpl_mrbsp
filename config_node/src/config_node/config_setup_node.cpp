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
 * @file: belief_laser_node.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include "config_node/config_setup.h"
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "configSetup");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ANPL::ConfigSetup config(nh, pnh);

    ros::spin();
    return 0;
}