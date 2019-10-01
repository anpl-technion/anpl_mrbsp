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
 * @file: state_machine_default_node.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include "state_machine_default.h"
#include <ros/ros.h>
#include <signal.h>
#include <mrbsp_utils/gtsam_serialization.h>

void mySigintHandler(int sig) {
    ROS_INFO("Shutting down state machine node...");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ROS_INFO_STREAM("Starting default state machine node...");
    ros::init(argc, argv, "stateMachineNode");

    ANPL::StateMachineDefault state_machine_default;

    signal(SIGINT, mySigintHandler);

    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();

    ros::waitForShutdown();

//  ros::spin();
    return 0;
}