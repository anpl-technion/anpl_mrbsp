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
 * @file: collision_detection_octomap_node.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include "collision_detection/collision_detection_octomap.h"
#include <ros/ros.h>
#include <signal.h>

void mySigintHandler(int sig) {
    ROS_INFO("Shutting down collision detection node...");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collisionDetectionOctomap");
    signal(SIGINT, mySigintHandler);

    std::cout << "start collision detection.." << std::endl;
    MRBSP::CollisionDetectionOctomap collision_detection_octomap;

    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();

    ros::waitForShutdown();

    //ros::spin();

  return 0;
}
