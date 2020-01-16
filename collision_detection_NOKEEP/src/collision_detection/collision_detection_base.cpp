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
 * @file: collision_detection_base.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include "collision_detection/collision_detection_base.h"

#include <mrbsp_utils/conversion.h>

#include <std_msgs/Bool.h>

using namespace MRBSP;

CollisionDetectionBase::CollisionDetectionBase()
{
    ros::NodeHandle nh_private("~");

    nh_private.param("robot_name", b_m_robot_name, std::string("Robot_A"));
    std::cout << "Robot name: " << b_m_robot_name << std::endl;
    std::string robot_id;
    nh_private.param("robot_id", robot_id, std::string("A"));
    std::cout << "Robot ID: " << robot_id << std::endl;
    b_m_robot_id = robot_id.at(0);
    b_m_robot_ns = std::string("/Robot_" + robot_id);

    std::string odom_topic;
    nh_private.param("odometry/topic", odom_topic, std::string("odom"));
    if(!b_m_robot_name.empty()) {
        odom_topic = std::string("/" + b_m_robot_name + odom_topic);
    }
    std::cout << "Odometry topic: "  << odom_topic << std::endl;
    b_m_odom_sub = b_m_collision_detector_node.subscribe(std::string(odom_topic),
                                                         1, &CollisionDetectionBase::odomCallback, this);

    std::string robot_ns("Robot_" + robot_id);
    std::string collision_warning_topic(b_m_robot_ns + "/Collision_warning");
    b_m_collision_warning_pub = b_m_collision_detector_node.advertise<std_msgs::Bool>(collision_warning_topic, 1);

    std::string collision_with_other_warning_topic(b_m_robot_ns + "/Stop_for_other");
    b_m_emergency_stop_pub = b_m_collision_detector_node.advertise<std_msgs::Bool>(collision_with_other_warning_topic, 1);

    ROS_INFO_STREAM("Collision detector base initialized");
}

CollisionDetectionBase::CollisionDetectionBase(const CollisionDetectionBase& other):
        CollisionDetectionBase()
{

}

CollisionDetectionBase& CollisionDetectionBase::operator=(const CollisionDetectionBase& other) {
    CollisionDetectionBase();
}

void CollisionDetectionBase::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
    b_m_current_odom_msg = *odom;
}

CollisionDetectionBase::~CollisionDetectionBase() {

}
