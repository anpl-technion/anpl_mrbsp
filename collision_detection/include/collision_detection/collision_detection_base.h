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
 * @file: collision_detection_base.h
 * @brief: An interface for collision detection
 * @author: Asaf Feniger
 *
 */

#ifndef COLLISSION_DETECTION_BASE_H
#define COLLISSION_DETECTION_BASE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <spdlog/spdlog.h>


namespace MRBSP {

    class CollisionDetectionBase {
    public:
        /**
         * Default constructor
         */
        CollisionDetectionBase();

        /**
         * Copy constructor
         * @param other
         */
        CollisionDetectionBase(const CollisionDetectionBase& other);

        /**
         * Copy assignment operator.
         * @param other
         * @return
         */
        CollisionDetectionBase& operator=(const CollisionDetectionBase& other);

        /**
         * Destructor
         */
        virtual ~CollisionDetectionBase();


    private:


    protected:

        // Logger
        /// pointer to logger
        std::shared_ptr<spdlog::logger> b_m_p_cd_logger;

        /// string for logger msgs
        std::stringstream b_m_logger_msg;

        /// path to current run folder
        std::string b_m_path_to_log_folder;

        // Robot identification
        /// robot name
        std::string b_m_robot_name;

        /// robot id
        char b_m_robot_id;

        /// robot namespace
        std::string b_m_robot_ns;

        // ros node publishers and subscribers
        /// ros node
        ros::NodeHandle b_m_collision_detector_node;

        /// odometry msgs ros subscriber
        ros::Subscriber b_m_odom_sub;

        /// ros publisher for collision warnings
        ros::Publisher b_m_collision_warning_pub;

        /// current odometry ros msg
        nav_msgs::Odometry b_m_current_odom_msg;

        /**
         * @brief Callback function to store current velocity information
         * @param odom
         */
        void odomCallback(const nav_msgs::OdometryConstPtr& odom);
    };

} // namespace ANPL

#endif // COLLISSION_DETECTION_BASE_H
