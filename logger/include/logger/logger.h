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
 * @file: logger.h
 * @brief:
 * @author: Tal Regev
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <ros/ros.h>
#include <mrbsp_utils/function_logger.h>


namespace MRBSP {
    class Logger {
    public:
        /**
         *
         * @param pnh - ros private node handle
         */
        Logger(ros::NodeHandle& privateNodeHandle);
        ~Logger();

    private:


        void loadParameter();

        /// private ros node handel (with the node name space)
        ros::NodeHandle m_privateNodeHandle;

        /// string for logger msgs
        std::stringstream m_logger_msg;

        /// senario prefix path
        std::string m_scenario_prefix;

        /// researcher name
        std::string m_researcher_name;

        /// tag of current node
        Utils::LogTag m_tag;

    };
}

#endif // LOGGER_H
