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
 * @file: config_setup.h
 * @brief:
 * @author: Asaf Feniger
 */

#ifndef CONFIG_SETUP_H
#define CONFIG_SETUP_H

#include <ros/ros.h>
#include <spdlog/spdlog.h>


namespace ANPL {
    class ConfigSetup {
    public:
        /**
         *
         * @param nh - ros node handle
         * @param pnh - ros private node handle
         */
        ConfigSetup(ros::NodeHandle& nh, ros::NodeHandle& pnh);

        std::string getFolderLog();

        ~ConfigSetup();

    private:
        ros::NodeHandle m_config_node;

        /// researcher name
        std::string m_researcher_name;

        /// path to the scenario folder
        std::string m_scenario_folder;

        /// folder
        std::stringstream m_folder_log;

        /// pointer to logger
        std::shared_ptr<spdlog::logger> m_p_config_logger;

        /// string for logger msgs
        std::stringstream m_logger_msg;

        /**
         *
         */
        void init();

    };
}

#endif // CONFIG_SETUP_H