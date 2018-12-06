/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (MRBSP),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */

/**
 * @file: mrbsp_utils.h
 * @brief: implementation of utils function for ANPL C++ code
 * @author: Tal Regev
 */


#ifndef MRBSP_UTILS_H
#define MRBSP_UTILS_H

#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <ostream>
#include <map>
#include <unordered_set>

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "mrbsp_utils/function_logger.h"

namespace MRBSP {

    namespace Utils {

        #define LOG_LVL(a) (a <= Globals::log_level)
        #define LOG_INFO_LVL 1
        #define END_LINE "\n"


        // taken from: http://stackoverflow.com/questions/18837857/cant-use-enum-class-as-unordered-map-key
        struct EnumClassHash    // is needed to use unordered_set with enums
        {
            template<typename T>
            std::size_t operator()(T t) const {
                return static_cast<std::size_t>(t);
            }
        };

        //from: http://stackoverflow.com/questions/19929681/c-global-variable-declaration
        /**
        * Global variables collected in one class"
        */
        class Globals {
        public:
            /// File separator per os
            static std::string file_separator;

            /// For writing log messages
            static std::stringstream string_stream;

            /// logger full path
            static std::string folder_log;

            /// Flag. true if file logger is initialized, false otherwise
            static bool is_file_logger_init;

            /// Flag. true if to save log to file logger, false otherwise
            static bool is_file_logger;

            /// Flag. true if to print log to screen, false otherwise
            static bool is_console_logger;

            static bool is_ros_console;

            ///Flag. true if loggin is over network
            static bool is_over_network_logger;

            /// number represents the amount of log messages
            static int log_level;

            /// holds the tags for logger
            static std::unordered_set<LogTag, EnumClassHash> log_tags;

            /// Scenario name
            static std::string scenario_name;

            /// Researcher Name
            static std::string researcher_name;

            static std::string node_name;
        };

        /**
        * Initialize logger file or console
        * @param nh - node handler, for read ros param (Not private node)
        */
        void initLogger(const ros::NodeHandle &nh);

        /**
        * Log function
        * log to show on screen / write to file
        * @param log_type  - type of log (info, warn, error critical)
        * @param log_level - number represents the amount of log messages
        * @param message   - log message
        * @param is_file   - Flag. true if written to file, false otherwise. default value true
        * @param is_consol - Flag. true if displayed on the screen, false otherwise. default value true
        */
        void logMessage(LogType log_type, int log_level, std::string message, LogTag log_tag);

        /**
        * Log function
        * log to show on screen / write to file
        * @param log_type      - type of log (info, warn, error critical)
        * @param log_level     - number represents the amount of log messages
        * @param string_stream - log message
        * @param is_file       - Flag. true if written to file, false otherwise. default value true
        * @param is_consol     - Flag. true if displayed on the screen, false otherwise. default value true
        */
        void logMessage(LogType log_type, int log_level, std::stringstream &string_stream, LogTag log_tag);


        /**
        * Create folder with given name
        * @param folder_name - name of folder to create
        */
        void createFolder(const std::string &folder_name);

        /**
        * Return the current time
        * @return the current time
        */
        std::string getCurrentTime();


        /**
        * For debugging
        * Write text file with isam data
        * with the new values and new factors that will be added.
        * @param file_name  - text file name to write to
        * @param isam              - isam data
        * @param new_graph         - new factor added to isam
        * @param new_values        - new values added to isam
        */
        void saveFactorsToFile(const std::string &file_name, const gtsam::ISAM2 &isam,
                               const gtsam::NonlinearFactorGraph &new_graph, const gtsam::Values &new_values);


        /**
        *
        * @param file_name
        * @param isam
        * @param robot_id
        */
        void saveIsamResultsToFile(const std::string &file_name, const gtsam::ISAM2 &isam, const char &robot_id);


        /**
        * For Debugging
        * Write text file with pose and iteration number for ground truth of the robot location
        * @param file_name         - text file name to create
        * @param iteration_number  - iteration number of the robot
        * @param robot_location    - location of the robot
        */
        void savePoseToFile(const std::string &file_name, const int &iteration_number,
                            const gtsam::Pose3 &robot_location);

        /**
        *
        * @param file_name
        * @param iteration_number
        * @param pose
        * @param cov
        */
        void saveIncrementalPoseWithCovToFile(const std::string &file_name, const int &iteration_number,
                                              const gtsam::Pose3 &pose, const gtsam::Matrix &cov);

    }
}

#endif // MRBSP_UTILS_H
