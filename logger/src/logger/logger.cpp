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
 * @file: logger.cpp
 * @brief:
 * @author: Tal Regev
 */

#include "logger/logger.h"
#include <mrbsp_utils/mrbsp_utils.h>

#include <boost/filesystem.hpp>

#define SCENARIO_STRING "/scenarios/"

using namespace MRBSP;
using namespace MRBSP::Utils;


int main(int argc, char** argv)
{
    FUNCTION_LOGGER(LogTag::logger);

    ros::init(argc, argv, "logger");
    ros::NodeHandle pnh("~");

    Logger logger(pnh);

    ros::spin();
    return 0;
}

Logger::Logger(ros::NodeHandle& privateNodeHandle) :
        m_privateNodeHandle(privateNodeHandle)
{
    m_tag = LogTag::logger;

    FUNCTION_LOGGER(m_tag);

    if (!m_privateNodeHandle.hasParam("/scenario_folder"))
    {
        m_logger_msg << ros::this_node::getName().substr(1) << " node load without parameters.\n exiting node.";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        ros::shutdown();
        return;
    }

    m_privateNodeHandle.getParam("/scenario_folder", m_scenario_prefix);
    m_privateNodeHandle.getParam("researcher_name", m_researcher_name);
    std::replace(m_researcher_name.begin(), m_researcher_name.end(), ' ', '_');

    std::string timestemp = getCurrentTime();
    // create results folder, if not exist
    std::stringstream loggerPath;
    std::string scenario_string = SCENARIO_STRING;
    std::string scenario_name = m_scenario_prefix.substr(m_scenario_prefix.find(scenario_string) + scenario_string.length());

    loggerPath << m_scenario_prefix << Globals::file_separator
                << "results" << Globals::file_separator;
    createFolder(loggerPath.str());

//    loggerPath  << m_researcher_name << Globals::file_separator;
//    createFolder(loggerPath.str());

//    loggerPath  << scenario_name << "_"
    loggerPath  << m_researcher_name << "_"
                << getCurrentTime() << Globals::file_separator;
    createFolder(loggerPath.str());

    privateNodeHandle.setParam("loggerPath", loggerPath.str());

    initLogger(privateNodeHandle);
    loadParameter();



    // create folder for results to analyse with matlab scripts
    std::stringstream matlab_dir;
    matlab_dir << loggerPath.str() << "matlab";
    createFolder(matlab_dir.str());


    // create folder for serialized isam files
    std::stringstream serialized_dir;
    serialized_dir << loggerPath.str() << "serialized_files";
    createFolder(serialized_dir.str());
}

Logger::~Logger() {
    FUNCTION_LOGGER(m_tag);

}

void Logger::loadParameter() {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << ros::this_node::getName().substr(1) << " node parameters:";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "=====================";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    int         log_level;
    int         flush_freq;
    bool        is_file_logger;
    bool        is_console_logger;
    bool        is_ros_console;
    std::string log_tags;

    m_privateNodeHandle.getParam("/scenario_folder", m_scenario_prefix);
    m_privateNodeHandle.getParam("researcher_name", m_researcher_name);
    m_privateNodeHandle.getParam("log_level", log_level);
    m_privateNodeHandle.getParam("is_file_logger", is_file_logger);
    m_privateNodeHandle.getParam("is_console_logger", is_console_logger);
    m_privateNodeHandle.getParam("is_ros_console", is_ros_console);
    m_privateNodeHandle.getParam("log_tags", log_tags);
    m_privateNodeHandle.getParam("logger_flash_frequency", flush_freq);


    m_logger_msg <<  "/scenario_folder: " << m_scenario_prefix;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "researcher_name: " << m_researcher_name;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg <<  "log_level: " << log_level;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "is_file_logger: " << is_file_logger;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "is_console_logger: " << is_console_logger;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg <<  "is_ros_console: " << is_ros_console;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "log_tags: " << log_tags;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "logger_flash_frequency: " << flush_freq;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "=====================";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

}
