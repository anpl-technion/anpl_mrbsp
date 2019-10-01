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
 * @file: config_setup.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include "config_node/config_setup.h"
#include <mrbsp_utils/mrbsp_utils.h>

#include <boost/filesystem.hpp>

using namespace ANPL;

ConfigSetup::ConfigSetup(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    m_config_node(nh)
{
    pnh.getParam("researcher_name", m_researcher_name);
    std::cout << "Researcher name: " << m_researcher_name << std::endl;

    pnh.getParam("/scenario_folder", m_scenario_folder);
    std::cout << "Scenario folder: " << m_scenario_folder << std::endl;

    init();
}

void ConfigSetup::init() {
    //ADD_FIRST_LEVEL_INFO_LOG(LogTag::anpl_utils);

    const char sep = boost::filesystem::path::preferred_separator;
    std::string file_separator = std::string(&sep, 1);

    // create results folder, if not exist
    std::stringstream scenario_results_dir;
    scenario_results_dir << m_scenario_folder << file_separator << "results";
    MRBSP::Utils::createFolder(scenario_results_dir.str());

    // create folder for current run
    m_folder_log << scenario_results_dir.str() << file_separator << m_researcher_name << "_" << MRBSP::Utils::getCurrentTime();
    MRBSP::Utils::createFolder(m_folder_log.str());
    m_config_node.setParam("/config_setup/log_folder", m_folder_log.str());

    // initialize logger file
    std::string folder_log(m_folder_log.str());
    MRBSP::Utils::initLogger(m_config_node);

    m_logger_msg << "Config node logger initialized.";
    //ANPL::anplLogMessage(m_p_config_logger, info, 0, m_logger_msg);

    m_logger_msg << "Researcher: " + m_researcher_name;
    //ANPL::anplLogMessage(m_p_config_logger, info, 0, m_logger_msg);

    std::size_t scenarios_pos = m_scenario_folder.find("/scenarios/");
    std::string scenario = m_scenario_folder.substr(scenarios_pos + 11);
    m_logger_msg << "Scenario: " << scenario;
    //ANPL::anplLogMessage(m_p_config_logger, info, 0, m_logger_msg);


    // create folder for results to analyse with matlab scripts
    std::stringstream matlab_dir;
    matlab_dir << m_folder_log.str() << file_separator << "matlab";
    MRBSP::Utils::createFolder(matlab_dir.str());
    m_logger_msg << "Create matlab folder.";
    //ANPL::anplLogMessage(m_p_config_logger, info, 0, m_logger_msg);

    // create folder for serialized isam files
    std::stringstream serialized_dir;
    serialized_dir << m_folder_log.str() << file_separator << "serialized_files";
    MRBSP::Utils::createFolder(serialized_dir.str());
    m_logger_msg << "Create serialized files folder.";
    //ANPL::anplLogMessage(m_p_config_logger, info, 0, m_logger_msg);
}


std::string ConfigSetup::getFolderLog() {
    return m_folder_log.str();
}

ConfigSetup::~ConfigSetup() {
    m_logger_msg << "Exit config node.";
    //ANPL::anplLogMessage(m_p_config_logger, info, 0, m_logger_msg);
}