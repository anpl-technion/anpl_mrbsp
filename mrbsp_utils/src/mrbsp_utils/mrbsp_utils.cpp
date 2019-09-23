/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (MRBSP),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 *
 * See LICENSE for the license information
 * GPLv3
 * -------------------------------------------------------------------------- */

/**
 * @file: mrbsp_utils.cpp
 * @brief: implementation of utils function for ANPL C++ code
 * @author: Tal Regev
 */



#include "mrbsp_utils/mrbsp_utils.h"
#include "mrbsp_utils/gtsam_serialization.h"

#include <exception>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>


#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>


#include <gtsam/slam/dataset.h>
#include <gtsam/inference/Symbol.h>

//from: https://github.com/gabime/spdlog

using namespace MRBSP::Utils;

#define LOG_LEVEL               1
#define IS_FILE_LOGGER          1
#define IS_CONSOLE_LOGGER       1
#define IS_OVER_NETWORK_LOGGER  0
#define IS_ROS_CONSOLE          0

#define FOLDER_LOG              "logs"


class Logger {
public:
    typedef std::shared_ptr<spdlog::logger> Ptr;
};

void logMessage_spdf_h(Logger::Ptr &logger, LogType log_type, std::string message);
void logMessage_ros_h(LogType log_type, std::string message);
bool isTagFitting(LogTag tag);

//from: https://github.com/gabime/spdlog
//from: http://stackoverflow.com/questions/185844/initializing-private-static-members
// Console logger with color
static Logger::Ptr console = spdlog::stdout_color_mt("mrbsp_utils");
static Logger::Ptr file_logger;

std::string                                 Globals::file_separator         = std::string(1, boost::filesystem::path::preferred_separator);
std::stringstream                           Globals::string_stream;
std::string                                 Globals::folder_log             = FOLDER_LOG;
bool                                        Globals::is_file_logger_init    = false;
int                                         Globals::log_level              = LOG_LEVEL;
bool  				                        Globals::is_file_logger         = IS_FILE_LOGGER;
bool 				                        Globals::is_console_logger      = IS_CONSOLE_LOGGER;
bool                                        Globals::is_ros_console         = IS_ROS_CONSOLE;
bool                                        Globals::is_over_network_logger = IS_OVER_NETWORK_LOGGER;
std::unordered_set<LogTag, EnumClassHash>   Globals::log_tags               = {LogTag::all};
std::string                                 Globals::researcher_name;
std::string                                 Globals::scenario_name;
std::string                                 Globals::node_name;

LogTag m_tag = LogTag::utils;


static std::map<std::string, LogTag> strToLogTagMap = {
        {"all",                 LogTag::all},
        {"utils",               LogTag::utils},
        {"logger",              LogTag::logger},
        {"odometry",            LogTag::odometry},
        {"da",                  LogTag::da},
        {"belief",              LogTag::belief},
};


void MRBSP::Utils::initLogger(const ros::NodeHandle &pnh) {
    FUNCTION_LOGGER(m_tag);

    std::string node = ros::this_node::getName().substr(1);

    while(!pnh.hasParam("/logger/loggerPath"))
    {
        // sleep for 1 sec
        ros::Duration(1).sleep();
    }

    std::string loggerPath;
    std::string log_tags;
    int flush_freq;

    pnh.getParam("/logger/loggerPath", loggerPath);
    pnh.getParam("/logger/researcher_name", Globals::researcher_name);
    pnh.param("/logger/log_level", Globals::log_level, 1);
    pnh.param("/logger/is_file_logger", Globals::is_file_logger, true);
    pnh.param("/logger/is_console_logger", Globals::is_console_logger, true);
    pnh.param("/logger/is_ros_console", Globals::is_ros_console, false);
    pnh.param("/logger/log_tags", log_tags, std::string("all"));
    pnh.param("/logger/logger_flash_frequency", flush_freq, 5);

    if(!log_tags.empty()) {
        Globals::log_tags.clear();
        std::set<std::string> tags;

        // taken from: http://stackoverflow.com/questions/5607589/right-way-to-split-an-stdstring-into-a-vectorstring
        boost::split(tags, log_tags, boost::is_any_of(", \t"), boost::token_compress_on);

        for (std::string tag_name : tags) {
            std::map<std::string, LogTag>::const_iterator it = strToLogTagMap.find(tag_name);// check if tag is valid
            if(it == strToLogTagMap.end()) {
                throw std::invalid_argument(std::string("invalid logging tag: ") + tag_name);
            }
            Globals::log_tags.insert(it->second);
        }
    }

    Globals::folder_log = loggerPath;
    std::string node_name = ros::this_node::getName().substr(1);
    Globals::node_name = node_name;
    console = spdlog::stdout_color_mt(node_name);

    loggerPath.append(node_name);
    loggerPath.append(".txt");
    file_logger = spdlog::basic_logger_mt(node_name + "_file", loggerPath);
    spdlog::flush_every(std::chrono::seconds(flush_freq));
    Globals::is_file_logger_init = true;
}

void MRBSP::Utils::logMessage(LogType log_type, int log_level, std::string message, LogTag log_tag) {
    if (LOG_LVL(log_level)) {
        if(isTagFitting(log_tag)) {
            if (Globals::is_console_logger) {
                if(Globals::is_ros_console) {
                    logMessage_ros_h(log_type, message);
                }
                else {
                    logMessage_spdf_h(console, log_type, message);
                }
            }

            if (Globals::is_file_logger && Globals::is_file_logger_init) {
                logMessage_spdf_h(file_logger, log_type, message);
            }
        }
    }
}

void MRBSP::Utils::logMessage(LogType log_type, int log_level, std::stringstream &string_stream, LogTag log_tag) {
    logMessage(log_type, log_level, string_stream.str(), log_tag);
    string_stream.str(std::string());
}

bool MRBSP::Utils::isFolderExist(const std::string &folder_name)
{
    boost::filesystem::path dir(folder_name);
    return boost::filesystem::is_directory(dir);
}

void MRBSP::Utils::createFolder(const std::string &folder_name) {
    FUNCTION_LOGGER(m_tag);

    boost::filesystem::path dir(folder_name);

    if(isFolderExist(folder_name))
    {
        Globals::string_stream << folder_name << " folder exist";
        logMessage(warn, 0, Globals::string_stream, m_tag);
    }
    else
    {
        //from: http://stackoverflow.com/questions/9235679/create-a-directory-if-it-doesnt-exist
        if(!boost::filesystem::create_directory(dir))
        {
            Globals::string_stream << folder_name << " folder fail to create";
            logMessage(error, 0, Globals::string_stream, m_tag);
        }
        else
        {
            Globals::string_stream << "Create " << folder_name << " folder";
            logMessage(info, 0, Globals::string_stream, m_tag);
        }
    }
}

std::string MRBSP::Utils::getCurrentTime() {
    FUNCTION_LOGGER(m_tag);

    //from: http://stackoverflow.com/questions/16357999/current-date-and-time-as-string
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%Y-%m-%d_%I-%M-%S",timeinfo);
    std::string timestemp(buffer);

    return timestemp;
}

void logMessage_spdf_h(Logger::Ptr &logger, LogType log_type, std::string message) {
    switch (log_type) {
        case info:
            logger->info(message);
            break;
        case warn:
            logger->warn(message);
            break;
        case error:
            logger->error(message);
            break;
        case critical:
            logger->critical(message);
            break;
    }
}

void logMessage_ros_h(LogType log_type, std::string message) {
    Globals::string_stream.str(std::string());
    Globals::string_stream << "[" << Globals::node_name << "] " << message;
    std::string log_str = Globals::string_stream.str();
    switch (log_type) {
        case info:
            ROS_INFO(log_str.c_str());
            break;
        case warn:
            ROS_WARN(log_str.c_str());
            break;
        case error:
            ROS_ERROR(log_str.c_str());
            break;
        case critical:
            ROS_FATAL(log_str.c_str());
            break;
    }
    Globals::string_stream.str(std::string());
}

bool isTagFitting(LogTag tag) {
    return (Globals::log_tags.find(LogTag::all) != Globals::log_tags.end()) || (Globals::log_tags.find(tag) != Globals::log_tags.end());
}


void MRBSP::Utils::saveFactorsToFile(const std::string &file_name, const gtsam::ISAM2 &isam,
                                     const gtsam::NonlinearFactorGraph &new_graph, const gtsam::Values &new_values) {
    FUNCTION_LOGGER(m_tag);

    std::fstream file;
    file.open(file_name ,std::fstream::out);

    file << "New graph: \n";
    file << gtsam::serialize(new_graph);

    file << "New values: \n";
    file << gtsam::serialize(new_values);

    gtsam::NonlinearFactorGraph isam_graph  = isam.getFactorsUnsafe();
    gtsam::Values               isam_values = isam.calculateBestEstimate();

    file << "ISAM graph: \n";
    file << gtsam::serialize(isam_graph);

    file << "ISAM values: \n";
    file << gtsam::serialize(isam_values);

    file.close();
    std::string file_name_to_save(file_name);
    file_name_to_save.append(".g2o");
    gtsam::writeG2o(isam_graph, isam_values, file_name);
}

void MRBSP::Utils::savePoseToFile(const std::string &file_name, const int &iteration_number,
                                  const gtsam::Pose3 &robot_location) {
    FUNCTION_LOGGER(m_tag);

    std::fstream file;
    if(iteration_number <= 1) {
        file.open(file_name, std::fstream::out);
    }
    else {
        file.open(file_name, std::fstream::out | std::ofstream::app);
    }
    file << "Iter: " << iteration_number << "\n";
    file << robot_location << "\n";

    file.close();
}

void MRBSP::Utils::saveIncrementalPoseWithCovToFile(const std::string &file_name, const int &iteration_number,
                                                    const gtsam::Pose3 &pose, const gtsam::Matrix &cov) {
    FUNCTION_LOGGER(m_tag);

    std::fstream file;
    if(iteration_number <= 1) {
        file.open(file_name, std::fstream::out);
    }
    else {
        file.open(file_name, std::fstream::out | std::ofstream::app);
    }
    file << iteration_number << " " <<
         pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z() << " " <<
         pose.rotation().matrix()(0,0) << " " << pose.rotation().matrix()(0,1) << " " << pose.rotation().matrix()(0,2) << " " <<
         pose.rotation().matrix()(1,0) << " " << pose.rotation().matrix()(1,1) << " " << pose.rotation().matrix()(1,2) << " " <<
         pose.rotation().matrix()(2,0) << " " << pose.rotation().matrix()(2,1) << " " << pose.rotation().matrix()(2,2) << " " <<
         cov(0,0) << " " << cov(0,1) << " " << cov(0,2) << " " << cov(0,3) << " " << cov(0,4) << " " << cov(0,5) << " " <<
         cov(1,0) << " " << cov(1,1) << " " << cov(1,2) << " " << cov(1,3) << " " << cov(1,4) << " " << cov(1,5) << " " <<
         cov(2,0) << " " << cov(2,1) << " " << cov(2,2) << " " << cov(2,3) << " " << cov(2,4) << " " << cov(2,5) << " " <<
         cov(3,0) << " " << cov(3,1) << " " << cov(3,2) << " " << cov(3,3) << " " << cov(3,4) << " " << cov(3,5) << " " <<
         cov(4,0) << " " << cov(4,1) << " " << cov(4,2) << " " << cov(4,3) << " " << cov(4,4) << " " << cov(4,5) << " " <<
         cov(5,0) << " " << cov(5,1) << " " << cov(5,2) << " " << cov(5,3) << " " << cov(5,4) << " " << cov(5,5) << "\n";

    file.close();
}

void MRBSP::Utils::saveIsamResultsToFile(const std::string &file_name, const gtsam::ISAM2 &isam, const char &robot_id) {
    FUNCTION_LOGGER(m_tag);

    std::fstream file;
    file.open(file_name, std::fstream::out);

    gtsam::Values values = isam.calculateBestEstimate();
    for(unsigned int i = 1; i < isam.size(); i++)
    {
        gtsam::Symbol key = gtsam::Symbol(robot_id, i);
        gtsam::Pose3 pose = values.at<gtsam::Pose3>(key);
        gtsam::Matrix cov = isam.marginalCovariance(key);
        saveIncrementalPoseWithCovToFile(file_name, i, pose, cov);
    }
    file.close();
}
