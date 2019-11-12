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
 * @file: belief_isam2.cpp
 * @brief:
 * @author: Tal Regev
 */

#include "belief/belief_isam2.h"

#include <fstream>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <mrbsp_msgs/GtsamSerValues.h>
#include <mrbsp_msgs/GtsamSerPose3.h>
#include <mrbsp_utils/mrbsp_utils.h>
#include <mrbsp_utils/gtsam_serialization.h>

#include <gtsam/base/serialization.h>

#include <signal.h>

using namespace MRBSP;
using namespace MRBSP::Utils;

void mySigintHandler(int sig) {
    Utils::LogTag tag = LogTag::belief;
    FUNCTION_LOGGER(tag);

    std::stringstream logger_msg;


    logger_msg << "Shutting down " << ros::this_node::getName().substr(1) << " node";
    logMessage(info, LOG_INFO_LVL, logger_msg, tag);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    FUNCTION_LOGGER(LogTag::belief);

    ros::init(argc, argv, "BeliefIsam2");
    signal(SIGINT, mySigintHandler);
    ros::NodeHandle pnh("~");

    BeliefIsam2 belief_isam2(pnh);

    ros::spin();
    return 0;
}

BeliefIsam2::BeliefIsam2(const ros::NodeHandle &nh_private) :
        m_is_rosbag(false),
        m_privateNodeHandle(nh_private)
{
    m_tag = LogTag::belief;
    FUNCTION_LOGGER(m_tag);

    if (!m_privateNodeHandle.hasParam("/scenario_folder"))
    {
        m_logger_msg << m_node_name << " node load without parameters.\n exiting node.";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        ros::shutdown();
        return;
    }
    m_privateNodeHandle.getParam("/logger/loggerPath", m_logger_path);
    initLogger(m_privateNodeHandle);
    loadParameter();

    //m_central_prefix = "/Central/";
    m_central_prefix = "/";

    
    m_factors_and_values_sub    = m_privateNodeHandle.subscribe(m_central_prefix + "belief_input", 100, &BeliefIsam2::factorAndValuesCallback, this);
    m_memory_update_pub         = m_privateNodeHandle.advertise<mrbsp_msgs::GtsamSerValues>(m_central_prefix + "Belief/memory_update", 100);

    m_get_belief_service = m_privateNodeHandle.advertiseService(m_central_prefix + "request_belief", &BeliefIsam2::getBeliefCallback, this);

    m_logger_msg << "Belief: Belief node initialized";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}

void BeliefIsam2::loadParameter()
{
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << m_node_name << " node parameters:";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "=====================";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_privateNodeHandle.param("is_publish_optimize_pose", m_is_publish_optimize_pose, true);
    m_privateNodeHandle.param("is_save_isam_files", m_is_save_isam_files, true);



    m_logger_msg << "is_publish_optimize_pose: " << m_is_publish_optimize_pose;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "is_save_isam_files: " << m_is_save_isam_files;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "=====================";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

}

gtsam::Values BeliefIsam2::getFactorsAndValues(std::vector<NewFactorsAndValues>& all_factors_and_values, std::string path_to_log_folder) {
    FUNCTION_LOGGER(m_tag);

    m_is_rosbag = true;
    m_isam.clear();
    int i = 0;
    for(auto iter = all_factors_and_values.begin(); iter != all_factors_and_values.end(); ++iter) {
        handleNewFactorsAndValues(*iter);
        ++i;
    }

    std::string file_name(path_to_log_folder + "/matlab/rosbag_final_graph");
    saveIsamResultsToFile(file_name, m_isam);

    return m_isam.calculateBestEstimate();
}

gtsam::Values BeliefIsam2::getLastFactorsAndValues(NewFactorsAndValues& new_factors_and_values) {
    FUNCTION_LOGGER(m_tag);

    m_is_rosbag = true;
    handleNewFactorsAndValues(new_factors_and_values);
    return m_isam.calculateBestEstimate();
}

void BeliefIsam2::factorAndValuesCallback(const mrbsp_msgs::GtsamFactorGraphConstPtr& graph_input_msg) {
    FUNCTION_LOGGER(m_tag);

    gtsam::NonlinearFactorGraph graph;
    gtsam::deserialize(graph_input_msg->ser_factors, graph);
    gtsam::Values vals;
    gtsam::deserialize(graph_input_msg->ser_values, vals);

    NewFactorsAndValues new_factors_and_values = std::make_pair(graph, vals);
    handleNewFactorsAndValues(new_factors_and_values);

    // update memory in da node
    //TODO: find string msg with header
    gtsam::Values optimize_vals(m_isam.calculateBestEstimate());
    mrbsp_msgs::GtsamSerValues vals_ser_msg;
    vals_ser_msg.header.seq = 0; // TODO: add counter
    vals_ser_msg.header.frame_id = "world";
    vals_ser_msg.header.stamp = ros::Time::now();
    vals_ser_msg.ser_values = gtsam::serialize(optimize_vals);

    m_memory_update_pub.publish(vals_ser_msg);
}

void BeliefIsam2::handleNewFactorsAndValues(NewFactorsAndValues& new_factors_and_values) {
    FUNCTION_LOGGER(m_tag);

    gtsam::NonlinearFactorGraph new_graph(new_factors_and_values.first);
    gtsam::Values new_values(new_factors_and_values.second);

    gtsam::KeyList new_keys = new_values.keys();
    for(auto key : new_keys) {
        gtsam::Symbol symbol(key);
        char robot_id = symbol.chr();
        if(robot_id != 'l') {
            m_logger_msg << "Belief: New value from robot: " << robot_id << ", index: " << symbol.index();
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

            if(m_last_values_map.empty() || m_last_values_map.find(symbol.chr()) == m_last_values_map.end()) {
                m_last_values_map.emplace(symbol.chr(), symbol.index());
            }
            else if(symbol.index() > m_last_values_map.find(symbol.chr())->second) {
                m_last_values_map.at(symbol.chr()) = static_cast<unsigned int>(symbol.index());
            }
        }
    }

    for(auto factor : new_graph) {
        std::vector<char> robot_ids;
        std::vector<size_t> factor_indexes;
        for(auto factors_key : factor.get()->keys()) {
            gtsam::Symbol symbol(factors_key);
            // detect multi robot factors
            if(!(std::find(robot_ids.begin(), robot_ids.end(), symbol.chr()) != robot_ids.end())) {
                robot_ids.push_back(symbol.chr());
            }
            factor_indexes.push_back(symbol.index());
        }

        // debug information
        // assume only priors and between factors
        if(factor_indexes.size() > 1) {
            // not a prior factor

            if((robot_ids.size() == 1) && (fabs(factor_indexes.at(1) - factor_indexes.at(0)) > 1)) {
                // loop closure factor
                m_logger_msg << "Belief: Loop closure factor added for robot " << robot_ids.at(0)
                             << ", between index " << factor_indexes.at(0) << " and index " << factor_indexes.at(1);
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
            }
            else if(robot_ids.size() > 1) {
                // multi robot factor
                m_logger_msg << "Belief: Multi robot factor added between "
                             << "robot " << robot_ids.at(0) << "index " << factor_indexes.at(0)
                             << ", and robot " << robot_ids.at(1) << " index " << factor_indexes.at(1);
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
            }
        }
        else {
            // add new prior
            m_logger_msg << "Belief: Add new prior factor for robot " << robot_ids.at(0);
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        }
    }

    m_isam.update(new_graph, new_values);

    if (m_is_save_isam_files) {
        if (new_values.size() == 1) {
            gtsam::Symbol sym = *new_values.keys().begin();
            std::string matlab_filename(m_logger_path + "/matlab/belief_" +
                                        std::string(1, sym.chr()) + std::to_string(sym.index()));
            saveIsamResultsToFile(matlab_filename, m_isam);

            std::string ser_filename(m_logger_path + "/serialized_files/belief_" +
                                     std::string(1, sym.chr()) + std::to_string(sym.index()));
            serializeResultsToFile(ser_filename, m_isam);
        }
    }


    if(!m_is_rosbag && m_is_publish_optimize_pose) {
        // publish optimized poses to the robots
        for (auto last_index_iter = m_last_values_map.begin();
             last_index_iter != m_last_values_map.end(); ++last_index_iter) {
            std::cout << "Belief update: robot " << last_index_iter->first << " index " << last_index_iter->second
                      << std::endl;
            mrbsp_msgs::GtsamSerPose3 pose3_ser_msg;
            pose3_ser_msg.header.seq = 0; // TODO: add counter
            pose3_ser_msg.header.frame_id = std::string(std::string(1, last_index_iter->first) + std::to_string(last_index_iter->second));
            pose3_ser_msg.header.stamp = ros::Time::now();
            gtsam::Pose3 opt_pose = m_isam.calculateBestEstimate().at<gtsam::Pose3>
                    (gtsam::symbol(last_index_iter->first, last_index_iter->second));
            pose3_ser_msg.ser_pose3 = gtsam::serialize(opt_pose);
            // check if the robot publisher already exist. If not initialize it.
            char current_robot_id = last_index_iter->first;
            if(m_optimized_pose_pub.find(current_robot_id) == m_optimized_pose_pub.end()) {
                std::string optimize_pose_topic("/Robot_" + std::string(1, current_robot_id) + "/optimized_pose");
                ros::Publisher optimized_pose_pub(m_privateNodeHandle.advertise<mrbsp_msgs::GtsamSerPose3>(optimize_pose_topic, 100));
                m_optimized_pose_pub.emplace(current_robot_id, optimized_pose_pub);
            }
            m_optimized_pose_pub.at(current_robot_id).publish(pose3_ser_msg);
        }
    }
}

void BeliefIsam2::savePoseWithCovToFile(const std::string& file_name, const gtsam::ISAM2& isam) {
    FUNCTION_LOGGER(m_tag);

    std::fstream file;
    file.open(file_name, std::fstream::out);

    gtsam::Values vals = isam.calculateBestEstimate();
    gtsam::KeyList values_key_list(vals.keys());
    for(auto value_key : values_key_list)
    {
        gtsam::Symbol current_key = gtsam::Symbol(value_key);
        gtsam::Pose3 pose = vals.at<gtsam::Pose3>(current_key);
        gtsam::Matrix cov = isam.marginalCovariance(current_key);

        file << current_key.chr() << " " << current_key.index() << " " <<
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
    }

    file.close();
}

void BeliefIsam2::saveFactorsListToFile(const std::string& file_name, const gtsam::NonlinearFactorGraph& graph) {
    FUNCTION_LOGGER(m_tag);

    std::fstream file;
    file.open(file_name, std::fstream::out);

    for(auto factor : graph) {
        std::vector<char> robot_ids;
        std::vector<size_t> factor_indexes;
        for (auto factors_key : factor.get()->keys()) {
            gtsam::Symbol symbol(factors_key);
            // detect multi robot factors
            if (!(std::find(robot_ids.begin(), robot_ids.end(), symbol.chr()) != robot_ids.end())) {
                robot_ids.push_back(symbol.chr());
            }
            factor_indexes.push_back(symbol.index());
        }

        if(factor_indexes.size() == 1) {
            file << "Prior" << " " <<
                 robot_ids.at(0) << " " << factor_indexes.at(0) << " " << robot_ids.at(0) << " " << factor_indexes.at(0) << "\n";
        }
        else if(factor_indexes.size() > 1 && robot_ids.size() == 1) {
            if(fabs(factor_indexes.at(1) - factor_indexes.at(0)) == 1) {
                file << "Between" << " " <<
                     robot_ids.at(0) << " " << factor_indexes.at(0) << " " << robot_ids.at(0) << " " << factor_indexes.at(1) << "\n";
            }
            else {
                file << "Between_lc" << " " <<
                     robot_ids.at(0) << " " << factor_indexes.at(0) << " " << robot_ids.at(0) << " " << factor_indexes.at(1) << "\n";
            }
        }
        else {
            file << "Between_mr" << " " <<
                 robot_ids.at(0) << " " << factor_indexes.at(0) << " " << robot_ids.at(1) << " " << factor_indexes.at(1) << "\n";
        }
    }

    file.close();
}

void BeliefIsam2::saveIsamResultsToFile(const std::string& file_name, const gtsam::ISAM2& isam) {
    FUNCTION_LOGGER(m_tag);

    std::string values_file_name(file_name + "_values.txt");
    savePoseWithCovToFile(values_file_name, isam);

    std::string factors_file_name(file_name + "_factors.txt");
    gtsam::NonlinearFactorGraph graph(isam.getFactorsUnsafe());
    saveFactorsListToFile(factors_file_name, graph);
}

void BeliefIsam2::serializeResultsToFile(const std::string& file_name, const gtsam::ISAM2& isam) {
    FUNCTION_LOGGER(m_tag);

    std::string isam_ser_filename(file_name + "_ser_isam.txt");
    gtsam::serializeToFile(isam, isam_ser_filename);

    std::string values_ser_filename(file_name + "_ser_values.txt");
    gtsam::serializeToFile(isam.calculateBestEstimate(), values_ser_filename);

    std::string factors_ser_filename(file_name + "_ser_factors.txt");
    gtsam::serializeToFile(isam.getFactorsUnsafe(), factors_ser_filename);
}

bool BeliefIsam2::getBeliefCallback(mrbsp_msgs::RequestBelief::Request& req, mrbsp_msgs::RequestBelief::Response& res) {
    FUNCTION_LOGGER(m_tag);

    gtsam::NonlinearFactorGraph sm_graph(m_isam.getFactorsUnsafe());
    gtsam::Values sm_vals(m_isam.calculateEstimate());

    std::string graph_serialized;
    std::string values_serialized;
    try {
        graph_serialized = gtsam::serialize(sm_graph);
        values_serialized = gtsam::serialize(sm_vals);
    }
    catch (...) {
        m_logger_msg << "Belief: Unable to serialize belief, send empty string";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    //graph_serialized = "graph_serialized";
    res.graph_string = graph_serialized;
    res.values_string = values_serialized;

    m_logger_msg << "Belief: Send factor graph with "  << sm_graph.size() << " factors and " << sm_vals.size() << " values to state machine.";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    m_logger_msg << "Belief: graph string size "  << res.graph_string.size() << " Values string size " << res.values_string.size() << " values to state machine.";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}

BeliefIsam2::~BeliefIsam2() {
    FUNCTION_LOGGER(m_tag);
}
