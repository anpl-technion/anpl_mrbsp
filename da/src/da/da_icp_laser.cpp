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
 * @file: da_icp_laser.cpp
 * @brief:
 * @author: Tal Regev
 */


#include "da/da_icp_laser.h"

#include <fstream>

#include <planar_icp/planarICP.h>

#include <mrbsp_msgs/GtsamFactorGraph.h>
#include <mrbsp_msgs/MapData.h>
#include <mrbsp_msgs/MapData3D.h>
#include <mrbsp_utils/mrbsp_utils.h>
#include <mrbsp_utils/gtsam_serialization.h>

#include <gtsam/base/serialization.h>

#include <signal.h>

// defined duo to deserialization round in matlab
#define MIN_ROTATION_NOISE_SIGMA 1E-6
#define MIN_TRANSLATION_NOISE_SIGMA 1E-3

using namespace MRBSP;
using namespace MRBSP::Utils;

void mySigintHandler(int sig) {
    Utils::LogTag tag = LogTag::da;
    FUNCTION_LOGGER(tag);

    std::stringstream logger_msg;

    logger_msg << "Shutting down " << ros::this_node::getName().substr(1) << " node";
    logMessage(info, LOG_INFO_LVL, logger_msg, tag);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    FUNCTION_LOGGER(LogTag::da);

    ros::init(argc, argv, "DaIcpLaser");
    signal(SIGINT, mySigintHandler);
    ros::NodeHandle pnh("~");

    DaIcpLaser da_laser(pnh);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    /*
     * 1) Sliding window thread
     * 2) Loop closure thread
     * 3) Multi-Robot thread
     * 4) Memory update thread
     */
    spinner.start();
    ros::waitForShutdown();

    return 0;
}


DaIcpLaser::DaIcpLaser(const ros::NodeHandle &nh_private) :
    m_privateNodeHandle(nh_private),
    m_is_init(false)
{
    m_tag = LogTag::odometry;
    FUNCTION_LOGGER(m_tag);

    m_node_name = ros::this_node::getName().substr(1);

    if (!m_privateNodeHandle.hasParam("/scenario_folder"))
    {
        m_logger_msg << m_node_name << " node load without parameters.\n exiting node.";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        ros::shutdown();
        return;
    }

    initLogger(m_privateNodeHandle);
    loadParameter();

    m_central_prefix = "/Central/";
    m_robots_prefix  = "/Robots/";
    
    // keyframes with pointclouds
    m_keyframe_init_rgbd_sub = m_privateNodeHandle.subscribe(m_robots_prefix + "Odometry/keyframe/withPointcloud", 1, &DaIcpLaser::KeyframeRgbdCallback, this);
    m_map_data_3D_pub = m_privateNodeHandle.advertise<mrbsp_msgs::MapData3D>(m_central_prefix + "DA/map_data/3D", 1);

    // keyframes without pointclouds
    m_keyframe_init_sub = m_privateNodeHandle.subscribe(m_robots_prefix + "Odometry/keyframe", 1, &DaIcpLaser::KeyframeCallback, this);
    m_map_data_2D_pub = m_privateNodeHandle.advertise<mrbsp_msgs::MapData>(m_central_prefix + "DA/map_data/2D", 1);

    m_updated_values_sub = m_privateNodeHandle.subscribe(m_central_prefix + "Belief/memory_update", 1, &DaIcpLaser::updateMemoryCallback, this);

    m_loop_closure_pub = m_privateNodeHandle.advertise<mrbsp_msgs::Keyframe>(m_central_prefix + "DA/LoopClosure", 5);
    m_loop_closure_sub = m_privateNodeHandle.subscribe(m_central_prefix + "DA/LoopClosure", 1, &DaIcpLaser::loopClosureCallback, this);
    m_multirobot_pub = m_privateNodeHandle.advertise<mrbsp_msgs::Keyframe>(m_central_prefix + "DA/MultiRobot", 5);
    m_multirobot_sub = m_privateNodeHandle.subscribe(m_central_prefix + "DA/MultiRobot", 1, &DaIcpLaser::multiRobotCallback, this);

    m_factors_and_values_pub = m_privateNodeHandle.advertise<mrbsp_msgs::GtsamFactorGraph>(m_central_prefix + "belief_input", 1);

    m_da_check_last_index = m_privateNodeHandle.advertiseService(m_central_prefix + "da_check_last_index", &DaIcpLaser::daCheckLastIndex, this);

    m_is_init = true;
    m_da_init_check_service = m_privateNodeHandle.advertiseService(m_central_prefix + "da_init_check", &DaIcpLaser::daInitCheck, this);

    m_logger_msg << "da node initialized...";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}


void DaIcpLaser::loadParameter() {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << m_node_name << " node parameters:";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "=====================";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    // get parameters for loop closure detection and multi robot factors
    m_privateNodeHandle.param("loop_closure/perform", m_perform_lc, true);
    m_privateNodeHandle.param("loop_closure/parallel", m_parallel_lc, true);
    m_privateNodeHandle.param("loop_closure/min_index_range", m_lc_min_idx_range, 10.0);
    m_privateNodeHandle.param("loop_closure/spatial_search_range", m_lc_spatial_search_range, 5.0);
    m_privateNodeHandle.param("multi_robot/perform", m_perform_mr, true);
    m_privateNodeHandle.param("multi_robot/parallel", m_parallel_mr, true);
    m_privateNodeHandle.param("multi_robot/spatial_search_range", m_mr_spatial_search_range, 2.0);
    m_privateNodeHandle.param("enforce_perfect_DA", m_enforcing_perfect_DA, false);

    // get icp thresholds parameters
    m_privateNodeHandle.param("icp/max_iters", m_icp_max_iters, 50);
    m_privateNodeHandle.param("icp/stopping_thresh", m_icp_stopping_thresh, 0.000001); // [m]
    m_privateNodeHandle.param("icp/inlier_thresh_sq", m_icp_inlier_thresh_sq, 0.5); // [m]
    m_privateNodeHandle.param("icp/nn_matching_percentage/sliding_window", m_icp_window_nn_pct_thresh, 0.0); // [%]
    m_privateNodeHandle.param("icp/nn_matching_percentage/loop_closure", m_icp_lc_nn_pct_thresh, 0.0); // [%]
    m_privateNodeHandle.param("icp/nn_matching_percentage/multi_robot", m_icp_mr_nn_pct_thresh, 0.0); // [%]

    // get noise model parameters
    double sigma_yaw_prior;
    double sigma_pitch_prior;
    double sigma_roll_prior;
    double sigma_x_prior;
    double sigma_y_prior;
    double sigma_z_prior;

    double sigma_yaw_odom;
    double sigma_pitch_odom;
    double sigma_roll_odom;
    double sigma_x_odom;
    double sigma_y_odom;
    double sigma_z_odom;

    double sigma_yaw_icp;
    double sigma_pitch_icp;
    double sigma_roll_icp;
    double sigma_x_icp;
    double sigma_y_icp;
    double sigma_z_icp;

    // prior noise model
    m_privateNodeHandle.param("noise_model/prior/sigma_yaw", sigma_yaw_prior, 0.000001);
    m_privateNodeHandle.param("noise_model/prior/sigma_pitch", sigma_pitch_prior, 0.000001);
    m_privateNodeHandle.param("noise_model/prior/sigma_roll", sigma_roll_prior, 0.000001);
    m_privateNodeHandle.param("noise_model/prior/sigma_x", sigma_x_prior, 0.001);
    m_privateNodeHandle.param("noise_model/prior/sigma_y", sigma_y_prior, 0.001);
    m_privateNodeHandle.param("noise_model/prior/sigma_z", sigma_z_prior, 0.001);
    m_prior_model = gtsam::noiseModel::Diagonal::Sigmas
            ((gtsam::Vector(6) << sigma_yaw_prior, sigma_pitch_prior, sigma_roll_prior, sigma_x_prior, sigma_y_prior, sigma_z_prior));

    // odom noise model
    m_privateNodeHandle.param("noise_model/odom/sigma_yaw", sigma_yaw_odom, 0.085); // 0.085rad = 5deg
    m_privateNodeHandle.param("noise_model/odom/sigma_pitch", sigma_pitch_odom, 0.085);
    m_privateNodeHandle.param("noise_model/odom/sigma_roll", sigma_roll_odom, 0.085);
    m_privateNodeHandle.param("noise_model/odom/sigma_x", sigma_x_odom, 0.1);
    m_privateNodeHandle.param("noise_model/odom/sigma_y", sigma_y_odom, 0.1);
    m_privateNodeHandle.param("noise_model/odom/sigma_z", sigma_z_odom, 0.1);
    m_odom_model = gtsam::noiseModel::Diagonal::Sigmas
            ((gtsam::Vector(6) << sigma_yaw_odom, sigma_pitch_odom, sigma_roll_odom, sigma_x_odom, sigma_y_odom, sigma_z_odom));

    // icp noise model
    m_privateNodeHandle.param("noise_model/icp/dynamic_error_percentage", m_error_dynamic_percentage, 0.1); // if <=0 -> const noise
    m_privateNodeHandle.param("noise_model/icp/sigma_yaw", sigma_yaw_icp, 0.085);
    m_privateNodeHandle.param("noise_model/icp/sigma_pitch", sigma_pitch_icp, 0.085);
    m_privateNodeHandle.param("noise_model/icp/sigma_roll", sigma_roll_icp, 0.085);
    m_privateNodeHandle.param("noise_model/icp/sigma_x", sigma_x_icp, 0.1);
    m_privateNodeHandle.param("noise_model/icp/sigma_y", sigma_y_icp, 0.1);
    m_privateNodeHandle.param("noise_model/icp/sigma_z", sigma_z_icp, 0.1);
    m_icp_model = gtsam::noiseModel::Diagonal::Sigmas
            ((gtsam::Vector(6) << sigma_yaw_icp, sigma_pitch_icp, sigma_roll_icp, sigma_x_icp, sigma_y_icp, sigma_z_icp));

    m_privateNodeHandle.param("range_max", m_range_max, 10.0);

    m_logger_msg << "loop_closure/perform: " << m_perform_lc;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "loop_closure/parallel: " << m_parallel_lc;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "loop_closure/min_index_range: " << m_lc_min_idx_range;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "loop_closure/spatial_search_range: " << m_lc_spatial_search_range;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "multi_robot/perform: " << m_perform_mr;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "multi_robot/parallel: " << m_parallel_mr;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "multi_robot/spatial_search_range: " << m_mr_spatial_search_range;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "enforce_perfect_DA: " << m_enforcing_perfect_DA;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "icp/max_iters: " << m_icp_max_iters;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "icp/stopping_thresh[m]: " << m_icp_stopping_thresh;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "icp/inlier_thresh_sq[m]: " << m_icp_inlier_thresh_sq;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "icp/nn_matching_percentage/sliding_window: " << m_icp_window_nn_pct_thresh;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "icp/nn_matching_percentage/loop_closure: " << m_icp_lc_nn_pct_thresh;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "icp/nn_matching_percentage/multi_robot: " << m_icp_mr_nn_pct_thresh;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/prior/sigma_yaw[rad]: " << sigma_yaw_prior;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/prior/sigma_pitch[rad]: " << sigma_pitch_prior;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/prior/sigma_roll[rad]: " << sigma_roll_prior;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/prior/sigma_x[m]: " << sigma_x_prior;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/prior/sigma_y[m]: " << sigma_y_prior;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/prior/sigma_z[m]: " << sigma_z_prior;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/odom/sigma_yaw[rad]: " << sigma_yaw_odom;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/odom/sigma_pitch[rad]: " << sigma_pitch_odom;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/odom/sigma_roll[rad]: " << sigma_roll_odom;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/odom/sigma_x[m]: " << sigma_x_odom;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/odom/sigma_y[m]: " << sigma_y_odom;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/odom/sigma_z[m]: " << sigma_z_odom;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/icp/dynamic_error_percentage: " << m_error_dynamic_percentage;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/icp/sigma_yaw[rad]: " << sigma_yaw_icp;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/icp/sigma_pitch[rad]: " << sigma_pitch_icp;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/icp/sigma_roll[rad]: " << sigma_roll_icp;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/icp/sigma_x[m]: " << sigma_x_icp;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/icp/sigma_y[m]: " << sigma_y_icp;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "noise_model/icp/sigma_z[m]: " << sigma_z_icp;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "range_max: " << m_range_max;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "=====================";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

}

bool DaIcpLaser::daInitCheck(mrbsp_msgs::InitCheck::Request& req, mrbsp_msgs::InitCheck::Response& res) {
    FUNCTION_LOGGER(m_tag);

    res.init_check_answer = static_cast<unsigned char>(m_is_init);
    return true;
}

bool DaIcpLaser::daCheckLastIndex(mrbsp_msgs::LastIndexInDa::Request& req, mrbsp_msgs::LastIndexInDa::Response& res) {
    FUNCTION_LOGGER(m_tag);

    char robot_id = req.robot_id.at(0);
    if(m_all_keyframes.find(robot_id) != m_all_keyframes.end()) {
        res.last_index = m_all_keyframes.at(robot_id).size() - 1;
    }
    else {
        res.last_index = -1;
    }

    return true;
}

std::vector<NewFactorsAndValues> DaIcpLaser::getKeyframes(std::vector<std::vector<Keyframe>>& robots_keyframes) {
    FUNCTION_LOGGER(m_tag);

    m_is_rosbag = true;
    double number_of_robots = robots_keyframes.size();
    std::vector<unsigned int> robot_keyframe_idx(robots_keyframes.size(), 0);
    std::vector<double> robot_current_time(robots_keyframes.size(), 0);

    while(number_of_robots > 0) {
        number_of_robots = robots_keyframes.size();
        for (unsigned int i_robot = 0; i_robot < robots_keyframes.size(); ++i_robot) {
            try {
                robot_current_time.at(i_robot) = std::get<1>(robots_keyframes.at(i_robot).at(robot_keyframe_idx.at(i_robot)));
            }
            catch(const std::out_of_range& e) {
                robot_current_time.at(i_robot) = std::numeric_limits<double>::max();
                --number_of_robots;
            }
        }
        if (number_of_robots == 0) {
            break;
        }

        unsigned int current_robot;
        current_robot = (unsigned int) std::distance(robot_current_time.begin(), std::min_element(robot_current_time.begin(), robot_current_time.end()));

        robot_keyframe_idx[current_robot]++;
    }

    m_logger_msg << "\n"
                 << "Summery:" << "\n"
                 << "--------------------------------------------" << "\n"
                 << "Detected robots:" << "\n" << "\n";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    for (auto iter = m_all_keyframes.begin(); iter != m_all_keyframes.end(); ++iter) {
        m_logger_msg << "Robot ID: " << iter->first << ", Number of keyframes: " << iter->second.size() << "\n";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        gtsam::Pose3 final_pose = iter->second.rbegin()->second.second;
        m_logger_msg << "Final position: " << final_pose.x() << ", " << final_pose.y() << ", " << final_pose.z() << "\n";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    return m_all_factors_and_values;
}

void DaIcpLaser::KeyframeCallback(const mrbsp_msgs::KeyframeConstPtr& keyframe_init_msg) {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << "receive new 2D keyframe information...";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_is_3D_vis = false;
    Keyframe keyframe_init;
    KeyframeMsgParser(keyframe_init_msg, keyframe_init);
    m_is_rosbag = false;
    matchInSlidingWindow(keyframe_init);

    if(m_perform_lc) {
        if(m_parallel_lc) {
            m_loop_closure_pub.publish(keyframe_init_msg);
        }
        else {
            detectLoopClosures(keyframe_init);
        }
    }
    if(m_perform_mr) {
        if(m_parallel_mr) {
            m_multirobot_pub.publish(keyframe_init_msg);
        }
        else {
            detectMultiRobotFactors(keyframe_init);
        }
    }
}

void DaIcpLaser::KeyframeRgbdCallback(const mrbsp_msgs::KeyframeRgbdConstPtr& keyframe_init_rgbd_msg) {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << "receive new 3D keyframe information...";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_is_3D_vis = true;
    Keyframe keyframe_init;
    mrbsp_msgs::Keyframe keyframe_init_msg;
    keyframe_init_msg.header = keyframe_init_rgbd_msg->header;
    keyframe_init_msg.ser_odom_from_prev_keyframe = keyframe_init_rgbd_msg->ser_odom_from_prev_keyframe;
    keyframe_init_msg.ser_symbol = keyframe_init_rgbd_msg->ser_symbol;
    keyframe_init_msg.laser_scan = keyframe_init_rgbd_msg->laser_scan;
    m_current_pointcloud_msg = keyframe_init_rgbd_msg->pointcloud;

    mrbsp_msgs::KeyframeConstPtr keyframe_init_msg_const(new mrbsp_msgs::Keyframe(keyframe_init_msg));
    KeyframeMsgParser(keyframe_init_msg_const, keyframe_init);
    m_is_rosbag = false;
    matchInSlidingWindow(keyframe_init);

    if(m_perform_lc) {
        if(m_parallel_lc) {
            m_loop_closure_pub.publish(keyframe_init_msg);
        }
        else {
            detectLoopClosures(keyframe_init);
        }
    }
    if(m_perform_mr) {
        if(m_parallel_mr) {
            m_multirobot_pub.publish(keyframe_init_msg);
        }
        else {
            detectMultiRobotFactors(keyframe_init);
        }
    }
}

void DaIcpLaser::KeyframeMsgParser(const mrbsp_msgs::KeyframeConstPtr& keyframe_init_msg, Keyframe &keyframe_init) {
    FUNCTION_LOGGER(m_tag);

    gtsam::Symbol current_symbol;
    gtsam::deserialize(keyframe_init_msg->ser_symbol, current_symbol);

    sensor_msgs::LaserScan laser_scan(keyframe_init_msg->laser_scan);

    gtsam::Pose3 odom_from_prev_keyframe;
    gtsam::deserialize(keyframe_init_msg->ser_odom_from_prev_keyframe, odom_from_prev_keyframe);

    std::string keyframe_idx(std::string(1,current_symbol.chr()) + std::to_string(current_symbol.index()));
    double current_time = keyframe_init_msg->header.stamp.toSec();

    keyframe_init = std::make_tuple(keyframe_idx, current_time, laser_scan, odom_from_prev_keyframe, keyframe_init_msg->ground_truth_pose);
}

void DaIcpLaser::matchInSlidingWindow(Keyframe &keyframe_init) {
    FUNCTION_LOGGER(m_tag);

    // gtsam facor graph initialization
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    // extract new data;
    std::string current_index;
    double current_time;
    sensor_msgs::LaserScan current_scan_msg;
    std::vector<gtsam::Point2> current_scan;
    gtsam::Pose3 current_odom;
    gtsam::Pose3 current_pose;
    geometry_msgs::Pose ground_truth_pose;
    std::tie(current_index, current_time, current_scan_msg, current_odom, ground_truth_pose) = keyframe_init;

    // set gtsam symbol
    char index_char = current_index.at(0);
    int  index_num  = std::stoi(current_index.substr(1));
    gtsam::Symbol current_key(index_char, index_num);


    m_logger_msg << "DA: Robot " << index_char << ", index " << index_num << ": Matching in sliding window";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    // cast laser msg to vector of gtsam point2 objects - for csm icp
    rosLaserToCsmGtsam(current_scan_msg, current_scan);

    // initialize pose for the new keyframe
    if(index_num == 0) {
        // new robot detected
        m_logger_msg << "Add new robot, ID " << index_char;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        SingleRobotKeyframe single_robot_keyframes;
        m_all_keyframes.emplace(index_char, single_robot_keyframes);
        current_pose = current_odom; // odom of the first keyframe is the initial pose of the robot
        gtsam::PriorFactor<gtsam::Pose3> f_prior(current_key, current_pose, m_prior_model);
        graph.push_back(f_prior);

        //current_pose.print("Current pose:\n");
    }
    else {
        // calculate current pose with previous scan from this robot
        std::vector<gtsam::Point2> previous_scan(m_all_keyframes.at(index_char).at(index_num - 1).first);
        gtsam::Pose3 previous_pose(m_all_keyframes.at(index_char).at(index_num - 1).second);
        gtsam::Symbol previous_key(index_char, index_num - 1);
        gtsam::Pose3 icp_transformation;

        //bool use_icp = false;
        std::string indexes(current_index + "," + std::string(1, index_char) + std::to_string(index_num - 1));
        if (performCsmIcp(current_scan, previous_scan, current_odom, icp_transformation, m_icp_window_nn_pct_thresh, indexes)) {
        //if(use_icp) {
            // use icp result
            gtsam::noiseModel::Diagonal::shared_ptr icp_dynamic_model(getDynamicModel(icp_transformation));
            gtsam::BetweenFactor<gtsam::Pose3> f_icp(previous_key, current_key, icp_transformation, icp_dynamic_model);
            graph.push_back(f_icp);
            current_pose = previous_pose.compose(icp_transformation);

            //current_pose.print("Current pose:\n");

            if (current_odom.equals(icp_transformation, 0.5)) {

            } else {
                m_logger_msg << "DA: Big difference between initial guess and icp results, temp index " << index_num;
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

                //current_odom.print("current odom:\n");
                m_logger_msg << "DA: current odom:\n" << current_odom;
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

                //icp_transformation.print("icp:\n");
                m_logger_msg << "DA: Icp transformation:\n" << icp_transformation;
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
            }

        } else {
            // use odometry
            m_logger_msg << "Icp failed to converge, use odomety instead.";
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

            gtsam::noiseModel::Diagonal::shared_ptr odom_dynamic_model(getDynamicModel(current_odom));
            gtsam::BetweenFactor<gtsam::Pose3> f_odom(previous_key, current_key, current_odom, odom_dynamic_model);
            graph.push_back(f_odom);
            current_pose = previous_pose.compose(current_odom);

            //current_pose.print("Current pose:\n");
        }
    }

    values.insert(current_key, current_pose);
    m_logger_msg << "DA: Robot " << index_char << ", keyframe " << index_num << " initialized...";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    KeyframeData keyframe_data = std::make_pair(current_scan, current_pose);
    m_all_keyframes.at(index_char).emplace(index_num, keyframe_data);

    if(m_is_3D_vis) {
        MapData3D current_map_data = std::make_pair(m_current_pointcloud_msg, current_pose);
        m_all_map_data_3d.emplace(current_index, current_map_data);
    }
    else {
        MapData current_map_data = std::make_pair(current_scan_msg, current_pose);
        m_all_map_data.emplace(current_index, current_map_data);
    }

    NewFactorsAndValues new_factors_and_values = std::make_pair(graph, values);
    m_all_factors_and_values.push_back(new_factors_and_values);
    m_last_factors_and_values = new_factors_and_values;

    if(!m_is_rosbag) {
        m_logger_msg << "DA: Publish sliding window data.";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        publishDataToBelief(graph, values);
        if(m_is_3D_vis) {
            publishDataToMap3D(m_current_pointcloud_msg, values);
        }
        else {
            publishDataToMap(current_scan_msg, values);
        }
    }
}

void DaIcpLaser::loopClosureCallback(const mrbsp_msgs::KeyframeConstPtr& keyframe_init_msg) {
    FUNCTION_LOGGER(m_tag);

    Keyframe keyframe_init;
    KeyframeMsgParser(keyframe_init_msg, keyframe_init);
    detectLoopClosures(keyframe_init);
}

void DaIcpLaser::detectLoopClosures(Keyframe &keyframe_init) {
    FUNCTION_LOGGER(m_tag);

    // gtsam facor graph initialization
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    // extract new data;
    std::string current_index;
    double current_time;
    sensor_msgs::LaserScan current_scan_msg;
    std::vector<gtsam::Point2> current_scan;
    gtsam::Pose3 current_odom;
    geometry_msgs::Pose ground_truth_pose;
    std::tie(current_index, current_time, current_scan_msg, current_odom, ground_truth_pose) = keyframe_init;

    // set gtsam symbol
    char index_char = current_index.at(0);
    int  index_num  = std::stoi(current_index.substr(1));
    gtsam::Symbol current_key(index_char, index_num);
    gtsam::Pose3 current_pose(m_all_keyframes.at(index_char).at(index_num).second);


    m_logger_msg << "DA: Robot " << index_char << ", index " << index_num << ": Searching for loop closure";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    // cast laser msg to vector of gtsam point2 objects - for csm icp
    rosLaserToCsmGtsam(current_scan_msg, current_scan);

    // detect loop closures factors
    //size_t last_lc_detection_index = -m_lc_min_idx_range;
    for(auto lc_iter = m_all_keyframes.at(index_char).begin(); lc_iter != m_all_keyframes.at(index_char).end(); ++lc_iter) {
        size_t temp_index = lc_iter->first;
        if(index_num - temp_index < m_lc_min_idx_range) {
            break;
        }

        std::vector<gtsam::Point2> temp_scan(m_all_keyframes.at(index_char).at(temp_index).first);
        gtsam::Pose3 temp_pose(m_all_keyframes.at(index_char).at(temp_index).second);
        gtsam::Symbol temp_key(index_char, temp_index);
        gtsam::Pose3 temp_odom(temp_pose.between(current_pose));

        m_logger_msg << "DA: Search for loop closure, Robot " << index_char
                     << " indexes: " << index_num << ", " << temp_index;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "DA: temp_pose: X = " << temp_pose.x() << ", Y = " << temp_pose.y() << ", Z = " << temp_pose.z();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "DA: current_pose: X = " << current_pose.x() << ", Y = " << current_pose.y() << ", Z = " << current_pose.z();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "DA: temp_odom: X = " << temp_odom.x() << ", Y = " << temp_odom.y() << ", Z = " << temp_odom.z();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "DA: range: " << temp_odom.range(gtsam::Pose3())
                     << ", angle difference: : " << fabs(temp_odom.rotation().yaw());
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        //if(temp_odom.range(gtsam::Pose3()) < m_lc_spatial_search_range && fabs(temp_odom.rotation().yaw()) < M_PI_4) {
        if(temp_odom.range(gtsam::Pose3()) < m_lc_spatial_search_range) {

            gtsam::Pose3 lc_icp_transformation;
            std::string indexes(current_index + "," + std::string(1, index_char) + std::to_string(temp_index));
            if (performCsmIcpLc(current_scan, temp_scan, temp_odom, lc_icp_transformation, m_icp_lc_nn_pct_thresh, indexes)) {
                /*bool save_lc_scan(true);
                if(save_lc_scan) {
                }*/

                bool DA_consistent;
                if (m_enforcing_perfect_DA) {

                    gtsam::Pose3 gtsam_gt_pose(gtsam::Rot3::quaternion(ground_truth_pose.orientation.w, ground_truth_pose.orientation.x, ground_truth_pose.orientation.y, ground_truth_pose.orientation.z),
                                               gtsam::Point3(ground_truth_pose.position.x, ground_truth_pose.position.y, ground_truth_pose.position.z));

                    gtsam::Pose3 icp_current_pose = temp_pose.compose(lc_icp_transformation);
                    gtsam::Pose3 absolute_error = gtsam_gt_pose.between(icp_current_pose);
                    DA_consistent = absolute_error.translation().equals(gtsam::Point3(), 0.5) &&
                            absolute_error.rotation().pitch() < 0.1 &&
                            absolute_error.rotation().roll() < 0.1 &&
                            absolute_error.rotation().yaw() < 0.3;

                }
                else
                    DA_consistent = temp_odom.translation().equals(lc_icp_transformation.translation(), 1); // tollerance on displacement 1 m

                if(DA_consistent) {
                    m_logger_msg << "DA: Loop closure detected, Robot " << index_char
                                 << " indexes: " << index_num << ", " << temp_index;
                    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

                    gtsam::noiseModel::Diagonal::shared_ptr icp_dynamic_model(
                            getDynamicModel(lc_icp_transformation));
                    gtsam::BetweenFactor<gtsam::Pose3> f_icp(temp_key, current_key, lc_icp_transformation,
                                                             icp_dynamic_model);

                    graph.push_back(f_icp);

                }
                else {
                    m_logger_msg << "DA: loop closure : Big difference between initial guess/GT and icp results, temp index. Skipping this factor. "
                                 << temp_index;
                    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
                }


                if(temp_index + m_lc_min_idx_range >= m_all_keyframes.at(index_char).size()) {
                    break;
                }
                else {
                    std::advance(lc_iter, m_lc_min_idx_range - 1);
                }
            }
        }
    }

    if(!m_is_rosbag && graph.size() > 0) {
        m_logger_msg << "DA: Publish loop closure data.";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        publishDataToBelief(graph, values);
    }
}

void DaIcpLaser::multiRobotCallback(const mrbsp_msgs::KeyframeConstPtr& keyframe_init_msg) {
    FUNCTION_LOGGER(m_tag);

    Keyframe keyframe_init;
    KeyframeMsgParser(keyframe_init_msg, keyframe_init);
    detectMultiRobotFactors(keyframe_init);
}

void DaIcpLaser::detectMultiRobotFactors(Keyframe &keyframe_init) {
    FUNCTION_LOGGER(m_tag);

    // gtsam facor graph initialization
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    // extract new data;
    std::string current_index;
    double current_time;
    sensor_msgs::LaserScan current_scan_msg;
    std::vector<gtsam::Point2> current_scan;
    gtsam::Pose3 current_odom;
    geometry_msgs::Pose ground_truth_pose;
    std::tie(current_index, current_time, current_scan_msg, current_odom, ground_truth_pose) = keyframe_init;

    // set gtsam symbol
    char index_char = current_index.at(0);
    int  index_num  = std::stoi(current_index.substr(1));
    gtsam::Symbol current_key(index_char, index_num);
    gtsam::Pose3 current_pose(m_all_keyframes.at(index_char).at(index_num).second);

    m_logger_msg << "DA: Robot " << index_char << ", index " << index_num << ":  Searching for multirobot factors";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    // cast laser msg to vector of gtsam point2 objects - for csm icp
    rosLaserToCsmGtsam(current_scan_msg, current_scan);

    // detect multi-robot factors
    for(auto robot_id_iter = m_all_keyframes.begin(); robot_id_iter != m_all_keyframes.end(); ++robot_id_iter) {
        char temp_id = robot_id_iter->first;
        if(temp_id != index_char) {
            for(auto mr_iter = robot_id_iter->second.begin(); mr_iter != robot_id_iter->second.end(); ++mr_iter) {
                size_t temp_index = mr_iter->first;

                std::vector<gtsam::Point2> temp_scan(m_all_keyframes.at(temp_id).at(temp_index).first);
                gtsam::Pose3 temp_pose(m_all_keyframes.at(temp_id).at(temp_index).second);
                gtsam::Symbol temp_key(temp_id, temp_index);
                gtsam::Pose3 temp_odom(temp_pose.between(current_pose));

                if (temp_odom.range(gtsam::Pose3()) < m_mr_spatial_search_range) {
                    gtsam::Pose3 mr_icp_transformation;
                    std::string indexes(current_index + "," + std::string(1, temp_id) + std::to_string(temp_index));
                    if (performCsmIcpMr(current_scan, temp_scan, temp_odom, mr_icp_transformation, m_icp_mr_nn_pct_thresh, indexes)) {
                        gtsam::noiseModel::Diagonal::shared_ptr icp_dynamic_model(
                                getDynamicModel(mr_icp_transformation));
                        gtsam::BetweenFactor<gtsam::Pose3> f_icp(temp_key, current_key, mr_icp_transformation,
                                                                 icp_dynamic_model);

                        graph.push_back(f_icp);

                        if(temp_index + m_lc_min_idx_range >= robot_id_iter->second.size()) {
                            break;
                        }
                        else {
                            std::advance(mr_iter, m_lc_min_idx_range - 1);
                        }


                        if(temp_odom.equals(mr_icp_transformation, 1)) {
                            m_logger_msg << "DA: Multi robot factors detected,"
                                         << " Robot " << index_char <<  "index" << index_num
                                         << " Robot " << temp_id <<  "index" << temp_index;
                            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
                        }
                        else {
                            m_logger_msg << "DA: Multi robot factors : Big difference between initial guess and icp results,"
                                         << " Robot " << index_char <<  "index" << index_num
                                         << " Robot " << temp_id <<  "index" << temp_index;
                            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
                        }
                    }
                }
            }
        }
    }

    if(!m_is_rosbag && graph.size() > 0) {
        m_logger_msg << "DA: Publish multi-robot data.";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        publishDataToBelief(graph, values);
    }
}

void DaIcpLaser::publishDataToBelief(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << "DA: Number of new factors: " << graph.size()
                 << "Number of new values: " << values.size();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);


    // publish factors and values to belief
    mrbsp_msgs::GtsamFactorGraph factor_graph;
    factor_graph.ser_factors = gtsam::serialize(graph);
    factor_graph.ser_values = gtsam::serialize(values);
    m_factors_and_values_pub.publish(factor_graph);
}

void DaIcpLaser::publishDataToMap(const sensor_msgs::LaserScan& scan, const gtsam::Values& values) {
    FUNCTION_LOGGER(m_tag);

    // publish new scan to the map node
    mrbsp_msgs::MapData map_data;
    map_data.header.stamp = ros::Time::now();
    map_data.laser_scan = scan;
    map_data.ser_values = gtsam::serialize(values);
    map_data.is_new_scan = 1;
    m_map_data_2D_pub.publish(map_data);

    m_logger_msg << "DA: Publish 2D map data, new scan. number of values: " << values.size();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}

void DaIcpLaser::publishDataToMap3D(const sensor_msgs::PointCloud2& pointcloud, const gtsam::Values& values) {
    FUNCTION_LOGGER(m_tag);

    // publish new scan to the map node
    mrbsp_msgs::MapData3D map_data3D;
    map_data3D.header.stamp = ros::Time::now();
    map_data3D.pointcloud = pointcloud;
    map_data3D.ser_values = gtsam::serialize(values);
    map_data3D.is_new_scan = 1;
    m_map_data_3D_pub.publish(map_data3D);

    m_logger_msg << "DA: Publish 3D map data, new scan. number of values: " << values.size();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}

void DaIcpLaser::rosLaserToCsmGtsam(sensor_msgs::LaserScan& laser_msg, std::vector<gtsam::Point2>& current_scan) {
    FUNCTION_LOGGER(m_tag);

    current_scan.clear();
    std::vector<float> ranges = laser_msg.ranges;
    float current_angle = laser_msg.angle_min;
    for (auto &&range : ranges) {
        if ((range > laser_msg.range_min) && (range < m_range_max) &&
            (current_angle >= laser_msg.angle_min) && (current_angle <= laser_msg.angle_max)) {
            double x = range * cos(current_angle);
            double y = range * sin(current_angle);
            gtsam::Point2 scan(x, y);
            current_scan.push_back(scan);
        }
        current_angle += laser_msg.angle_increment;
    }
}

bool DaIcpLaser::performCsmIcp(std::vector<gtsam::Point2>& current_measurement,
                                       std::vector<gtsam::Point2>& prev_measurement, const gtsam::Pose3& initial_guess,
                                       gtsam::Pose3& icp_transformation, const double nn_matching_threshold, std::string indexes/*= std::string()*/) {
    FUNCTION_LOGGER(m_tag);
    if(current_measurement.empty() || prev_measurement.empty()) {
        return false;
    }

    if(!indexes.empty()) {
        m_logger_msg << "PLANNAR_ICP: Matching scans between indexes: " << indexes;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    /// ICP thresholds
    const int maxIters = m_icp_max_iters;
    const double stoppingThresh = m_icp_stopping_thresh;
    const double inlierThreshSq = m_icp_inlier_thresh_sq;

    gtsam::Pose2 initial_guess_pose2(initial_guess.x(), initial_guess.y(), initial_guess.rotation().yaw());

    ReturnValue return_value = laserScanICP(prev_measurement, current_measurement, initial_guess_pose2, maxIters, stoppingThresh, inlierThreshSq);

    icp_transformation = gtsam::Pose3(return_value.getDelta());
    double status = return_value.getStatus();

    double scan_size_ratio = double(prev_measurement.size()) / double(current_measurement.size());
    double scan_size_ratio_threshold = 0.95;
    if(scan_size_ratio < scan_size_ratio_threshold || scan_size_ratio > 1/scan_size_ratio_threshold) {
        m_logger_msg << "PLANNAR_ICP: prev_measurement.size() = " << prev_measurement.size();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "PLANNAR_ICP: current_measurement.size() = " << current_measurement.size();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "PLANNAR_ICP: Scans size ratio = " << scan_size_ratio << ", check inverse matching";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        ReturnValue return_value_rev = laserScanICP(current_measurement, prev_measurement, initial_guess_pose2.inverse(), maxIters, stoppingThresh, inlierThreshSq);
        // assign lowest status value
        if(status > return_value_rev.getStatus()) {
            status = return_value_rev.getStatus();
        }
    }

    bool print_icp_results(true);
    if(print_icp_results) {
        m_logger_msg << "PLANNAR_ICP:\n"
                     << "delta: X = " << return_value.getDelta().x() << ", Y = " << return_value.getDelta().y()
                     << ", Yaw = " << return_value.getDelta().rotation().theta() << "\n"
                     << "Number of iterations: " << return_value.getNumIters() << "\n"
                     << "Stoping threshold: " << return_value.getStopping() << "\n"
                     << "Min status score: " << status;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    }

    if(status > 0 && status < nn_matching_threshold) {
        m_logger_msg << "PLANNAR_ICP:\n"
                     << "ICP succeeded but below nn matching threshold." << "\n"
                     << "score: " << status
                     << ", nn matching threshold: " << nn_matching_threshold;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    return status > nn_matching_threshold;
}

bool DaIcpLaser::performCsmIcpLc(std::vector<gtsam::Point2>& current_measurement,
                                       std::vector<gtsam::Point2>& prev_measurement, const gtsam::Pose3& initial_guess,
                                       gtsam::Pose3& icp_transformation, const double nn_matching_threshold, std::string indexes/*= std::string()*/) {
    FUNCTION_LOGGER(m_tag);

    if(current_measurement.empty() || prev_measurement.empty()) {
        return false;
    }

    if(!indexes.empty()) {
        m_logger_msg << "PLANNAR_ICP: Matching scans between indexes: " << indexes;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    /// ICP thresholds
    const int maxIters = m_icp_max_iters;
    const double stoppingThresh = m_icp_stopping_thresh;
    const double inlierThreshSq = m_icp_inlier_thresh_sq;

    gtsam::Pose2 initial_guess_pose2(initial_guess.x(), initial_guess.y(), initial_guess.rotation().yaw());

    ReturnValue return_value = laserScanICP(prev_measurement, current_measurement, initial_guess_pose2, maxIters, stoppingThresh, inlierThreshSq);

    icp_transformation = gtsam::Pose3(return_value.getDelta());
    double status = return_value.getStatus();

    double scan_size_ratio = double(prev_measurement.size()) / double(current_measurement.size());
    double scan_size_ratio_threshold = 0.95;
    if(scan_size_ratio < scan_size_ratio_threshold || scan_size_ratio > 1/scan_size_ratio_threshold) {
        m_logger_msg << "PLANNAR_ICP: prev_measurement.size() = " << prev_measurement.size();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "PLANNAR_ICP: current_measurement.size() = " << current_measurement.size();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "PLANNAR_ICP: Scans size ratio = " << scan_size_ratio << ", check inverse matching";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        ReturnValue return_value_rev = laserScanICP(current_measurement, prev_measurement, initial_guess_pose2.inverse(), maxIters, stoppingThresh, inlierThreshSq);
        // assign lowest status value
        if(status > return_value_rev.getStatus()) {
            status = return_value_rev.getStatus();
        }
    }

    bool print_icp_results(true);
    if(print_icp_results) {
        m_logger_msg << "PLANNAR_ICP:\n"
                     << "delta: X = " << return_value.getDelta().x() << ", Y = " << return_value.getDelta().y()
                     << ", Yaw = " << return_value.getDelta().rotation().theta() << "\n"
                     << "Number of iterations: " << return_value.getNumIters() << "\n"
                     << "Stoping threshold: " << return_value.getStopping() << "\n"
                     << "Min status score: " << status;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    }

    if(status > 0 && status < nn_matching_threshold) {
        m_logger_msg << "PLANNAR_ICP:\n"
                     << "ICP succeeded but below nn matching threshold." << "\n"
                     << "score: " << status
                     << ", nn matching threshold: " << nn_matching_threshold;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    return status > nn_matching_threshold;
}

bool DaIcpLaser::performCsmIcpMr(std::vector<gtsam::Point2>& current_measurement,
                                       std::vector<gtsam::Point2>& prev_measurement, const gtsam::Pose3& initial_guess,
                                       gtsam::Pose3& icp_transformation, const double nn_matching_threshold, std::string indexes/*= std::string()*/) {
    FUNCTION_LOGGER(m_tag);
    if(current_measurement.empty() || prev_measurement.empty()) {
        return false;
    }

    if(!indexes.empty()) {
        m_logger_msg << "PLANNAR_ICP: Matching scans between indexes: " << indexes;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    /// ICP thresholds
    const int maxIters = m_icp_max_iters;
    const double stoppingThresh = m_icp_stopping_thresh;
    const double inlierThreshSq = m_icp_inlier_thresh_sq;

    gtsam::Pose2 initial_guess_pose2(initial_guess.x(), initial_guess.y(), initial_guess.rotation().yaw());

    ReturnValue return_value = laserScanICP(prev_measurement, current_measurement, initial_guess_pose2, maxIters, stoppingThresh, inlierThreshSq);

    icp_transformation = gtsam::Pose3(return_value.getDelta());
    double status = return_value.getStatus();

    double scan_size_ratio = double(prev_measurement.size()) / double(current_measurement.size());
    double scan_size_ratio_threshold = 0.95;
    if(scan_size_ratio < scan_size_ratio_threshold || scan_size_ratio > 1/scan_size_ratio_threshold) {
        m_logger_msg << "PLANNAR_ICP: prev_measurement.size() = " << prev_measurement.size();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "PLANNAR_ICP: current_measurement.size() = " << current_measurement.size();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        m_logger_msg << "PLANNAR_ICP: Scans size ratio = " << scan_size_ratio << ", check inverse matching";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        ReturnValue return_value_rev = laserScanICP(current_measurement, prev_measurement, initial_guess_pose2.inverse(), maxIters, stoppingThresh, inlierThreshSq);
        // assign lowest status value
        if(status > return_value_rev.getStatus()) {
            status = return_value_rev.getStatus();
        }
    }

    bool print_icp_results(true);
    if(print_icp_results) {
        m_logger_msg << "PLANNAR_ICP:\n"
                     << "delta: X = " << return_value.getDelta().x() << ", Y = " << return_value.getDelta().y()
                     << ", Yaw = " << return_value.getDelta().rotation().theta() << "\n"
                     << "Number of iterations: " << return_value.getNumIters() << "\n"
                     << "Stoping threshold: " << return_value.getStopping() << "\n"
                     << "Min status score: " << status;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    }

    if(status > 0 && status < nn_matching_threshold) {
        m_logger_msg << "PLANNAR_ICP:\n"
                     << "ICP succeeded but below nn matching threshold." << "\n"
                     << "score: " << status
                     << ", nn matching threshold: " << nn_matching_threshold;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    return status > nn_matching_threshold;
}

gtsam::noiseModel::Diagonal::shared_ptr DaIcpLaser::getDynamicModel(gtsam::Pose3& action) {
    FUNCTION_LOGGER(m_tag);

    if(m_error_dynamic_percentage > 0) {
        gtsam::Point3 translation = action.translation();
        gtsam::Rot3 rotation = action.rotation();

        double roll     = std::max(rotation.roll()  * m_error_dynamic_percentage, MIN_ROTATION_NOISE_SIGMA);
        double pitch    = std::max(rotation.pitch() * m_error_dynamic_percentage, MIN_ROTATION_NOISE_SIGMA);
        double yaw      = std::max(rotation.yaw()   * m_error_dynamic_percentage, MIN_ROTATION_NOISE_SIGMA);
        double X        = std::max(translation.x()  * m_error_dynamic_percentage, MIN_TRANSLATION_NOISE_SIGMA);
        double Y        = std::max(translation.y()  * m_error_dynamic_percentage, MIN_TRANSLATION_NOISE_SIGMA);
        double Z        = std::max(translation.z()  * m_error_dynamic_percentage, MIN_TRANSLATION_NOISE_SIGMA);

        gtsam::Vector vector(6);
        vector << roll, pitch, yaw, X, Y, Z;

        return gtsam::noiseModel::Diagonal::Sigmas(vector);
    }
    else {
        return m_icp_model;
    }
}

void DaIcpLaser::saveScan(std::vector<gtsam::Point2>& scan, std::string& index) {
    FUNCTION_LOGGER(m_tag);

    std::fstream file;
    file.open(std::string("scans/scan_" + index + ".txt"), std::fstream::out);

    for(auto iter = scan.begin(); iter != scan.end(); ++iter) {
        file << iter->x() << " " << iter->y() << "\n";
    }
    file.close();
}

void DaIcpLaser::updateMemoryCallback(const mrbsp_msgs::GtsamSerValuesConstPtr& serialized_values_msg) {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << "DA: receive serialized values.";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    gtsam::Values updated_values;
    try {
        gtsam::deserialize(serialized_values_msg->ser_values, updated_values);

        m_logger_msg << "DA: Number of optimized values: " << updated_values.size();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        updateMemory(updated_values);
    }
    catch (...) {
        m_logger_msg << "DA: Deserialization failed!";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }
}

void DaIcpLaser::updateMemory(gtsam::Values& updated_values) {
    FUNCTION_LOGGER(m_tag);

    gtsam::Values vals_for_map;
    gtsam::KeyList updated_keys = updated_values.keys();
    for(auto key : updated_keys) {
        gtsam::Symbol symbol(key);
        char robot_id = symbol.chr();
        unsigned int index = symbol.index();

        // update keyframes data structure
        auto updated_iter = m_all_keyframes.at(robot_id).find(index);
        if(updated_iter !=  m_all_keyframes.at(robot_id).end()) {
            if(!updated_iter->second.second.equals(updated_values.at<gtsam::Pose3>(symbol),0.1)) {
                //updated_iter->second.second.print("Old value:\n");
                updated_iter->second.second = updated_values.at<gtsam::Pose3>(symbol);
                //updated_iter->second.second.print("New value:\n");

                // update map data structure
                std::string index_string((1, robot_id) + std::to_string(index));
                auto map_iter = m_all_map_data.find(index_string);
                if(map_iter != m_all_map_data.end()) {
                    map_iter->second.second = updated_values.at<gtsam::Pose3>(symbol);
                    vals_for_map.insert(symbol, updated_values.at<gtsam::Pose3>(symbol));
                }
            }
        }
    }

    if(!m_is_rosbag && vals_for_map.size() > 0) {
        // publish updated poses to the map node
        mrbsp_msgs::MapData map_data;
        map_data.header.stamp = ros::Time::now();
        map_data.ser_values = gtsam::serialize(vals_for_map);
        map_data.is_new_scan = 0;
        m_map_data_2D_pub.publish(map_data);
        //m_map_data_3D_pub.publish(map_data);

        m_logger_msg << "DA: Publish map data, update scan poses. number of values: " << vals_for_map.size();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    // print last pose for each robot
    m_logger_msg << "--------------------------";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "Last poses of each robot:";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "--------------------------";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    for(auto robot_iter = m_all_keyframes.begin(); robot_iter != m_all_keyframes.end(); ++robot_iter) {
        char robot_id = robot_iter->first;
        unsigned int last_index = robot_iter->second.rbegin()->first;
        gtsam::Pose3 last_pose = robot_iter->second.rbegin()->second.second;

        m_logger_msg << "Robot id: " << robot_id << ", index: " << last_index;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        m_logger_msg << "Robot position: X = " << last_pose.x() << ", Y = " << last_pose.y() << ", Z = " << last_pose.z();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }
}

std::map<std::string, MapData> DaIcpLaser::getMapData() {
    FUNCTION_LOGGER(m_tag);

    return m_all_map_data;
}

DaIcpLaser::~DaIcpLaser() {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg <<  "Destructor " << m_node_name << " node";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}
