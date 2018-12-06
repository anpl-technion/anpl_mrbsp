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
 * @file: odometry_external.cpp
 * @brief:
 * @author: Tal Regev
 */


#include "odometry/odometry_external.h"

#include <mrbsp_msgs/LastIndexInDa.h>
#include <std_srvs/Empty.h>

#include <random>
#include <chrono>

#include <gtsam/base/serialization.h>
#include <gtsam/inference/Symbol.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <planar_icp/Saliency.h>


#include "odometry/odometry_external.h"
#include <ros/ros.h>
#include <signal.h>

#include <mrbsp_utils/gtsam_serialization.h>


using namespace MRBSP;
using namespace MRBSP::Utils;

void mySigintHandler(int sig) {
    std::stringstream logger_msg;
    Utils::LogTag tag = LogTag::odometry;

    logger_msg << "Shutting down " << ros::this_node::getName().substr(1) << " node";
    logMessage(info, LOG_INFO_LVL, logger_msg, tag);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "OdometryExternal");
    ros::NodeHandle pnh("~");
    signal(SIGINT, mySigintHandler);

    OdometryExternal odometry_external(pnh);

    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();

    ros::waitForShutdown();

    return 0;
}

void OdometryExternal::loadParameter() {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << m_node_name << " node parameters:";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "=====================";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    std::string robot_id;
    double x, y, z, yaw, pitch, roll;
    m_privateNodeHandle.param("robot_name", m_robot_name, std::string("Robot_A"));

    m_privateNodeHandle.param("robot_id", robot_id, std::string("A"));
    m_robot_id = robot_id.at(0);


    m_privateNodeHandle.param("init_pose/position/X", x, double(0.0));
    m_privateNodeHandle.param("init_pose/position/Y", y, double(0.0));
    m_privateNodeHandle.param("init_pose/position/Z", z, double(0.0));
    m_privateNodeHandle.param("init_pose/orientation/Yaw",   yaw,   double(0.0));
    m_privateNodeHandle.param("init_pose/orientation/Pitch", pitch, double(0.0));
    m_privateNodeHandle.param("init_pose/orientation/Roll",  roll,  double(0.0));

    gtsam::Rot3 R = gtsam::Rot3::ypr(yaw, pitch, roll);
    gtsam::Point3 t(x, y, z);
    gtsam::Pose3 initial_pose(R, t);
    m_current_pose = initial_pose;

    m_privateNodeHandle.getParam("topics/odom", m_odom_topic);
    m_privateNodeHandle.getParam("topics/laser", m_laser_topic);
    m_privateNodeHandle.getParam("topics/image", m_image_topic);
    m_privateNodeHandle.getParam("topics/pointcloud", m_pointcloud_topic);
    m_privateNodeHandle.param("is_visualize_laser", m_is_laser_vis, false);
    m_privateNodeHandle.param("is_3D_vis", m_is_3D_vis, false);

    m_privateNodeHandle.getParam("ground_truth_source", m_gt_source);
    m_privateNodeHandle.getParam("topics/ground_truth", m_gt_topic_sub);

    // defualt odometry rate: 10 [Hz]
    m_privateNodeHandle.param("velocity_update_rate", m_vel_update_rate, 10.0);

    // condition to create new keyframe
    m_privateNodeHandle.param("informative_condition/distance", m_informative_condition_distance, 1.0);
    m_privateNodeHandle.param("informative_condition/yaw", m_informative_condition_yaw, 0.2618);
    m_privateNodeHandle.param("is_odom_noised", m_is_odom_noised, false);
    m_privateNodeHandle.param("error_dynamic_percentage", m_error_dynamic_percentage, 0.1);

    m_privateNodeHandle.param("index_timeout_threshold", m_optimize_pose_index_timeout_threshold, 50);

    m_privateNodeHandle.param("is_print_icp_results", m_is_print_icp_results, true);

    m_logger_msg << "robot_name: " << m_robot_name;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "robot_id: " << m_robot_id;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "init_pose: " << m_current_pose;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "topics/odom: " << m_odom_topic;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "topics/laser: " << m_laser_topic;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "topics/image: " << m_image_topic;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "topics/pointcloud: " << m_pointcloud_topic;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "is_visualize_laser: " << m_is_laser_vis;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "is_3D_vis: " << m_is_3D_vis;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "ground_truth_source: " << m_gt_source;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "topics/ground_truth: " << m_gt_topic_sub;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "velocity_update_rate: " << m_vel_update_rate;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg  << "informative_condition/distance: " << m_informative_condition_distance;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "informative_condition/yaw: " << m_informative_condition_yaw;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "is_odom_noised: " << m_is_odom_noised;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "error_dynamic_percentage: " << m_error_dynamic_percentage;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "index_timeout_threshold: " << m_optimize_pose_index_timeout_threshold;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "is_print_icp_results: " << m_is_print_icp_results;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "=====================";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

}

OdometryExternal::OdometryExternal(const ros::NodeHandle &nh_private) :
        m_is_perceive(false),
        m_privateNodeHandle(nh_private),
        m_first_odom_msg(true),
        m_first_laser_msg(true),
        m_is_da_init(false)
{
    m_tag = LogTag::odometry;
    FUNCTION_LOGGER(m_tag);

    if (!m_privateNodeHandle.hasParam("/scenario_folder"))
    {
        m_logger_msg << m_node_name << " node load without parameters.\n exiting node.";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        ros::shutdown();
        return;
    }

    initLogger(m_privateNodeHandle);
    loadParameter();


    m_current_odom_pose = m_current_pose;
    m_last_optimize_pose = m_current_pose;
    m_last_keyframe_pose = m_current_pose;


    m_odom_sub = m_privateNodeHandle.subscribe(std::string(m_robot_name + m_odom_topic), 1, &OdometryExternal::odomCallback, this);
    

	m_laser_sub = m_privateNodeHandle.subscribe(std::string(m_robot_name + m_laser_topic), 1, &OdometryExternal::laserCallback, this);
	
	
    if(m_is_laser_vis) {
        m_laser_pub = m_privateNodeHandle.advertise<sensor_msgs::LaserScan>(std::string(m_robot_name + m_laser_topic + "/vis"), 1);
    }

    
    if(m_is_3D_vis) {
        // set keyframe type (with pointclouds)
        m_pointcloud_sub = m_privateNodeHandle.subscribe(std::string(m_robot_name + m_pointcloud_topic), 1, &OdometryExternal::pointcloudCallback, this);
        m_keyframe_info_pub = m_privateNodeHandle.advertise<mrbsp_msgs::KeyframeInitRgbd>("/Odometry/keyframe_init/withPointcloud",1);

        m_logger_msg << "Robot " << m_robot_id << ": publish keyframes info with 3D scans";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    }
    else {
        // set keyframe type here (without pointclouds)
        m_keyframe_info_pub = m_privateNodeHandle.advertise<mrbsp_msgs::KeyframeInit>("/Odometry/keyframe_init",1);

        m_logger_msg << "Robot " << m_robot_id << ": publish keyframes info with 2D scans";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    std::string optimize_pose_topic("/Robot_" + std::string(1, m_robot_id) + "/optimized_pose");
	m_optimize_pose_sub = m_privateNodeHandle.subscribe(optimize_pose_topic, 1, &OdometryExternal::optimizePoseCallback, this);


    m_data_source = DataSource::live;
    m_last_optimize_index = 0;
    m_last_keyframe_index = 0;



    m_GT_available = false;
    if (m_gt_source == "gazebo") {
        m_ground_truth_sub = m_privateNodeHandle.subscribe("/gazebo/model_states", 1, &OdometryExternal::gtGazeboCallback, this);
        m_GT_available = true;
    }
    if (m_gt_source == "mocap") {
        m_ground_truth_sub = m_privateNodeHandle.subscribe(std::string(m_robot_name + m_gt_topic_sub), 1, &OdometryExternal::gtMoCapCallback, this);
        m_GT_available = true;
    }

    std::string gt_topic_pub("/Robot_" + std::string(1, m_robot_id) + "/ground_truth_path");
    m_robot_ground_truth_pub = m_privateNodeHandle.advertise<nav_msgs::Path>(gt_topic_pub, 1);

    std::string estimated_path_topic("/Robot_" + std::string(1, m_robot_id) + "/estimated_path");
    m_estimated_path_pub = m_privateNodeHandle.advertise<nav_msgs::Path>(estimated_path_topic, 1);
    
    
    m_da_last_index_client = m_privateNodeHandle.serviceClient<mrbsp_msgs::LastIndexInDa>("da_check_last_index");

    std::string service_name(m_robot_name + "/check_if_perceive");
    m_preceive_init_check_service = m_privateNodeHandle.advertiseService(service_name, &OdometryExternal::odometryInitCheck, this);

    std::string odom_pub_topic("/Robot_" + std::string(1, m_robot_id) + "/odometry");
    m_current_odom_pub = m_privateNodeHandle.advertise<nav_msgs::Odometry>(odom_pub_topic, 1);

	//m_relative_motion.push_back(gtsam::Pose3());
    m_logger_msg << "Robot " << m_robot_id << " initialized";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "Robot " << m_robot_id <<  " Initial robot position: X =  " << m_current_pose.x()
                 << ", Y = " << m_current_pose.y() << ", Z = " << m_current_pose.z();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "Robot " << m_robot_id <<  " Initial yaw: " << m_current_pose.rotation().yaw();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}


bool OdometryExternal::odometryInitCheck(mrbsp_msgs::InitCheck::Request& req, mrbsp_msgs::InitCheck::Response& res) {
    FUNCTION_LOGGER(m_tag);

    res.init_check_answer = static_cast<unsigned char>(m_is_perceive);
    return true;
}

std::vector<KeyframeInit> OdometryExternal::getDataFromBag(const std::string& path_to_bagfile, const std::string& odom_topic, const std::string& laser_topic) {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << "--------------------";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "Robot id: "           << m_robot_id;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "rosbag file name: "   << path_to_bagfile;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "ros odometry topic: " << odom_topic;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "ros laser topic: "    << laser_topic;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    rosbag::Bag bag(path_to_bagfile);

    m_logger_msg << "Robot " << m_robot_id << ", start reading data...";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    rosbag::View view_odom(bag, rosbag::TopicQuery(odom_topic));
    rosbag::View view_laser(bag, rosbag::TopicQuery(laser_topic));
    double odom_time = 0;
    double laser_time = 0;
    auto laser_iterator = view_laser.begin();
    auto odom_iterator = view_odom.begin();
    bool init_laser = false; // start the algorithm only after getting first laser scan
    bool init_odom = false;

    m_logger_msg << "Number of odometry messages: " << view_odom.size();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "Number of laser messages: "    << view_laser.size();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_initial_time = std::min(view_laser.begin()->getTime().toSec(), view_odom.begin()->getTime().toSec());

    // TODO: add initial condition check
    m_logger_msg << "Initial robot position: X = " << m_current_pose.x() << ", Y = " << m_current_pose.y() << ", Z = " << m_current_pose.z();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "Initial yaw: " <<  m_current_pose.rotation().yaw();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    if(view_odom.size() > view_laser.size()) {
        for (odom_iterator = view_odom.begin(); odom_iterator != view_odom.end(); ++odom_iterator) {
            auto messageInstance = *odom_iterator;
            nav_msgs::OdometryConstPtr odomConstPtr = messageInstance.instantiate<nav_msgs::Odometry>();
            if (odomConstPtr != NULL) {
                odom_time = odomConstPtr->header.stamp.toSec();

                if ((laser_time < odom_time) && (laser_iterator != view_laser.end())) {
                    auto messageInstance = *laser_iterator;
                    sensor_msgs::LaserScanConstPtr laserConstPtr = messageInstance.instantiate<sensor_msgs::LaserScan>();
                    if (laserConstPtr != NULL) {
                        m_current_laser_msg = *laserConstPtr;
                        laser_time = laserConstPtr->header.stamp.toSec();

                        if (!init_laser) {
                            std::string keyframe_idx(m_robot_id + std::to_string(m_last_keyframe_index));
                            KeyframeInit current_keyframe = std::make_tuple(keyframe_idx, 0, m_current_laser_msg,
                                                                            m_current_pose, m_ground_truth_pose_stamped.pose);
                            m_keyframes.push_back(current_keyframe);

                            m_logger_msg << "keyframe laser seq " << m_current_laser_msg.header.seq;
                            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

                            init_laser = true;

                            m_last_keyframe_index++;
                        }
                    }
                    ++laser_iterator;
                }

                if (!init_laser) {
                    continue;
                } else {
                    handleOdomData(odomConstPtr);
                }
            }
        }
    }
    else {
        for (laser_iterator = view_laser.begin(); laser_iterator != view_laser.end(); ++laser_iterator) {
            auto messageInstance = *laser_iterator;
            sensor_msgs::LaserScanConstPtr laserConstPtr = messageInstance.instantiate<sensor_msgs::LaserScan>();

            if (laserConstPtr != NULL) {
                m_current_laser_msg = *laserConstPtr;
                laser_time = laserConstPtr->header.stamp.toSec();

                if ((odom_time < laser_time) && (odom_iterator != view_odom.end())) {
                    auto messageInstance = *odom_iterator;
                    nav_msgs::OdometryConstPtr odomConstPtr = messageInstance.instantiate<nav_msgs::Odometry>();
                    if (odomConstPtr != NULL) {
                        odom_time = odomConstPtr->header.stamp.toSec();

                        if (!init_odom) {
                            std::string keyframe_idx(m_robot_id + std::to_string(m_last_keyframe_index));
                            KeyframeInit current_keyframe = std::make_tuple(keyframe_idx, 0, m_current_laser_msg,
                                                                            m_current_pose, m_ground_truth_pose_stamped.pose);
                            m_keyframes.push_back(current_keyframe);
                            init_odom = true;

                            m_last_keyframe_index++;
                        }

                        handleOdomData(odomConstPtr);
                    }
                    ++odom_iterator;
                }
            }
        }
    }
    double odom_distance = 0;
    for(auto relative_iter = m_relative_motion.begin(); relative_iter != m_relative_motion.end(); ++relative_iter) {
        odom_distance += relative_iter->range(gtsam::Pose3());
    }

    m_logger_msg << "Total odometry distance: " << odom_distance;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    return m_keyframes;
}

void OdometryExternal::handleOdomData(nav_msgs::OdometryConstPtr odom_msg) {
    FUNCTION_LOGGER(m_tag);

    m_is_perceive = true;

    // odometry calculator
    gtsam::Pose3 previous_odom_pose = m_current_odom_pose;
    rosOdomeMsgToGtsam(odom_msg);
    gtsam::Pose3 current_odom = previous_odom_pose.between(m_current_odom_pose);

    if(m_is_odom_noised) {
        std::default_random_engine generator;
        std::normal_distribution<double> x_distribution(0.0,current_odom.x() * m_error_dynamic_percentage);
        std::normal_distribution<double> y_distribution(0.0,current_odom.y() * m_error_dynamic_percentage);
        std::normal_distribution<double> z_distribution(0.0,current_odom.z() * m_error_dynamic_percentage);
        std::normal_distribution<double> yaw_distribution(0.0,current_odom.rotation().yaw() * m_error_dynamic_percentage);
        std::normal_distribution<double> roll_distribution(0.0,current_odom.rotation().roll() * m_error_dynamic_percentage);
        std::normal_distribution<double> pitch_distribution(0.0,current_odom.rotation().pitch() * m_error_dynamic_percentage);

        double x_noise = x_distribution(generator);
        double y_noise = y_distribution(generator);
        double z_noise = z_distribution(generator);
        double yaw_noise = yaw_distribution(generator);
        double roll_noise = roll_distribution(generator);
        double pitch_noise = pitch_distribution(generator);

        gtsam::Point3 t_noised(current_odom.x() + x_noise, current_odom.y() + y_noise, current_odom.z() + z_noise);
        gtsam::Rot3   R_noised(gtsam::Rot3::ypr(current_odom.rotation().yaw() + yaw_noise,
                                                current_odom.rotation().pitch() + pitch_noise,
                                                current_odom.rotation().roll() + roll_noise));
        current_odom = gtsam::Pose3(R_noised, t_noised);
    }

    // update robot pose
    m_relative_motion_from_last_keyframe = m_relative_motion_from_last_keyframe.compose(current_odom); // update last object in relative motion vector
    m_current_pose = m_last_keyframe_pose.compose(m_relative_motion_from_last_keyframe);


    if(m_data_source == DataSource::live) {
        // broadcast current robot pose with tf
        broadcastCurrentPose(m_current_pose, std::string("Robot_" + std::string(1,m_robot_id)), current_odom);
    }

    // keyframes information
    if(isInformative(m_relative_motion_from_last_keyframe, m_current_laser_msg) || m_last_keyframe_index == 0)
    {
        // check what is the last index stored in da.
        mrbsp_msgs::LastIndexInDa da_last_index_srv;
        da_last_index_srv.request.robot_id = std::string(1, m_robot_id);
        if (m_da_last_index_client.call(da_last_index_srv)) {
            if(m_last_keyframe_index - da_last_index_srv.response.last_index != 1) {
                m_logger_msg << "Robot " << m_robot_id << ": DA didn't receive last keyframe. Not publishing new one.";
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

                m_logger_msg << "Robot " << m_robot_id << ": : Last odometry keyframe: " << m_last_keyframe_index <<
                        ", last keyframe in da: " << da_last_index_srv.response.last_index;
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
                return;
            }
            else {
                //ROS_WARN("Robot %c: Last odometry keyframe: %d, last keyframe in da: %d", m_robot_id, m_last_keyframe_index, da_last_index_srv.response.last_index);
            }
        }
        else {
            return;
        }

        m_logger_msg << "Robot " << m_robot_id << ": Add new keyframe, index: " << m_last_keyframe_index;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        m_logger_msg << "Robot " << m_robot_id << ": Current position: X = " << m_current_pose.x() <<
                ", Y = " << m_current_pose.y() << ", Z = " << m_current_pose.z();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        switch(m_data_source) {
            case DataSource::bagfile :
            {
                std::string keyframe_idx(m_robot_id + std::to_string(m_last_keyframe_index));
                double current_time = odom_msg->header.stamp.toSec() - m_initial_time;
                KeyframeInit current_keyframe = std::make_tuple(keyframe_idx, current_time, m_current_laser_msg, m_relative_motion_from_last_keyframe, m_ground_truth_pose_stamped.pose);
                m_keyframes.push_back(current_keyframe);
                break;
            }

            case DataSource::live :
            {
                m_logger_msg << "Robot " << m_robot_id << ": Publish keyframe " << m_last_keyframe_index;
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

		        //publish keyframes
                if(!m_is_3D_vis) {
                    mrbsp_msgs::KeyframeInit keyframe;
                    keyframe.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
                    keyframe.header.seq = m_last_keyframe_index;
                    keyframe.header.stamp = odom_msg->header.stamp;
                    keyframe.ser_symbol = gtsam::serialize(gtsam::Symbol(m_robot_id, m_last_keyframe_index));
                    keyframe.laser_scan = m_current_laser_msg;
                    keyframe.ground_truth_pose = m_ground_truth_pose_stamped.pose;

                    if (m_last_keyframe_index == 0) {
                        // initial pose
                        keyframe.ser_odom_from_prev_keyframe = gtsam::serialize(m_current_pose);
                    } else {
                        keyframe.ser_odom_from_prev_keyframe = gtsam::serialize(m_relative_motion_from_last_keyframe);
                    }

                    m_keyframe_info_pub.publish(keyframe);
                }
                else {
                    // keyframes with 3D data
                    mrbsp_msgs::KeyframeInitRgbd keyframe_3d;
                    keyframe_3d.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
                    keyframe_3d.header.seq = m_last_keyframe_index;
                    keyframe_3d.header.stamp =  odom_msg->header.stamp;
                    keyframe_3d.ser_symbol = gtsam::serialize(gtsam::Symbol(m_robot_id, m_last_keyframe_index));
                    keyframe_3d.laser_scan = m_current_laser_msg;
                    keyframe_3d.ground_truth_pose = m_ground_truth_pose_stamped.pose;

                    if (m_last_keyframe_index == 0) {
                        // initial pose
                        keyframe_3d.ser_odom_from_prev_keyframe = gtsam::serialize(m_current_pose);
                    } else {
                        keyframe_3d.ser_odom_from_prev_keyframe = gtsam::serialize(m_relative_motion_from_last_keyframe);
                    }

                    keyframe_3d.pointcloud = m_current_pointcloud_msg;

                    m_logger_msg << "Robot " << m_robot_id << ": Publish pointcloud, height: " <<
                                 m_current_pointcloud_msg.height << ", width: " << m_current_pointcloud_msg.width;
                    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

                    m_keyframe_info_pub.publish(keyframe_3d);
                }

                if(isOptimizePoseTimout()) {
                    // publish warning
                }
                break;
            }
        }

        if(m_last_keyframe_index > 0) {
            m_relative_motion.push_back(m_relative_motion_from_last_keyframe);
        }
        m_last_keyframe_pose = m_last_optimize_pose;
        for(auto iter_rel = m_relative_motion.begin() + m_last_optimize_index ; iter_rel != m_relative_motion.end(); ++iter_rel) {
            m_last_keyframe_pose = m_last_keyframe_pose.compose(*iter_rel);
        }
        m_relative_motion_from_last_keyframe = gtsam::Pose3();
        m_last_keyframe_index++;
    }

}

void OdometryExternal::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
    FUNCTION_LOGGER(m_tag);

    // Check if the da node is initialized

    if(!m_is_da_init) {
        ros::ServiceClient da_init_check_client = m_privateNodeHandle.serviceClient<mrbsp_msgs::InitCheck>("da_init_check");
        mrbsp_msgs::InitCheck da_init_check_srv;
        if (da_init_check_client.call(da_init_check_srv))
        {
            m_is_da_init = da_init_check_srv.response.init_check_answer;
            if(!m_is_da_init) {
                m_logger_msg << "Robot " << m_robot_id << ": DA node isn't ready, service return false";
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
            }
            else {
                m_logger_msg << "Robot " << m_robot_id << ": DA node initialized";
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
            }
        }
        else
        {
            m_logger_msg << "Robot " << m_robot_id << ": DA node isn't ready, service call failed";
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        }

        m_logger_msg << "Robot " << m_robot_id << ": Wait for 5 sec...";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        ros::Duration(5.0).sleep(); // sleep for 5 seconds
        return;
    }


    // make sure we receive a laser scan
    if(m_first_laser_msg) {
        return;
    }

    if(m_first_odom_msg) {
        rosOdomeMsgToGtsam(odom);
        m_first_odom_msg = false;

        m_logger_msg << "Robot " << m_robot_id << ": Receive first odom msg...";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    }
    handleOdomData(odom);
}

void OdometryExternal::rosOdomeMsgToGtsam(const nav_msgs::OdometryConstPtr& odom_msg) {
    FUNCTION_LOGGER(m_tag);

    geometry_msgs::Pose curr_odom_ros = odom_msg->pose.pose;
    geometry_msgs::Quaternion curr_quat_ros = curr_odom_ros.orientation;
    geometry_msgs::Point curr_point_ros = curr_odom_ros.position;

    gtsam::Rot3 curr_rot_gtsam = gtsam::Rot3::quaternion(curr_quat_ros.w, curr_quat_ros.x, curr_quat_ros.y, curr_quat_ros.z);
    gtsam::Point3 curr_point_gtsam = gtsam::Point3(curr_point_ros.x, curr_point_ros.y, curr_point_ros.z);
    m_current_odom_pose = gtsam::Pose3(curr_rot_gtsam, curr_point_gtsam);
}

void OdometryExternal::optimizePoseCallback(const mrbsp_msgs::GtsamSerPose3ConstPtr& optimized_pose) {
    FUNCTION_LOGGER(m_tag);

    bool optimize_pose = true;

    if(optimize_pose) {
        std::string symbol_str(optimized_pose->header.frame_id);
        char optimized_pose_id = symbol_str.at(0);
        int optimize_pose_idx = std::stoi(symbol_str.substr(1));

        if (m_robot_id == optimized_pose_id && m_last_optimize_index <= optimize_pose_idx) {

            m_logger_msg << "Robot " << m_robot_id << ": Old optimized pose, index " << m_last_optimize_index;
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

            m_logger_msg << "m_last_optimize_pose" << m_last_optimize_pose;
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

            m_logger_msg << "Robot " << optimized_pose_id << ": New optimized pose, index " << optimize_pose_idx;
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

            gtsam::Pose3 opt_pose;
            gtsam::deserialize(optimized_pose->ser_pose3, opt_pose);

            m_logger_msg << "opt_pose: " << opt_pose;
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

            // update last keyframe pose
            m_last_optimize_index = optimize_pose_idx;
            m_last_optimize_pose = opt_pose;
            gtsam::Pose3 last_keyframe_pose(m_last_optimize_pose);
            for (auto iter_rel = m_relative_motion.begin() + m_last_optimize_index;
                 iter_rel != m_relative_motion.end(); ++iter_rel) {
                last_keyframe_pose = m_last_keyframe_pose.compose(*iter_rel);
            }
            m_last_keyframe_pose = last_keyframe_pose;
            m_logger_msg << "Robot " << optimized_pose_id << ": last keyframe pose updated.";
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        } else {
            m_logger_msg << "Robot " << optimized_pose_id << ": Something was wrong ...." << "\n" <<
                         "Old optimize pose index: " << m_last_optimize_index << "\n" <<
                         "New optimize pose ID: "    << optimized_pose_id     << "\n" <<
                         "New optimize pose index: " << optimize_pose_idx ;
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        }
    }
}

void OdometryExternal::laserCallback(const sensor_msgs::LaserScanConstPtr& scan) {
    FUNCTION_LOGGER(m_tag);

    if(m_first_laser_msg) {
        m_first_laser_msg = false;
        m_logger_msg << "Robot " << m_robot_id << ": Receive first laser msg...";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }
    m_current_laser_msg = *scan;
    if(m_is_laser_vis) {
        m_current_laser_msg.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
        m_current_laser_msg.header.stamp = ros::Time::now();
        m_laser_pub.publish(m_current_laser_msg);
    }
}

void OdometryExternal::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud) {
    FUNCTION_LOGGER(m_tag);

    m_current_pointcloud_msg = *pointcloud;
}

bool OdometryExternal::isInformative(gtsam::Pose3& relative_pose, sensor_msgs::LaserScan& current_scan) {
    FUNCTION_LOGGER(m_tag);

    double distance_between_keyframes = m_relative_motion_from_last_keyframe.range(gtsam::Pose3());
    double yaw_diff = fabs(m_last_keyframe_pose.rotation().yaw() - m_current_pose.rotation().yaw());

    // check distance condition
    if (distance_between_keyframes > m_informative_condition_distance) {
        m_logger_msg << "Robot " << m_robot_id << ": Distance from last keyframe: " << distance_between_keyframes;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        return true;
    }
    // check yaw condition
    if (yaw_diff > m_informative_condition_yaw && yaw_diff < 2 * M_PI - m_informative_condition_yaw) {
        m_logger_msg << "Robot " << m_robot_id << ": Difference in yaw from last keyframe: " << yaw_diff;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
        return true;
    }

    return false;
}

void OdometryExternal::rosLaserMesgToGtsamMatrix(sensor_msgs::LaserScan& laser_msg, gtsam::Matrix& scan) {
    FUNCTION_LOGGER(m_tag);

    float angle_max = M_2_PI;
    float angle_min = -M_2_PI;
    std::vector<float> ranges = laser_msg.ranges;
    float current_angle = laser_msg.angle_min;
    for(auto&& range : ranges) {
        if ((range > laser_msg.range_min) && (range < laser_msg.range_max)) {
            //if((current_angle >= laser_msg.angle_min) && (current_angle <= laser_msg.angle_max)) {
            if((current_angle >= angle_min) && (current_angle <= angle_max)) {
                double x = range * cos(current_angle);
                double y = range * sin(current_angle);
                scan << x << y ;
            }
        }
        current_angle += laser_msg.angle_increment;
    }
}

void OdometryExternal::broadcastCurrentPose(const gtsam::Pose3& current_pose, const std::string& frame_id, const gtsam::Pose3& current_odom_gtsam) {
    FUNCTION_LOGGER(m_tag);

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(current_pose.x(), current_pose.y(), current_pose.z()));
    tf::Quaternion q;
    q.setRPY(current_pose.rotation().roll(), current_pose.rotation().pitch(), current_pose.rotation().yaw());
    current_pose.rotation().quaternion();
    transform.setRotation(q);
    //TODO make base_footprint as a member robot_root_frame.
    //TODO use namespace
    m_current_pose_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", frame_id));

    // publish estimated path
    static int seq = 0;
    geometry_msgs::Pose ros_pose;
    tf::poseTFToMsg(transform, ros_pose);
    geometry_msgs::PoseStamped ros_pose_stamp;
    ros_pose_stamp.pose = ros_pose;
    ros_pose_stamp.header.seq = seq;
    ros_pose_stamp.header.stamp = ros::Time::now();
    ros_pose_stamp.header.frame_id = std::string(frame_id + "/estimated_path");

    static nav_msgs::Path path_msg;
    path_msg.poses.push_back(ros_pose_stamp);
    path_msg.header.seq = seq;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = std::string("world");

    m_estimated_path_pub.publish(path_msg);

    static gtsam::Pose3 integrated_odom;
    static ros::Time prev_odom_time(ros::Time::now());
    static geometry_msgs::Twist last_twist_msg;
    nav_msgs::Odometry current_odom_ros;
    current_odom_ros.header = path_msg.header;
    current_odom_ros.child_frame_id = frame_id;
    current_odom_ros.pose.pose = ros_pose;

    double dt = (ros::Time::now() - prev_odom_time).toSec();
    integrated_odom = integrated_odom.compose(current_odom_gtsam);
    if(dt > (1 / m_vel_update_rate)) {
        current_odom_ros.twist.twist.linear.x = integrated_odom.x() / dt;
        current_odom_ros.twist.twist.linear.y = integrated_odom.y() / dt;
        current_odom_ros.twist.twist.linear.z = integrated_odom.z() / dt;

        current_odom_ros.twist.twist.angular.x = integrated_odom.rotation().roll() / dt;
        current_odom_ros.twist.twist.angular.y = integrated_odom.rotation().pitch() / dt;
        current_odom_ros.twist.twist.angular.z = integrated_odom.rotation().yaw() / dt;

        //m_logger_msg << current_odom_ros.twist.twist.linear.x << ", "
        //             << current_odom_ros.twist.twist.linear.y << ", "
        //             << current_odom_ros.twist.twist.linear.z << ", "
        //             << current_odom_ros.twist.twist.angular.x << ", "
        //             << current_odom_ros.twist.twist.angular.y << ", "
        //             << current_odom_ros.twist.twist.angular.z << ", "
        //             << dt;
        //logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        integrated_odom = gtsam::Pose3();
        last_twist_msg = current_odom_ros.twist.twist;
        prev_odom_time = ros::Time::now();
    }
    else {
        current_odom_ros.twist.twist = last_twist_msg;
    }

    m_current_odom_pub.publish(current_odom_ros);

    ++seq;
}

bool OdometryExternal::isOptimizePoseTimout() {
    FUNCTION_LOGGER(m_tag);

    int index_dist = m_last_keyframe_index- m_last_optimize_index;
    if(index_dist > m_optimize_pose_index_timeout_threshold) {
        m_logger_msg << "Robot " << m_robot_id << ": Optimized pose index is " << index_dist << " indexes behind last keyframe index";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }
}

void OdometryExternal::gtsamPose3ToRosPoseMsg(gtsam::Pose3& gtsam_pose, geometry_msgs::Pose& ros_pose_msg) {
    FUNCTION_LOGGER(m_tag);

    ros_pose_msg.position.x = gtsam_pose.x();
    ros_pose_msg.position.y = gtsam_pose.y();
    ros_pose_msg.position.z = gtsam_pose.z();
    ros_pose_msg.orientation.w = gtsam_pose.rotation().quaternion().x();
    ros_pose_msg.orientation.x = gtsam_pose.rotation().quaternion().y();
    ros_pose_msg.orientation.y = gtsam_pose.rotation().quaternion().z();
    ros_pose_msg.orientation.z = gtsam_pose.rotation().quaternion().w();
}

// Ground truth from Gazebo
void OdometryExternal::gtGazeboCallback(const gazebo_msgs::ModelStatesConstPtr& gt_msg) {
    FUNCTION_LOGGER(m_tag);

    static nav_msgs::Path gt_path;
    static int seq = 0;

    // check if the Gazebo model contains this robot
    m_GT_available = false;
    std::vector<std::string> gt_names(gt_msg->name);
    int robot_gt_pose_ids = 0;
    for(auto iter_gt = gt_names.begin(); iter_gt != gt_names.end(); ++iter_gt) {
        if(iter_gt->compare(m_robot_name) == 0) {
            m_GT_available = true;
            break;
        }
        else {
            ++robot_gt_pose_ids;
        }
    }

    if (m_GT_available) {
        m_ground_truth_pose_stamped.pose = gt_msg->pose.at(robot_gt_pose_ids);
        m_ground_truth_pose_stamped.header.seq = seq;
        m_ground_truth_pose_stamped.header.frame_id = std::string(
                "Robot_" + std::string(1, m_robot_id) + "/ground_truth");
        m_ground_truth_pose_stamped.header.stamp = ros::Time::now();
        gt_path.header.seq = seq;
        gt_path.header.frame_id = std::string("world");
        gt_path.header.stamp = m_ground_truth_pose_stamped.header.stamp;
        gt_path.poses.push_back(m_ground_truth_pose_stamped);

        m_robot_ground_truth_pub.publish(gt_path);
    }
}

// Ground truth from Optitrack
void OdometryExternal::gtMoCapCallback(const geometry_msgs::PoseStampedConstPtr& gt_msg) {
    FUNCTION_LOGGER(m_tag);

    static nav_msgs::Path gt_path;
    static int seq = 0;

    m_ground_truth_pose_stamped.pose = gt_msg->pose;
    m_ground_truth_pose_stamped.header.seq = seq;
    m_ground_truth_pose_stamped.header.frame_id = std::string("Robot_" + std::string(1,m_robot_id) + "/ground_truth");
    m_ground_truth_pose_stamped.header.stamp = gt_msg->header.stamp;
    gt_path.header.seq = seq;
    gt_path.header.frame_id = std::string("world");
    gt_path.header.stamp = m_ground_truth_pose_stamped.header.stamp;
    gt_path.poses.push_back(m_ground_truth_pose_stamped);

    m_robot_ground_truth_pub.publish(gt_path);
}

OdometryExternal::~OdometryExternal() {
    FUNCTION_LOGGER(m_tag);

    m_logger_msg << "Robot " << m_robot_id << ": Destroy" << m_node_name << " node";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}
