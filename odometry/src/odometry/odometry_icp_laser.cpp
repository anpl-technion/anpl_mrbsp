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
 * @file: odometry_icp_laser.cpp
 * @brief:
 * @author: Tal Regev
 */


#include "odometry/odometry_icp_laser.h"

#include <mrbsp_msgs/LastIndexInDa.h>
#include <std_srvs/Empty.h>

#include <random>
#include <chrono>

#include <gtsam/base/serialization.h>
#include <gtsam/inference/Symbol.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>

#include <planar_icp/Saliency.h>

#include "odometry/odometry_icp_laser.h"
#include <ros/ros.h>
#include <signal.h>

#include <mrbsp_utils/gtsam_serialization.h>


using namespace MRBSP;
using namespace MRBSP::Utils;

void mySigintHandler(int sig) {
    Utils::LogTag tag = LogTag::odometry;
    FUNCTION_LOGGER(tag);

    std::stringstream logger_msg;


    logger_msg << "Shutting down " << ros::this_node::getName().substr(1) << " node";
    logMessage(info, LOG_INFO_LVL, logger_msg, tag);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    FUNCTION_LOGGER(LogTag::odometry);

    ros::init(argc, argv, "OdometryIcpLaser");
    ros::NodeHandle pnh("~");
    signal(SIGINT, mySigintHandler);

    OdometryIcpLaser odometry_icp_laser(pnh);

    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();

    ros::waitForShutdown();

    return 0;
}

void OdometryIcpLaser::loadParameter() {
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
    m_privateNodeHandle.param("record_keyframe_bagfile", m_record_keyframe_bagfile, false);
    m_privateNodeHandle.param("gt_buffer_size", m_gt_buffer_size, 100);


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

    m_logger_msg << "record_keyframe_bagfile: " << m_record_keyframe_bagfile;
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "=====================";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

}

OdometryIcpLaser::OdometryIcpLaser(const ros::NodeHandle &nh_private) :
        m_is_perceive(false),
        m_privateNodeHandle(nh_private),
        m_first_odom_msg(true),
        m_first_laser_msg(true),
        m_is_da_init(false)
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

	m_current_odom_pose = m_current_pose;
    m_last_optimize_pose = m_current_pose;
    m_last_keyframe_pose = m_current_pose;

    std::string name_space_sub = "/" + m_robot_name;
    std::string name_space_pub = "/Robot_" +  std::string(1,m_robot_id);

    m_odom_sub = m_privateNodeHandle.subscribe(name_space_sub + m_odom_topic, 1, &OdometryIcpLaser::odomCallback, this);



	m_laser_sub = m_privateNodeHandle.subscribe(name_space_sub+ m_laser_topic, 1, &OdometryIcpLaser::laserCallback, this);


    if(m_is_laser_vis) {
        m_laser_pub = m_privateNodeHandle.advertise<sensor_msgs::LaserScan>(std::string(name_space_sub + m_laser_topic + "/vis"), 1);
    }



    if(m_is_3D_vis) {
        // set keyframe type (with pointclouds)
        m_pointcloud_sub = m_privateNodeHandle.subscribe(name_space_sub + m_pointcloud_topic, 1, &OdometryIcpLaser::pointcloudCallback, this);
        m_keyframe_info_pub = m_privateNodeHandle.advertise<mrbsp_msgs::KeyframeRgbd>("/Robots/Odometry/keyframe/withPointcloud",1);

        m_logger_msg << "Robot " << m_robot_id << ": publish keyframes info with 3D scans";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    }
    else {
        // set keyframe type here (without pointclouds)
        m_keyframe_info_pub = m_privateNodeHandle.advertise<mrbsp_msgs::Keyframe>("/Robots/Odometry/keyframe",1);

        m_logger_msg << "Robot " << m_robot_id << ": publish keyframes info with 2D scans";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    ;
	m_optimize_pose_sub = m_privateNodeHandle.subscribe(std::string(name_space_pub + "/optimized_pose"), 1, &OdometryIcpLaser::optimizePoseCallback, this);


    m_data_source = DataSource::live;
    m_last_optimize_index = 0;
    m_last_keyframe_index = 0;

    m_GT_available = false;
    if (m_gt_source == "gazebo") {
        m_ground_truth_sub = m_privateNodeHandle.subscribe("/gazebo/model_states", 1, &OdometryIcpLaser::gtGazeboCallback, this);
        m_GT_available = true;
    }
    if (m_gt_source == "mocap") {
        m_ground_truth_sub = m_privateNodeHandle.subscribe(std::string(m_robot_name + m_gt_topic_sub), 1, &OdometryIcpLaser::gtMoCapCallback, this);
        m_GT_available = true;
    }

    m_robot_ground_truth_pub = m_privateNodeHandle.advertise<nav_msgs::Path>(std::string(name_space_pub + "/ground_truth_path"), 1);


    m_estimated_path_pub = m_privateNodeHandle.advertise<nav_msgs::Path>(std::string(name_space_pub + "/estimated_path"), 1);


    m_da_last_index_client = m_privateNodeHandle.serviceClient<mrbsp_msgs::LastIndexInDa>("/Centralize/da_check_last_index");

    std::string service_name(m_robot_name + "/check_if_perceive");
    m_da_init_check_service = m_privateNodeHandle.advertiseService(service_name, &OdometryIcpLaser::odometryInitCheck, this);


    if(m_record_keyframe_bagfile) {
        std::string m_keyframe_bag_name(Globals::folder_log + Globals::file_separator + m_robot_name + "_keyframe_bag.bag");
        m_keyframe_bag.open(m_keyframe_bag_name, rosbag::bagmode::Write);

        m_image_sub = m_privateNodeHandle.subscribe(std::string(m_robot_name + m_image_topic), 1,
                                                &OdometryIcpLaser::imageCallback, this);

        if(!m_is_3D_vis) {
            std::string pointcloud_topic;
            m_pointcloud_sub = m_privateNodeHandle.subscribe(name_space_sub + m_pointcloud_topic, 1, &OdometryIcpLaser::pointcloudCallback, this);
        }
    }

    m_current_odom_pub = m_privateNodeHandle.advertise<nav_msgs::Odometry>(std::string(name_space_pub + "/odometry"), 1);

	//m_relative_motion.push_back(gtsam::Pose3());
    m_logger_msg << "Robot " << m_robot_id << " initialized";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "Robot " << m_robot_id <<  " Initial robot position: X =  " << m_current_pose.x()
                 << ", Y = " << m_current_pose.y() << ", Z = " << m_current_pose.z();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    m_logger_msg << "Robot " << m_robot_id <<  " Initial yaw: " << m_current_pose.rotation().yaw();
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}


bool OdometryIcpLaser::odometryInitCheck(mrbsp_msgs::InitCheck::Request& req, mrbsp_msgs::InitCheck::Response& res) {
    FUNCTION_LOGGER(m_tag);

    res.init_check_answer = static_cast<unsigned char>(m_is_perceive);
    return true;
}

void OdometryIcpLaser::handleOdomData(nav_msgs::OdometryConstPtr odom_msg) {
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
    //info m_robot_id m_current_pose

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
                //warn m_robot_id, m_last_keyframe_index, da_last_index_srv.response.last_index
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
                Keyframe current_keyframe = std::make_tuple(keyframe_idx, current_time, m_current_laser_msg, m_relative_motion_from_last_keyframe, m_current_gt_pose.pose);
                m_keyframes.push_back(current_keyframe);
                break;
            }

            case DataSource::live :
            {
                m_logger_msg << "Robot " << m_robot_id << ": Publish keyframe " << m_last_keyframe_index;
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

		        //publish keyframes
                if(!m_is_3D_vis) {
                    mrbsp_msgs::Keyframe keyframe;
                    keyframe.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
                    keyframe.header.seq = m_last_keyframe_index;
                    keyframe.header.stamp = odom_msg->header.stamp;
                    keyframe.ser_symbol = gtsam::serialize(gtsam::Symbol(m_robot_id, m_last_keyframe_index));
                    keyframe.laser_scan = m_current_laser_msg;
                    keyframe.ground_truth_pose = m_current_gt_pose.pose;

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
                    mrbsp_msgs::KeyframeRgbd keyframe_3d;
                    keyframe_3d.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
                    keyframe_3d.header.seq = m_last_keyframe_index;
                    keyframe_3d.header.stamp = odom_msg->header.stamp;
                    keyframe_3d.ser_symbol = gtsam::serialize(gtsam::Symbol(m_robot_id, m_last_keyframe_index));
                    keyframe_3d.laser_scan = m_current_laser_msg;
                    keyframe_3d.ground_truth_pose = m_current_gt_pose.pose;

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
                    m_logger_msg << "optimize Pose Timout";
                    //logMessage(warn, LOG_INFO_LVL, m_logger_msg, m_tag);

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

void OdometryIcpLaser::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
    FUNCTION_LOGGER(m_tag);

    // Check if the da node is initialized
    if(!m_is_da_init) {
        ros::ServiceClient da_init_check_client = m_privateNodeHandle.serviceClient<mrbsp_msgs::InitCheck>("/Centralize/da_init_check");
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
    //handleOdomData(odom);
}

void OdometryIcpLaser::rosOdomeMsgToGtsam(const nav_msgs::OdometryConstPtr& odom_msg) {
    FUNCTION_LOGGER(m_tag);

    geometry_msgs::Pose curr_odom_ros = odom_msg->pose.pose;
    geometry_msgs::Quaternion curr_quat_ros = curr_odom_ros.orientation;
    geometry_msgs::Point curr_point_ros = curr_odom_ros.position;

    gtsam::Rot3 curr_rot_gtsam = gtsam::Rot3::quaternion(curr_quat_ros.w, curr_quat_ros.x, curr_quat_ros.y, curr_quat_ros.z);
    gtsam::Point3 curr_point_gtsam = gtsam::Point3(curr_point_ros.x, curr_point_ros.y, curr_point_ros.z);
    m_current_odom_pose = gtsam::Pose3(curr_rot_gtsam, curr_point_gtsam);
}

void OdometryIcpLaser::optimizePoseCallback(const mrbsp_msgs::GtsamSerPose3ConstPtr& optimized_pose) {
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

void OdometryIcpLaser::laserCallback(const sensor_msgs::LaserScanConstPtr& scan) {
    FUNCTION_LOGGER(m_tag);

    // Debag: releated to quad bagfiles...
    static int first_msg_seq;

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
                m_logger_msg << "Robot " << m_robot_id << ": Wait for 5 sec...";
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
                ros::Duration(5.0).sleep(); // sleep for 5 seconds
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
            m_logger_msg << "Robot " << m_robot_id << ": Wait for 5 sec...";
            logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
            ros::Duration(5.0).sleep(); // sleep for 5 seconds
        }

        return;
    }

    if(m_first_laser_msg) {
        m_first_laser_msg = false;
        m_logger_msg << "Robot " << m_robot_id << ": Receive first laser msg...";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

        // Debag: releated to quad bagfiles...
        first_msg_seq = scan->header.seq;
    }

    if(m_is_laser_vis) {
        m_current_laser_msg.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
        m_current_laser_msg.header.stamp = ros::Time::now();
        m_laser_pub.publish(m_current_laser_msg);
    }

    // Debag: releated to quad bagfiles...
    if(scan->header.seq > first_msg_seq + 100) {
        handleLaserData(scan);
    }
}

void OdometryIcpLaser::handleLaserData(const sensor_msgs::LaserScanConstPtr& laser_msg) {
    FUNCTION_LOGGER(m_tag);

    m_is_perceive = true;

    // odometry calculator
    std::vector<gtsam::Point2> previous_scan = m_current_scan;
    m_current_laser_msg = *laser_msg;
    rosLaserToCsmGtsam(m_current_laser_msg, m_current_scan);
    gtsam::Pose3 initial_guess;
    gtsam::Pose3 current_odom;
    double nn_matching_threshold = 0.0;
    if(performCsmIcp(m_current_scan, previous_scan, initial_guess,current_odom, nn_matching_threshold)) {

    }
    else {
        m_logger_msg << "Robot " << m_robot_id << ": ICP failed!, set odometry to (0,0,0)";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    // update robot pose
    m_relative_motion_from_last_keyframe = m_relative_motion_from_last_keyframe.compose(current_odom); // update last object in relative motion vector
    m_current_pose = m_last_keyframe_pose.compose(m_relative_motion_from_last_keyframe);
    //info m_robot_id, m_current_pose

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
                double current_time = m_current_laser_msg.header.stamp.toSec() - m_initial_time;
                Keyframe current_keyframe = std::make_tuple(keyframe_idx, current_time, m_current_laser_msg, m_relative_motion_from_last_keyframe, m_current_gt_pose.pose);
                m_keyframes.push_back(current_keyframe);
                break;
            }

            case DataSource::live :
            {
                m_logger_msg << "Robot " << m_robot_id << ": Publish keyframe " << m_last_keyframe_index;
                logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

                //publish keyframes
                if(!m_is_3D_vis) {
                    mrbsp_msgs::Keyframe keyframe;
                    keyframe.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
                    keyframe.header.seq = m_last_keyframe_index;
                    keyframe.header.stamp = laser_msg->header.stamp;
                    keyframe.ser_symbol = gtsam::serialize(gtsam::Symbol(m_robot_id, m_last_keyframe_index));
                    keyframe.laser_scan = m_current_laser_msg;
                    keyframe.ground_truth_pose = m_current_gt_pose.pose;

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
                    mrbsp_msgs::KeyframeRgbd keyframe_3d;
                    keyframe_3d.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
                    keyframe_3d.header.seq = m_last_keyframe_index;
                    keyframe_3d.header.stamp = ros::Time::now();
                    keyframe_3d.ser_symbol = gtsam::serialize(gtsam::Symbol(m_robot_id, m_last_keyframe_index));
                    keyframe_3d.laser_scan = m_current_laser_msg;

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

        // save keyframe raw data to bagfile for later inspection
        if(m_record_keyframe_bagfile) {
            ros::Time time = ros::Time::now();
            std_msgs::String index_msg;
            index_msg.data = std::string(std::string(1, m_robot_id) + std::to_string(m_last_keyframe_index));
            m_keyframe_bag.write("index", time, index_msg);

            geometry_msgs::PoseStamped estimated_pose_msg;
            estimated_pose_msg.header.seq = m_last_keyframe_index;
            estimated_pose_msg.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
            estimated_pose_msg.header.stamp = time;
            geometry_msgs::Pose est_pose_msg;
            gtsamPose3ToRosPoseMsg(m_current_pose, est_pose_msg);
            estimated_pose_msg.pose = est_pose_msg;
            m_keyframe_bag.write("estimated_pose", time, estimated_pose_msg);

            m_current_gt_pose.header.stamp = time;
            m_keyframe_bag.write("ground_truth_pose", time, m_current_gt_pose);

            m_current_laser_msg.header.stamp = time;
            m_keyframe_bag.write("scan", time, m_current_laser_msg);

            m_current_pointcloud_msg.header.stamp = time;
            m_keyframe_bag.write("pointcloud", time, m_current_pointcloud_msg);

            m_current_img.header.stamp = time;
            m_keyframe_bag.write("image", time, m_current_img);

            geometry_msgs::PoseStamped odometry_msg;
            odometry_msg.header.seq = m_last_keyframe_index;
            odometry_msg.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id));
            odometry_msg.header.stamp = time;
            geometry_msgs::Pose odom_msg;
            gtsamPose3ToRosPoseMsg(current_odom, odom_msg);
            odometry_msg.pose = odom_msg;
            m_keyframe_bag.write("odometry", time, odometry_msg);

        }

        m_last_keyframe_index++;
    }
}

void OdometryIcpLaser::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud) {
    FUNCTION_LOGGER(m_tag);

    m_current_pointcloud_msg = *pointcloud;
}

bool OdometryIcpLaser::isInformative(gtsam::Pose3& relative_pose, sensor_msgs::LaserScan& current_scan) {
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

void OdometryIcpLaser::rosLaserMesgToGtsamMatrix(sensor_msgs::LaserScan& laser_msg, gtsam::Matrix& scan) {
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

void OdometryIcpLaser::broadcastCurrentPose(const gtsam::Pose3& current_pose, const std::string& frame_id, const gtsam::Pose3& current_odom_gtsam) {
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

    if (path_msg.poses.empty()) {
        path_msg.poses.push_back(ros_pose_stamp);
        //path_msg.header.seq = seq;
        path_msg.header.stamp = ros_pose_stamp.header.stamp;
        path_msg.header.frame_id = std::string("world");
        m_estimated_path_pub.publish(path_msg);

    }  else if  (m_estimated_path_pub.getNumSubscribers() > 0 ) { // publish est. path only if someone interested to this topic and robot moved enough according to informative condition

        geometry_msgs::PoseStamped prev_est_pose = path_msg.poses.back();
        gtsam::Point3 ds(ros_pose_stamp.pose.position.x-prev_est_pose.pose.position.x,
                         ros_pose_stamp.pose.position.y-prev_est_pose.pose.position.y,
                         ros_pose_stamp.pose.position.z-prev_est_pose.pose.position.z);
        gtsam::Quaternion q1(ros_pose_stamp.pose.orientation.x, ros_pose_stamp.pose.orientation.y, ros_pose_stamp.pose.orientation.z, ros_pose_stamp.pose.orientation.w);
        gtsam::Quaternion q2(prev_est_pose.pose.orientation.x, prev_est_pose.pose.orientation.y, prev_est_pose.pose.orientation.z, prev_est_pose.pose.orientation.w);

        if (ds.norm() > m_informative_condition_distance ||
            q1.angularDistance(q2) > m_informative_condition_yaw) {
            path_msg.poses.push_back(ros_pose_stamp);
            //path_msg.header.seq = seq;
            path_msg.header.stamp = ros_pose_stamp.header.stamp;
            path_msg.header.frame_id = std::string("world");
            m_estimated_path_pub.publish(path_msg);
        }
    }


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
        current_odom_ros.twist.twist.linear.x = current_odom_gtsam.x() / dt;
        current_odom_ros.twist.twist.linear.y = current_odom_gtsam.y() / dt;
        current_odom_ros.twist.twist.linear.z = current_odom_gtsam.z() / dt;

        current_odom_ros.twist.twist.angular.x = current_odom_gtsam.rotation().roll() / dt;
        current_odom_ros.twist.twist.angular.y = current_odom_gtsam.rotation().pitch() / dt;
        current_odom_ros.twist.twist.angular.z = current_odom_gtsam.rotation().yaw() / dt;

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

bool OdometryIcpLaser::isOptimizePoseTimout() {
    FUNCTION_LOGGER(m_tag);

    int index_dist = m_last_keyframe_index- m_last_optimize_index;
    if(index_dist > m_optimize_pose_index_timeout_threshold) {
        m_logger_msg << "Robot " << m_robot_id << ": Optimized pose index is " << index_dist << " indexes behind last keyframe index";
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }
}

void OdometryIcpLaser::gtsamPose3ToRosPoseMsg(gtsam::Pose3& gtsam_pose, geometry_msgs::Pose& ros_pose_msg) {
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
void OdometryIcpLaser::gtGazeboCallback(const gazebo_msgs::ModelStatesConstPtr& gt_msg) {
    FUNCTION_LOGGER(m_tag);

    nav_msgs::Path gt_path;
    static std::list<geometry_msgs::PoseStamped> gt_poses_list;
    static int seq = 0;
    static tf::Transform tf_p_prev = tf::Transform::getIdentity();

    // check if the Gazebo model contains this robot
    m_GT_available = false;
    std::vector<std::string> gt_names(gt_msg->name);
    int robot_gt_pose_ids = 0;
    for (auto iter_gt = gt_names.begin(); iter_gt != gt_names.end(); ++iter_gt) {
        if (iter_gt->compare(m_robot_name) == 0) {
            m_GT_available = true;
            break;
        } else {
            ++robot_gt_pose_ids;
        }
    }

    if (m_GT_available && m_robot_ground_truth_pub.getNumSubscribers() > 0) {
        ros::Time stamp = ros::Time::now();
        m_current_gt_pose.pose = gt_msg->pose.at(robot_gt_pose_ids);
        m_current_gt_pose.header.seq = seq;
        m_current_gt_pose.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id) + "/ground_truth");
        m_current_gt_pose.header.stamp = stamp;
        geometry_msgs::PoseStamped pose_stamped;

        tf::Transform tf_p_curr, tf_p_delta;
        pose_stamped.pose = m_current_gt_pose.pose;
        tf::poseMsgToTF(pose_stamped.pose, tf_p_curr);

        tf_p_delta = tf_p_prev.inverse() * tf_p_curr;

        if (tf_p_delta.getOrigin().length() > m_informative_condition_distance / 5 ||
            tf_p_delta.getRotation().getAngle() > m_informative_condition_yaw / 3) {

            pose_stamped.header.seq = seq;
            pose_stamped.header.frame_id = std::string("Robot_" + std::string(1, m_robot_id) + "/ground_truth");
            pose_stamped.header.stamp = stamp;
            gt_path.header.seq = seq;
            gt_path.header.frame_id = std::string("world");
            gt_path.header.stamp = pose_stamped.header.stamp;

            if (gt_poses_list.size() == m_gt_buffer_size) {
                gt_poses_list.pop_front();
            }
            gt_poses_list.push_back(pose_stamped);
            gt_path.poses.assign(gt_poses_list.begin(), gt_poses_list.end());

            tf_p_prev = tf_p_curr;
            m_robot_ground_truth_pub.publish(gt_path);
        }
    }
}



// Ground truth from Optitrack
void OdometryIcpLaser::gtMoCapCallback(const geometry_msgs::PoseStampedConstPtr& gt_msg) {
    FUNCTION_LOGGER(m_tag);

    nav_msgs::Path gt_path;
    static std::list<geometry_msgs::PoseStamped> gt_poses_list;
    static int seq = 0;
    static tf::Transform tf_p_prev = tf::Transform::getIdentity();

    m_current_gt_pose = *gt_msg;
    geometry_msgs::PoseStamped pose_stamped;

    if (m_robot_ground_truth_pub.getNumSubscribers()>0) {
        tf::Transform tf_p_curr, tf_p_delta;
        pose_stamped.pose = gt_msg->pose;
        tf::poseMsgToTF(pose_stamped.pose, tf_p_curr);

        tf_p_delta = tf_p_prev.inverse()*tf_p_curr;

        if (tf_p_delta.getOrigin().length() > m_informative_condition_distance/5 || tf_p_delta.getRotation().getAngle() > m_informative_condition_yaw/3) {

            pose_stamped.header.seq = seq;
            pose_stamped.header.frame_id = std::string("Robot_" + std::string(1,m_robot_id) + "/ground_truth");
            pose_stamped.header.stamp = gt_msg->header.stamp;
            gt_path.header.seq = seq;
            gt_path.header.frame_id = std::string("world");
            gt_path.header.stamp = pose_stamped.header.stamp;

            if (gt_poses_list.size() == m_gt_buffer_size) {
                gt_poses_list.pop_front();
            }
            gt_poses_list.push_back(pose_stamped);
            gt_path.poses.assign(gt_poses_list.begin(), gt_poses_list.end());

            tf_p_prev = tf_p_curr;
            m_robot_ground_truth_pub.publish(gt_path);
        }
    }
}


void OdometryIcpLaser::rosLaserToCsmGtsam(sensor_msgs::LaserScan& laser_msg, std::vector<gtsam::Point2>& current_scan) {
    FUNCTION_LOGGER(m_tag);

    current_scan.clear();
    float angle_max = M_PI_2;  // M_PI_2 = (pi/2) =  90 [deg]
    float angle_min = -M_PI_2; // -90 [deg]
    float range_max = 10;
    std::vector<float> ranges = laser_msg.ranges;
    float current_angle = laser_msg.angle_min;
    for(auto&& range : ranges) {
        //if ((range > laser_msg.range_min) && (range < laser_msg.range_max)) {
        if ((range > laser_msg.range_min) && (range < range_max)) {
            if((current_angle >= laser_msg.angle_min) && (current_angle <= laser_msg.angle_max)) {
                //if((current_angle >= angle_min) && (current_angle <= angle_max)) {
                double x = range * cos(current_angle);
                double y = range * sin(current_angle);
                gtsam::Point2 scan(x, y);
                current_scan.push_back(scan);
            }
        }
        current_angle += laser_msg.angle_increment;
    }
}

bool OdometryIcpLaser::performCsmIcp(std::vector<gtsam::Point2>& current_measurement,
                                       std::vector<gtsam::Point2>& prev_measurement, const gtsam::Pose3& initial_guess,
                                       gtsam::Pose3& icp_transformation, const double nn_matching_threshold) {
    FUNCTION_LOGGER(m_tag);

    if(current_measurement.empty() || prev_measurement.empty()) {
        return false;
    }

    /// ICP thresholds
    const int maxIters = 50;
    const double stoppingThresh = 0.000001;
    const double inlierThreshSq = 0.5;

    gtsam::Pose2 initial_guess_pose2(initial_guess.x(), initial_guess.y(), initial_guess.rotation().yaw());

    ReturnValue return_value = laserScanICP(prev_measurement, current_measurement, initial_guess_pose2, maxIters, stoppingThresh, inlierThreshSq);

    icp_transformation = gtsam::Pose3(return_value.getDelta());


    if(m_is_print_icp_results) {
        m_logger_msg << "PLANNAR_ICP:\n"
                     << "delta: X = " << return_value.getDelta().x() << ", Y = " << return_value.getDelta().y()
                     << ", Yaw = " << return_value.getDelta().rotation().theta() << "\n"
                     << "Number of iterations: " << return_value.getNumIters() << "\n"
                     << "Stoping threshold: " << return_value.getStopping() << "\n"
                     << "Status score: " << return_value.getStatus();
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);

    }

    if(return_value.getStatus() > 0 && return_value.getStatus() < nn_matching_threshold) {
        m_logger_msg << "PLANNAR_ICP:\n"
                     << "ICP succeeded but below nn matching threshold." << "\n"
                     << "score: " << return_value.getStatus()
                     << ", nn matching threshold: " << nn_matching_threshold;
        logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
    }

    return return_value.getStatus() > nn_matching_threshold;
}

void OdometryIcpLaser::imageCallback(const sensor_msgs::ImageConstPtr& image) {
    FUNCTION_LOGGER(m_tag);

    m_current_img = *image;
}

OdometryIcpLaser::~OdometryIcpLaser() {
    FUNCTION_LOGGER(m_tag);

    m_keyframe_bag.close();

    m_logger_msg << "Robot " << m_robot_id << ": destructor " << m_node_name << " node";
    logMessage(info, LOG_INFO_LVL, m_logger_msg, m_tag);
}
