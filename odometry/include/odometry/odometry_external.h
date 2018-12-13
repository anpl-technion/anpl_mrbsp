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
 * @file: odometry_external.h
 * @brief:
 * @author: Tal Regev
 */

#ifndef ODOMETRY_EXTERNAL_H
#define ODOMETRY_EXTERNAL_H

#include <mrbsp_utils/mrbsp_types.h>
#include "mrbsp_utils/mrbsp_utils.h"

#include <mrbsp_msgs/Keyframe.h>
#include <mrbsp_msgs/KeyframeRgbd.h>
#include <mrbsp_msgs/GtsamSerPose3.h>
#include <mrbsp_msgs/InitCheck.h>

#include <string>
#include <tuple>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>

#include <spdlog/spdlog.h>
#include <mrbsp_utils/function_logger.h>


//typedef std::tuple<std::string, double, sensor_msgs::LaserScan, gtsam::Pose3> Keyframe;

namespace MRBSP {


    /*
     *
     */
    class OdometryExternal {
    public:

        /**
         *
         * @param nh_private
         */
        OdometryExternal(const ros::NodeHandle &nh_private);

	    virtual ~OdometryExternal();

        /**
         * Read information from bag file
         * @param path_to_bagfile - full path to bagfile
         * @param odom_topic - odom topic name
         * @param laser_topic - laser topic name
         */
        std::vector<Utils::Keyframe> getDataFromBag(const std::string& path_to_bagfile, const std::string& odom_topic, const std::string& laser_topic);

    private:

        void loadParameter();

        /// ros service to check if the DA node has been initialized
        ros::ServiceServer m_preceive_init_check_service;

        /// boolean set to true after the node start t opublish keyframes
        bool m_is_perceive;

        /**
         *
         * @param req
         * @param res
         * @return
         */
        bool odometryInitCheck(mrbsp_msgs::InitCheck::Request& req, mrbsp_msgs::InitCheck::Response& res);

        /// pointer to logger
        std::shared_ptr<spdlog::logger> m_p_odom_logger;

        /// string for logger msgs
        std::stringstream m_logger_msg;

        /// path to current run folder
        std::string m_path_to_log_folder;

        /// data sources (live, bagfile)
        enum class DataSource {
            live,
            bagfile,
        } m_data_source;

        /// robot id for factor graph
        char m_robot_id;

        /// robot name in ground truth source
        std::string m_robot_name;

        /// rosbag initial time, in seconds
        double m_initial_time;

        /// container for keyframe information
        std::vector<Utils::Keyframe> m_keyframes;

        /// ros node
        ros::NodeHandle m_privateNodeHandle;

        /// bool to check if received first odom msg
        bool m_first_odom_msg;

        /// bool to check if received first laser msg
        bool m_first_laser_msg;

        /// bool to check if da node is initialized
        bool m_is_da_init;

        /// ground truth subscriber
        ros::Subscriber m_ground_truth_sub;

        /// flag to mark availability of ground truth data
        bool m_GT_available;

        /// current ground truth pose with timestamp
        geometry_msgs::PoseStamped m_ground_truth_pose_stamped;

        /// ground truth path msg publisher (for rviz)
        ros::Publisher m_robot_ground_truth_pub;

        /**
         *
         * @param gt_msg
         */
        void gtGazeboCallback(const gazebo_msgs::ModelStatesConstPtr& gt_msg);
        void gtMoCapCallback(const geometry_msgs::PoseStampedConstPtr& gt_msg);


        // odometry calculator

        /// ros subscriber for odometry msgs
        ros::Subscriber m_odom_sub;

        /// current odom msg as gtsam Pose3 object
        gtsam::Pose3 m_current_odom_pose;

        /**
        * Callback function to catch ros odometry messages
        * @param odom - pointer to ros odometry message
        */
        void odomCallback(const nav_msgs::OdometryConstPtr& odom);

        /**
         * Use new odometry msg to calculate odometry
         * @param odom_msg - pointer for ros odometry message
         */
        void handleOdomData(nav_msgs::OdometryConstPtr odom_msg);

        /**
         * Cast ros odometry msg to gtsam Pose3 object
         * @param odom_msg - pointer to ros odometry message
         */
        void rosOdomeMsgToGtsam(const nav_msgs::OdometryConstPtr& odom_msg);


        // update robot pose

        /// relative motion from the last keyframe
        gtsam::Pose3 m_relative_motion_from_last_keyframe;

        /// vector to store relative motion between keyframes
        std::vector<gtsam::Pose3> m_relative_motion;

        /// ros subscriber for last optimize pose of the robot
        ros::Subscriber m_optimize_pose_sub;

        /// last optimize robot pose from the belief
        gtsam::Pose3 m_last_optimize_pose;

        /// last optimize robot pose index
        unsigned int m_last_optimize_index;

        /// last keyframe pose relative to the last optimize pose
        gtsam::Pose3 m_last_keyframe_pose;

        /// last keyframe pose index
        unsigned int m_last_keyframe_index;

        /// current estimation of the robot pose
        gtsam::Pose3 m_current_pose;

        /// tf broadcaster for the robot current pose
        //tf2_ros::TransformBroadcaster m_current_pose_pub;

        /// ros tf broadcaster for robot pose in global frame
        tf::TransformBroadcaster m_current_pose_tf_broadcaster;

        /// ros publisher for the robot estimated path
        ros::Publisher m_estimated_path_pub;

        /**
         *
         * @param current_pose
         * @param frame_id
         */
        void broadcastCurrentPose(const gtsam::Pose3& current_pose, const std::string& frame_id, const gtsam::Pose3& current_odom_gtsam);

        /**
        * Callback function to catch robot optimize pose
        * @param optimize_pose - pointer to ros pose message
        */
        void optimizePoseCallback(const mrbsp_msgs::GtsamSerPose3ConstPtr& optimized_pose);


        // keyframes information

        /// ros subscriber for laser msgs
        ros::Subscriber m_laser_sub;

        /// current ros laser message
        sensor_msgs::LaserScan m_current_laser_msg;

        /// Whether to publish laser scan for visualization
        bool m_is_laser_vis;

        /// laser msg publisher (for visualization when working with rosbags)
        ros::Publisher m_laser_pub;

        /// Whether to save pointclouds for 3D visualization
        bool m_is_3D_vis;

        /// Ros subscriber for pointclouds msgs
        ros::Subscriber m_pointcloud_sub;

        /// ros publisher for keyframe information
        ros::Publisher m_keyframe_info_pub;

        /// current ros pointcloud msg
        sensor_msgs::PointCloud2 m_current_pointcloud_msg;

        /**
        * Function to cast gtsam Pose3 object to ros Pose msg
        * @param gtsam_pose
        * @param ros_pose_msg
        */
        void gtsamPose3ToRosPoseMsg(gtsam::Pose3& gtsam_pose, geometry_msgs::Pose& ros_pose_msg);

        /**
         * Callback function to catch ros laser messages
         * @param scan - pointer to ros laser message
         */
        void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);

        /**
         * Callback function to store ros pointcloud messages
         * @param pointcloud - pointer to ros pointcloud2 message
         */
        void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud);

        /// distance condition for new keyframe
        double m_informative_condition_distance;

        /// angle condition for new keyframe
        double m_informative_condition_yaw;

        /**
         * Check whether the current laser scan is informative
         * @param relative_pose - the relative motion from the last keyframe
         * @param current_scan - pointer to ros laser messages
         * @return true if the scan is informative
         */
        bool isInformative(gtsam::Pose3& relative_pose, sensor_msgs::LaserScan& current_scan);

        /**
         *
         * @param ros_msg
         * @param scan
         */
        void rosLaserMesgToGtsamMatrix(sensor_msgs::LaserScan& laser_msg, gtsam::Matrix& scan);

        ros::ServiceClient m_da_last_index_client;

        /// threshold for slow optimized pose update.
        /// how many keyframes pose haven't been optimized at least one time
        int m_optimize_pose_index_timeout_threshold;

        /// publisher for optimized pose timeout
        ros::Publisher m_optimized_timout_pub;

        /**
         * check if the gap between last optimized pose index and last keyframe index is higher than the timout threshold.
         * @return true if the threshold been exceed.
         */
        bool isOptimizePoseTimout();

        /// ros publisher for current odometry msg
        ros::Publisher m_current_odom_pub;

        /// velocity publish rate (for more accurate velocity)
        double m_vel_update_rate;


        Utils::LogTag  m_tag;
        std::string m_odom_topic;
        std::string m_laser_topic;
        std::string m_image_topic;
        std::string m_pointcloud_topic;

        std::string m_gt_topic_sub;
        std::string m_gt_source;

        std::string m_node_name;
        bool        m_is_odom_noised;
        double      m_error_dynamic_percentage;
        bool        m_is_print_icp_results;
        int         m_gt_buffer_size;

    };
}

#endif // ODOMETRY_EXTERNAL_H
