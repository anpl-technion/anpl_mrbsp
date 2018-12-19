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
 * @file: da_icp_laser.h
 * @brief:
 * @author: Tal Regev
 */

#ifndef DA_ICP_LASER_H
#define DA_ICP_LASER_H

#include <mrbsp_utils/mrbsp_types.h>

#include <mrbsp_msgs/Keyframe.h>
#include <mrbsp_msgs/KeyframeRgbd.h>
#include <mrbsp_msgs/GtsamSerValues.h>
#include <mrbsp_msgs/InitCheck.h>
#include <mrbsp_msgs/LastIndexInDa.h>

#include <ros/ros.h>


#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/Value.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point2.h>
#include <mrbsp_utils/function_logger.h>

namespace MRBSP {

    using namespace MRBSP::Utils;

    class DaIcpLaser {
    public:

	DaIcpLaser(const ros::NodeHandle &nh_private);

        /**
         *
         */
        virtual ~DaIcpLaser();

        /**
         *
         * @param robots_keyframes
         * @return
         */
        std::vector<NewFactorsAndValues> getKeyframes(std::vector<std::vector<Keyframe>> &robots_keyframes);

        /**
         *
         * @param updated_values
         */
        void updateMemory(gtsam::Values& updated_values);

        /**
         *
         * @return map data for octomat construction
         */
        std::map<std::string, MapData> getMapData();

    private:
        

        /// string for logger msgs
        std::stringstream m_logger_msg;

        /// path to current run folder
        std::string m_path_to_log_folder;

        /// data source type, true if rosbag, false if stream
        bool m_is_rosbag;

        /// Whether to save pointclouds for 3D visualization
        bool m_is_3D_vis;

        /// belief object for incremental optimization with rosbag data source
        //RobotBeliefIsam2 m_belief;

        /// ros node
        ros::NodeHandle m_privateNodeHandle;

        /// flag for loop closure detecting
        bool m_perform_lc;

        /// flag for multi robot factors detecting
        bool m_perform_mr;

        /// flag whether to calculate loop closure in parallel process
        bool m_parallel_lc;

        /// flag whether to calculate multi robot factors in parallel process
        bool m_parallel_mr;

        /// make sure that DA is perfect by comparing it to the ground-truth
        bool m_enforcing_perfect_DA;

        /// return object for rosbag data source
        std::vector<NewFactorsAndValues> m_all_factors_and_values;

        /// ros service to check if the DA node has been initialized
        ros::ServiceServer m_da_init_check_service;

        /// boolean set to true after the constructor end the initialization
        bool m_is_init;

        /**
         *
         * @param res
         * @return
         */
        bool daInitCheck(mrbsp_msgs::InitCheck::Request& req, mrbsp_msgs::InitCheck::Response& res);

        // data extraction

        /// map of all keyframes received - memory object
        AllKeyframes m_all_keyframes;

        /// factors and values added from last keyframe
        NewFactorsAndValues m_last_factors_and_values;

        /// ros subscriber for keyframe initialization messages without pointcloud
        ros::Subscriber m_keyframe_init_sub;

        /// ros subscriber for keyframe initialization messages with pointcloud
        ros::Subscriber m_keyframe_init_rgbd_sub;

        /// ros subscriber updated values from the belief node
        ros::Subscriber m_updated_values_sub;

        /// ros publisher for parallel computation of loop closure
        ros::Publisher m_loop_closure_pub;

        /// ros subscriber for parallel computation of loop closure
        ros::Subscriber m_loop_closure_sub;

        /// ros publisher for parallel computation of multi-robot factors
        ros::Publisher m_multirobot_pub;

        /// ros subscriber for parallel computation of multi-robot factors
        ros::Subscriber m_multirobot_sub;

        /// current sensor_msg pointcloud2
        sensor_msgs::PointCloud2 m_current_pointcloud_msg;

        /**
        *
        *  @param keyframe_init_msg
        */
        void KeyframeCallback(const mrbsp_msgs::KeyframeConstPtr& keyframe_init_msg);

        /**
        *
        *  @param keyframe_init_msg
        */
        void KeyframeRgbdCallback(const mrbsp_msgs::KeyframeRgbdConstPtr& keyframe_init_rgbd_msg);

        /**
         *
         * @param keyframe_init
         */
        void matchInSlidingWindow(Keyframe &keyframe_init);

        /**
         *
         * @param keyframe_init_msg
         */
        void loopClosureCallback(const mrbsp_msgs::KeyframeConstPtr& keyframe_init_msg);

        /**
         *
         * @param keyframe_init
         */
        void detectLoopClosures(Keyframe &keyframe_init);

        /**
         *
         * @param keyframe_init_msg
         */
        void multiRobotCallback(const mrbsp_msgs::KeyframeConstPtr& keyframe_init_msg);

        /**
         *
         * @param keyframe_init
         */
        void detectMultiRobotFactors(Keyframe &keyframe_init);

        /**
         * get the updated values after belief optimization
         * @param serialized_values - string msg with serialized gtsam::values object
         */

        /**
         *
         * @param keyframe_init_msg
         * @param keyframe_init
         */
        void KeyframeMsgParser(const mrbsp_msgs::KeyframeConstPtr& keyframe_init_msg, Keyframe &keyframe_init);

        /// ros service to check if the DA node has been initialized
        ros::ServiceServer m_da_check_last_index;

        /**
         *
         * @param res
         * @return
         */
        bool daCheckLastIndex(mrbsp_msgs::LastIndexInDa::Request& req, mrbsp_msgs::LastIndexInDa::Response& res);

        /**
         *
         * @param graph
         * @param values
         */
        void publishDataToBelief(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values);

        /**
         *
         * @param map_data
         */
        void publishDataToMap(const sensor_msgs::LaserScan& scan, const gtsam::Values& values);

        /**
         *
         * @param pointcloud
         * @param values
         */
        void publishDataToMap3D(const sensor_msgs::PointCloud2& pointcloud, const gtsam::Values& values);

        /**
         *
         * @param serialized_values_msg
         */
        void updateMemoryCallback(const mrbsp_msgs::GtsamSerValuesConstPtr& serialized_values_msg);

        /// min index difference to search for loop closures
        double m_lc_min_idx_range;

        /// min spatial search for loop closures
        double m_lc_spatial_search_range;

        /// min spatial search for multi robots factors
        double m_mr_spatial_search_range;

        /// icp maximum iterations
        int m_icp_max_iters;

        /// icp stopping threshold
        double m_icp_stopping_thresh; // [m]

        /// icp inlier threshold sequence
        double m_icp_inlier_thresh_sq; // [m]

        /// threshold for nearest neighbors matching percentage for sliding window icp matching
        double m_icp_window_nn_pct_thresh; // [%]

        /// threshold for nearest neighbors matching percentage for loop closures icp matching
        double m_icp_lc_nn_pct_thresh; // [%]

        /// threshold for nearest neighbors matching percentage for multi robot icp matching
        double m_icp_mr_nn_pct_thresh; // [%]

        /// if -1: fixed noise, if > 0: noise is %(m_error_dynamic_percentage) of the distance
        double m_error_dynamic_percentage;

        /**
         * Function to get and store ros parameters
         */
        void loadParameter();

        /**
         * Cast ros laser message to vector of gtsam::Point2 for csm icp
         */
        void rosLaserToCsmGtsam(sensor_msgs::LaserScan &laser_msg, std::vector<gtsam::Point2> &current_scan);

        /**
         * Write scan to file for further review
         * @param scan - laser scan for CSM icp
         * @param index - keyframe index for the scan
         */
        void saveScan(std::vector<gtsam::Point2> &scan, std::string &index);

        /**
         *
         * @param current_measurement
         * @param prev_measurement
         * @param initial_guess
         * @return true if icp converged
         */
        bool performCsmIcp(std::vector<gtsam::Point2> &current_measurement,
                           std::vector<gtsam::Point2> &prev_measurement, const gtsam::Pose3 &initial_guess,
                           gtsam::Pose3& icp_transformation, const double nn_matching_threshold, std::string indexes = std::string());

        bool performCsmIcpLc(std::vector<gtsam::Point2> &current_measurement,
                           std::vector<gtsam::Point2> &prev_measurement, const gtsam::Pose3 &initial_guess,
                           gtsam::Pose3& icp_transformation, const double nn_matching_threshold, std::string indexes = std::string());

        bool performCsmIcpMr(std::vector<gtsam::Point2> &current_measurement,
                           std::vector<gtsam::Point2> &prev_measurement, const gtsam::Pose3 &initial_guess,
                           gtsam::Pose3& icp_transformation, const double nn_matching_threshold, std::string indexes = std::string());

        // gtsam factors

        /// prior noise model
        gtsam::noiseModel::Diagonal::shared_ptr m_prior_model;

        /// icp noise model
        gtsam::noiseModel::Diagonal::shared_ptr m_icp_model;

        /// odom noise model
        gtsam::noiseModel::Diagonal::shared_ptr m_odom_model;

        /**
         * Calculate dynamic noise model for factors with respect to the distance moved from last keyframe
         * @param action - relative motion from the previous pose
         * @return - dynamic noise model for odometry and icp factors (with respect to the distance passed)
         */
        gtsam::noiseModel::Diagonal::shared_ptr getDynamicModel(gtsam::Pose3& action);

        /// ros publisher for factors and values
        ros::Publisher m_factors_and_values_pub;

        /// ros publisher for 2D (laser) map data
        ros::Publisher m_map_data_2D_pub;

        /// ros publisher for 3D (pointclouds) map data
        ros::Publisher m_map_data_3D_pub;

        /// data structure with 2D scans
        std::map<std::string, MapData> m_all_map_data;

        /// data structure with 3d scans
        std::map<std::string, MapData3D> m_all_map_data_3d;

        /// tag
        LogTag m_tag;
        std::string m_node_name;

        double m_range_max;

    };
}

#endif // DA_ICP_LASER_H
