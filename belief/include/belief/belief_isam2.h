/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (ANPL),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 * Authors: Vadim Indelman, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information.
 *
 * -------------------------------------------------------------------------- */

/**
 * @file: belief_isam2.h
 * @brief:
 * @author: Tal Regev
 * @author: Asaf Feniger
 */

#ifndef BELIEF_ISAM2_H
#define BELIEF_ISAM2_H

#include <mrbsp_utils/mrbsp_types.h>
#include <mrbsp_utils/mrbsp_utils.h>
#include <mrbsp_msgs/GtsamFactorGraph.h>
#include <mrbsp_msgs/RequestBelief.h>

#include <ros/ros.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>


namespace MRBSP {
    using namespace MRBSP::Utils;

    class BeliefIsam2 {
    public:

        BeliefIsam2(const ros::NodeHandle &nh_private);

        /**
         *
         */
        virtual ~BeliefIsam2();

        /**
         *
         * @param new_factors_and_values
         */
        gtsam::Values getFactorsAndValues(std::vector<NewFactorsAndValues>& all_factors_and_values, std::string path_to_log_folder);

        /**
         * get one
         * @param new_factors_and_values
         * @return
         */
        gtsam::Values getLastFactorsAndValues(NewFactorsAndValues& new_factors_and_values);

        /**
         * Arrange factors and values and pass them to the saving functions
         * @param file_name - filename to save
         * @param isam - isam object to save
         */
        void saveIsamResultsToFile(const std::string& file_name, const gtsam::ISAM2& isam);

        /**
         *
         * @param file_name
         * @param isam
         */
        void serializeResultsToFile(const std::string& file_name, const gtsam::ISAM2& isam);

    private:

        /// string for logger msgs
        std::stringstream m_logger_msg;

        /// path to current run folder
        std::string m_logger_path;

        /// data source type, true if rosbag, false if stream
        bool m_is_rosbag;

        /// flag to publish optimized pose to odometry node
        bool m_is_publish_optimize_pose;

        /// ros node
        ros::NodeHandle m_privateNodeHandle;

        /// ISAM object to calculate the factor graph
        gtsam::ISAM2 m_isam;

        /// map to store the last indexes of al participated robots
        std::map<char, unsigned int> m_last_values_map;

        /// ros subscriber for factors and values
        ros::Subscriber m_factors_and_values_sub;

        /// ros publisherss for all odometry nodes of the participating robots
        std::map<char, ros::Publisher> m_optimized_pose_pub;

        /// ros publisher for optimized gtsam::values
        ros::Publisher m_memory_update_pub;

        /// ros service server to rqeust the controller to move the robot
        ros::ServiceServer m_get_belief_service;

        /**
         * Add new factors and values into the belief
         * @param all_factors_and_values - new factors and values
         */
        void handleNewFactorsAndValues(NewFactorsAndValues& all_factors_and_values);

        /**
         * Save gtsam values to txt file + covariances for each value. Readable with loadResults.m matlab script
         * @param file_name - filname to save
         * @param vals - gtsam values to save
         */
        void savePoseWithCovToFile(const std::string& file_name, const gtsam::ISAM2& isam);

        /**
         * Save gtsam factors list to txt file + covariances for each value. Readable with loadResults.m matlab script
         * @param file_name - filname to save
         * @param graph - factor grapg with the factors to save
         */
        void saveFactorsListToFile(const std::string& file_name, const gtsam::NonlinearFactorGraph& graph);

        /**
         *
         * @param graph_input_msg
         */
        void factorAndValuesCallback(const mrbsp_msgs::GtsamFactorGraphConstPtr& graph_input_msg);

        /**
         * service callback for move request
         * @param req
         * @param res
         * @return
         */
        bool getBeliefCallback(mrbsp_msgs::RequestBelief::Request& req, mrbsp_msgs::RequestBelief::Response& res);


        void loadParameter();

        /// tag
        LogTag m_tag;
        std::string m_node_name;
        bool m_is_save_isam_files;
        std::string m_central_prefix;

    };
}

#endif // BELIEF_ISAM2_H
