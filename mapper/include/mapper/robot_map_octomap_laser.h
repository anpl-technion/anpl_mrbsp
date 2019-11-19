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
 * @file: robot_map_octomap_laser.h
 * @brief:
 * @author: Asaf Feniger
 */

#ifndef ROBOT_MAP_OCTOMAP_LASER_H
#define ROBOT_MAP_OCTOMAP_LASER_H

#include <mrbsp_utils/mrbsp_types.h>

#include <mrbsp_msgs/MapData.h>
#include <mrbsp_msgs/MapData3D.h>
#include <mrbsp_msgs/ReDrawMap.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <octomap/OcTree.h>

#include <spdlog/spdlog.h>


namespace MRBSP {

    class RobotMapOctomapLaser {
    public:
        /**
         *
         */
        RobotMapOctomapLaser();

        /**
        *
        * @param nh
        * @param nh_private
        */
        RobotMapOctomapLaser(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

        /**
         *
         */
        virtual ~RobotMapOctomapLaser();
        /**
         *
         * @param all_map_data
         */
        void generateMapFromMapData(std::map<std::string, MRBSP::Utils::MapData>& all_map_data, std::string path_to_log_folder);

    private:

        /// pointer to logger
        std::shared_ptr<spdlog::logger> m_p_map_logger;

        /// string for logger msgs
        std::stringstream m_logger_msg;

        /// path to current run folder
        std::string m_path_to_log_folder;

        /// ros node
        ros::NodeHandle m_map_node;

        /// Whether to save pointclouds for 3D visualization
        bool m_is_3D_vis;

        /// resolution of the map
        double m_resolution;

        /// smart pointer to octomap
        std::shared_ptr<octomap::OcTree> m_p_octree_map;

        /// mapping sensor pose with respect to the robot frame
        std::map<std::string, gtsam::Pose3> m_sensor_pose;

        /// ros subscriber for mapdata (laser scans + poses)
        ros::Subscriber m_map_data_2D_sub;

        /// ros subscriber for mapdata3D (pointclouds + poses)
        ros::Subscriber m_map_data_3D_sub;

        /// ros publisher for octomap msgs
        ros::Publisher m_octomap_pub;

        /// ros service server to re-draw the octomap
        ros::ServiceServer m_re_draw_map_srv;

        /// saving map incremental map
        bool m_save_incremental_map;

        /// file name to store incremental map file
        std::string m_map_file_name;

        /// data structure for building the map
        std::map<std::string, MRBSP::Utils::MapData> m_map_data;

        /// data structure for building the map with pointclouds
        std::map<std::string, MRBSP::Utils::MapData3D> m_map_data_3d;

        /**
         *
         * @param map_data_msg
         */
        void mapDataCallback(const mrbsp_msgs::MapDataConstPtr& map_data_msg);

        /**
         *
         * @param map_data_3d_msg
         */
        void mapData3DCallback(const mrbsp_msgs::MapData3DConstPtr& map_data_3d_msg);

        /**
         *
         * @param map_data
         */
        void insertLaserScan(MRBSP::Utils::MapData& map_data, std::string& robot_id);

        /**
         *
         * @param map_data_3d
         */
        void insertPointcloud(MRBSP::Utils::MapData3D& map_data_3d, std::string& robot_id);

        /**
         *
         * @param updated_values
         */
        void updatePointcloudPose(gtsam::Values& updated_values);

        /**
         *
         */
        void reDrawMap();


        bool reDrawCallback(mrbsp_msgs::ReDrawMap::Request& req, mrbsp_msgs::ReDrawMap::Response& res);

        /**
         *
         * @param bagfile_name
         */
        void WriteMapToBagfile(const std::string& bagfile_name);
    };
}

#endif // ROBOT_MAP_OCTOMAP_LASER_H
