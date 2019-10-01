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
 * @file: robot_map_octomap_laser.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include "map_node/robot_map_octoamp_laser.h"

#include <mrbsp_utils/conversion.h>
#include <mrbsp_utils/mrbsp_utils.h>

#include <octomap_ros/conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <rosbag/bag.h>
#include <nav_msgs/OccupancyGrid.h>

#include <gtsam/inference/Symbol.h>

#include "gtsam/base/serialization.h"

#define MAP_RESOLUTION 0.1
#define MAP_FILENAME "octomap.ot"

using namespace ANPL;

RobotMapOctomapLaser::RobotMapOctomapLaser():
        m_resolution(MAP_RESOLUTION),
        m_p_octree_map(new octomap::OcTree(m_resolution)),
        m_map_file_name(MAP_FILENAME),
        m_save_incremental_map(false)
{

}

RobotMapOctomapLaser::RobotMapOctomapLaser(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
        // read from param server...
        m_resolution(MAP_RESOLUTION),
        m_map_node(nh),
        m_p_octree_map(new octomap::OcTree(m_resolution)),
        m_save_incremental_map(false)
{
    if(!m_map_node.hasParam("/config_setup/log_folder")) {
        ROS_WARN("Unable to find log folder...");
        ros::Duration(1.0).sleep();
        if(m_map_node.hasParam("/config_setup/log_folder")) {
            nh_private.getParam("/config_setup/log_folder", m_path_to_log_folder);
        }
        else {
            m_path_to_log_folder = "~/.ros";
            MRBSP::Utils::createFolder(m_path_to_log_folder);
        }
    }
    else {
        nh_private.getParam("/config_setup/log_folder", m_path_to_log_folder);
    }

    MRBSP::Utils::initLogger(nh_private);
    //std::string sensor_type;
    //nh_private.param("/config_setup/is_3D_vis", m_is_3D_vis, false);
    //if(m_is_3D_vis) {
        // set keyframe type (with pointclouds)
        m_map_data_3D_sub = m_map_node.subscribe("/DA/map_data/3D", 100, &RobotMapOctomapLaser::mapData3DCallback, this);
    //    sensor_type = "pointcloud";
    //    ROS_INFO("Map: publish 3D map");
    //}
    //else {
        m_map_data_2D_sub = m_map_node.subscribe("/DA/map_data/2D", 100, &RobotMapOctomapLaser::mapDataCallback, this);
    //    sensor_type = "laser";
    //    ROS_INFO("Map: publish 2D map");
    //}

    nh_private.param("map_resolution", m_resolution, double(0.1));
    std::string filename;
    nh_private.param("save_incremental_map", m_save_incremental_map, false);
    nh_private.param("map_filename", filename, std::string("octomap.ot"));

    if(m_map_node.hasParam("/config_setup/log_folder")) {
        std::string path_to_log_folder;
        nh_private.getParam("/config_setup/log_folder", path_to_log_folder);
        m_map_file_name = std::string(path_to_log_folder + "/" + filename);
    }
    else {
        ROS_WARN("Cant find path to log folder...");
        std::string scenario_folder;
        nh_private.getParam("/senario_folder", scenario_folder);
        m_map_file_name = std::string(scenario_folder + "/" + filename);
    }

    m_octomap_pub = m_map_node.advertise<octomap_msgs::Octomap>("Map/octomap_publisher", 100);

    m_re_draw_map_srv = m_map_node.advertiseService("Map/re_draw_map", &RobotMapOctomapLaser::reDrawCallback, this);

    m_logger_msg << "Map: Map node initialized";
    //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);
}

void RobotMapOctomapLaser::generateMapFromMapData(std::map<std::string, MRBSP::Utils::MapData>& all_map_data, std::string path_to_log_folder) {
    for(auto map_data_iter = all_map_data.begin(); map_data_iter != all_map_data.end(); ++map_data_iter) {
        std::string index = map_data_iter->first;
        insertLaserScan(map_data_iter->second, index);
    }

    std::string full_map_filename(path_to_log_folder + "/" + m_map_file_name);

    m_logger_msg << "Map: Saving map to file, file name: " << full_map_filename.c_str();
    //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);
    m_p_octree_map->write(full_map_filename);
}

void RobotMapOctomapLaser::mapDataCallback(const mrbsp_msgs::MapDataConstPtr& map_data_msg) {
    gtsam::Values updated_vals;
    gtsam::deserialize(map_data_msg->ser_values, updated_vals);

    m_logger_msg << "Map: Receive map data, number of values: " << updated_vals.size();
    //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);

    if(map_data_msg->is_new_scan) {
        // add new scan to the octomap
        if(updated_vals.size() == 1) {
            gtsam::Symbol symbol(*(updated_vals.keys().begin()));

            std::string current_robot_id = std::string(1,symbol.chr());
            auto sensor_pose_iter(m_sensor_pose.find(current_robot_id));
            if (sensor_pose_iter == m_sensor_pose.end()) {
                double X_sensor, Y_sensor, Z_sensor, qX, qY, qZ, qW;
                std::string robot_ns = std::string("/robot_" + current_robot_id);
                m_map_node.param(std::string(robot_ns + "/sensor_pose/laser/position/X"), X_sensor, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/laser/position/Y"), Y_sensor, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/laser/position/Z"), Z_sensor, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/laser/orientation/qX"), qX, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/laser/orientation/qY"), qY, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/laser/orientation/qZ"), qZ, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/laser/orientation/qW"), qW, double(0.0));
                gtsam::Rot3 R_sensor = gtsam::Rot3::quaternion(qW, qX, qY, qZ);
                gtsam::Point3 t_sensor(X_sensor, Y_sensor, Z_sensor);
                gtsam::Pose3 sensor_pose = gtsam::Pose3(R_sensor, t_sensor);

                /*m_logger_msg << "Map: Set laser sensor pose." << "\n"
                             << "Robot " << current_robot_id << "\n"
                             << ": laser sensor, pose:" << "\n" << ANPL::gtsamPose3ToStringStream(sensor_pose);
                ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);*/

                m_sensor_pose.emplace(std::make_pair(current_robot_id, sensor_pose));

                
            }

            gtsam::Pose3 current_pose(updated_vals.at<gtsam::Pose3>(symbol));
            MRBSP::Utils::MapData current_map_data = std::make_pair(map_data_msg->laser_scan, current_pose);
            insertLaserScan(current_map_data, current_robot_id);

            char robot_id = symbol.chr();
            unsigned int index = symbol.index();
            std::string index_string((1, robot_id) + std::to_string(index));
            m_map_data.emplace(index_string, current_map_data);
        }
        else {
            m_logger_msg << "Wrong number of values, not adding scan!, Number of values: " << updated_vals.size();
            //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);
        }
    }
    else {
        //update scans poses
        m_logger_msg << "Map: Updating map data, number of updated values: " << updated_vals.size();
        //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);
        updatePointcloudPose(updated_vals);
    }

    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id = "world"; //xml
    map_msg.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*m_p_octree_map, map_msg); // (.ot)
    m_octomap_pub.publish(map_msg);

    if (m_save_incremental_map) {
        m_logger_msg << "Map: Saving map to file, file name: " << m_map_file_name.c_str();
        //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);
        m_p_octree_map->write(m_map_file_name);
    }
}

void RobotMapOctomapLaser::mapData3DCallback(const mrbsp_msgs::MapData3DConstPtr& map_data_3d_msg) {
    gtsam::Values updated_vals;
    gtsam::deserialize(map_data_3d_msg->ser_values, updated_vals);

    m_logger_msg << "Map: Receive map data, number of values: " << updated_vals.size();
    //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);

    if(map_data_3d_msg->is_new_scan) {
        // add new scan to the octomap
        if(updated_vals.size() == 1) {
            gtsam::Symbol symbol(*(updated_vals.keys().begin()));
            std::string current_robot_id = std::string(1,symbol.chr());
            auto sensor_pose_iter(m_sensor_pose.find(current_robot_id));
            if (sensor_pose_iter == m_sensor_pose.end()) {
                double X_sensor, Y_sensor, Z_sensor, qX, qY, qZ, qW;
                std::string robot_ns = std::string("/robot_" + current_robot_id);
                m_map_node.param(std::string(robot_ns + "/sensor_pose/pointcloud/position/X"), X_sensor, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/pointcloud/position/Y"), Y_sensor, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/pointcloud/position/Z"), Z_sensor, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/pointcloud/orientation/qX"), qX, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/pointcloud/orientation/qY"), qY, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/pointcloud/orientation/qZ"), qZ, double(0.0));
                m_map_node.param(std::string(robot_ns + "/sensor_pose/pointcloud/orientation/qW"), qW, double(0.0));
                gtsam::Rot3 R_sensor = gtsam::Rot3::quaternion(qW, qX, qY, qZ);
                gtsam::Point3 t_sensor(X_sensor, Y_sensor, Z_sensor);
                gtsam::Pose3 sensor_pose = gtsam::Pose3(R_sensor, t_sensor);

                /*m_logger_msg << "Map: Set pointcloud sensor pose." << "\n"
                             << "Robot " << current_robot_id << "\n"
                             << ": pointcloud sensor, pose:" << "\n" << ANPL::gtsamPose3ToStringStream(sensor_pose);
                ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);*/
                m_sensor_pose.emplace(std::make_pair(current_robot_id, sensor_pose));
            }

            gtsam::Pose3 current_pose(updated_vals.at<gtsam::Pose3>(symbol));
            MRBSP::Utils::MapData3D current_map_data = std::make_pair(map_data_3d_msg->pointcloud, current_pose);
            insertPointcloud(current_map_data, current_robot_id);

            char robot_id = symbol.chr();
            unsigned int index = symbol.index();
            std::string index_string((1, robot_id) + std::to_string(index));
            m_map_data_3d.emplace(index_string, current_map_data);
        }
        else {
            m_logger_msg << "Wrong number of values, not adding scan!, Number of values: " << updated_vals.size();
            //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);
        }
    }
    else {
        //update scans poses
        m_logger_msg << "Map: Updating map data, number of updated values: " << updated_vals.size();
        //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);
        updatePointcloudPose(updated_vals);
    }

    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id = "world"; //xml
    map_msg.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*m_p_octree_map, map_msg); // (.ot)
    m_octomap_pub.publish(map_msg);

    if (m_save_incremental_map) {
        m_logger_msg << "Map: Saving map to file, file name: " << m_map_file_name.c_str();
        //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);

        m_p_octree_map->write(m_map_file_name);
    }
}

void RobotMapOctomapLaser::insertLaserScan(MRBSP::Utils::MapData& map_data, std::string& robot_id) {
    sensor_msgs::LaserScan scan = map_data.first;
    gtsam::Pose3           pose = map_data.second;

    if(scan.ranges.size() == 0) {
        m_logger_msg << "Map: Received empty cloud";
        //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);
        return;
    }

    // cast laser msg to ros pointcloud
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(scan, cloud);

    // cast ros pointcloud to octomap pointcloud
    octomap::Pointcloud octomap_scan;
    octomap::pointCloud2ToOctomap(cloud, octomap_scan);

    octomap::point3d sensor_origin = octomap::point3d(0, 0, 0);

    gtsam::Pose3 current_sensor_pose;
    auto sensor_pose_iter(m_sensor_pose.find(robot_id));
    if (sensor_pose_iter != m_sensor_pose.end()) {
        current_sensor_pose = pose.compose(m_sensor_pose.at(robot_id));
    }
    else {
        m_logger_msg << "Map: can't find sensor pose for current robot.";
        //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);
    }

    octomap::pose6d frame_origin = Conversion<octomap::pose6d>::as(current_sensor_pose);

    m_p_octree_map->insertPointCloud(octomap_scan, sensor_origin, frame_origin, scan.range_max);
    m_p_octree_map->updateInnerOccupancy();
}

void RobotMapOctomapLaser::insertPointcloud(MRBSP::Utils::MapData3D& map_data_3d, std::string& robot_id) {
    sensor_msgs::PointCloud2 cloud = map_data_3d.first;
    gtsam::Pose3             pose = map_data_3d.second;
    double range_max = 5;

    if(cloud.height == 0 || cloud.width == 0) {
        m_logger_msg << "Map: Received empty cloud";
        //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);
        return;
    }

    // cast ros pointcloud to octomap pointcloud
    octomap::Pointcloud octomap_scan;
    octomap::pointCloud2ToOctomap(cloud, octomap_scan);
    octomap::point3d sensor_origin = octomap::point3d(0, 0, 0);

    gtsam::Pose3 current_sensor_pose;
    auto sensor_pose_iter(m_sensor_pose.find(robot_id));
    if (sensor_pose_iter != m_sensor_pose.end()) {
        current_sensor_pose = pose.compose(m_sensor_pose.at(robot_id));
    }
    else {
        m_logger_msg << "Map: can't find sensor pose for current robot.";
        //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);
    }

    octomap::
    pose6d frame_origin = Conversion<octomap::pose6d>::as(current_sensor_pose);

    m_p_octree_map->insertPointCloud(octomap_scan, sensor_origin, frame_origin, range_max);
    m_p_octree_map->updateInnerOccupancy();
}

void RobotMapOctomapLaser::updatePointcloudPose(gtsam::Values& updated_values) {
    gtsam::KeyList updated_keys = updated_values.keys();
    for(auto key : updated_keys) {
        gtsam::Symbol symbol(key);
        char robot_id = symbol.chr();
        size_t index = symbol.index();
        std::string index_string((1, robot_id) + std::to_string(index));
        gtsam::Pose3 updated_pose(updated_values.at<gtsam::Pose3>(symbol));


        // update 2D map
        auto data_it = m_map_data.find(index_string);
        if (data_it != m_map_data.end()) {
            m_map_data.at(index_string).second = updated_pose;
        }

        // update 3D map
        auto data3D_it = m_map_data_3d.find(index_string);
        if (data3D_it != m_map_data_3d.end()) {
            m_map_data_3d.at(index_string).second = updated_pose;
        }

        m_logger_msg << "Map: Update pose of index " << index_string;;
        //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);
    }
}

void RobotMapOctomapLaser::reDrawMap() {
    m_logger_msg << "Map: Clear map and re-draw.";
    //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);

    //m_logger_msg << "Map: Map size before clear: " << m_p_octree_map->size();;
    //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);

    m_p_octree_map->clear();

    //m_logger_msg << "Map: Map size after clear: " << m_p_octree_map->size();;
    //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);

    if(!m_map_data.empty()) {
        // draw 2D map
        for(auto data : m_map_data) {
            std::string robot_id(1,data.first.at(0));
            insertLaserScan(data.second, robot_id);
        }
    }
    if(!m_map_data_3d.empty()) {
        // draw 3D map
        for(auto data : m_map_data_3d) {
            std::string robot_id(1,data.first.at(0));
            insertPointcloud(data.second, robot_id);
        }
    }

    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id = "world"; //xml
    map_msg.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*m_p_octree_map, map_msg); // (.ot)
    m_octomap_pub.publish(map_msg);
}

bool RobotMapOctomapLaser::reDrawCallback(mrbsp_msgs::ReDrawMap::Request& req, mrbsp_msgs::ReDrawMap::Response& res) {
    m_logger_msg << "Map: Re draw map";
    //ANPL::anplLogMessage(m_p_map_logger, info, 0, m_logger_msg);
    reDrawMap();

    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id = "world"; //xml
    map_msg.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*m_p_octree_map, map_msg); // (.ot)

    res.incremental_map = map_msg;
    return true;
}

void RobotMapOctomapLaser::WriteMapToBagfile(const std::string& bagfile_name) {
    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Write);

    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id = "world"; //xml
    map_msg.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*m_p_octree_map, map_msg); // (.ot)

    /*
    nav_msgs::OccupancyGrid occ_grid_msg;
    occ_grid_msg.info.height = m_p_octree_map->getBBXBounds().x();
    occ_grid_msg.info.height = m_p_octree_map->getBBXBounds().y();
    occ_grid_msg.info.resolution = map_msg.resolution;
    */

    bag.write("octomap", ros::Time::now(), map_msg);
    bag.close();
}


RobotMapOctomapLaser::~RobotMapOctomapLaser() {
    std::size_t dot_pos = m_map_file_name.find(".");
    std::string final_map_file(m_map_file_name.substr(0,dot_pos) + "_final_" + m_map_file_name.substr(dot_pos));

    m_logger_msg << "Exiting map node. Saving map to file, file name: " << final_map_file;
    //ANPL::anplLogMessage(m_p_map_logger, warn, 0, m_logger_msg);

    reDrawMap();

    m_p_octree_map->write(final_map_file);

    std::string map_bagfile(m_map_file_name.substr(0,dot_pos) + "_bagfile.bag");
    WriteMapToBagfile(map_bagfile);

}
