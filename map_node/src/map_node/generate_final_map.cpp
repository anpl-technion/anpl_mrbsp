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
 * @file: generate_final_map.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include "map_node/robot_map_octoamp_laser.h"
#include <mrbsp_utils/gtsam_serialization.h>
#include <mrbsp_utils/mrbsp_types.h>
#include <mrbsp_utils/conversion.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include "gtsam/base/serialization.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

#include <octomap/OcTree.h>

void createMapData3D(std::map<std::string, MRBSP::Utils::MapData3D>& map_data_3d, std::string& path_to_values,
                     std::vector<std::string>& path_to_bagfiles);
void insertPointcloud(std::shared_ptr<octomap::OcTree>& p_octree_map, MRBSP::Utils::MapData3D& map_data_3d);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generateFinalMap");
    ros::NodeHandle nh;

    // path to files
    std::string scenario_folder("/home/asafeniger/ANPL/code/mrbsp_ws/src/mrbsp/mrbsp_scenarios/scenarios");
    std::string scenario_name("mr_centralized_laser");
    std::string run_name("mr_lc_nn_80_03");
    std::string data_folder(scenario_folder + "/" + scenario_name + "/results/" + run_name);

    // get gtsam values
    std::string serialized_filename("belief_A70_ser_values.txt");
    std::string path_to_values_file(data_folder + "/serialized_files/" + serialized_filename);



    // get pointcloud data
    std::vector<std::string> path_to_bagfiles;
    std::string bag_A_filename("Robot_A_keyframe_bag.bag");
    std::string path_to_bag_A(data_folder + "/" + bag_A_filename);
    path_to_bagfiles.push_back(path_to_bag_A);

    std::string bag_B_filename("Robot_B_keyframe_bag.bag");
    std::string path_to_bag_B(data_folder + "/" + bag_B_filename);
    path_to_bagfiles.push_back(path_to_bag_B);

    // arrange data
    std::map<std::string, MRBSP::Utils::MapData3D> data_3d_container;
    createMapData3D(data_3d_container, path_to_values_file, path_to_bagfiles);
    std::cout << "Number of scans: " << data_3d_container.size() << std::endl;

    // generate octomap
    std::string map_name("final_octomap_3d.ot");
    std::string path_to_map(data_folder + "/" + map_name);
    double map_resulotion = 0.1;
    std::shared_ptr<octomap::OcTree> p_octree_map(new octomap::OcTree(map_resulotion));


    std::cout << "map size: " << p_octree_map->size() << std::endl;

    unsigned int scan_idx = 1;
    for(auto iter = data_3d_container.begin(); iter != data_3d_container.end(); ++iter) {
        std::cout << "Adding scan #" << scan_idx << "/" << data_3d_container.size() << " to the map." << std::endl;
        insertPointcloud(p_octree_map, iter->second);
        ++scan_idx;
    }
    std::cout << "map size: " << p_octree_map->size() << std::endl;
    p_octree_map->updateInnerOccupancy();
    p_octree_map->write(path_to_map);

    return 0;
}

void createMapData3D(std::map<std::string, MRBSP::Utils::MapData3D>& data_3d_container, std::string& path_to_values,
                     std::vector<std::string>& path_to_bagfiles) {

    // arange keyframe poses
    std::map<std::string, gtsam::Pose3> keyframes_poses;
    gtsam::Values vals;
    gtsam::deserializeFromFile(path_to_values, vals);

    gtsam::KeyList keys = vals.keys();
    for(auto key : keys) {
        gtsam::Symbol symbol(key);
        char robot_id = symbol.chr();
        size_t index = symbol.index();
        std::string index_string((1, robot_id) + std::to_string(index));
        gtsam::Pose3 pose(vals.at<gtsam::Pose3>(symbol));

        keyframes_poses.insert(std::pair<std::string, gtsam::Pose3>(index_string,pose));
    }

    // arrange keyframe scans
    std::string pointcloud_topic("pointcloud");
    std::string index_topic("index");
    for(auto iter = path_to_bagfiles.begin(); iter != path_to_bagfiles.end(); ++iter) {
        rosbag::Bag bag;
        bag.open(*iter, rosbag::bagmode::Read);
        rosbag::View view_pointcloud(bag, rosbag::TopicQuery(pointcloud_topic));
        rosbag::View view_index(bag, rosbag::TopicQuery(index_topic));
        auto index_iter = view_index.begin();

        for(auto pointcloud_iter = view_pointcloud.begin(); pointcloud_iter != view_pointcloud.end(); ++pointcloud_iter, ++index_iter) {
            auto pt_messageInstance = *pointcloud_iter;
            sensor_msgs::PointCloud2ConstPtr pt_const_ptr = pt_messageInstance.instantiate<sensor_msgs::PointCloud2>();
            auto idx_messageInstance = *index_iter;
            std_msgs::StringConstPtr idx_const_ptr = idx_messageInstance.instantiate<std_msgs::String>();

            if (pt_const_ptr != NULL && idx_const_ptr != NULL) {
                sensor_msgs::PointCloud2 pointcloud = *pt_const_ptr;
                std_msgs::String index = *idx_const_ptr;

                std::cout << "index: " << index.data << ", pointcloud size: " << pointcloud.data.size() << std::endl;

                gtsam::Pose3 pose = keyframes_poses.at(index.data);

                MRBSP::Utils::MapData3D map_data = std::pair<sensor_msgs::PointCloud2, gtsam::Pose3>(pointcloud, pose);

                data_3d_container.insert(std::pair<std::string, MRBSP::Utils::MapData3D>(index.data, map_data));
            }
        }
    }
}


void insertPointcloud(std::shared_ptr<octomap::OcTree>& p_octree_map, MRBSP::Utils::MapData3D& map_data_3d) {

    sensor_msgs::PointCloud2 cloud = map_data_3d.first;
    gtsam::Pose3             pose = map_data_3d.second;
    double range_max = 5;

    if(cloud.height == 0 || cloud.width == 0) {
        std::cout << "Received empty cloud" << std::endl;
        return;
    }

    // cast ros pointcloud to octomap pointcloud
    octomap::Pointcloud octomap_scan;
    octomap::pointCloud2ToOctomap(cloud, octomap_scan);
    octomap::point3d sensor_origin = octomap::point3d(0, 0, 0);


    gtsam::Pose3 current_sensor_pose = pose;
    /*
    auto sensor_pose_iter(m_sensor_pose.find(robot_id));
    if (sensor_pose_iter != m_sensor_pose.end()) {
        current_sensor_pose = pose.compose(m_sensor_pose.at(robot_id));
    }
    else {
        std::cout << "Map: can't find sensor pose for current robot." <<  std::endl;
    }
    */

    std::cout << "convert gtsam object to octomap object..." << std::endl;
    octomap::pose6d frame_origin = Conversion<octomap::pose6d>::as(current_sensor_pose);

    std::cout << "insert pointcloud to octomap..." << std::endl;
    p_octree_map->insertPointCloud(octomap_scan, sensor_origin, frame_origin, range_max);

    std::cout << "Update inner occupancy.." << std::endl;
    p_octree_map->updateInnerOccupancy();
}
