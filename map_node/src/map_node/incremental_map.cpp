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
 * @file: incremental_map.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include <mrbsp_utils/gtsam_serialization.h>
#include <mrbsp_utils/mrbsp_types.h>
#include <mrbsp_utils/conversion.h>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map> //std::map

#include "gtsam/base/serialization.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap/OcTree.h>

#include <signal.h>

enum class Colors {
    red,
    cyan,
};

void arrangeScans(std::map<std::string, std::pair<sensor_msgs::LaserScan, sensor_msgs::Image>>& keyframes_scans, std::vector<std::string>& path_to_bagfiles);
void updateDataContainer(std::map<std::string, MRBSP::Utils::MapData>& data_container, gtsam::Values& updated_values,
                         nav_msgs::Path& path_A, nav_msgs::Path& path_B);
void reDrawMap(std::shared_ptr<octomap::OcTree>& p_octree_map, std::map<std::string, MRBSP::Utils::MapData>& data_container);
void insertLaserScan(std::shared_ptr<octomap::OcTree>& p_octree_map, MRBSP::Utils::MapData& map_data);
void gtsamPose3ToRosPose(geometry_msgs::Pose& ros_pose, gtsam::Pose3& gtsam_pose);
visualization_msgs::MarkerArray visualizeFactors(const std::map<std::string, MRBSP::Utils::MapData>& data_contaner, std::vector<std::string>& factor_inf);
void reduceScanRangesByFactor(sensor_msgs::LaserScan& scan);
visualization_msgs::MarkerArray creatTitlesMarkers();

void mySigintHandler(int sig) {
    ROS_INFO("Shutting down incremental mapping node...");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    // ros initialization
    ros::init(argc, argv, "incMap");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    ros::Publisher map_publisher(nh.advertise<octomap_msgs::Octomap>("Map/inc_map",100));
    ros::Publisher robot_A_path(nh.advertise<nav_msgs::Path>("Map/Robot_A/path",10));
    ros::Publisher robot_B_path(nh.advertise<nav_msgs::Path>("Map/Robot_B/path",10));
    ros::Publisher robot_A_img(nh.advertise<sensor_msgs::Image>("Map/Robot_A/image",10));
    ros::Publisher robot_B_img(nh.advertise<sensor_msgs::Image>("Map/Robot_B/image",10));
    ros::Publisher scan1_pub(nh.advertise<sensor_msgs::LaserScan>("Map/scan1",10));
    ros::Publisher scan2_pub(nh.advertise<sensor_msgs::LaserScan>("Map/scan2",10));
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "Map/visualization_marker", 0 );

    // path to files
    std::string scenario_folder("/home/asafeniger/ANPL/code/mrbsp_ws/src/mrbsp/mrbsp_scenarios/scenarios");
    std::string scenario_name("mr_centralized_laser");
    std::string run_name("mr_lc_nn_80");
    std::stringstream data_folder;
    data_folder << scenario_folder << "/" << scenario_name << "/results/" << run_name;

    // path to bagfiles
    std::vector<std::string> path_to_bagfiles;
    std::string bag_A_filename("Robot_A_keyframe_bag.bag");
    std::string path_to_bag_A(data_folder.str() + "/" + bag_A_filename);
    path_to_bagfiles.push_back(path_to_bag_A);

    std::string bag_B_filename("Robot_B_keyframe_bag.bag");
    std::string path_to_bag_B(data_folder.str() + "/" + bag_B_filename);
    path_to_bagfiles.push_back(path_to_bag_B);

    // arrange scans and images
    std::map<std::string, std::pair<sensor_msgs::LaserScan, sensor_msgs::Image>> keyframes_scans;
    arrangeScans(keyframes_scans, path_to_bagfiles);


    // create container for map data
    std::map<std::string, MRBSP::Utils::MapData> data_container;

    // generate octomap object
    double map_resulotion = 0.05;
    std::shared_ptr<octomap::OcTree> p_octree_map(new octomap::OcTree(map_resulotion));

    // tf broadcaster to publish tf between 2 scans
    tf::TransformBroadcaster relative_pose_tf_broadcaster;

    ros::Duration(5.0).sleep();

    visualization_msgs::MarkerArray marker_array = creatTitlesMarkers();
    std::cout << "Number of titles: " << marker_array.markers.size() << std::endl;
    vis_pub.publish(marker_array);
    ros::spinOnce();

    for(auto marker : marker_array.markers) {
        std::cout << marker.text << ": " << marker.pose.position.x << "," << marker.pose.position.y << std::endl;
    }

    marker_array.markers.clear();

    // reference file name to set final belief
    std::string ref_filename("belief_A69_factors.txt");
    std::stringstream index_order_ref;
    index_order_ref << data_folder.str() << "/matlab/" << ref_filename;
    std::string line;
    std::ifstream myfile (index_order_ref.str());
    std::cout << "Try to read " << index_order_ref.str() << std::endl;

    if (myfile.is_open())
    {
        // create strings from each line in the txt file
        while ( getline (myfile,line) && ros::ok() )
        {
            // convert the string to vector of string (seperated with space in the txt file)
            std::istringstream iss(line);
            std::vector<std::string> tokens{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};

            // ignore loop closures and multi robot factors inputs
            if(tokens.at(0) == "Between" || tokens.at(0) == "Prior") {
                // interpret id and index from strings
                std::string id_index_str(tokens.at(3) + tokens.at(4));
                unsigned char robot_id = static_cast<unsigned char>(std::string(tokens.at(3)).at(0));
                size_t index;
                std::stringstream index_strs(tokens.at(4));
                index_strs >> index;

                // deserialize gtsam values
                std::stringstream serialized_filename;
                serialized_filename << "belief_" << id_index_str << "_ser_values.txt";
                std::stringstream path_to_values_file;
                path_to_values_file << data_folder.str() << "/serialized_files/" << serialized_filename.str();

                gtsam::Values vals;
                gtsam::deserializeFromFile(path_to_values_file.str(), vals);

                // get current pose
                gtsam::Symbol symbol(robot_id, index);
                gtsam::Pose3 pose = vals.at<gtsam::Pose3>(symbol);

                std::cout << "add: " << id_index_str << ": X = " << pose.x() << ", Y = " << pose.y() << ", Z = " << pose.z() << std::endl;

                MRBSP::Utils::MapData map_data = std::pair<sensor_msgs::LaserScan, gtsam::Pose3>(keyframes_scans.at(id_index_str).first, pose);
                data_container.insert(std::pair<std::string, MRBSP::Utils::MapData>(id_index_str, map_data));

                // update previous scans poses
                nav_msgs::Path path_A, path_B; // objects to store robots path
                updateDataContainer(data_container, vals, path_A, path_B);

                // re-draw map
                reDrawMap(p_octree_map, data_container);

                // publish robots path
                robot_A_path.publish(path_A);
                robot_B_path.publish(path_B);

                visualization_msgs::Marker marker;
                marker.header.frame_id = "world";
                marker.header.stamp = ros::Time().now();
                marker.ns = std::string(tokens.at(3)).at(0);
                marker.type = visualization_msgs::Marker::POINTS;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1;
                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.01;
                marker.color.a = 1.0; // Don't forget to set the alpha!

                switch (robot_id) {
                    case 'A' :
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                        marker.id = 0;
                        break;
                    case 'B' :
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        marker.id = 1;
                        break;
                }

                geometry_msgs::Point point1 = Conversion<geometry_msgs::Point>::as(pose.translation());
                marker.points.push_back(point1);

                visualization_msgs::Marker text_marker(marker);
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.text = id_index_str;
                text_marker.pose.position = point1;
                text_marker.pose.position.z = 1;
                text_marker.pose.position.y = point1.y + 1;
                text_marker.scale.z = 1;
                text_marker.ns = "text";

                marker_array.markers.clear();
                marker_array.markers.push_back(marker);
                marker_array.markers.push_back(text_marker);
                vis_pub.publish(marker_array);

                switch (robot_id) {
                    case 'A' :
                        robot_A_img.publish(keyframes_scans.at(id_index_str).second);
                        break;
                    case 'B' :
                        robot_B_img.publish(keyframes_scans.at(id_index_str).second);
                        break;
                }

                // publish map
                octomap_msgs::Octomap map_msg;
                map_msg.header.frame_id = "world"; //xml
                map_msg.header.stamp = ros::Time::now();
                octomap_msgs::fullMapToMsg(*p_octree_map, map_msg); // (.ot)
                map_publisher.publish(map_msg);


            }
            else {

                //marker_array.markers.clear();
                marker_array = visualizeFactors(data_container, tokens);
                vis_pub.publish(marker_array);

                std::string robot1_id_index_str(tokens.at(1) + tokens.at(2));
                std::string robot2_id_index_str(tokens.at(3) + tokens.at(4));

                gtsam::Pose3 pose1(data_container.at(robot1_id_index_str).second);
                gtsam::Pose3 pose2(data_container.at(robot2_id_index_str).second);

                sensor_msgs::LaserScan scan1(keyframes_scans.at(robot1_id_index_str).first);
                scan1.header.stamp = ros::Time::now();
                reduceScanRangesByFactor(scan1);
                sensor_msgs::LaserScan scan2(keyframes_scans.at(robot2_id_index_str).first);
                scan2.header.stamp = ros::Time::now();
                scan2.header.frame_id = "laser2";
                reduceScanRangesByFactor(scan2);

                gtsam::Pose3 relative_pose_gtsam(pose1.between(pose2));
                geometry_msgs::Pose relative_pose_ros;
                gtsamPose3ToRosPose(relative_pose_ros, relative_pose_gtsam);
                tf::Transform transform;
                tf::poseMsgToTF(relative_pose_ros, transform);
                relative_pose_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "laser", scan2.header.frame_id));
                ros::Duration(1.0).sleep();

                //std::cout << "Scan 1 size: " << scan1.ranges.size() << std::endl;
                //std::cout << "Scan 2 size: " << scan2.ranges.size() << std::endl;
                scan1_pub.publish(scan1);
                scan2_pub.publish(scan2);

                //ros::Duration(0.5).sleep();
            }

            ros::spinOnce();
        }
        myfile.close();

        marker_array.markers.clear();
        visualization_msgs::Marker clear_markers;
        clear_markers.action = 3u; //visualization_msgs::Marker::DELETEALL;
        clear_markers.pose.orientation.w = 1;
        marker_array.markers.push_back(clear_markers);
        vis_pub.publish(marker_array);
    }
    else {
        std::cout << "Unable to open file" << std::endl;
    }

    return 0;
}

void arrangeScans(std::map<std::string, std::pair<sensor_msgs::LaserScan, sensor_msgs::Image>>& keyframes_scans, std::vector<std::string>& path_to_bagfiles) {
    // arrange keyframe scans
    std::string scan_topic("scan");
    std::string image_topic("image");
    std::string index_topic("index");
    for(auto iter = path_to_bagfiles.begin(); iter != path_to_bagfiles.end(); ++iter) {
        rosbag::Bag bag;
        bag.open(*iter, rosbag::bagmode::Read);
        rosbag::View view_pointcloud(bag, rosbag::TopicQuery(scan_topic));
        rosbag::View view_image(bag, rosbag::TopicQuery(image_topic));
        rosbag::View view_index(bag, rosbag::TopicQuery(index_topic));
        auto image_iter = view_image.begin();
        auto index_iter = view_index.begin();

        for(auto scan_iter = view_pointcloud.begin(); scan_iter != view_pointcloud.end(); ++scan_iter, ++image_iter, ++index_iter) {
            auto laser_messageInstance = *scan_iter;
            sensor_msgs::LaserScanConstPtr laser_const_ptr = laser_messageInstance.instantiate<sensor_msgs::LaserScan>();
            auto img_messageInstance = *image_iter;
            sensor_msgs::ImageConstPtr img_const_ptr = img_messageInstance.instantiate<sensor_msgs::Image>();
            auto idx_messageInstance = *index_iter;
            std_msgs::StringConstPtr idx_const_ptr = idx_messageInstance.instantiate<std_msgs::String>();

            if (laser_const_ptr != NULL && idx_const_ptr != NULL) {
                sensor_msgs::LaserScan scan = *laser_const_ptr;
                sensor_msgs::Image image = *img_const_ptr;
                std_msgs::String index = *idx_const_ptr;

                //std::cout << "index: " << index.data << ", scan size: " << scan.ranges.size() << std::endl;
                std::pair<sensor_msgs::LaserScan, sensor_msgs::Image> keyframe_measurements = std::make_pair(scan, image);
                keyframes_scans.insert(std::pair<std::string, std::pair<sensor_msgs::LaserScan, sensor_msgs::Image>>
                                               (index.data,keyframe_measurements));
            }
        }
    }
}

void updateDataContainer(std::map<std::string, MRBSP::Utils::MapData>& data_container, gtsam::Values& updated_values,
                         nav_msgs::Path& path_A, nav_msgs::Path& path_B) {
    int num_of_updated_poses = 0;
    int seq_A(0);
    int seq_B(0);

    gtsam::KeyList updated_keys = updated_values.keys();
    for(auto key : updated_keys) {
        gtsam::Symbol symbol(key);
        char robot_id = symbol.chr();
        size_t index = symbol.index();
        std::string index_string((1, robot_id) + std::to_string(index));
        gtsam::Pose3 updated_pose(updated_values.at<gtsam::Pose3>(symbol));


        // update 2D map
        auto data_it = data_container.find(index_string);
        if (data_it != data_container.end()) {
            data_container.at(index_string).second = updated_pose;

            geometry_msgs::Pose ros_pose;
            gtsamPose3ToRosPose(ros_pose, updated_pose);
            geometry_msgs::PoseStamped ros_pose_stamp;
            ros_pose_stamp.pose = ros_pose;
            ros_pose_stamp.header.stamp = ros::Time::now();

            switch (robot_id) {
                case 'A':
                    ros_pose_stamp.header.frame_id = std::string("Robot_A");
                    ros_pose_stamp.header.seq = seq_A;
                    path_A.header.seq = seq_A;
                    path_A.header.stamp = ros::Time::now();
                    path_A.header.frame_id = std::string("world");
                    path_A.poses.push_back(ros_pose_stamp);
                    ++seq_A;
                    break;

                case 'B':
                    ros_pose_stamp.header.frame_id = std::string("Robot_B");
                    ros_pose_stamp.header.seq = seq_B;
                    path_B.header.seq = seq_B;
                    path_B.header.stamp = ros::Time::now();
                    path_B.header.frame_id = std::string("world");
                    path_B.poses.push_back(ros_pose_stamp);
                    ++seq_B;
                    break;
            }

            ++num_of_updated_poses;
        }
    }

    //std::cout << "Update " << num_of_updated_poses << " of " << data_container.size() << std::endl;
}

void reDrawMap(std::shared_ptr<octomap::OcTree>& p_octree_map, std::map<std::string, MRBSP::Utils::MapData>& data_container) {
    p_octree_map->clear();
    //std::cout << "Clear map." << std::endl;

    unsigned int scan_number(0);
    if(!data_container.empty()) {
        // draw 2D map
        for(auto data : data_container) {
            std::string robot_id(1,data.first.at(0));
            insertLaserScan(p_octree_map, data.second);
            ++scan_number;

            //std::cout << "add scan #" << scan_number << "/" << data_container.size() << std::endl;
        }
    }
}

void insertLaserScan(std::shared_ptr<octomap::OcTree>& p_octree_map, MRBSP::Utils::MapData& map_data) {

    sensor_msgs::LaserScan scan = map_data.first;
    gtsam::Pose3           pose = map_data.second;
    double range_max = 5;

    if(scan.ranges.size() == 0) {
        std::cout << "Received empty scan" << std::endl;
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

    //std::cout << "convert gtsam object to octomap object..." << std::endl;
    octomap::pose6d frame_origin = Conversion<octomap::pose6d>::as(current_sensor_pose);

    //std::cout << "insert pointcloud to octomap..." << std::endl;
    p_octree_map->insertPointCloud(octomap_scan, sensor_origin, frame_origin, range_max);

    //std::cout << "Update inner occupancy.." << std::endl;
    p_octree_map->updateInnerOccupancy();
}

void gtsamPose3ToRosPose(geometry_msgs::Pose& ros_pose, gtsam::Pose3& gtsam_pose) {
    ros_pose.position.x = gtsam_pose.x();
    ros_pose.position.y = gtsam_pose.y();
    ros_pose.position.z = gtsam_pose.z();
    ros_pose.orientation.w = gtsam_pose.rotation().quaternion().x();
    ros_pose.orientation.x = gtsam_pose.rotation().quaternion().y();
    ros_pose.orientation.y = gtsam_pose.rotation().quaternion().z();
    ros_pose.orientation.z = gtsam_pose.rotation().quaternion().w();
}

visualization_msgs::MarkerArray visualizeFactors(const std::map<std::string, MRBSP::Utils::MapData>& data_contaner, std::vector<std::string>& factor_inf) {
    static int factor_idx(0);
    visualization_msgs::MarkerArray marker_array;

    std::string index1(factor_inf.at(1) + factor_inf.at(2));
    std::string index2(factor_inf.at(3) + factor_inf.at(4));

    gtsam::Pose3 gtsam_pose1 = data_contaner.at(index1).second;
    gtsam::Pose3 gtsam_pose2 = data_contaner.at(index2).second;


    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time().now();
    marker.ns = "keyframes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!


    Colors color;
    std::string factor_type;
    if(factor_inf.at(0) == "Between_lc") {
        color = Colors::red;
        factor_type = std::string("loop closure");
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else {
        color = Colors::cyan;
        factor_type = std::string("multi robot");
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
    }


    geometry_msgs::Point point1 = Conversion<geometry_msgs::Point>::as(gtsam_pose1.translation());
    geometry_msgs::Point point2 = Conversion<geometry_msgs::Point>::as(gtsam_pose2.translation());
    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker_array.markers.push_back(marker);

    visualization_msgs::Marker factor_line(marker);
    factor_line.type = visualization_msgs::Marker::LINE_LIST;
    factor_line.ns = "factors connection";
    //factor_line.id = factor_idx;
    ++factor_idx;
    factor_line.scale.x = 0.1;
    marker_array.markers.push_back(factor_line);


    visualization_msgs::Marker text_marker1(marker);
    text_marker1.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker1.text = index1;
    text_marker1.pose.position = point1;
    text_marker1.pose.position.z = 1;
    text_marker1.pose.position.y = point1.y + 1;
    text_marker1.scale.z = 1;
    text_marker1.ns = "text";
    text_marker1.id = 2;
    marker_array.markers.push_back(text_marker1);

    visualization_msgs::Marker text_marker2(text_marker1);
    text_marker2.text = index2;
    text_marker2.pose.position = point2;
    text_marker2.pose.position.y = point2.y + 1;
    text_marker2.id = 3;
    marker_array.markers.push_back(text_marker2);
    return marker_array;
}

void reduceScanRangesByFactor(sensor_msgs::LaserScan& scan) {
    for(auto iter = scan.ranges.begin(); iter != scan.ranges.end(); ++iter) {
        *iter = *iter/2;
    }
}

visualization_msgs::MarkerArray creatTitlesMarkers() {
    visualization_msgs::MarkerArray title_marker_array;

    visualization_msgs::Marker map_title_marker;
    map_title_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    map_title_marker.action = visualization_msgs::Marker::ADD;
    map_title_marker.header.frame_id = "world";
    map_title_marker.header.stamp = ros::Time::now();
    map_title_marker.text = "Joint Map";
    map_title_marker.pose.position.x = 8;
    map_title_marker.pose.position.y = 17;
    map_title_marker.pose.orientation.w = 1;
    map_title_marker.scale.x = 0.5;
    map_title_marker.scale.y = 0.5;
    map_title_marker.scale.z = 2;
    map_title_marker.ns = "titles";
    map_title_marker.id = 0;
    map_title_marker.color.a = 1.0; // Don't forget to set the alpha!
    map_title_marker.color.r = 1.0;
    map_title_marker.color.g = 1.0;
    map_title_marker.color.b = 1.0;
    title_marker_array.markers.push_back(map_title_marker);

    visualization_msgs::Marker robot_A_legend_marker(map_title_marker);
    robot_A_legend_marker.text = "Robot A keyframes";
    robot_A_legend_marker.pose.position.x = 8;
    robot_A_legend_marker.pose.position.y = -5;
    robot_A_legend_marker.scale.z = 1;
    robot_A_legend_marker.id = 1;
    robot_A_legend_marker.color.r = 0.0;
    robot_A_legend_marker.color.g = 0.0;
    robot_A_legend_marker.color.b = 1.0;
    title_marker_array.markers.push_back(robot_A_legend_marker);

    visualization_msgs::Marker robot_B_legend_marker(robot_A_legend_marker);
    robot_B_legend_marker.text = "Robot B keyframes";
    robot_B_legend_marker.pose.position.y = -6;
    robot_B_legend_marker.id = 2;
    robot_B_legend_marker.color.r = 0.0;
    robot_B_legend_marker.color.g = 1.0;
    robot_B_legend_marker.color.b = 0.0;
    title_marker_array.markers.push_back(robot_B_legend_marker);

    visualization_msgs::Marker lc_legend_marker(robot_A_legend_marker);
    lc_legend_marker.text = "Loop closure factors";
    lc_legend_marker.pose.position.y = -7;
    lc_legend_marker.id = 3;
    lc_legend_marker.color.r = 1.0;
    lc_legend_marker.color.g = 0.0;
    lc_legend_marker.color.b = 0.0;
    title_marker_array.markers.push_back(lc_legend_marker);

    visualization_msgs::Marker mr_legend_marker(robot_A_legend_marker);
    mr_legend_marker.text = "Multi robot factors";
    mr_legend_marker.pose.position.y = -8;
    mr_legend_marker.id = 4;
    mr_legend_marker.color.r = 0.0;
    mr_legend_marker.color.g = 1.0;
    mr_legend_marker.color.b = 1.0;
    title_marker_array.markers.push_back(mr_legend_marker);

    return title_marker_array;
}