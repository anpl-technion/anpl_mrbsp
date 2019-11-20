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
 * @file: collision_detection_octomap.cpp
 * @brief:
 * @author: Asaf Feniger
 */


#include "collision_detection/collision_detection_octomap.h"

#include <mrbsp_utils/conversion.h>
#include <mrbsp_utils/mrbsp_utils.h>

#include <laser_geometry/laser_geometry.h>

#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <fcl/collision.h>
#include <fcl/continuous_collision.h>

#include <math.h>
#include <std_msgs/Bool.h>

using namespace MRBSP;

// default constructor
CollisionDetectionOctomap::CollisionDetectionOctomap():
    CollisionDetectionBase()
{
    ros::NodeHandle nh_private("~");

    /*if(!nh_private.hasParam("/logger/loggerPath")) {
        ROS_WARN("Unable to find log folder...");
        ros::Duration(1.0).sleep();
        if(nh_private.hasParam("/logger/loggerPath")) {
            nh_private.getParam("/logger/loggerPath", b_m_path_to_log_folder);
        }
        else {
			const char * home = getenv ("HOME");
    		if (home == NULL)
	            b_m_path_to_log_folder = "./";
    		else
	            b_m_path_to_log_folder = std::string(home) + "/.ros/";
			ROS_WARN("CD logging set to %s", b_m_path_to_log_folder.c_str());

            MRBSP::Utils::createFolder(b_m_path_to_log_folder);
        }
    }
    else {
        nh_private.getParam("/logger/loggerPath", b_m_path_to_log_folder);
    }
    std::cout << b_m_path_to_log_folder << std::endl;*/
    MRBSP::Utils::initLogger(nh_private);

    b_m_logger_msg << "CD: Initialize octomap collision detector logger";
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    // get sensor topic parameter
    std::string sensor_topic;
    nh_private.param("depth_sensor/topic", sensor_topic, std::string("scan"));
    if(!b_m_robot_name.empty()) {
        sensor_topic = std::string("/" + b_m_robot_name + sensor_topic);
    }
    b_m_logger_msg << "CD: Sensor topic: " << sensor_topic;
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    // initialize ros laser messages subscriber
    m_laser_sub = b_m_collision_detector_node.subscribe(std::string(sensor_topic),
                                                        1, &CollisionDetectionOctomap::laserCallback, this);

    // initialize ros octomap publisher
    m_local_map_pub = b_m_collision_detector_node.advertise<octomap_msgs::Octomap>(b_m_robot_ns + "/Collision/local_map", 1, true);

    // set publish empty collision msg flag to false
    m_publish_empty_msg = false;

    // get local map resolution parameter
    nh_private.param("local_map/resolution", m_map_resolution, 0.1);

    // get sensor pose in the robot body frame
    std::string sensor_type;
    nh_private.param("depth_sensor/type", sensor_type, std::string("laser"));
    b_m_logger_msg << "CD: Sensor Type: " << sensor_type;
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    double X_sensor, Y_sensor, Z_sensor, qX, qY, qZ, qW;
    std::string robot_ns = std::string("/Robot_" + std::string(1,b_m_robot_id));
    std::cout << "Robot ns: " << robot_ns << std::endl;
    b_m_collision_detector_node.param(std::string(robot_ns + "/sensor_pose/" + sensor_type + "/position/X"), X_sensor, double(0.0));
    b_m_collision_detector_node.param(std::string(robot_ns + "/sensor_pose/" + sensor_type + "/position/Y"), Y_sensor, double(0.0));
    b_m_collision_detector_node.param(std::string(robot_ns + "/sensor_pose/" + sensor_type + "/position/Z"), Z_sensor, double(0.0));
    b_m_collision_detector_node.param(std::string(robot_ns + "/sensor_pose/" + sensor_type + "/orientation/qX"), qX, double(0.0));
    b_m_collision_detector_node.param(std::string(robot_ns + "/sensor_pose/" + sensor_type + "/orientation/qY"), qY, double(0.0));
    b_m_collision_detector_node.param(std::string(robot_ns + "/sensor_pose/" + sensor_type + "/orientation/qZ"), qZ, double(0.0));
    b_m_collision_detector_node.param(std::string(robot_ns + "/sensor_pose/" + sensor_type + "/orientation/qW"), qW, double(1.0));

    b_m_logger_msg << "CD: Sensor pose param name: " << std::string(robot_ns + "/sensor_pose/" + sensor_type);
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);
    b_m_logger_msg << "CD: trans: (" << X_sensor << "," << Y_sensor << "," << Z_sensor << "), "
                   << "orientation: (" << qW << "," << qX << "," << qY << "," << qZ << ")";
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    double quaternion_size = sqrt(qX*qX + qY*qY + qZ*qZ + qW*qW);
    if(quaternion_size != 1) {
        b_m_logger_msg << "CD: Orientation quaternion size is not equal to 1!!";
        //ANPL::anplLogMessage(b_m_p_cd_logger, warn, 0, b_m_logger_msg);
    }

    octomap::point3d sensor_position(static_cast<float>(X_sensor), static_cast<float>(Y_sensor), static_cast<float>(Z_sensor));
    octomath::Quaternion sensor_orientation(static_cast<float>(qW), static_cast<float>(qX), static_cast<float>(qY), static_cast<float>(qZ));
    m_senor_pose = octomap::pose6d(sensor_position, sensor_orientation);

    // get robot box size parameters
    double robot_box_size_x, robot_box_size_y, robot_box_size_z;
    nh_private.param("robot_size/box/X", robot_box_size_x, 0.5);
    nh_private.param("robot_size/box/Y", robot_box_size_y, 0.5);
    nh_private.param("robot_size/box/Z", robot_box_size_z, 0.5);
    m_robot_box = std::shared_ptr<fcl::Box>(new fcl::Box(robot_box_size_x, robot_box_size_y, robot_box_size_z));
    b_m_logger_msg << "CD: Robot " << b_m_robot_id << " box size: (" << robot_box_size_x
                   << "," << robot_box_size_y << "," << robot_box_size_z;
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    // get robot maximum acceleration
    nh_private.param<double>("robot_accel_max", m_a_max, 0.5);

    b_m_logger_msg << "CD: Octomap collision detector initialized";
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);
}


// Copy constructor
CollisionDetectionOctomap::CollisionDetectionOctomap(const CollisionDetectionOctomap& other) {
    CollisionDetectionOctomap();
}

// Copy assignment operator.
CollisionDetectionOctomap& CollisionDetectionOctomap::operator=(const CollisionDetectionOctomap& other) {

    CollisionDetectionOctomap();
}

void CollisionDetectionOctomap::laserCallback(const sensor_msgs::LaserScanConstPtr& scan) {
    std::shared_ptr<octomap::OcTree> p_octree_map(new octomap::OcTree(m_map_resolution));

    insertLaserScan(scan, p_octree_map);

    //b_m_logger_msg << "CD: Local map size: " << p_octree_map->size();
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    // publish map
    if (m_local_map_pub.getNumSubscribers() > 0) {
        octomap_msgs::Octomap map_msg;
        map_msg.header.frame_id = "world"; //xml
        map_msg.header.stamp = ros::Time::now();
        octomap_msgs::fullMapToMsg(*p_octree_map, map_msg); // (.ot)
        m_local_map_pub.publish(map_msg);
        //b_m_logger_msg << "CD: Publish map";
        //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);
    }

    fcl::OcTree tree(p_octree_map);
    std::vector<fcl::CollisionObject> boxes;
    generateBoxesFromOctomap(tree, boxes);
    //b_m_logger_msg << "CD: Generate boxes from octomap";
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    double lin_vel_x(b_m_current_odom_msg.twist.twist.linear.x);
    double lin_vel_y(b_m_current_odom_msg.twist.twist.linear.y);
    double lin_vel_z(b_m_current_odom_msg.twist.twist.linear.z);
    double ang_vel_x(b_m_current_odom_msg.twist.twist.angular.x);
    double ang_vel_y(b_m_current_odom_msg.twist.twist.angular.y);
    double ang_vel_z(b_m_current_odom_msg.twist.twist.angular.z);

    double lin_vec_size = sqrt(lin_vel_x*lin_vel_x + lin_vel_y*lin_vel_y + lin_vel_z*lin_vel_z);
    double norm_lin_vel_x = lin_vel_x / lin_vec_size;
    double norm_lin_vel_y = lin_vel_y / lin_vec_size;
    double norm_lin_vel_z = lin_vel_z / lin_vec_size;
    double ang_vec_size = sqrt(ang_vel_x*ang_vel_x + ang_vel_y*ang_vel_y + ang_vel_z*ang_vel_z);
    double norm_ang_vel_x = ang_vel_x / ang_vec_size;
    double norm_ang_vel_y = ang_vel_y / ang_vec_size;
    double norm_ang_vel_z = ang_vel_z / ang_vec_size;

    //b_m_logger_msg << "CD: Normalized linear velocity vec: (" << norm_lin_vel_x << "," << norm_lin_vel_y << "," << norm_lin_vel_z << ")";
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);
    //b_m_logger_msg << "CD: Linear speed: " << sqrt(lin_vel_x*lin_vel_x + lin_vel_y*lin_vel_y + lin_vel_z*lin_vel_z);
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);
    //b_m_logger_msg << "CD: Normalized angular velocity vec: (" << norm_ang_vel_x << "," << norm_ang_vel_y << "," << norm_ang_vel_z << ")";
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);
    //b_m_logger_msg << "CD: Angular speed: " << sqrt(ang_vel_x*ang_vel_x + ang_vel_y*ang_vel_y + ang_vel_z*ang_vel_z);
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

 /*   double max_stop_dist(0.8);
    double stop_dist;
    if(lin_vec_size < 0.01) {
        stop_dist = 0;
    }
    else {
        stop_dist = max_stop_dist;
    }
    fcl::Vec3f vel_vec(stop_dist*norm_lin_vel_x, stop_dist*norm_lin_vel_y, stop_dist*norm_lin_vel_z);
*/

    std_msgs::Bool collision_detected_msg;

    collision_detected_msg.data = false;
    
    fcl::Vec3f vel_vec(lin_vel_x, lin_vel_y, lin_vel_z);

    double stop_dist = vel_vec.sqrLength()/ (2*m_a_max); // scale equals to stoping distance of the robot given its maximal acceleration: v^2/(2 a_max)
    fcl::Vec3f unit_vel_vec (vel_vec/vel_vec.length());

    if(isObstacleAheadFCL(boxes, stop_dist * unit_vel_vec)) {
        collision_detected_msg.data = true;
	ROS_WARN("Collision warning!!!");

	/*  TODO Node Crash inside the if block!!!
/* INFO] [1519831088.262972356]: CD: Collision warning @ 179.443[deg]Index: 837
terminate called after throwing an instance of 'std::out_of_range'
  what():  vector::_M_range_check: __n (which is 837) >= this->size() (which is 720) */

        /*if(stop_dist != 0) {
            double collision_angle(atan2(norm_lin_vel_y, norm_lin_vel_x));
            int collosion_angle_index = (int) std::round(
                    (collision_angle - (scan->angle_min)) / (scan->angle_increment));

            b_m_logger_msg << "CD: Collision warning @ " << collision_angle * 180 / M_PI << "[deg]"
                           << "Index: " << collosion_angle_index;
            ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

            float distance_to_collision(0);
            try {
                distance_to_collision = scan->ranges.at(collosion_angle_index);
            }
            catch (...) {

            }

            b_m_logger_msg << "CD: Collision warning @ " << distance_to_collision << " [m], "
                           << collision_angle * 180 / M_PI << "[deg]";
            ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

        }*/

        b_m_logger_msg << "CD: Collision detected!!";
        //ANPL::anplLogMessage(b_m_p_cd_logger, warn, 0, b_m_logger_msg);

        b_m_logger_msg << "CD: Collision goal check: " << stop_dist * norm_lin_vel_x << ","
                       << stop_dist * norm_lin_vel_y   << "," << stop_dist * norm_lin_vel_z;
        //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

        auto min_range_iter = std::min_element(scan->ranges.begin(), scan->ranges.end());
        double angle_min = scan->angle_min;
        double angle_inc = scan->angle_increment;
        double angle_min_range = angle_min + angle_inc * (std::distance(scan->ranges.begin(), min_range_iter));
        b_m_logger_msg << "CD: Closest object at: " << *min_range_iter << "[m], " << angle_min_range << "[rad]";
        //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    }

    b_m_collision_warning_pub.publish(collision_detected_msg);
}

void CollisionDetectionOctomap::insertLaserScan(const sensor_msgs::LaserScanConstPtr& scan,
                                                std::shared_ptr<octomap::OcTree> p_octree_map) {
    if(scan->ranges.size() == 0) {
        b_m_logger_msg << "CD: Received empty cloud";
        //ANPL::anplLogMessage(b_m_p_cd_logger, warn, 0, b_m_logger_msg);
        return;
    }

    //b_m_logger_msg << "CD: Scan size: " << scan->ranges.size();
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    // cast laser msg to ros pointcloud
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan, cloud);

    //b_m_logger_msg << "CD: Cloud size: " << cloud.data.size();
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);

    // cast ros pointcloud to octomap pointcloud
    octomap::Pointcloud octomap_scan;
    octomap::pointCloud2ToOctomap(cloud, octomap_scan);

    octomap::point3d sensor_origin = octomap::point3d(0, 0, 0);

    p_octree_map->insertPointCloud(octomap_scan, sensor_origin, m_senor_pose, scan->range_max);
    p_octree_map->updateInnerOccupancy();
}

void CollisionDetectionOctomap::generateBoxesFromOctomap(const fcl::OcTree &tree, std::vector<fcl::CollisionObject> &boxes){
    for(auto&& box_ : tree.toBoxes()) {
        fcl::FCL_REAL x = box_[0];
        fcl::FCL_REAL y = box_[1];
        fcl::FCL_REAL z = box_[2];
        fcl::FCL_REAL size = box_[3];
        fcl::FCL_REAL cost = box_[4];
        fcl::FCL_REAL threshold = box_[5];
        fcl::Box* box = new fcl::Box(size, size, size);
        box->cost_density = cost;
        box->threshold_occupied = threshold;
        fcl::CollisionObject obj(std::shared_ptr<fcl::CollisionGeometry>(box), fcl::Transform3f(fcl::Vec3f(x, y, z)));
        boxes.push_back(obj);
    }
}

bool CollisionDetectionOctomap::isObstacleAheadFCL(const std::vector<fcl::CollisionObject>& boxes, const fcl::Vec3f& vel_vec) {
    //https://github.com/kuri-kustar/laser_collision_detection/blob/master/src/laser_obstacle_detect.cpp#L135-L228
    const static ros::Duration duration(0.01);

    fcl::Transform3f tf_orig;
    //fcl::Transform3f tf_orig = Conversion<fcl::Transform3f>::as(gtsam::Pose3());
    fcl::CollisionObject robot(m_robot_box, tf_orig);

    fcl::Quaternion3f q_goal; // Default quaternion is identity rotation.    
    fcl::Transform3f tf_goal(q_goal, vel_vec); // Construct transform from rotation and translation. 

    for (auto &&box : boxes) {
        fcl::ContinuousCollisionResult result;
        //http://ompl.kavrakilab.org/FCLMethodWrapper_8h_source.html
        //fcl::ContinuousCollisionRequest request(10, 0.0001, fcl::CCDM_SCREW, fcl::GST_LIBCCD, fcl::CCDC_CONSERVATIVE_ADVANCEMENT);
	//ContinuousCollisionRequest (std::size_t num_max_iterations_=10, S toc_err_=0.0001, CCDMotionType ccd_motion_type_=CCDM_TRANS, GJKSolverType gjk_solver_type_=GST_LIBCCD, CCDSolverType ccd_solver_type_=CCDC_NAIVE)
        fcl::ContinuousCollisionRequest request(15);
        fcl::Transform3f tf_box(box.getAABB().center());
        fcl::continuousCollide(&robot, tf_goal, &box, tf_box, request, result);
        if(result.is_collide) {
            //b_m_logger_msg << "CD: Collision detected";
            //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);
            return true;
        }
    }

    //b_m_logger_msg << "CD: No collision detected";
    //ANPL::anplLogMessage(b_m_p_cd_logger, info, 0, b_m_logger_msg);
    return false;
    //return isObstacleAhead(waypoint_origin, waypoint_end);
}

CollisionDetectionOctomap::~CollisionDetectionOctomap() {

}
