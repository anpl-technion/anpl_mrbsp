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
 * @file: collision_detection_octomap.h
 * @brief:
 * @author: Asaf Feniger
 */

#ifndef COLLISION_DETECTIONS_OCTOMAP_H
#define COLLISION_DETECTIONS_OCTOMAP_H

#include "collision_detection_base.h"

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <octomap/OcTree.h>

#include <fcl/octree.h>
#include <fcl/shape/geometric_shapes.h>

// ros headers
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
// gtsam headers
#include <gtsam/geometry/Pose3.h>

namespace MRBSP {

    /**

    */

    class CollisionDetectionOctomap : public CollisionDetectionBase {
    public:
        /**
         * Default constructor - empty
         */
        CollisionDetectionOctomap();

        /**
         * Copy constructor
         * @param collision_detection
         */
        CollisionDetectionOctomap(const CollisionDetectionOctomap& other);

        /**
         * Copy assignment operator.
         * @param collision_detection
         * @return
         */
        CollisionDetectionOctomap& operator=(const CollisionDetectionOctomap& other);

        /**
        * Destructor
        */
        virtual ~CollisionDetectionOctomap();

    private:

        // **ROS publishers and subscribers**
        /// ros subscriber for laser scans messages
        ros::Subscriber m_laser_sub;

        /// ros publisher for local map
        ros::Publisher m_local_map_pub;

        /// flag whether to publish empty collision warning
        bool m_publish_empty_msg;

        /// flag whether to prioritize movement
        bool m_stop_for_other_robot;
        /// minimum distance allowed from other robots
        bool m_min_dist_other_robot;

        /// subscribers to other robots poses
        std::vector<ros::Subscriber> m_stop_sub;
        /// current poses of other robots
        std::vector<geometry_msgs::Pose> m_current_pose; 

        /**
        * @brief Callback function to handle ros laser messages
        * @param scan - pointer to ros laser message
        */
        void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);


        // **Octomap**
        /// local map resolution
        double m_map_resolution;

        /// sensor pose in the robot body frame
        octomap::pose6d m_senor_pose;

        /**
         * @brief Function to generate octomap from laser scan message
         * @param scan - ros laser scan msg
         * @param p_octree_local_map - pointer to octree object
         */
        void insertLaserScan(const sensor_msgs::LaserScanConstPtr& scan,
                             std::shared_ptr<octomap::OcTree> p_octree_map);


        // **FCL**
        /// smart pointer to robot box fcl model
        std::shared_ptr<fcl::Box> m_robot_box;

        /// maximum robot acceleration to calculate safe stopping distance at the current speed before hitting obstacles
        double m_a_max;

        /**
         * @brief Function to generate FCL boxes from fcl::octree object
         * @param tree - fcl's octree representation
         * @param boxes - collision objects from the octree
         */
        void generateBoxesFromOctomap(const fcl::OcTree &tree, std::vector<fcl::CollisionObject> &boxes);

        /**
         * @brief Function to detect collisions with respect to the robot size
         * @param boxes - collision object to check for collisions
         * @return
         */
        bool isObstacleAheadFCL(const std::vector<fcl::CollisionObject>& boxes, const fcl::Vec3f& vel_vec);
    };
}

#endif // COLLISION_DETECTIONS_OCTOMAP_H
