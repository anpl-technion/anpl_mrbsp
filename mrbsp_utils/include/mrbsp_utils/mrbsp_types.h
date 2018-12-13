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
 * @file: mrbsp_types.h
 * @brief:
 * @author: Tal Regev
 */

#ifndef MRBPS_TYPES_H
#define MRBPS_TYPES_H

#include <tuple>
#include <string>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/Value.h>
#include <gtsam/geometry/Pose3.h>


namespace MRBSP {

    namespace Utils {

        typedef std::tuple<std::string, double, sensor_msgs::LaserScan, gtsam::Pose3, geometry_msgs::Pose> Keyframe;
        typedef std::tuple<std::string, double, sensor_msgs::LaserScan, gtsam::Pose3, sensor_msgs::PointCloud2, geometry_msgs::Pose> KeyframeRgbd;
        typedef std::pair<std::vector<gtsam::Point2>, gtsam::Pose3> KeyframeData;
        typedef std::map<unsigned int, KeyframeData> SingleRobotKeyframe;
        typedef std::map<char, SingleRobotKeyframe> AllKeyframes;
        typedef std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> NewFactorsAndValues;
        typedef std::pair<sensor_msgs::LaserScan, gtsam::Pose3> MapData;
        typedef std::pair<sensor_msgs::PointCloud2, gtsam::Pose3> MapData3D;
    }
}

#endif // MRBPS_TYPES_H

