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
 * @file: config.h
 * @brief: Planning constants
 * @author: Andrej Kitanov
 *
 */

#ifndef PLANNER_CONFIG_H
#define PLANNER_CONFIG_H

#define NUM_ROBOTS 1 // robots ids are 'A', 'B', ...
// be sure to define number of colors at least equal to the number of robots.
/* Colors can be specified using one of four formats:
 1. "#%2x%2x%2x"	Red-Green-Blue (RGB)
 2. "#%2x%2x%2x%2x"	Red-Green-Blue-Alpha (RGBA)
 3. "H[, ]+S[, ]+V"	Hue-Saturation-Value (HSV) 0.0 <= H,S,V <= 1.0
 4. string	color name */
static const std::string ROBOT_COLORS[] = {"red", "cyan", "yellowgreen", "yellow", "wheat", "violet", "turquoise", "tomato", "thistle"};
#define LC_DISTANCE_THRESHOLD 2.0
#define LC_RELATIVE_ORIENTATION_THRESHOLD 1.0
#define USING_MR_FACTORS true
// define Matlab message codes
enum MATLAB_MSG_CODES {MATLAB_SEND_ONLY_POSTERIOR, MATLAB_SEND_PRIOR_AND_POSTERIOR};
// ... and what to send
#define MATLAB_SEND_MSG_CODE MATLAB_SEND_PRIOR_AND_POSTERIOR

using namespace gtsam;

// prior noise model
static const noiseModel::Diagonal::shared_ptr priorNoiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(1e-6), Vector3::Constant(0.001))); //Vector3::Constant(3*M_PI/180), Vector3::Constant(0.01))); // 3 degrees std on roll,pitch,yaw and 10 cm std on x,y,z

// Odometry noise model
static const noiseModel::Diagonal::shared_ptr odometryNoiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1)); //0.01, 0.01, 0.1, 0.2, 0.2, 0.2)); // RPY, XYZ

// Loop closures noise model
static const noiseModel::Diagonal::shared_ptr measurementNoiseModel = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1)); //0.01, 0.01, 0.1, 0.2, 0.2, 0.2));  // RPY, XYZ

// Noise model for multi-robot relative pose measurements
static const noiseModel::Diagonal::shared_ptr mr_rel_pose_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.015, 0.015, 0.015, 0.1, 0.1, 0.1)); //0.01, 0.01, 0.1, 0.2, 0.2, 0.2));  // RPY, XYZ


#endif //PLANNER_CONFIG_H
