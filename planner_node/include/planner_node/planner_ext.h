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
 * @file: planner_matlab.h
 * @brief: Extended Planner class with Matlab and Julia interface
 * @author: Andrej Kitanov
 *
 */


#ifndef PLANNER_MATLAB_H
#define PLANNER_MATLAB_H

#include "planner_node/planner.h"

#include <gtsam/geometry/Pose2.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <math.h>

//namespace ANPL {

class PlannerExt : public Planner
{
public:

    /**
     * constructor
     */
    PlannerExt(std::string planner_type);

    /**
     * destructor
     */
    virtual ~PlannerExt();

    /**
     * Read description in parent class
     */

    // overridden method
    unsigned int evaluateObjFn(std::vector<NonlinearFactorGraph>& graph, std::vector<Values>& initialEstimate, bool inBatchMode);
    std::string type; // "cpp" or "matlab"

protected:

    ros::Publisher      m_ext_pub;
    ros::Subscriber     m_ext_sub;

    std::string         m_ext_request_topic;
    std::string         m_ext_response_topic;

    std::string         m_matlab_boost_archive_version;
    std::string         m_ros_boost_archive_version;

    bool                m_is_received;
    bool                m_is_correct;

    unsigned int        m_ext_answer;
    std::string         m_request;
    std::string         m_delimiter;

    void extPlannerCallback(const std_msgs::UInt32& msg);

};
//}

#endif // PLANNER_MATLAB_H
