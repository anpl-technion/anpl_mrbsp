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
 * @file: state_machine_default.h
 * @brief:
 * @author: Asaf Feniger
 *
 */

#ifndef STATE_MACHINE_DEFAULT_H
#define STATE_MACHINE_DEFAULT_H

#include "state_machine_node_base.h"

#include <mrbsp_msgs/WaitForActions.h>
#include <mrbsp_msgs/Actions.h>
#include <std_msgs/Bool.h>

namespace ANPL {

    class StateMachineDefault : public StateMachineNodeBase {
    public:
        /**
         * Default constructor
         */
        StateMachineDefault();

        /**
         * Copy constructor
         * @param other
         */
        StateMachineDefault(const StateMachineDefault& other);

        /**
         * Copy assignment operator.
         * @param other
         * @return
         */
        StateMachineDefault& operator=(const StateMachineDefault& other);

        /**
         * Destructor
         */
        virtual ~StateMachineDefault();

    private:

        /// vectors of goals to reach
        std::vector<gtsam::Point3> m_goals;

        /// vector of actions to perform
        std::vector<gtsam::Point3> m_waypoints;

        /// ros service client to ask the controller to move the robot
        ros::ServiceClient m_controller_move_robot_client;

        /// ros service client to get the current belief from the belief node
        ros::ServiceClient m_get_belief_client;

        /// ros service client to re draw the map
        ros::ServiceClient m_re_draw_map_client;

        /// ros service client to request the planner node to plan
        ros::ServiceClient m_request_to_plan_client;

        /// ros subscriber to get the optimal actions from the planner
        ros::Subscriber m_optimal_action_sub;

        /// ros publisher to publish current waypoint
        ros::Publisher m_waypoint_pub;

        /// flag whether to call the controller via ros service or ros publisher/subscriber
        bool m_request_to_move_with_service;

        /// flag to check if the state machine receive new actions
        bool m_receive_actions;

        /// ros service server to continue
        ros::ServiceServer m_skip_next_waypoint_service;

        /// passive planner parameter full name
        std::string m_passive_planner_param;

        /// mpc parameter full name
        std::string m_mpc_param;

        /// mpc time horizon -  execute this number of immediate actions before triggering mpc again
        int m_Ts;

        /// mpc prediction horizon
        int m_L;

        /// mpc parameter - when true do mpc planning session, but stay with previous plan until the next goal
        /// (mostly used for testing in static environments with ground truth actions)
        bool m_stay_on_global_path;

        /// each mpc step replan to the goal, otherwise look at prediction horizon
        bool m_goal_oriented_mpc;

        /// action generation method
        std::string m_action_generation_method;

        /// run state machine until the current state is Exit
        void initStateMachine();

        /**
         * Functions to set the states conditions
         */
        void setStateConditions();

        /**
         * Activate plan node
         */
        void requestToPlan();

        /**
         * Activate controller node
         */
        void requestToMove();

        /**
         * Check if the robot reached current goal
         * @return
         */
        bool isGoalAchived(gtsam::Pose3& current_pose);

        /**
         * Check if the robot reached current waypoint
         * @return
         */
        bool isWaypointReached(gtsam::Pose3& current_pose);

        /**
         * service callback for move request
         * @param req
         * @param res
         * @return
         */
        bool waitForActions(mrbsp_msgs::WaitForActions::Request& req, mrbsp_msgs::WaitForActions::Response& res);

        /**
         *
         * @param actions
         */
        void optimalActionsCallback(const mrbsp_msgs::ActionsConstPtr& actions);

        /**
         *
         * @param wyapoints
         */
        std::string printActions(std::vector<gtsam::Point3>& waypoints);

        /// ros subscriber for immediate collision warnings
        ros::Subscriber m_immediate_collision_sub;

        /**
         *
         * @param collision_warning
         */
        void collisionDetectionCallback(const std_msgs::BoolConstPtr& collision_warning);

        /// immediate collision flag
        bool m_immediate_collision;

        /**
         * service callback to skip next waypoint
         * @param req
         * @param res
         * @return
         */
        bool skipNextWaypointCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    };

} // namespace ANPL

#endif // STATE_MACHINE_DEFAULT_H
