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
 * @file: state_machine_node_base.h
 * @brief: An interface for state machines
 * @author: Asaf Feniger
 *
 */

#ifndef STATE_MACHINE_NODE_BASE_H
#define STATE_MACHINE_NODE_BASE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

#include <gtsam/geometry/Pose3.h>

#include <spdlog/spdlog.h>

#define PLAN_FAIL_THRESHOLD 5

namespace ANPL {

    enum class State {
        Standby,
        Plan,
        Move,
        Exit,
        UsrCmd
    };

    struct StatesConditions {
        bool have_goals = false;
        bool have_actions = false;
        bool request_to_plan = false;
        bool received_actions = false;
        bool reached_waypoint = false;
        bool perceive = false;
        int plan_fail = false;
        bool mpc = false;
        bool passive_planner = false;
        bool pause_state_machine = false;
        bool stop_state_machine = false;
    };

    class StateMachineNodeBase {
    public:
        /**
         * Default constructor
         */
        StateMachineNodeBase();

        /**
         * Copy constructor
         * @param other
         */
        StateMachineNodeBase(const StateMachineNodeBase& other);

        /**
         * Copy assignment operator.
         * @param other
         * @return
         */
        StateMachineNodeBase& operator=(const StateMachineNodeBase& other);

        /**
         * Destructor
         */
        virtual ~StateMachineNodeBase();

        /**
         * save State name to string
         */
        std::string stateToString(const State& state_to_print);

        /**
         *
         * @return current state in the state machine
         */
        State getCurrentState();

    private:
        /// current state at the state machine
        State b_m_current_state;

    protected:

        /// pointer to logger
        std::shared_ptr<spdlog::logger> b_m_p_sm_logger;

        /// string for logger msgs
        std::stringstream b_m_logger_msg;

        /// path to current run folder
        std::string b_m_path_to_log_folder;

        /// ros service server to stop the state machine and exit
        ros::ServiceServer b_m_stop_state_machine_service;

        /// ros service server to pause the state machine
        ros::ServiceServer b_m_pause_state_machine_service;

        /// ros service server to continue
        ros::ServiceServer b_m_continue_state_machine_service;

        /// robot id
        char b_m_robot_id;

        /// robot name
        std::string b_m_robot_ns;

        /// ros node
        ros::NodeHandle b_m_state_machine_node;

        /// ros publisher for current state
        ros::Publisher b_m_state_pub;

        /// condition to determine next state
        StatesConditions b_m_states_conditions;

        /// tf listener to get the robot current pose
        tf::TransformListener b_m_tf_listener;

        /// current pose of the robot as gtsam pose3 object
        gtsam::Pose3 b_m_current_robot_pose_gtsam;

        /// current robot pose as geometry_msgs Pose object
        geometry_msgs::Pose b_m_current_robot_pose_ros;

        /// flag to check if the robot succeed to plan at least one time
        bool b_m_planned_once;

        /// number of intermediate actions executed, to control replanning for fixed prediction horizon
        unsigned int b_m_l;

        /**
         * Function to determine next state according to the states conditions
         */
        void nextState();

        /**
         * print current conditions to screen
         */
        void printCurrentStatesConditions();

        /**
         *
         * @return string with the current state conditions
         */
        std::string currentStatesConditionsToString();


        /**
         * get the current pose of the robot using tf listener
         */
        void getCurrentPose();

        /**
         * service callback to stop the state machine
         * @param req
         * @param res
         * @return
         */
        bool stopStateMachineCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        /**
         * service callback to pause the state machine
         * @param req
         * @param res
         * @return
         */
        bool pauseStateMachineCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        /**
         * service callback to continue the state machine
         * @param req
         * @param res
         * @return
         */
        bool continueStateMachineCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        /**
         * close node handles to shut down node
         */
        void closeNodeHandles();

    };

} // namespace ANPL

#endif // STATE_MACHINE_NODE_BASE_H
