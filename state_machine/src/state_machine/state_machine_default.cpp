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
 * @file: state_machine_node_base.cpp
 * @brief:
 * @author: Asaf Feniger
 */

#include "state_machine_default.h"
// mrbsp headers
#include <mrbsp_utils/conversion.h>
#include <mrbsp_utils/mrbsp_utils.h>
#include <mrbsp_msgs/RequestToMove.h>
#include <mrbsp_msgs/RequestBelief.h>
#include <mrbsp_msgs/InitCheck.h>
#include <mrbsp_msgs/RequestToPlan.h>
#include <mrbsp_msgs/ReDrawMap.h>
// gtsam headers
#include <gtsam/base/serialization.h>
#include <gtsam/nonlinear/ISAM2.h>
//
#include <math.h>
// octomap
#include <octomap_msgs/Octomap.h>


using namespace MRBSP;

StateMachineDefault::StateMachineDefault():
        StateMachineNodeBase(),
        m_receive_actions(false)
{
    ros::NodeHandle pnh("~");

/*    if(!pnh.hasParam("/logger/loggerPath")) {
        ROS_WARN("Unable to find log folder...");
        ros::Duration(1.0).sleep();
        if(pnh.hasParam("/logger/loggerPath")) {
            pnh.getParam("/logger/loggerPath", b_m_path_to_log_folder);
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
        pnh.getParam("/logger/loggerPath", b_m_path_to_log_folder);
    }*/
    MRBSP::Utils::initLogger(pnh);

    b_m_logger_msg << "SM: Initialize state machine logger";
    //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

    std::string srvice_name(b_m_robot_ns + "/controller_request_to_move");
    m_controller_move_robot_client = b_m_state_machine_node.serviceClient<mrbsp_msgs::RequestToMove>(srvice_name);
    m_get_belief_client = b_m_state_machine_node.serviceClient<mrbsp_msgs::RequestBelief>("/request_belief");
    m_re_draw_map_client = b_m_state_machine_node.serviceClient<mrbsp_msgs::ReDrawMap>("/Map/re_draw_map");
    m_request_to_plan_client = b_m_state_machine_node.serviceClient<mrbsp_msgs::RequestToPlan>("/send_plan_data");

    m_optimal_action_sub = b_m_state_machine_node.subscribe("/optimal_joint_action", 10, &StateMachineDefault::optimalActionsCallback, this);


    b_m_logger_msg << "Current state: " << stateToString(getCurrentState());
    //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

    unsigned int goal_number = 1;
    while(true) {
        std::string goal_name("goals/goal" + std::to_string(goal_number));
        if (pnh.hasParam(goal_name)) {
            // expect to get vector with 3 parameters (x,y,z)
            std::vector<double> goal_ros;
            pnh.getParam(goal_name, goal_ros);
            gtsam::Point3 point(goal_ros.at(0), goal_ros.at(1), goal_ros.at(2));
            m_goals.emplace(m_goals.end(), point);

            b_m_logger_msg << "SM: Goal #" << goal_number << ": X = " << point.x() << ", Y = " << point.y() << ", Z = " << point.z();
            //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

            goal_number++;
        }
        else {
            b_m_logger_msg << "SM: Cant find " << goal_name;
            //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

            break;
        }
    }

    if(!m_goals.empty()) {
        b_m_logger_msg << "SM: Receive " << goal_number -1 << " goals";
        //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
    }
    else {
        b_m_logger_msg << "SM: Didn't receive goals!";
        //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
    }

    std::string collision_detector_topic(b_m_robot_ns + "/Collision_warning");
    m_immediate_collision_sub = b_m_state_machine_node.subscribe(collision_detector_topic, 1,
                                                                 &StateMachineDefault::collisionDetectionCallback, this);

    m_immediate_collision = false;

    std::string waypoint_pub_topic(b_m_robot_ns + "/next_waypoint");
    m_waypoint_pub = b_m_state_machine_node.advertise<geometry_msgs::Point>(waypoint_pub_topic, 1);

    // get parameter whether to use service to call the controller
    pnh.param("request_to_move_with_service", m_request_to_move_with_service, true);

    std::string skip_waypoint_topic(b_m_robot_ns + "/skip_next_waypoint");
    m_skip_next_waypoint_service = b_m_state_machine_node.advertiseService(skip_waypoint_topic, &StateMachineDefault::skipNextWaypointCallback, this);

    std::string state_switching_params_ns(ros::this_node::getName() + "/state_switching_conditions/");
    m_passive_planner_param = std::string(state_switching_params_ns + "passive_planner");
    b_m_state_machine_node.param(m_passive_planner_param, b_m_states_conditions.passive_planner, false);

    m_mpc_param = std::string(state_switching_params_ns + "mpc");
    b_m_state_machine_node.param(m_mpc_param, b_m_states_conditions.mpc, false);
    std::string Ts_param = state_switching_params_ns + "time_horizon";
    b_m_state_machine_node.param(Ts_param, m_Ts, 1);
    std::string L_param = state_switching_params_ns + "prediction_horizon";
    b_m_state_machine_node.param(L_param, m_L, 1);
    b_m_l = 0;

    std::string global_path_param(ros::this_node::getName() + "/stay_on_global_path");
    b_m_state_machine_node.param(global_path_param, m_stay_on_global_path, false);
    std::string goal_oriented_mpc_param(ros::this_node::getName() + "/goal_oriented_mpc");
    b_m_state_machine_node.param(goal_oriented_mpc_param, m_goal_oriented_mpc, true);


    if (b_m_states_conditions.mpc) {
        if (m_goal_oriented_mpc)
            ROS_WARN("SM: MPC style planning with time step %d and goal oriented", m_Ts);
        else
            ROS_WARN("SM: MPC style planning with time step %d and prediction horizon = %d", m_Ts, m_L);
    }


    // action generation method. if == "interactive", isGoalAchived will always return false.
    b_m_state_machine_node.getParam("/action_generator/method", m_action_generation_method);

    b_m_logger_msg << "Initialize default state machine.";
    //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

    initStateMachine();

    //std::cout << "Initial conditions:" << std::endl;
    //printCurrentStatesConditions();
    //std::cout << "Current state: " << stateToString(getCurrentState()) << std::endl; // Standby
    //std::cout << std::endl;

    b_m_logger_msg << "Initial conditions: \n" << currentStatesConditionsToString();
    //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

    closeNodeHandles();
}

StateMachineDefault::StateMachineDefault(const StateMachineDefault& other):
        StateMachineDefault()
{

}

StateMachineDefault& StateMachineDefault::operator=(const StateMachineDefault& other) {
    StateMachineDefault();
}

void StateMachineDefault::initStateMachine() {
    static int counter = 0;
    while (ros::ok()) {
        switch(getCurrentState()) {
            case State::UsrCmd:
                // check if the user finish the run
                break;

            case State::Standby:
                requestToMove();
                break;

            case State::Plan:
                // remove old plan (?)
                ros::Duration(1.0).sleep(); // at least stay 1.0 s in standby to allow nodes to initialize/finish work before planning
                requestToPlan();
                break;

            case State::Move:
                requestToMove();
                if (b_m_states_conditions.mpc) {
                    b_m_states_conditions.have_actions = false; // this forces the robot to replan on the next waypoint
                }
                break;

            case State::Exit:
                m_waypoints.clear();
                m_goals.clear();
                requestToMove();
                return;
        }

        setStateConditions();
        //printCurrentStatesConditions();
        State current_state = getCurrentState();
        nextState();
        if(current_state != getCurrentState()) {
            b_m_logger_msg << "SM: " << counter << ") New state: " << stateToString(getCurrentState());
            //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
            counter++;

            if(getCurrentState() == State::Plan) { // on every change to the planning state
                b_m_states_conditions.request_to_plan = false;
                b_m_states_conditions.have_actions = false;
                b_m_states_conditions.received_actions = false;
                b_m_states_conditions.reached_waypoint = false;
                if (!m_stay_on_global_path)
                    m_waypoints.clear();
                b_m_l = 0;
                ROS_WARN("Resetting planner");
                printCurrentStatesConditions();
            }
        }

        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
}

void StateMachineDefault::setStateConditions() {
    // Check immediate collision
    if(m_request_to_move_with_service && m_immediate_collision) { // replan in case of collision detection, otherwise recover somehow
        m_waypoints.clear();
        b_m_states_conditions.have_actions = false;
    }

    // Check if the user change the state switching conditions parameter (passive planner / mpc)
    if(b_m_state_machine_node.hasParam(m_passive_planner_param)) {
        b_m_state_machine_node.getParam(m_passive_planner_param, b_m_states_conditions.passive_planner);
    }
    if(b_m_state_machine_node.hasParam(m_mpc_param) && b_m_l == m_Ts) {
        b_m_state_machine_node.getParam(m_mpc_param, b_m_states_conditions.mpc); // rising edge of MPC variable
        if ( b_m_states_conditions.mpc)
            ROS_WARN("MPC horizon reached.");
    }

    // Check perceiving status
    if(!b_m_states_conditions.perceive) {
        std::string service_name("/" + b_m_robot_ns + "/check_if_perceive");
        ros::ServiceClient check_if_perceive_client = b_m_state_machine_node.serviceClient<mrbsp_msgs::InitCheck>(service_name);
        mrbsp_msgs::InitCheck check_if_perceive_srv;

        b_m_logger_msg << "SM: call service: " << service_name;
        //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

        if(check_if_perceive_client.call(check_if_perceive_srv)) {
            b_m_states_conditions.perceive = check_if_perceive_srv.response.init_check_answer;

            if(!b_m_states_conditions.perceive)
                b_m_logger_msg << "Wait for 5 seconds...";
                //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
                ros::Duration(5.0).sleep(); // sleep for 5 seconds
        }
        else {
            b_m_logger_msg << "Unable to check perceiving status.";
            //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
            b_m_states_conditions.perceive = false;
        }
    }

    getCurrentPose();

    // check if the robot has goals
    if(m_goals.empty()) {
        b_m_states_conditions.have_goals = false;
    }
    else {
        b_m_states_conditions.have_goals = true;
        if(isGoalAchived(b_m_current_robot_pose_gtsam)){
            m_goals.erase(m_goals.begin());
            if(m_goals.empty()) {
                b_m_states_conditions.have_goals = false;
            }
        }

    }

    if(m_waypoints.empty()) {
        b_m_states_conditions.have_actions = false;
    }
    else if (!b_m_states_conditions.mpc) {
        b_m_states_conditions.have_actions = true;
    }

    // immediate_collision   -> collision detector node

    // future_collision      -> collision detector node (?)

    // uncertainty_belief    -> belief node

    // uncertainty_odometry  -> odometry node

    // low_battery           -> robot driver

    // time_limit            -> pre-define parameter

    // sensors_on            -> odometry_node (?)

    // goal_achieved         -> state_machine_node

    // reach_waypoint        -> state_machine_node

    // mpc_flag              -> pre-define parameter

}

void StateMachineDefault::requestToPlan() {
    // empty
    // current pose
    // belief
    // current pose + goal
    // belief + goal

    // Request to plan
    // geometry_msgs/Pose start
    // geometry_msgs/Pose goal
    // string gtsam_values_str
    // string gtsam_factors_str
    // char robot_id

    static bool print_msg = true;
    if(b_m_states_conditions.request_to_plan) {
        if (!b_m_states_conditions.received_actions) {
            if (print_msg) {
                b_m_logger_msg << "Wait for optimal actions from planner...";
                //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
                print_msg = false;
            }
            return;
        }
        else {
            b_m_states_conditions.request_to_plan = false;
        }
    }
    else {
        b_m_states_conditions.received_actions = false;
        print_msg = true;

        // translate goal to ros geometry_msgs::Pose object
        gtsam::Pose3 goal_gtsam = gtsam::Pose3(gtsam::Rot3(), m_goals.at(0));
        geometry_msgs::Pose current_robot_goal_ros = Conversion<geometry_msgs::Pose>::as(goal_gtsam);

        // get current belief
        mrbsp_msgs::RequestBelief belief_request;

        std::string planner_mode;
        ros::param::param<std::string>("/planner/mode", planner_mode, "batch");
        if (planner_mode.compare("incremental") == 0)
            belief_request.request.getDeltas.data = true; // only added factors and vars will be retured if this is true, everything otherwise
        else
            belief_request.request.getDeltas.data = false;

        std::string graph_serialized;
        std::string values_serialized;

        b_m_logger_msg << "SM: Ask for current belief in " << planner_mode << " mode.";
        //ANPL::anplLogMessage(b_m_p_sm_logger, info, 1, b_m_logger_msg);
        //b_m_p_sm_logger->flush();

        if (m_get_belief_client.call(belief_request)) {
            graph_serialized = belief_request.response.graph_string;
            values_serialized = belief_request.response.values_string;

            gtsam::Values vals;
            try {
                //b_m_logger_msg << "SM: values_serialized length: " << values_serialized.size();
                //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
                gtsam::deserialize(values_serialized, vals);

                b_m_logger_msg << "SM: Deserialized values. Receive belief with " << vals.size() << " values";
                //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
            }
            catch (...) {
                b_m_logger_msg << "SM: Unable to deserialize values";
                //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
            }

            if(vals.empty() && !belief_request.request.getDeltas.data) { // values can not be empty in batch mode, while in incremental they can
                b_m_logger_msg << "SM: Receive empty values";
                //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
                return;
            }

            gtsam::NonlinearFactorGraph graph;
            try {
                //b_m_logger_msg << "SM: graph_serialized length: " << graph_serialized.size();
                //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
                gtsam::deserialize(graph_serialized, graph);

                b_m_logger_msg << "SM: Deserialized graph. Receive belief with " << graph.size() << " factors";
                //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
            }
            catch (...) {
                b_m_logger_msg << "SM: Unable to deserialize graph";
                //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
            }

            if(graph.empty() && !belief_request.request.getDeltas.data) { // graph can not be empty in batch mode, while in incremental it can
                b_m_logger_msg << "SM: Receive empty graph";
                //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
                return;
            }

        } else {
            b_m_logger_msg << "SM: Fail receive current belief";
            //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
            return; // check in different branch...
        }

        mrbsp_msgs::ReDrawMap re_draw_map;
        octomap_msgs::Octomap incremental_map;
        unsigned int re_draw_attemp = 0;
        while(b_m_state_machine_node.ok() && re_draw_attemp < 10) {
            if (m_re_draw_map_client.call(re_draw_map)) {
                incremental_map = re_draw_map.response.incremental_map;
                b_m_logger_msg << "SM: Re-draw map";
                //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
                break;
            } else {
                b_m_logger_msg << "SM: Unable to re-draw map, try again in 5 sec";
                //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
                ros::Duration(5.0).sleep();
                ++re_draw_attemp;
            }
            b_m_logger_msg << "SM: Re-draw map failed, proceed without re-drawing";
            //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
        }

        mrbsp_msgs::RequestToPlan request_to_plan;
        request_to_plan.request.robot_id = static_cast<unsigned char>(b_m_robot_id);
        request_to_plan.request.start = b_m_current_robot_pose_ros;
        if (b_m_states_conditions.mpc && !m_goal_oriented_mpc && m_waypoints.size() >= m_L)
            request_to_plan.request.goal = Conversion<geometry_msgs::Pose>::as(gtsam::Pose3(b_m_current_robot_pose_gtsam.rotation(), m_waypoints.at(m_L-1)));
        else
            request_to_plan.request.goal = current_robot_goal_ros;
        request_to_plan.request.gtsam_factors_str = graph_serialized;
        request_to_plan.request.gtsam_values_str = values_serialized;
        request_to_plan.request.edges_list = belief_request.response.edges_list;
        request_to_plan.request.nodes_list = belief_request.response.nodes_list;
        request_to_plan.request.incremental_map = incremental_map;
        ROS_WARN("SM: Planner calling...");
        if (m_request_to_plan_client.call(request_to_plan)) {
            // Wait for actions
            b_m_states_conditions.request_to_plan = true;
            ROS_WARN("SM: Planner responded.");
        } else {
            b_m_logger_msg << "SM: Fail to call planner";
            //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
        }
        printCurrentStatesConditions();
    }
}

void StateMachineDefault::requestToMove() {
    // send request to controller with:
    // 1) current pose
    // 2) current waypoint

    if(m_request_to_move_with_service) {
        // Call controller via ros service
        mrbsp_msgs::RequestToMove controller_request_to_move;
        // if state isn't Move, stop the robot (request with empty string.

        if (getCurrentState() == State::Move) {
            controller_request_to_move.request.gtsam_pose = gtsam::serialize(b_m_current_robot_pose_gtsam);
            controller_request_to_move.request.gtsam_goal = gtsam::serialize(m_waypoints.at(0));
        }

        if(m_controller_move_robot_client.call(controller_request_to_move)) {
            b_m_states_conditions.reached_waypoint = controller_request_to_move.response.reached_waypoint;
            if(b_m_states_conditions.reached_waypoint && !m_waypoints.empty()) {
                b_m_logger_msg << "SM: Reached waypoint "
                               << "(X = " << m_waypoints.begin()->x()
                               << " Y = " << m_waypoints.begin()->y()
                               << " Z = " << m_waypoints.begin()->z() << ")";
                //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
                m_waypoints.erase(m_waypoints.begin());
            }
        }
    }

    else {
        // Call controller via ros publisher/subscriber

        /// next waypoint to go to
        geometry_msgs::Point m_next_waypoint_ros;

        if (getCurrentState() == State::Move) {
            m_next_waypoint_ros = Conversion<geometry_msgs::Point>::as(m_waypoints.at(0));

            b_m_states_conditions.reached_waypoint = isWaypointReached(b_m_current_robot_pose_gtsam);

            if(b_m_states_conditions.reached_waypoint && !m_waypoints.empty()) {
                b_m_logger_msg << "SM: Reached waypoint "
                               << "(X = " << m_waypoints.begin()->x()
                               << " Y = " << m_waypoints.begin()->y()
                               << " Z = " << m_waypoints.begin()->z() << ")";
                b_m_logger_msg << "/ Robot at pose: (" << b_m_current_robot_pose_gtsam.x() << ", " << b_m_current_robot_pose_gtsam.y() << ", " << b_m_current_robot_pose_gtsam.z();
                //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
                m_waypoints.erase(m_waypoints.begin());
            }

            m_waypoint_pub.publish(m_next_waypoint_ros);

        }
        else {
            m_next_waypoint_ros = Conversion<geometry_msgs::Point>::as(b_m_current_robot_pose_gtsam.translation());
        }	 
    }


}

bool StateMachineDefault::isGoalAchived(gtsam::Pose3& current_pose) {
    static int goal_counter = 0;

    if(m_action_generation_method.compare(std::string("interactive")) == 0) {
        return false;
    }

    if(b_m_states_conditions.have_actions && m_waypoints.empty() && !m_goals.empty()) {
        //m_goals.erase(m_goals.begin());
        b_m_logger_msg << "SM: Reached goal #" << goal_counter
                       << "(X = " << m_goals.begin()->x()
                       << " Y = " << m_goals.begin()->y()
                       << " Z = " << m_goals.begin()->z() << ")";
        //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
        goal_counter++;
        return true;
    }
    return false;
}

bool StateMachineDefault::isWaypointReached(gtsam::Pose3& current_pose) {
    double dist_threshold;
    // TODO consider making dist_threshold a function of position covariance
    b_m_state_machine_node.param<double>("/" + b_m_robot_ns + "/controller_pioneer/goal_radius", dist_threshold, 0.1);
    //ROS_WARN_STREAM("Dist THR set to " << dist_threshold);
    return current_pose.translation().distance(m_waypoints.at(0)) < dist_threshold;
}


void StateMachineDefault::optimalActionsCallback(const mrbsp_msgs::ActionsConstPtr& actions) {
    b_m_logger_msg << "SM: Receive actions msg from the planner.";
    //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

    if (m_stay_on_global_path && !m_waypoints.empty()) { // if staying on a non-empty global path, ignore returned actions and set flags

        b_m_states_conditions.received_actions = true;
        b_m_states_conditions.have_actions = true;
        b_m_states_conditions.plan_fail = 0;
        b_m_states_conditions.mpc = false; // falling edge of MPC
        return;
    }

    m_waypoints.clear();
    auto robot_num = static_cast<unsigned int>(b_m_robot_id - 'A');
    gtsam::Pose3 b_m_current_robot_pose_gtsam;

    if(!actions->actions.empty()) {
        if(!actions->actions.at(robot_num).poses.empty()) {
            /*if (b_m_states_conditions.mpc){
                b_m_current_robot_pose_gtsam = Conversion<gtsam::Pose3>::as(
                        actions->actions.at(robot_num).poses.at(1).pose);
                m_waypoints.push_back(b_m_current_robot_pose_gtsam.translation());
                ROS_WARN("MPC CALLBACK ---------------------------------------");
            } else {*/
                for (int i = 1; i < actions->actions.at(robot_num).poses.size(); i++) {
                    b_m_current_robot_pose_gtsam = Conversion<gtsam::Pose3>::as(
                            actions->actions.at(robot_num).poses.at(i).pose);
                    m_waypoints.push_back(b_m_current_robot_pose_gtsam.translation());
                }

                //m_waypoints.erase(m_waypoints.begin());
            //}


            if (!m_waypoints.empty()) {
                b_m_states_conditions.received_actions = true;
                b_m_states_conditions.have_actions = true;
                b_m_states_conditions.plan_fail = 0;
                b_m_states_conditions.mpc = false; // falling edge of MPC
            }

            b_m_logger_msg << printActions(m_waypoints);
            //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

        } else {
            b_m_states_conditions.request_to_plan = false;
            b_m_states_conditions.plan_fail++;

            b_m_logger_msg << "SM: Planner didn't return action for robot " << b_m_robot_id;
            //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
        }
    } else {
        b_m_states_conditions.request_to_plan = false;
        b_m_states_conditions.plan_fail++;

        b_m_logger_msg << "SM: Planner returned empty message";
        //ANPL::anplLogMessage(b_m_p_sm_logger, warn, 0, b_m_logger_msg);
    }

    b_m_logger_msg << "SM: Receive " << m_waypoints.size() << " actions to perform.";
    //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);

    printCurrentStatesConditions();
}

std::string StateMachineDefault::printActions(std::vector<gtsam::Point3>& waypoints) {
    int wp_counter = 0;
    std::stringstream str_print;
    for(auto waypoint : waypoints) {
        str_print << "Waypoint #" << std::to_string(wp_counter) << ":"
                  << " X = " << waypoint.x() << ", Y = " << waypoint.y() << ", Z = " << waypoint.z() << "\n";
        wp_counter++;
    }
    return str_print.str();
}

void StateMachineDefault::collisionDetectionCallback(const std_msgs::BoolConstPtr& collision_warning) {
    if(collision_warning->data) {
        m_immediate_collision = true;
    }
    else {
        m_immediate_collision = false;
    }

}

bool StateMachineDefault::skipNextWaypointCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    if(!m_waypoints.empty()) {
        m_waypoints.erase(m_waypoints.begin());
        b_m_states_conditions.reached_waypoint = true;
        b_m_logger_msg << "SM: User remove next waypoint";
        //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
    }
    else {
        b_m_logger_msg << "SM: User attempt tp remove next waypoint. No waypoints to remove.";
        //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
    }
}

StateMachineDefault::~StateMachineDefault() {
    b_m_logger_msg << "Shutting down state machine...";
    //ANPL::anplLogMessage(b_m_p_sm_logger, info, 0, b_m_logger_msg);
}
