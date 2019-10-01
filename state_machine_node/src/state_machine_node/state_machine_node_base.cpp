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

#include "state_machine_node_base.h"

#include <mrbsp_utils/conversion.h>
#include <std_msgs/String.h>


using namespace ANPL;

StateMachineNodeBase::StateMachineNodeBase():
        b_m_current_state(State::Standby),
        b_m_planned_once(false)
{
    ros::NodeHandle pnh("~");
    std::string robot_id;
    pnh.param("robot_id", robot_id, std::string("A"));
    b_m_robot_id = robot_id.at(0);
    //pnh.param("robot_name", b_m_robot_name, std::string("Robot_A"));
    b_m_robot_ns = std::string("/Robot_" + robot_id);

    std::string state_pub_topic(b_m_robot_ns + "/robot_state");
    b_m_state_pub = b_m_state_machine_node.advertise<std_msgs::String>(state_pub_topic,10);

    // intervention services to stop, puase and continue state machine
    std::string stop_state_machine_service_topic(b_m_robot_ns + "/stop_state_machine");
    b_m_stop_state_machine_service = b_m_state_machine_node.advertiseService(stop_state_machine_service_topic, &StateMachineNodeBase::stopStateMachineCallback, this);
    std::string pause_machine_service_topic(b_m_robot_ns + "/pause_state_machine");
    b_m_pause_state_machine_service = b_m_state_machine_node.advertiseService(pause_machine_service_topic, &StateMachineNodeBase::pauseStateMachineCallback, this);
    std::string continue_machine_service_topic(b_m_robot_ns + "/continue_state_machine");
    b_m_continue_state_machine_service = b_m_state_machine_node.advertiseService(continue_machine_service_topic, &StateMachineNodeBase::continueStateMachineCallback, this);

    double init_position_x, init_position_y, init_position_z;
    double init_orientation_pitch, init_orientation_roll, init_orientation_yaw;
    b_m_state_machine_node.getParam(std::string(b_m_robot_ns + "/init_pose/position/X"), init_position_x);
    b_m_state_machine_node.getParam(std::string(b_m_robot_ns + "/init_pose/position/Y"), init_position_y);
    b_m_state_machine_node.getParam(std::string(b_m_robot_ns + "/init_pose/position/Z"), init_position_z);
    b_m_state_machine_node.getParam(std::string(b_m_robot_ns + "/init_pose/orientation/Pitch"), init_orientation_pitch);
    b_m_state_machine_node.getParam(std::string(b_m_robot_ns + "/init_pose/orientation/Roll"), init_orientation_roll);
    b_m_state_machine_node.getParam(std::string(b_m_robot_ns + "/init_pose/orientation/Yaw"), init_orientation_yaw);

    try {
        gtsam::Point3 init_position(init_position_x, init_position_y, init_position_z);
        gtsam::Rot3 init_orientation = gtsam::Rot3::ypr(init_orientation_yaw, init_orientation_pitch, init_orientation_roll);
        b_m_current_robot_pose_gtsam = gtsam::Pose3(init_orientation, init_position);
        b_m_current_robot_pose_ros = Conversion<geometry_msgs::Pose>::as(b_m_current_robot_pose_gtsam);
    }
    catch (...) {
        ROS_WARN_STREAM("Unable to read robot " << robot_id);
    }

    b_m_current_robot_pose_gtsam.print("SM: robot init pose\n");
    ROS_INFO_STREAM("Initialize state machine node base.");
}

StateMachineNodeBase::StateMachineNodeBase(const StateMachineNodeBase& other):
        StateMachineNodeBase()
{

}

StateMachineNodeBase& StateMachineNodeBase::operator=(const StateMachineNodeBase& other) {
    StateMachineNodeBase();
}

void StateMachineNodeBase::nextState() {
    State prev_state = b_m_current_state;

    if(b_m_states_conditions.stop_state_machine) {
        b_m_current_state = State::Exit;
    }
    else if(b_m_states_conditions.pause_state_machine) {
        b_m_current_state = State::Standby;
    }
    else {
        switch (b_m_current_state) {
            case State::Standby:
                if (b_m_states_conditions.passive_planner) {
                    b_m_current_state = State::UsrCmd;
                } else if (!b_m_states_conditions.perceive) {
                    b_m_current_state = State::Standby;
                } else if (b_m_states_conditions.plan_fail >= PLAN_FAIL_THRESHOLD) {
                    b_m_current_state = State::Exit;
                } else if (b_m_states_conditions.have_actions && !b_m_states_conditions.mpc) {
                    b_m_current_state = State::Move;
                } else if (b_m_states_conditions.have_goals &&
                           (!b_m_states_conditions.have_actions || b_m_states_conditions.mpc)) {
                    b_m_current_state = State::Plan;
                } else {
                    b_m_current_state = State::Exit;
                }
                break;

            case State::Plan:
                if (!b_m_states_conditions.have_goals || b_m_states_conditions.plan_fail >= PLAN_FAIL_THRESHOLD) {
                    b_m_current_state = State::Standby;
                } else if (b_m_states_conditions.plan_fail == 0 && b_m_states_conditions.have_actions) {
                    b_m_current_state = State::Move;
                    ROS_WARN("Plan -> Move");
                } else if ((b_m_states_conditions.plan_fail < PLAN_FAIL_THRESHOLD && b_m_states_conditions.perceive) ||
                           (b_m_states_conditions.request_to_plan && !b_m_states_conditions.received_actions)) {
                    b_m_current_state = State::Plan;
                } else {
                    b_m_current_state = State::Standby;
                }
                break;

            case State::Move:
                if (b_m_states_conditions.have_actions && !b_m_states_conditions.reached_waypoint) {
                    b_m_current_state = State::Move;
                } else {
                    b_m_current_state = State::Standby;
                    b_m_l++;
                }
                break;

            case State::Exit:
                // close all nodes...
                break;
            case State::UsrCmd:
                if(!b_m_states_conditions.passive_planner) {
                    b_m_current_state == State::Standby;
                }
                // wait until user stop and kill all nodes
                break;
        }
    }

    // Publish current state (if changed...)
    //if(b_m_current_state != prev_state) {
        std_msgs::String current_state_msg;
        current_state_msg.data = stateToString(b_m_current_state);
        b_m_state_pub.publish(current_state_msg);
    //}

}

std::string StateMachineNodeBase::stateToString(const State& state_to_print) {
    std::string string_to_return;
    switch(state_to_print) {
        case State::Exit:
            string_to_return = std::string("Exit");
            break;
        case State::Move:
            string_to_return = std::string("Move");
            break;
        case State::Plan:
            string_to_return = std::string("Plan");
            break;
        case State::Standby:
            string_to_return = std::string("Standby");
            break;
        case State::UsrCmd:
            string_to_return = std::string("UsrCmd");
            break;
    }

    return string_to_return;
}

State StateMachineNodeBase::getCurrentState() {
    return b_m_current_state;
}

void StateMachineNodeBase::printCurrentStatesConditions() {
    std::cout << "perceive: " << b_m_states_conditions.perceive << std::endl;
    std::cout << "passive_planner: " << b_m_states_conditions.passive_planner << std::endl;
    std::cout << "mpc: " << b_m_states_conditions.mpc << std::endl;
    std::cout << "have_actions: " << b_m_states_conditions.have_actions << std::endl;
    std::cout << "have_goals: " << b_m_states_conditions.have_goals << std::endl;
    std::cout << "plan_fail: " << b_m_states_conditions.plan_fail << std::endl;
    std::cout << "request_to_plan: " << b_m_states_conditions.request_to_plan << std::endl;
    std::cout << "received_actions: " << b_m_states_conditions.received_actions << std::endl;
    std::cout << "reached_waypoint: " << b_m_states_conditions.reached_waypoint << std::endl;
}

std::string StateMachineNodeBase::currentStatesConditionsToString() {
    std::stringstream conditions_ss;
    conditions_ss << "perceive: " << b_m_states_conditions.perceive << "\n"
                  << "passive_planner: " << b_m_states_conditions.passive_planner << "\n"
                  << "mpc: " << b_m_states_conditions.mpc << "\n"
                  << "have_actions: " << b_m_states_conditions.have_actions << "\n"
                  << "have_goals: " << b_m_states_conditions.have_goals << "\n"
                  << "plan_fail: " << b_m_states_conditions.plan_fail << "\n"
                  << "request_to_plan: " << b_m_states_conditions.request_to_plan << "\n"
                  << "received_actions: " << b_m_states_conditions.received_actions << "\n"
                  << "reached_waypoint: " << b_m_states_conditions.reached_waypoint << "\n";

    return conditions_ss.str();
}

void StateMachineNodeBase::getCurrentPose() {
    tf::StampedTransform transform;
    try {
        //std::string robot_name("Robot_A");
        std::string global_frame("world");
        b_m_tf_listener.waitForTransform(global_frame, b_m_robot_ns, ros::Time(0), ros::Duration(1.0));
        b_m_tf_listener.lookupTransform(global_frame, b_m_robot_ns, ros::Time(0), transform);
        tf::poseTFToMsg(transform, b_m_current_robot_pose_ros);

        /*
        geometry_msgs::Point position(current_robot_pose.position);
        geometry_msgs::Quaternion quat(current_robot_pose.orientation);
        std::cout << "Current robot pose: \n"
                  << "Position: X = " << position.x << ", Y = " << position.y << ", Z = " << position.z << "\n"
                  << "Orientation: W = " << quat.w << ", X = " << quat.x << ", Y = " << quat.y << ", Z = " << quat.z
                  << "\n"
                  << std::endl;
        */

        b_m_current_robot_pose_gtsam = Conversion<gtsam::Pose3>::as(b_m_current_robot_pose_ros);
        b_m_states_conditions.perceive = true;
    }
    catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        b_m_states_conditions.perceive = false;
    }


}


bool StateMachineNodeBase::stopStateMachineCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    b_m_states_conditions.stop_state_machine = true;
    ROS_INFO_STREAM("SM: User intervention: Stop");
    return true;
}


bool StateMachineNodeBase::pauseStateMachineCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    b_m_states_conditions.pause_state_machine = true;
    ROS_INFO_STREAM("SM: User intervention: Pause");
    return true;
}


bool StateMachineNodeBase::continueStateMachineCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    b_m_states_conditions.pause_state_machine = false;
    ROS_INFO_STREAM("SM: User intervention: Continue");
    return true;
}


void StateMachineNodeBase::closeNodeHandles() {
    b_m_state_machine_node.shutdown();
    std::cout << "shutdown node handles..." << std::endl;

    ros::shutdown();
}

StateMachineNodeBase::~StateMachineNodeBase() {

}