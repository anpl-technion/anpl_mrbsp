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
 * @file: planner_matlab.cpp
 * @brief: Ext Planner class
 * @author: Andrej Kitanov
 *
 */

#include "planner/planner_ext.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <planner/topology.h>

//using namespace ANPL;


PlannerExt::PlannerExt(std::string planner_type) :
        Planner(),
        type(planner_type)
{
    //ADD_FIRST_LEVEL_INFO_LOG(LogTag::planner);
    if (type == "matlab") {
        m_ext_request_topic = "/matlab_request";
        m_ext_response_topic = "/matlab_response";
        m_ext_pub = plannData._n.advertise<std_msgs::String>("/matlab_request", 1);
        m_ext_sub = plannData._n.subscribe("/matlab_response", 1, &PlannerExt::extPlannerCallback, this);
        ROS_INFO("Planner with Matlab interface started.");
        m_delimiter = ",";

        // Replace archive version number encoded in the string when serializing/deserializing GTSAM objects.
        // This is a workaround when using different boost versions which causes "unsupported version" exception
        ros::param::param<std::string>("/config_setup/matlab_boost_archive_version", m_matlab_boost_archive_version, "serialization::archive 11");
        ros::param::param<std::string>("/config_setup/ros_boost_archive_version", m_ros_boost_archive_version, "serialization::archive 15");


    } else if (type == "julia") {
        m_ext_request_topic = "/julia_request";
        m_ext_response_topic = "/julia_response";
        m_ext_pub = plannData._n.advertise<std_msgs::String>("/julia_request", 1);
        m_ext_sub = plannData._n.subscribe("/julia_response", 1, &PlannerExt::extPlannerCallback, this);
        ROS_INFO("Planner with Julia interface started.");
        m_delimiter = ",";
    } else { // "cpp"
        ROS_INFO("C++ Planner started.");
    }
}

PlannerExt::~PlannerExt() {

}

void PlannerExt::extPlannerCallback(const std_msgs::UInt32& msg) {
    //ADD_FIRST_LEVEL_INFO_LOG(LogTag::planner);

    m_is_received = true;
    m_is_correct = false;
    try {
        //m_ext_answer = (unsigned int) std::stoi(msg.data);
        m_ext_answer = msg.data;
        m_is_correct = true;
    }
    catch (...){}
}

unsigned int PlannerExt::evaluateObjFn(std::vector<NonlinearFactorGraph>& graph, std::vector<Values>& initialEstimate, bool inBatchMode) {
    std::stringstream to_send;
    //std::ofstream outfile;
    m_is_received = false;

    // Serialize GTSAM objects and send to Matlab

    // send message code so that in Matlab we know how to interpret it
    to_send << std::to_string(MATLAB_SEND_MSG_CODE)  << m_delimiter;

    if (MATLAB_SEND_MSG_CODE == MATLAB_SEND_PRIOR_AND_POSTERIOR) {
        // one more belief will be sent
        to_send << std::to_string(graph.size()+1)  << m_delimiter;

        // prior VN entropy for incremental operation
        to_send << T->prior_graph.signature.s_VN_incr << m_delimiter;

        // serialize factors
        std::string tempStr = gtsam::serialize(plannData.prior_graph);
        tempStr.replace(tempStr.find(m_ros_boost_archive_version), m_matlab_boost_archive_version.size(),
                        m_matlab_boost_archive_version);
        to_send << tempStr << m_delimiter;

        // serialize values
        tempStr = gtsam::serialize(plannData.prior_values);
        tempStr.replace(tempStr.find(m_ros_boost_archive_version), m_matlab_boost_archive_version.size(),
                        m_matlab_boost_archive_version);
        to_send << tempStr << m_delimiter;


    } else // only posteriors
        to_send << std::to_string(graph.size())  << m_delimiter;

    for (int i = 0; i < graph.size(); i++) {

        std::string tempStr;

        try {

            if (!inBatchMode) {
                // posterior joint belief
                //isam2->update(graph.at(i), initialEstimate.at(i));
                graph.at(i).add(plannData.prior_graph);
                initialEstimate.at(i).insert(plannData.prior_values);

                /*ROS_WARN("Calculate posterior topologies in INCR mode");
                Graph posterior_topological_graph = T->prior_graph;
                cout << "Edges of action " << i << ": " << delta_edges[0][i] << endl;
                cout << "Nodes of action " << i << ": " << delta_nodes[0][i] << endl;
                posterior_topological_graph.updateGraphAndSignature(delta_edges[0][i], delta_nodes[0][i], initialEstimate.at(i)); // single robot for now, TODO MR*/
            }


            // serialize factors
            tempStr = gtsam::serialize(graph.at(i));
            tempStr.replace(tempStr.find(m_ros_boost_archive_version), m_matlab_boost_archive_version.size(), m_matlab_boost_archive_version);
            to_send << tempStr << m_delimiter;

            // serialize values
            tempStr = gtsam::serialize(initialEstimate.at(i));
            tempStr.replace(tempStr.find(m_ros_boost_archive_version), m_matlab_boost_archive_version.size(), m_matlab_boost_archive_version);
            to_send << tempStr << m_delimiter;
            //}

        } catch (boost::archive::archive_exception& e) { // (gtsam::IndeterminantLinearSystemException& e) {

            //ROS_WARN("GTSAM Exception caught.");
            ROS_WARN("Boost Exception caught.");
            cout << e.what() << endl;
            graph[i].print();
            initialEstimate[i].print();

        }
    }

    /*delta_edges[0].clear(); // TODO MR
    delta_nodes[0].clear();*/

    std_msgs::String msg;
    msg.data = to_send.str();

    /*outfile.open("/usr/ANPLprefix/gtsam_toolbox/gtsam_tests/cpp_graph_str.txt");
    std::cout << "Serialized graph sent to Matlab >> " << msg.data << std::endl;
    outfile << msg.data;
    outfile.close();*/


    m_ext_pub.publish(msg);

    ros::Rate rate(5.); //Hz
    // busy wait
    while (!m_is_received && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    if(m_is_correct) {
        ROS_INFO("Matlab answered %d", m_ext_answer);
        return m_ext_answer;
    } else return 0;


}
