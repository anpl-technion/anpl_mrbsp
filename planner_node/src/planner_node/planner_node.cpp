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
 * @file: planner_node.cpp
 * @brief: Planner Main
 * @author: Andrej Kitanov
 *
 */


// ROS includes
#include <ros/ros.h>
#include <mrbsp_msgs/GenerateActions.h>
#include <planner_node/planner_ext.h>
//#include <planner_node/topology.h>
#include <mrbsp_utils/gtsam_serialization.h>

#include <planner_node/eigen_utils.hpp>
#include <planner_node/topology.h>

int main(int argc, char** argv) {


    ros::init(argc, argv, "planner");
    ros::NodeHandle n("~");
    ros::Rate r(10); // 10 hz
    ros::ServiceClient client = n.serviceClient<mrbsp_msgs::GenerateActions>("/generate_actions");
    ros::Publisher opt_action_pub = n.advertise<mrbsp_msgs::Actions>("/optimal_joint_action", 10);
    mrbsp_msgs::GenerateActions srv[NUM_ROBOTS];
    unsigned int idx_opt[NUM_ROBOTS];
    std::string planner_type;
    if (n.getParam("planner_type", planner_type)) {
        ROS_WARN("PLANNER TYPE SET FROM CONFIG");
    } else {
        ROS_WARN("PLANNER TYPE NOT SET. USING DEFAULT. ");
        planner_type = "cpp";
    }
    PlannerExt planner(planner_type); // choose between C++ and Matlab BSP based on ROS parameter


    bool unit_tests;
    if (!n.getParam("unit_tests", unit_tests)) {
        unit_tests = false;
    }
    if (unit_tests) {
        ROS_WARN("UNIT TESTS ON");
        ROS_WARN("Wait for 3 s for nodes to initialize...");
        ros::Duration(3.0).sleep();
        ROS_INFO("Ready.");
        // set plan data
        planner.plannData.test_robots_ab_planning();
        planner.plannData.replan = true;
        ROS_INFO("Go.");
    }

    planner.T = new Topology();
    Topology::time_ofs << "% prior: isam2 Graph s_ST s_VN s_VNincr | num_cand_actions * posterior: isam2 Graph s_ST s_VN s_VNincr\n";
    gtsam::Matrix Omega3D = measurementNoiseModel->R().transpose()*measurementNoiseModel->R(); // Eigen::MatrixXd
    gtsam::Matrix Omega2D = Matrix3d::Zero();
    Omega2D(0,0) = Omega3D(3,3); // var x
    Omega2D(1,1) = Omega3D(4,4); // var y
    Omega2D(2,2) = Omega3D(2,2); // var yaw

    try {
        MatrixXd Ln;
        loadMatrixFromFile("Ln", Ln);
        std::cout << "Ln = " << Ln << std::endl;

    } catch (std::exception& e) {
        ROS_ERROR(e.what());
    }

    while (ros::ok()) {

        if (planner.plannData.replan) {

            planner.plannData.robots_updated = 0; // reset flags
            planner.plannData.replan = 0;
            planner.status = Planner::planner_status::SUCCESS;

            // call action generator service for each robot
            // - received generated actions (collision free) are contained in srv[robot_id].response
            // for each non-myopic action create a factor graph and for all robots
            for (int i = 0; i < NUM_ROBOTS; i++) {

                srv[i].request.start = planner.plannData.start[i];
                srv[i].request.goal = planner.plannData.goal[i];

                ROS_INFO("Calling AG for robot %d", i);
                if (client.call(srv[i])) {
                    ROS_INFO("Num actions received: %ld", (long int) srv[i].response.actions.actions.size());
                    if (srv[i].response.actions.actions.size() == 0) {
                        // i-th robot did not find feasible path
                        planner.status = Planner::planner_status::FAIL;
                        break;
                    }

                } else {
                    ROS_ERROR("Failed to call service /generate_actions");
                    return 1;
                }

                // clear previous session's posterior local FGs, in incremental mode they represent local information added to the prior of a robot i for all its actions
                planner.local_graphs[i].clear();
                planner.local_values[i].clear();

                if (planner.plannData.planner_mode == "batch") {
                    // clear also previous session's prior local FGs and MR factors
                    planner.prior_local_graph[i].resize(0);
                    planner.prior_mr_factors.resize(0);
                    planner.prior_local_values[i].clear();
                }
            }
            mrbsp_msgs::Actions opt_joint_action;
            if (planner.status == Planner::planner_status::FAIL) {
                ROS_WARN("Non-existing joint action. At least one robot did not find a path in requested configuration.");
                opt_action_pub.publish(opt_joint_action);
                continue;
            }

            // otherwise, generating actions succeeded
            if (planner.plannData.planner_mode == "batch") {

                // create prior topology from inference
                planner.T->updatePrior(planner.plannData.prior_edges, planner.plannData.prior_nodes, planner.plannData.prior_values, Omega2D, true);

                planner.decomposeMultiRobotBelief(planner.plannData.prior_graph, planner.plannData.prior_values,
                                                  planner.prior_local_graph, planner.prior_local_values, planner.prior_mr_factors);

            } else { // incremental


                // standard BSP
                // propagate belief to current planning time
                gttic_(isam2PriorUpdate);
                planner.isam2->update(planner.plannData.delta_prior_graph, planner.plannData.delta_prior_values);
                planner.plannData.prior_graph = planner.isam2->getFactorsUnsafe();
                planner.plannData.prior_values = planner.isam2->calculateBestEstimate();
                gttoc_(isam2PriorUpdate);
                tictoc_getNode(tisam2PriorUpdate, isam2PriorUpdate);
                Topology::time_ofs << tisam2PriorUpdate.get()->wall() << " ";


                // topological BSP
                // update prior topology from inference
                planner.plannData.prior_edges += planner.plannData.delta_edges;
                planner.plannData.prior_nodes += planner.plannData.delta_nodes;
                planner.T->updatePrior(planner.plannData.delta_edges, planner.plannData.delta_nodes, planner.plannData.prior_values, Omega2D, false);


                ROS_WARN("ISAM2 prior belief propagated:");
                //planner.plannData.prior_graph.print();
                //planner.plannData.prior_values.print();
                /*std::cout << "Press Enter to continue ...";
                std::string tempStr;
                std::getline(std::cin, tempStr);*/

                NonlinearFactorGraph delta_prior_local_graph[NUM_ROBOTS];
                Values delta_prior_local_values[NUM_ROBOTS];
                planner.decomposeMultiRobotBelief(planner.plannData.delta_prior_graph, planner.plannData.delta_prior_values,
                                                  delta_prior_local_graph, delta_prior_local_values, planner.prior_mr_factors); // local info extracted from deltas, mr factors added to previous prior

                // update prior local estimate of each robot from isam2 prior estimate
                for (int i = 0; i < NUM_ROBOTS; i++) {
                    planner.prior_local_graph[i].add(delta_prior_local_graph[i]);
                    planner.prior_local_values[i].insert(delta_prior_local_values[i]);
                    for(gtsam::Values::iterator key_value = planner.prior_local_values[i].begin(); key_value != planner.prior_local_values[i].end(); ++key_value)
                        key_value->value = planner.plannData.prior_values.at(key_value->key);
                }


                // clear deltas for the next session
                planner.plannData.delta_prior_graph.resize(0);
                planner.plannData.delta_prior_values.clear();
                planner.plannData.delta_edges.clear();
                planner.plannData.delta_nodes.clear();


            }

            bool isBatchMode = planner.plannData.planner_mode == "batch";

            // according to a motion model and simple neighbouring criteria function construct factors, i.e. relative pose observations with fixed noise covariance
            for (int i = 0; i < NUM_ROBOTS; i++)
                planner.constructLocalFGs(i+'A', srv[i].response.actions, isBatchMode);

            //- send to Matlab, or optimize here, evaluate Objective function and send chosen action in the form of waypoints to the controller
            // evaluate objective
            // publish the best actions

            if (NUM_ROBOTS == 1 || !USING_MR_FACTORS ) {
                for (int r = 0; r < NUM_ROBOTS; r++) {
                    if (planner_type == "cpp") // call the base planner C++ evaluateObjFn
                        idx_opt[r] = planner.Planner::evaluateObjFn(planner.local_graphs[r], planner.local_values[r], isBatchMode); // in incr. mode, local information refers to deltas of local FGs and VALS
                    else // call the matlab
                        idx_opt[r] = planner.evaluateObjFn(planner.local_graphs[r], planner.local_values[r], isBatchMode);

                    opt_joint_action.actions.push_back(srv[r].response.actions.actions[idx_opt[r]]);
                    std::cout << "Opt. action of robot " << (char)(r+'A') << " is action " << idx_opt[r] << std::endl;
                }
            } else { // using MR factors

                // clear previous session's multi-robot FGs
                planner.mr_graphs.clear();
                planner.mr_values.clear();
                planner.action_variations.clear();
                planner.constructMultiRobotFGsForTwoRobots(isBatchMode);
                unsigned int idx;
                if (planner_type == "cpp") // call the base planner C++ evaluateObjFn
                    idx = planner.Planner::evaluateObjFn(planner.mr_graphs, planner.mr_values, isBatchMode); // in incr. mode, MR information refers to deltas of MR FGs and VALS
                else // call the matlab
                    idx = planner.evaluateObjFn(planner.mr_graphs, planner.mr_values, isBatchMode);

                idx_opt[0] = planner.action_variations[idx].first;
                idx_opt[1] = planner.action_variations[idx].second;
                opt_joint_action.actions.push_back(srv[0].response.actions.actions[idx_opt[0]]);
                opt_joint_action.actions.push_back(srv[1].response.actions.actions[idx_opt[1]]);
                ROS_WARN("a* = (%d, %d)", idx_opt[0], idx_opt[1]);
            }
            opt_action_pub.publish(opt_joint_action);

            planner.optimal_joint_action = opt_joint_action;
            planner.visualizeOptJointAction();


        }

        ros::spinOnce();
        r.sleep();
    }

    if (planner.T)
        delete planner.T;

    return 0;
}

