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
 * @file: planner.h
 * @brief: Base Planner class
 * @author: Andrej Kitanov
 *
 */

#ifndef PLANNER_H
#define PLANNER_H

// In planar SLAM example we use Pose2 variables (x, y, theta) to represent the robot poses while
// in 3D SLAM Pose3 variables (Rot3, x, y, z)
// IMPORTANT: throughout code we use general 3D SLAM state representation even to model robot motion in 2D!
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>


// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>
// And here we will use simple integer Keys to refer to the robot poses.
//#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// constraint the first pose in 2D
#include <gtsam/nonlinear/NonlinearEquality.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori or Maximum Likelihood) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// a Gauss-Newton solver
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
// and here ISAM2
#include <gtsam/nonlinear/ISAM2.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Inter-process sharing of GTSAM objects by serialization
#include <gtsam/base/serialization.h>

// GTSAM timing utilities
#include <gtsam/base/timing.h>

using namespace std;


#include <ros/ros.h>
#include <mrbsp_msgs/RequestToPlan.h>
#include <mrbsp_msgs/Actions.h>
#include <std_srvs/Empty.h>
#include <bitset>
#include <math.h>
#include <random>
#include <mrbsp_utils/conversion.h>
#include "planner/config.h"

class PlannData {
private:
    ros::ServiceServer planner_service;
    boost::mutex mutex;
public:
    NonlinearFactorGraph delta_prior_graph;
    Values delta_prior_values;
    std::string prior_edges, delta_edges;
    std::string prior_nodes, delta_nodes;
    unsigned int n_session;

    geometry_msgs::Pose start[NUM_ROBOTS];
    geometry_msgs::Pose goal[NUM_ROBOTS];
    std::string gtsam_values_str[NUM_ROBOTS];
    std::string gtsam_factors_str[NUM_ROBOTS];
    NonlinearFactorGraph prior_graph;
    Values prior_values;
    bool MR_info_generated;
    std::string planner_mode;

    unsigned char robots_updated;
    bool replan;
    ros::NodeHandle _n;

    PlannData() {

        planner_service = _n.advertiseService("send_plan_data", &PlannData::initPlanner, this);
        robots_updated = 0;
        replan = false;


        ros::param::param<std::string>("/planner/mode", planner_mode, "batch");
        n_session = 0;

    }

    // test planning for up to 2 robots
    void test_robots_ab_planning() {

        char robot_ids[2] = {'A', 'B'};


        for (int n = 0; n < NUM_ROBOTS; n++) {

            unsigned char i = robot_ids[n] - 'A';

            switch (i) {
                case 0:
                    // set start pose
                    start[i].position.x = 19.9; //0.487;
                    start[i].position.y = -0.1; //-0.937;
                    start[i].position.z = 0;
                    start[i].orientation.w = 0; //1;
                    start[i].orientation.x = 0;
                    start[i].orientation.y = 0;
                    start[i].orientation.z = 1; //0;

                    break;

                case 1:
                    // set start pose
                    start[i].position.x = 26.9;
                    start[i].position.y = -6.07;
                    start[i].position.z = 0;
                    start[i].orientation.w = 0.707;
                    start[i].orientation.x = 0;
                    start[i].orientation.y = 0;
                    start[i].orientation.z = 0.707;

                    break;
            }

            // Add a prior on the first pose
            prior_graph.add(PriorFactor<Pose3>(Symbol('A'+i, 0),
                                               Conversion<gtsam::Pose3>::as(start[i]), priorNoiseModel));
            prior_values.insert(Symbol('A'+i, 0), Conversion<gtsam::Pose3>::as(start[i]));

            // Add a hard constraint on the first pose for planar SLAM, z = 0
            //prior_graph.add(NonlinearEquality<Pose3>(Symbol('A'+i, 0), Conversion<gtsam::Pose3>::as(start[i])));

            // let robots a and b have the same goal
            goal[i].position.x = 26; //1.981;
            goal[i].position.y = 9; //-6;
            goal[i].position.z = 0;
            goal[i].orientation.w = 1;
            goal[i].orientation.x = 0;
            goal[i].orientation.y = 0;
            goal[i].orientation.z = 0;

            robots_updated |= 1 << i;
            std::cout << std::bitset<NUM_ROBOTS>(robots_updated) << std::endl;
            //printf("robots updated = %04x\n", robots_updated);

        }
    }

protected:

    bool initPlanner(mrbsp_msgs::RequestToPlan::Request& req, mrbsp_msgs::RequestToPlan::Response& res) {
        boost::unique_lock<boost::mutex> lock(mutex);
        ROS_INFO("Received planning data from robot_%c", req.robot_id);
        printRequestInfo(req);
        unsigned char i = req.robot_id - 'A';
        robots_updated |= 1 << i;
        start[i] = req.start;
        goal[i] = req.goal;
        gtsam_values_str[i] = req.gtsam_values_str;
        gtsam_factors_str[i] = req.gtsam_factors_str;

        // CONDITION TO TRIGGER REPLANNING!!!
        replan = robots_updated == (1 << NUM_ROBOTS) - 1;

        if (planner_mode == "incremental") {

            //TODO call /request_belief service to get it

            ROS_WARN("SERIALIZATION CHECK");
            // NOTE! deserialization on factor graphs acts as integrator (accumulates factors), while on values as equality (destroys previous content)!!!
            try {
                //std::cout << "factors received by robot " << i << "\n" << gtsam_factors_str[i] << "\n";
                gtsam::deserialize(gtsam_factors_str[i], delta_prior_graph);
                //std::cout << "PL: Deserialized graph. Robot " << req.robot_id << " sent delta graph of size " << delta_prior_graph.size() << " factors" << std::endl;
                std::cout << "PL: Deserialized graph. Robot " << req.robot_id << " sent his delta graph. Delta graph size is now " << delta_prior_graph.size() << " factors" << std::endl;
                //delta_prior_graph.print();
            }
            catch (...) {
                std::cout << "PL: Unable to deserialize graph";
            }
            try {
                //std::cout << "values received by robot " << i << "\n" << gtsam_values_str[i] << "\n";
                //delta_prior_values.print("D-");
                gtsam::Values received_values;
                gtsam::deserialize(gtsam_values_str[i], received_values);
                try {
                    delta_prior_values.insert(received_values);
                } catch (ValuesKeyAlreadyExists& e) {
                    ROS_ERROR(e.what());
                }
                //std::cout << "PL: Deserialized values. Robot " << req.robot_id << " sent delta values of size " << delta_prior_values.size() << " states" << std::endl;
                std::cout << "PL: Deserialized values. Robot " << req.robot_id << " sent his delta values. Delta values size is now " << delta_prior_values.size() << " states" << std::endl;
                //delta_prior_values.print("D+");
            }
            catch (...) {
                std::cout << "PL: Unable to deserialize values" << std::endl;
            }

            delta_edges += req.edges_list;
            delta_nodes += req.nodes_list;

            if (replan) { // last robot in a group sent his planning request

                std::cout << "PL: Prior MR belief at previous planning session contained " << prior_graph.size() << " factors" << std::endl;
                std::cout << "PL: Prior MR belief at previous planning session contained " << prior_values.size() << " states" << std::endl;
                ROS_WARN_STREAM("Edges added during inference until current planning session: " << delta_edges);
                ROS_WARN_STREAM("Nodes added during inference until current planning session: " << delta_nodes);
                std::cout << "PL: Prior MR belief at current planning session contains " << prior_graph.size()+delta_prior_graph.size() << " factors" << std::endl;
                std::cout << "PL: Prior MR belief at current planning session contains " << prior_values.size()+delta_prior_values.size() << " states" << std::endl;

            }


        } else if (planner_mode == "batch" && replan) { // Assumption: "Last robot will send the most updated multi-robot belief at time k in batch mode"!!!

            // clear previous plannig session's multi-robot prior
            prior_graph.resize(0);
            prior_values.clear();

            //TODO call /request_belief service to get it
            ROS_WARN("SERIALIZATION CHECK");
            try {
                gtsam::deserialize(gtsam_factors_str[i], prior_graph);

                //std::cout << "Serialized graph received >> " << gtsam_factors_str[i] << std::endl;
                std::cout << "PL: Deserialized graph. Prior MR belief contains " << prior_graph.size() << " factors" << std::endl;

            }
            catch (...) {
                std::cout << "PL: Unable to deserialize graph";
            }
            try {
                gtsam::deserialize(gtsam_values_str[i], prior_values);

                //std::cout << "Serialized values received >> " << gtsam_values_str[i] << std::endl;
                std::cout << "PL: Deserialized values. Prior MR belief contains " << prior_values.size() << " states" << std::endl;
            }
            catch (...) {
                std::cout << "PL: Unable to deserialize values" << std::endl;

            }

            prior_edges = req.edges_list;
            prior_nodes = req.nodes_list;
        }

       if (replan) { // for both modes after information from all robots was sent
            MR_info_generated = false; // flag to signal that regenerating MR info is required

           /*std::string path_to_log_folder = ".";
           _n.getParam("/logger/loggerPath", path_to_log_folder);
           std::stringstream session_id;
           session_id << std::setw( 3 ) << std::setfill( '0' ) << n_session;
           gtsam::serializeToFile(prior_graph, path_to_log_folder+"/reconstructed_graph_session_" + session_id.str());
           gtsam::serializeToFile(prior_values, path_to_log_folder+"/reconstructed_values_session_" + session_id.str());*/
           n_session++;
        }

        return (res.success = true);
    }

    void printRequestInfo(mrbsp_msgs::RequestToPlan::Request& req) {
        std::cout << "REQUEST data received\n"
                  << "Robot ID: " << req.robot_id << "\n"
                  << "START position: (" << req.start.position.x << ", " << req.start.position.y << ", " << req.start.position.z << ")\n"
                  << "START orientation: (" << req.start.orientation.w << ", " << req.start.orientation.x << ", " << req.start.orientation.y << ", " << req.start.orientation.z << ")\n"
                  << "END position: (" << req.goal.position.x << ", " << req.goal.position.y << ", " << req.goal.position.z << ")\n"
                  << "END orientation: (" << req.goal.orientation.w << ", " << req.goal.orientation.x << ", " << req.goal.orientation.y << ", " << req.goal.orientation.z << ")\n";

        ROS_WARN_STREAM("Planner mode: " << planner_mode);
    }

};

//class PlannerExt;
class Topology;
class Planner {
    //friend class PlannerExt;
public:
    // constructor
    Planner();

    ~Planner() {

        if (isam2)
            delete isam2;

    }

    PlannData plannData;
    std::string planner_algorithm;
    enum planner_status {SUCCESS, FAIL, NA} status;
    std::vector<NonlinearFactorGraph> local_graphs[NUM_ROBOTS];
    std::vector<Values> local_values[NUM_ROBOTS];
    NonlinearFactorGraph prior_local_graph[NUM_ROBOTS];
    Values prior_local_values[NUM_ROBOTS];
    NonlinearFactorGraph prior_mr_factors;

    std::vector<NonlinearFactorGraph> mr_graphs; // multi-robot posterior beliefs of candidate control actions
    std::vector<Values> mr_values;
    // multi-robot posterior delta graphs such that
    // posterior topological graph[a] = plannData.prior_edges/nodes + delta_edges/nodes[a] for each action a
    std::vector<std::string> delta_nodes[NUM_ROBOTS], delta_edges[NUM_ROBOTS]; // topological effects of candidate actions
    Topology* T;

    std::vector<std::pair<int, int>> action_variations;
    ISAM2* isam2;
    mrbsp_msgs::Actions optimal_joint_action;

    void decomposeMultiRobotBelief(NonlinearFactorGraph &graph_in, Values &vals_in, NonlinearFactorGraph *graph_out,
                                   Values *vals_out, NonlinearFactorGraph &residual_graph);
    void constructLocalFGs(char robot_id, mrbsp_msgs::Actions& controls, bool isBatchMode);
    void constructMultiRobotFGsForTwoRobots(bool isBatchMode);
    unsigned int evaluateObjFn(const std::vector<NonlinearFactorGraph>& graph, const std::vector<Values>& initialEstimate, bool inBatchMode);

    void visualizeOptJointAction();

private:

    unsigned long int stateDimension[NUM_ROBOTS]; // robot state dimension, i.e number of pose samples by each robot at planning time k
    ros::Publisher vis_opt_actions_pub;

};

#endif

