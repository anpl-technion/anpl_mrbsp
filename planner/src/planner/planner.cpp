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
 * @file: planner.cpp
 * @brief: Base Planner class
 * @author: Andrej Kitanov
 *
 */

#include <gtsam/nonlinear/NonlinearEquality.h>
#include "planner/planner.h"
#include <visualization_msgs/MarkerArray.h>
#include <planner/topology.h>


Planner::Planner() {

    ros::NodeHandle pnh = ros::NodeHandle("~");
    vis_opt_actions_pub = pnh.advertise<visualization_msgs::MarkerArray>("optimal_actions_marker", 1);

    // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
    // and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
    // structure is available that allows the user to set various properties, such as the relinearization threshold
    // and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
    // will approach the batch result.
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    parameters.factorization = ISAM2Params::Factorization::QR;
    isam2 = new ISAM2(parameters);

    ros::param::param<std::string>("/planner/alg", planner_algorithm, "standard");
}

void Planner::constructLocalFGs(char robot_id, mrbsp_msgs::Actions& controls, bool isBatchMode) {


    std::default_random_engine generator;
    std::normal_distribution<double> pos_distribution(0.0,0.2);
    std::normal_distribution<double> ang_distribution(0.0,0.1);

    NonlinearFactorGraph graph;//, graph_k;
    Values initialEstimate;//, initialEstimate_k;

    // extract local information at time k from multi-robot belief
    // TODO consider doing this in a one-time call to populate all robots' priors, outside of this function
    // i_k contains index of the last pose of the given robot, assuming the first pose starts with 0
    //unsigned long int i_k = getRobotFactorGraphAndStates(robot_id, graph_k, initialEstimate_k)-1;
    unsigned long int i_k = stateDimension[robot_id-'A'] = prior_local_values[robot_id-'A'].size()-1;
    //unsigned long int i_k = stateDimension[robot_id-'A'];

    for (int a = 0; a < controls.actions.size(); a++) {

        // each action starts from the same prior
        graph = prior_local_graph[robot_id-'A'];
        initialEstimate = prior_local_values[robot_id-'A'];
        ROS_WARN_STREAM("Action " << a << " of robot " << robot_id << " starts from prior of size " << graph.size());

        // initialize an empty new local graph for each action, and new states ML estimate
        NonlinearFactorGraph new_local_graph;
        Values ML_estimate;
        std::string new_edges, new_nodes;
        std::stringstream sstr;

        nav_msgs::Path *path = &(controls.actions[a]);


        /*if (graph.empty()) {
            // Add a prior on the first pose
            // The prior is needed to fix/align the whole trajectory at world frame
            // A prior factor consists of a mean value and a noise model (covariance matrix)
            graph.add(PriorFactor<Pose3>(Symbol(robot_id, 0),
                                         Conversion<gtsam::Pose3>::as(path->poses[0].pose), priorNoiseModel));
            initialEstimate.insert(Symbol(robot_id, 0), Conversion<gtsam::Pose3>::as(path->poses[0].pose));
        }*/
        //unsigned long int i_k = 0; //initialEstimate.size()-1; // index of the last pose, assuming the first pose starts with 0


        // add new factors and states according to candidate actions
        for (int i = 0; i < path->poses.size(); i++) {

            Pose3 p1 = Conversion<gtsam::Pose3>::as(path->poses[i].pose);
            if (i < path->poses.size() - 1) {
                // Create odometry (Between) factors between consecutive poses
                Pose3 p2 = Conversion<gtsam::Pose3>::as(path->poses[i + 1].pose);
                new_local_graph.add(
                        BetweenFactor<Pose3>(Symbol(robot_id, i_k + i), Symbol(robot_id, i_k + i + 1), p1.between(p2),
                                             odometryNoiseModel));
                sstr << robot_id << i_k + i << "--" << robot_id << i_k + i + 1 << ";";
                new_edges += sstr.str();
                //std::stringstream().swap(sstr); // c++11 compiler
		sstr.str(std::string());

                // 3. Create the data structure to hold the initialEstimate estimate to the solution
                // For illustrative purposes, these have been deliberately set to incorrect values
                // by adding Gaussian white noise
                Pose2 w(pos_distribution(generator), pos_distribution(generator), ang_distribution(generator));
                p2.compose(Pose3(w));
                ML_estimate.insert(Symbol(robot_id, i_k + i + 1), p2);
                initialEstimate.insert(Symbol(robot_id, i_k + i + 1), p2);
                new_nodes += robot_id;
                new_nodes += std::to_string(i_k + i + 1) + ";";
            }

            // add local loop closings between {future-future} and {future - current, past} states
            for (int j = i_k + i - 2; j >= 0; j--) {
                // 2c. Add the loop closure constraint
                // This factor encodes the fact that we have returned to the same pose. In real systems,
                // these constraints may be identified in many ways, such as appearance-based techniques
                // with camera images. We will use another Between Factor to enforce this constraint:
                // check distance (i,j)
                //Pose3 p3 = Conversion<gtsam::Pose3>::as(path->poses[j].pose);
                Pose3 p3  = initialEstimate.at<gtsam::Pose3>(Symbol(robot_id, j));
                Pose3 delta = p1.between(p3);

                if (delta.translation().norm() < LC_DISTANCE_THRESHOLD  && abs(delta.rotation().yaw()) < LC_RELATIVE_ORIENTATION_THRESHOLD) {
                    new_local_graph.add(BetweenFactor<Pose3>(Symbol(robot_id, i_k + i), Symbol(robot_id, j), delta,
                                                             measurementNoiseModel));
                    sstr << robot_id << i_k + i << "--" << robot_id << j << ";";
                    new_edges += sstr.str();
                    //std::stringstream().swap(sstr); c++11 compiler
		    sstr.str(std::string());
                }

            }

        }

        /*if (NUM_ROBOTS > 1 && USING_MR_FACTORS) {
            // here we remove prior factors and states so that only newly added stay in the local_graphs/values
            // they can be added later on when constructing MR posterior belief and this is to avoid duplicate key/values/factors
            // the problem with this approach is that new_mr_factors will only contain future robots' states when constructing MR posterior
            graph.resize(0);
            initialEstimate.clear();
            graph.add(new_local_graph);
            initialEstimate.insert(ML_estimate);
        }*/

        // in batch mode local graph is the whole local FG, in incremental its subgraph corresponding to new states and local observations
        if (isBatchMode) {
            graph.add(new_local_graph);
            local_graphs[robot_id-'A'].push_back(graph);
            local_values[robot_id-'A'].push_back(initialEstimate);
        } else {
            local_graphs[robot_id-'A'].push_back(new_local_graph);
            local_values[robot_id-'A'].push_back(ML_estimate);
        }
        delta_edges[robot_id-'A'].push_back(new_edges);
        delta_nodes[robot_id-'A'].push_back(new_nodes);

        /*ROS_WARN_STREAM("--- Effect of Action " << a << " of robot " << robot_id );
        //graph.print("\nPredicted Factor Graph:\n"); // print
        //initialEstimate.print("\nPredicted Estimate:\n"); // print

        ROS_WARN_STREAM("Predicted local edges: " << new_edges);
        ROS_WARN_STREAM("Predicted local nodes: " << new_nodes);*/

    }

    ROS_WARN("Local factor graphs created.");

}


void Planner::constructMultiRobotFGsForTwoRobots(bool isBatchMode) {


    std::pair<int, int> pair_ij;


    // joint posterior factors and values
    for (int i  = 0;  i < local_graphs[0].size(); i++) {
        for (int j  = 0;  j < local_graphs[1].size(); j++) {

            NonlinearFactorGraph g = local_graphs[0].at(i);
            g.add(local_graphs[1].at(j));
            ROS_WARN("factors %d, %d joined", i, j);

            Values v, vi, vj;
            if (isBatchMode) {
                vi = local_values[0].at(i);
                vj = local_values[1].at(j);
            } else { // in incremental mode we combine priors and local variables (deltas) to get all local variables belonging to each robot
                vi = prior_local_values[0];
                vi.insert(local_values[0].at(i));
                vj = prior_local_values[1];
                vj.insert(local_values[1].at(j));
            }
            try {
                v.insert(vi);
                v.insert(vj);
                ROS_WARN("values %d, %d joined", i, j);
            } catch( const ValuesKeyAlreadyExists e) {
                ROS_WARN(e.what());
            }

            pair_ij.first = i;
            pair_ij.second = j;
            action_variations.push_back(pair_ij);

            // initial transformation
            gtsam::Pose3 p_00 = vi.at<gtsam::Pose3>(vi.keys().front());
            gtsam::Pose3 p_10 = vj.at<gtsam::Pose3>(vj.keys().front());
            g.add(BetweenFactor<Pose3>(Symbol('A'+0, 0), Symbol('A'+1, 0), p_00.between(p_10), mr_rel_pose_model));
            ROS_WARN("initial transform factor added");



            // loop over all values of the action 'i' and 'j' and add new MR factors
            // consider also MR factors between future and {current, past} states, i.e.
            // those with indexes either k > stateDimension[0] or m > stateDimension[1]
            // prior MR factors are cloned in batch mode
            for (int k = 0; k < vi.keys().size(); k++) {
                for (int m = 0; m < vj.keys().size(); m++) {
                    //KeyList ki = vi.keys();

                    if (k > stateDimension[0] || m > stateDimension[1]) { // at least one future state
                        gtsam::Pose3 p_0 = vi.at<gtsam::Pose3>(Symbol('A' + 0, k));
                        gtsam::Pose3 p_1 = vj.at<gtsam::Pose3>(Symbol('A' + 1, m));
                        Pose3 delta = p_0.between(p_1);
                        if (delta.translation().norm() < LC_DISTANCE_THRESHOLD)
                            // && abs(delta.theta()) < LC_RELATIVE_ORIENTATION_THRESHOLD
                            g.add(BetweenFactor<Pose3>(Symbol('A' + 0, k), Symbol('A' + 1, m), delta,
                                                       mr_rel_pose_model));
                    }
                }
            }

            // add prior MR factors in batch mode
            if (isBatchMode) {
                g.add(prior_mr_factors);

            } else { // add only new future states and new factors in INCR. mode
                v =  local_values[0].at(i);
                v.insert(local_values[1].at(j));
            }

            mr_graphs.push_back(g);
            mr_values.push_back(v);
        }
    }
    ROS_WARN("MR factor graphs created in %s", isBatchMode?"BATCH mode.":"INCR mode.");
    //mr_graphs.at(0).print();
    //mr_values.at(0).print();
}

// iterate over all beliefs and return the index of the best one
// in batch mode, whole factor graphs are given in the graph argument and their associated initial state estimates
// in incremental mode, prior belief is expected to be already calculated (ISAM2 object) and graphs represent new observations and motion factors that have been predicted by each action
unsigned int Planner::evaluateObjFn(const std::vector<NonlinearFactorGraph>& graph, const std::vector<Values>& initialEstimate, bool inBatchMode) {


    unsigned int idx_opt = 0, t_idx_opt = 0;
    double J_opt, J_t, s_opt;

    if (inBatchMode) { // batch standard and topological BSP

        ROS_WARN("Calculate posterior beliefs and topologies in BATCH mode");

        // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
        // The optimizer accepts an optional set of configuration parameters,
        // controlling things like convergence criteria, the type of linear
        // system solver to use, and the amount of information displayed during
        // optimization. We will set a few parameters as a demonstration.
        GaussNewtonParams parameters;
        // Stop iterating once the change in error between steps is less than this value
        parameters.relativeErrorTol = 1e-5;
        // Do not perform more than N iteration steps
        parameters.maxIterations = 100;

//        PATCH -- Evgeny ---------------------------------------------------
//        int countI = 0;
//        std::string post_EE[delta_edges[0].size()*delta_edges[1].size()];
//        std::string post_VV[delta_edges[0].size()*delta_edges[1].size()];
//        if ((USING_MR_FACTORS) && (NUM_ROBOTS > 1)) {
//            for (int i = 0; i < delta_edges[0].size(); i++) {
//                for (int j = 0; j < delta_edges[1].size(); j++) {
//                    post_EE[countI] = plannData.prior_edges + delta_edges[0][i] + delta_edges[1][j];
//                    post_VV[countI] = plannData.prior_nodes + delta_nodes[0][i] + delta_nodes[1][j];
//                    countI++;
//                }
//            }
//        }
//        ------------------------------------------------------------------

        for (int i = 0; i < graph.size(); i++) {

            ROS_WARN("|\nAction %ld", i);
            double J;
            Values result;
            // Create the optimizer ...
            try {

                GaussNewtonOptimizer optimizer(graph[i], initialEstimate[i], parameters);

                // ... and optimize
                result = optimizer.optimize();
                //result.print("Final Result:\n");

                // 5. Calculate and print marginal covariances for all variables
                cout.precision(3);
                Marginals marginals(graph[i], result);
                /*cout << "keys size of graph " << i << " is " << graph[i].keys().size() << endl;
                gtsam::Matrix cov_end_pose = marginals.marginalCovariance(initialEstimate[i].keys().back());
                Symbol last_pose(initialEstimate[i].keys().back());
                cout << "x(k+L) covariance [" << last_pose.chr() << last_pose.index() << "]:\n" << cov_end_pose
                     << endl; // uncertainty of the last pose of the i-th action
                J = cov_end_pose.determinant();*/
                result.erase(Key(Symbol('A', 0))); // this is deterministic var
                JointMarginal JointInf = marginals.jointMarginalInformation(KeyVector(result.keys()));
                gtsam::Matrix R_mat = gtsam::RtR(JointInf.fullMatrix()); // Cholesky factorization Lambda = R'R

                J = R_mat.rows()/2.0 * log(2*M_PI*M_E);
                double logDetR = 0;
                for (unsigned i = 0; i < R_mat.rows(); ++i) {
                    logDetR += std::log(fabs(R_mat(i, i)));
                    //logDetR2 += std::log(fabs(R.coeff(i, i)));
                }

                J -= 2*logDetR;

                if (i == 0) {
                    idx_opt = 0;
                    J_opt = J;

                } else if (J < J_opt) {
                    idx_opt = i;
                    J_opt = J;
                }
            } catch (gtsam::IndeterminantLinearSystemException &e) {

                ROS_WARN("GTSAM Exception caught.");
                cout << e.what() << endl;
                graph[i].print();
                initialEstimate[i].print();

            }

            if (false) { //PATCH - Evgeny
                // t-bsp
                std::string post_E;
                std::string post_V;
                Graph posterior_topological_graph;
                if ((USING_MR_FACTORS) && (NUM_ROBOTS > 1)) { // PATCH Evgeny
//                    std::string post_E = post_EE[i];
//                    std::string post_V = post_VV[i];
                } else {
                    std::string post_E = plannData.prior_edges + delta_edges[0][i]; // single robot for now, TODO MR
                    std::string post_V = plannData.prior_nodes + delta_nodes[0][i];
                }
                posterior_topological_graph.updateGraph(post_E, post_V, true);
                posterior_topological_graph.calculateSignature();

                if (i == 0) {
                    t_idx_opt = 0;
                    s_opt = posterior_topological_graph.signature.s_VN;
                    J_t = J;

                } else if (posterior_topological_graph.signature.s_VN > s_opt) {
                    t_idx_opt = i;
                    s_opt = posterior_topological_graph.signature.s_VN;
                    J_t = J;
                }
            }
        }
    } else { // incremental standard and topological BSP

        ROS_WARN("Calculate posterior beliefs and topologies in INCR mode");

        for (int i = 0; i < graph.size(); i++) {

            ROS_WARN("|\nAction %ld", i);

            gttic_(isam2BSP);

			gttic_(isam2Fact);
            ISAM2 posterior_belief = *isam2; // all actions share the same prior belief
            // posterior belief b(X_{k+L}| Z_{k+L}, U_{k+L})
            posterior_belief.update(graph[i], initialEstimate[i]);
            Values result = posterior_belief.calculateBestEstimate();
            NonlinearFactorGraph posterior_fg = posterior_belief.getFactorsUnsafe();
			gttoc_(isam2Fact);

			gttic_(marginals);
            // Calculate and print marginal covariances for variables of interest
            cout.precision(3);
            // Calculate and print marginal covariances for all variables
            Marginals marginals(posterior_fg, result);
            /*cout << "keys size of graph " << i << " is " << posterior_belief.getFactorsUnsafe().keys().size() << endl;
            gtsam::Matrix cov_end_pose = posterior_belief.marginalCovariance(initialEstimate[i].keys().back());
            Symbol last_pose(initialEstimate[i].keys().back());
            cout << "x(k+L) covariance [" << last_pose.chr() << last_pose.index() << "]:\n" << cov_end_pose
                 << endl; // uncertainty of the last pose of the i-th action*/

            //result.erase(result.keys().front());
            result.erase(Key(Symbol('A', 0))); // this is deterministic var
            JointMarginal JointInf = marginals.jointMarginalInformation(KeyVector(result.keys()));

			gttoc_(marginals);


            gttic_(obj);
            //gtsam::Matrix Lambda = JointInf.fullMatrix();
            //std::ofstream Lfs("/home/andrej/L.txt");
            //Lfs << Lambda.fullMatrix().toDenseMatrix(); // ok

            gttic_(gtsamChol);
            gtsam::Matrix R_mat = gtsam::RtR(JointInf.fullMatrix()); // Cholesky factorization Lambda = R'R
            //std::ofstream Rfs("/home/andrej/R.txt");
            //Rfs << R_mat; // ok, up to numerical error
            gttoc_(gtsamChol);

            /*SparseMatrix<double> Lambda_sparse(Lambda.rows(), Lambda.cols());
            for (int p = 0; p < Lambda.rows(); p++) {
                for (int q = 0; q < Lambda.cols(); q++) {
                    //if (Lambda(p,q) < FLT_EPSILON)
                        Lambda_sparse.coeffRef(p,q) = Lambda(p,q);
                }
            }
            Lambda_sparse.makeCompressed();


            SparseQR<SparseMatrix<double>, Eigen::COLAMDOrdering<int> > qr(Lambda_sparse); // QR factorization
            qr.analyzePattern(Lambda_sparse);
            qr.factorize(Lambda_sparse);
            SparseMatrix<double> R = qr.matrixR();

             */


           /* // Compute marginals
            try {
                int vi = 0;
                BOOST_FOREACH(Key key1, result.keys()) {
                                int vj = 0;
                                BOOST_FOREACH(Key key2, result.keys()) {
                                                gtsam::Matrix m;
                                                if (vi != vj) {
                                                    std::vector<Key> keys(2);
                                                    keys[0] = key1;
                                                    keys[1] = key2;
                                                    JointMarginal info = marginals.jointMarginalInformation(keys);
                                                    m = info.at(key1, key2);

                                                } else {
                                                    // diagonal
                                                    m = marginals.marginalInformation(key1);
                                                }
                                                Symbol s1(key1), s2(key2);
                                                int x = s1.index()-1, y = s2.index()-1; // one key was removed, anchor

                                                if (!m.isApproxToConstant(0, 1e-3)) {


                                                    ROS_WARN("%d, %d", x, y);
                                                    for (int p = 0; p < m.rows(); p++)
                                                        for (int q = 0; q < m.cols(); q++)
                                                            Lambda_sparse.coeffRef(6 * x + p, 6 * y + q) = m(p,
                                                                                                             q);
                                                }

                                                ++vj;

                                            }
                                ++vi;

                            }
            } catch (std::exception& e) {
                cout << e.what() << endl;
            }*/



            //double J = cov_end_pose.determinant();

            double J = R_mat.rows()/2.0 * log(2*M_PI*M_E);
            double logDetR = 0;
            for (unsigned i = 0; i < R_mat.rows(); ++i) {
                logDetR += std::log(fabs(R_mat(i, i)));
                //logDetR2 += std::log(fabs(R.coeff(i, i)));
            }

            J -= 2*logDetR;

			
            if (i == 0) {
                idx_opt = 0;
                J_opt = J;

            } else if (J < J_opt) {
                idx_opt = i;
                J_opt = J;
            }
            gttoc_(obj);


            gttoc_(isam2BSP);
            tictoc_getNode(tisam2BSP, isam2BSP);
            Topology::time_ofs << tisam2BSP.get()->wall() << " ";

            //result.print("Final estimate:\n");

            // t-bsp
            Graph posterior_topological_graph = T->prior_graph;
            posterior_topological_graph.signature.active[VN_incr] = true;
            posterior_topological_graph.signature.active[VN] = true; // only for testing and time comparison, otherwise can be turned off

            //cout << "Edges of action " << i << ": " << delta_edges[0][i] << endl;
            //cout << "Nodes of action " << i << ": " << delta_nodes[0][i] << endl;
            posterior_topological_graph.updateGraphAndSignature(delta_edges[0][i], delta_nodes[0][i], result); // single robot for now, TODO MR


            //delta_edges[i]

            if (i == 0) {
                t_idx_opt = 0;
                s_opt = posterior_topological_graph.signature.s_VN_incr;
                J_t = J;

            } else if (posterior_topological_graph.signature.s_VN_incr > s_opt) {
                t_idx_opt = i;
                s_opt = posterior_topological_graph.signature.s_VN_incr;
                J_t = J;
            }

            // save posterior factor graph as graphviz dot file
            // Render to PDF using "fdp fg.dot -Tpdf > fg.pdf"
            /*ofstream os("fg.dot");
            posterior_belief.getFactorsUnsafe().saveGraph(os, result);
            os.close();
            // save the isam2 object in gtsam format
            gtsam::serializeToFile(posterior_belief, "isam2.txt");
            gtsam::serializeToFile(posterior_belief.getFactorsUnsafe(), "factors.txt");
            Values initial_vals = isam2->calculateEstimate();
            initial_vals.insert(initialEstimate[i]);
            gtsam::serializeToFile(initial_vals, "initialEst.txt");
            gtsam::serializeToFile(result, "result.txt");*/




/*
            std::cout << "Press Enter to continue ...";
            std::string tempStr;
            std::getline(std::cin, tempStr);*/

        }
    }
    delta_edges[0].clear(); // TODO MR
    delta_nodes[0].clear();

    J_t = J_opt; // PATCH - Evgeny
    t_idx_opt = idx_opt; // PATCH - Evgeny

    Topology::time_ofs << "\n"; // terminate planning session
    tictoc_finishedIteration_();
    ROS_WARN_STREAM("Opt. action by: (standard bsp, t-bsp) = (" << idx_opt << ", " << t_idx_opt << ")");
    ROS_WARN("Relative error: %lf", (J_t-J_opt)/J_opt);

    // what action should be the selected
    if (planner_algorithm == "t-bsp") // topological bsp
        return t_idx_opt;
    else // standard bsp
        return idx_opt;

}


void Planner::decomposeMultiRobotBelief(NonlinearFactorGraph &graph_in, Values &vals_in, NonlinearFactorGraph* graph_out,
                                   Values* vals_out, NonlinearFactorGraph& residual_graph) {

    if (NUM_ROBOTS == 1) { // single robot case, no MR factors no need to waste time in the for loop

        graph_out[0] = graph_in;
        vals_out[0] = vals_in;


    } else { // separate MR factors from local for all robots

        char robot_id;

        for (auto factor : graph_in) {
            std::vector<char> robot_ids;
            std::vector<size_t> factor_indexes;
            for (auto factors_key : factor.get()->keys()) {
                gtsam::Symbol symbol(factors_key);
                // detect multi robot factors
                if (!(std::find(robot_ids.begin(), robot_ids.end(), symbol.chr()) != robot_ids.end())) {
                    robot_ids.push_back(symbol.chr());
                }
                factor_indexes.push_back(symbol.index());
            }


            // assume only priors and between factors
            if (robot_ids.size() > 1) { // multi robot factor

                residual_graph.add(factor);
                ROS_WARN_STREAM("Multi robot factor found between "
                                        << "robot " << robot_ids.at(0) << "index " << factor_indexes.at(0)
                                        << ", and robot " << robot_ids.at(1) << ", index " << factor_indexes.at(1));

            } else {

                for (unsigned int i = 0; i < NUM_ROBOTS; i++) {
                    robot_id = 'A' + i;

                    if (robot_ids.at(0) == robot_id) { // local factor for calling robot

                        graph_out[i].add(factor);
                        if (factor_indexes.size() == 1)
                            ROS_WARN_STREAM(
                                    "Robot prior factor found: " << robot_ids.at(0) << factor_indexes.at(0));
                        else
                            ROS_WARN_STREAM(
                                    "Local robot binary factor found: " << robot_ids.at(0) << factor_indexes.at(0)
                                                                         << robot_ids.at(0)
                                                                         << factor_indexes.at(1));
                        break;
                    }
                }
            }
        }

        for (auto value_key : vals_in.keys()) {
            gtsam::Symbol symbol(value_key);
            for (unsigned int i = 0; i < NUM_ROBOTS; i++) {
                robot_id = 'A' + i;

                if (symbol.chr() == robot_id) {
                    vals_out[i].insert(value_key, vals_in.at(value_key));
                    ROS_WARN_STREAM(
                            "State found: " << symbol.chr() << symbol.index());
                    break;
                }
            }
        }
    }

    plannData.MR_info_generated = true; // enough to call this function once to get MR factors in centralized inference
};

void Planner::visualizeOptJointAction(){

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "optimal_joint_action";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1;
    path_marker.scale.x = 0.3;
    path_marker.scale.y = 0.1;
    path_marker.scale.z = 0.1;
    path_marker.color.a = 1.0; // Don't forget to set the alpha!
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;

    geometry_msgs::Point point;

    visualization_msgs::MarkerArray vis_markers_array;


    for (std::vector<nav_msgs::Path>::iterator ipath = optimal_joint_action.actions.begin() ; ipath != optimal_joint_action.actions.end(); ++ipath) {


        for (std::vector<geometry_msgs::PoseStamped>::iterator ipose = ipath->poses.begin() ; ipose != ipath->poses.end(); ++ipose) {
            point.x = ipose->pose.position.x;
            point.y = ipose->pose.position.y;
            point.z = ipose->pose.position.z;
            path_marker.points.push_back(point);
        }

        vis_markers_array.markers.push_back(path_marker);
        path_marker.id++;
        path_marker.points.clear();
    }

    vis_opt_actions_pub.publish(vis_markers_array);
    vis_markers_array.markers.clear();
}

#include <planner/eigen_utils.hpp>