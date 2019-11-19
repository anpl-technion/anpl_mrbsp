#ifndef ACTION_GENERATOR_OMPL_PLANNERS_H
#define ACTION_GENERATOR_OMPL_PLANNERS_H

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <boost/graph/astar_search.hpp>
#include <iostream>


#define MAP_RESOLUTION 0.1
#define ROBOT_BOX_SIZE_X 0.6
#define ROBOT_BOX_SIZE_Y 0.6
#define ROBOT_BOX_SIZE_Z 0.6

#include <fcl/octree.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/collision.h>
#include <fcl/continuous_collision.h>

#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


//void omplPathToGtsamGraph(og::PathGeometric path);
// define this class:
class rayCastingMotionValidator : public ob::MotionValidator
{
public:
    rayCastingMotionValidator(
            const ob::SpaceInformationPtr& space_info, const ob::StateValidityCheckerPtr stateValidityCheckerPtr, const std::shared_ptr<octomap::OcTree> octree_map_ptr)
    : ob::MotionValidator(space_info),
      stateValidityChecker(stateValidityCheckerPtr),
    p_octree_map(octree_map_ptr) {

    }

    // implement checkMotion()
    bool checkMotion(const ob::State* s1, const ob::State* s2) const {
        std::pair<ob::State*, double> unused;
        return checkMotion(s1, s2, unused);
    }

    // Check motion returns *false* if invalid, *true* if valid.
    // last_valid is the state and percentage along the trajectory until invalid state
    bool checkMotion(const ob::State* s1, const ob::State* s2,
                             std::pair<ob::State*, double>& last_valid) const {

        /*A ray is cast from origin with a given direction, the first occupied cell is returned (as center coordinate).
         * If the starting coordinate is already occupied in the tree, this coordinate will be returned as a hit.

                Parameters:
        origin	starting coordinate of ray
        direction	A vector pointing in the direction of the raycast. Does not need to be normalized.
        end	returns the center of the cell that was hit by the ray, if successful
        ignoreUnknownCells	whether unknown cells are ignored. If false (default), the raycast aborts when an unkown cell is hit.
        maxRange	Maximum range after which the raycast is aborted (<= 0: no limit, default)

        Returns:
        whether an occupied cell was hit */

        octomap::point3d origin(s1->as<ob::SE2StateSpace::StateType>()->getX(), s1->as<ob::SE2StateSpace::StateType>()->getY(), PATH_Z_OFFSET);
        octomap::point3d direction(s2->as<ob::SE2StateSpace::StateType>()->getX(), s2->as<ob::SE2StateSpace::StateType>()->getY(), PATH_Z_OFFSET);
        direction -= origin;
        octomap::point3d end;

        if (p_octree_map->castRay(origin, direction, end, true, direction.norm()+p_octree_map->getResolution())) {

            if (last_valid.first != nullptr) {
                ob::ScopedState<ob::SE2StateSpace> last_valid_state(si_->getStateSpace());
                last_valid_state[0] = end.x();
                last_valid_state[1] = end.y();
                si_->copyState(last_valid.first, last_valid_state.get());
                last_valid.second = (end - origin).norm() / direction.norm();
            }
            return false;
        }
        else
            return true;

    }

protected:
    ob::StateValidityCheckerPtr stateValidityChecker;
    std::shared_ptr<octomap::OcTree> p_octree_map;

};


// Used for A* search.  Computes the heuristic distance from vertex v1 to the goal
ob::Cost distanceHeuristic(ob::PlannerData::Graph::Vertex v1,
                           const ob::GoalState* goal,
                           const ob::OptimizationObjective* obj,
                           const boost::property_map<ob::PlannerData::Graph::Type,
                                   vertex_type_t>::type& plannerDataVertices)
{
    return ob::Cost(obj->costToGo(plannerDataVertices[v1]->getState(), goal));
}


void readPlannerData(void)
{
    std::cout << std::endl;
    std::cout << "Reading PlannerData from './myPlannerData'" << std::endl;

    // Recreating the space information from the stored planner data instance
    ob::StateSpacePtr space(new ob::SE3StateSpace());
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    ob::PlannerDataStorage dataStorage;
    ob::PlannerData data(si);

    // Loading an instance of PlannerData from disk.
    dataStorage.load("myPlannerData", data);

    // Re-extract the shortest path from the loaded planner data
    if (data.numStartVertices() > 0 && data.numGoalVertices() > 0)
    {
        // Create an optimization objective for optimizing path length in A*
        ob::PathLengthOptimizationObjective opt(si);

        // Computing the weights of all edges based on the state space distance
        // This is not done by default for efficiency
        data.computeEdgeWeights(opt);

        // Getting a handle to the raw Boost.Graph data
        ob::PlannerData::Graph::Type& graph = data.toBoostGraph();

        // Now we can apply any Boost.Graph algorithm.  How about A*!

        // create a predecessor map to store A* results in
        boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(data.numVertices());

        // Retieve a property map with the PlannerDataVertex object pointers for quick lookup
        boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graph);

        // Run A* search over our planner data
        ob::GoalState goal(si);
        goal.setState(data.getGoalVertex(0).getState());
        ob::PlannerData::Graph::Vertex start = boost::vertex(data.getStartIndex(0), graph);
        boost::astar_search(graph, start,
                            boost::bind(&distanceHeuristic, _1, &goal, &opt, vertices),
                            boost::predecessor_map(prev).
                                    distance_compare(boost::bind(&ob::OptimizationObjective::
                            isCostBetterThan, &opt, _1, _2)).
                                    distance_combine(boost::bind(&ob::OptimizationObjective::
                            combineCosts, &opt, _1, _2)).
                                    distance_inf(opt.infiniteCost()).
                                    distance_zero(opt.identityCost()));

        // Extracting the path
        og::PathGeometric path(si);
        for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
             prev[pos] != pos;
             pos = prev[pos])
        {
            path.append(vertices[pos]->getState());
        }
        path.append(vertices[start]->getState());
        path.reverse();

        // print the path to screen
        //path.print(std::cout);
        std::cout << "Found stored solution with " << path.getStateCount() << " states and length " << path.length() << std::endl;
    }
}
/*
gtsam::Pose3* omplStateToGtsamPose3(ob::SE3StateSpace::StateType* state){
    gtsam::Point3 point3(state->getX(),state->getY(),state->getZ());
    const ob::SO3StateSpace::StateType& rotation = state->rotation();
    gtsam::Quaternion quaternion(rotation.w,rotation.x,rotation.y,rotation.z);
    gtsam::Rot3 rot3(quaternion);
    return new gtsam::Pose3(rot3,point3);
}

void omplPathToGtsamGraph(og::PathGeometric path) {
    // Create an empty nonlinear factor graph
    gtsam::NonlinearFactorGraph graph;

    // Add a prior on the first pose, setting it to the origin
    // A prior factor consists of a mean and a noise model (covariance matrix)
    std::vector<ob::State*>& states = path.getStates();


    gtsam::Pose3* startPose = omplStateToGtsamPose3(states.front()->as<ob::SE3StateSpace::StateType>());
    // prior at origin

    // containers
    vector<Symbol*>     symbols;
    vector<Pose3*>      poses;
    Values              values;

    Symbol* firstSymbol = new Symbol('X',0);
    symbols.push_back(firstSymbol);
    values.insert(firstSymbol->key(),*startPose);
    poses.push_back(startPose);

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4));
    gtsam::PriorFactor<gtsam::Pose3> priorFactor(firstSymbol->key(), *startPose, priorNoise);
    graph.add(priorFactor);
    long state_index = 0;

    for (vector<ob::State*>::iterator it = states.begin()+1; it != states.end(); ++it)
    {
        state_index++;
        Symbol* newSymbol = new Symbol('x',state_index);
        gtsam::Pose3* newPose = omplStateToGtsamPose3((*it)->as<ob::SE3StateSpace::StateType>());
        Symbol* previousSymbol  = symbols.back();
        Pose3* previousPose = poses.back();
        Pose3 control           = previousPose->between(*newPose);
        BetweenFactor<Pose3> factor = BetweenFactor<Pose3>(previousSymbol->key(), newSymbol->key(),control,priorNoise);
        graph.add(factor);
        symbols.push_back(newSymbol);
        poses.push_back(newPose);
        values.insert(newSymbol->key(),*newPose);
    }

      // optimize using Levenberg-Marquardt optimization
      Values result = LevenbergMarquardtOptimizer(graph, values).optimize();
      result.print("Final Result:\n");

      // Calculate and print marginal covariances for all variables
      cout.precision(2);
      Marginals marginals(graph, result);
      // show how the covariance evolves for first 3 poses
      for (int i = 0; i < 3; ++i) {
          cout << "x" << i << " covariance:\n" << marginals.marginalCovariance(symbols.at(i)->key()) << endl;
      }

}

*/

/*
bool PerceptionMapOctomap::isStateOccupied(const Waypoint &waypoint) const {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::map);

    if(NULL != m_p_octree_map) {
        octomap::point3d point3d = Conversion<octomap::point3d>::as(waypoint.translation());
        octomap::OcTreeNode *octree_node = m_p_octree_map->search(point3d);
        if (NULL != octree_node) {
            if (m_p_octree_map->isNodeOccupied(octree_node)) {
                return true;
            }
        }
    }
    return false;
}

bool PerceptionMapOctomap::isStateOccupiedFCL(const Waypoint &waypoint) const {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::map);

    //https://github.com/kuri-kustar/laser_collision_detection/blob/master/src/laser_obstacle_detect.cpp#L135-L228
    fcl::Transform3f tf = Conversion<fcl::Transform3f>::as(waypoint);
    fcl::CollisionObject robot(m_robot_box, tf);
    for (auto &&box : m_boxes) {
        fcl::CollisionResult result;
        fcl::CollisionRequest request;
        fcl::collide(&robot, &box, request, result);
        if(result.isCollision()) {
            return true;
        }
    }
    return false;
}

void PerceptionMapOctomap::generateBoxes() {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::map);

    m_boxes.clear();
    fcl::OcTree tree(m_p_octree_map);
    generateBoxesFromOctomap(tree, m_boxes);
}

void PerceptionMapOctomap::generateBoxesFromOctomap(const fcl::OcTree &tree, std::vector<fcl::CollisionObject> &boxes) const {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::map);

    for(auto&& box_ : tree.toBoxes()) {
        fcl::FCL_REAL x = box_[0];
        fcl::FCL_REAL y = box_[1];
        fcl::FCL_REAL z = box_[2];
        fcl::FCL_REAL size = box_[3];
        fcl::FCL_REAL cost = box_[4];
        fcl::FCL_REAL threshold = box_[5];
        fcl::Box* box = new fcl::Box(size, size, size);
        box->cost_density = cost;
        box->threshold_occupied = threshold;
        fcl::CollisionObject obj(std::shared_ptr<fcl::CollisionGeometry>(box), fcl::Transform3f(fcl::Vec3f(x, y, z)));
        boxes.push_back(obj);
    }
}

*/

#endif //ACTION_GENERATOR_OMPL_PLANNERS_H
