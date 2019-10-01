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
 * @file: generate_actions_prm.cpp
 * @brief: Implementation of base class for planners
 * @author: Tal Regev
 * @author: Asaf Feniger
 *
 */
#include "planner/generate_actions/generate_actions_prm.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <diverse_short_paths/TestData.h>
#include <diverse_short_paths/Levenshtein.h>
#include <diverse_short_paths/Neighborhood.h>
#include <diverse_short_paths/Eppstein.h>
#include <diverse_short_paths/Frechet.h>
#include <diverse_short_paths/Hausdorff.h>
#include <diverse_short_paths/Voss.h>

using namespace ANPL;

namespace ob  = ompl::base;
namespace og  = ompl::geometric;

namespace bg  = boost::geometry;
namespace bgi = boost::geometry::index;


GenerateActionsPRM::GenerateActionsPRM(const Perception *const p_perception, const Goals *const p_goals) :
    GenerateActions(p_perception, p_goals)
{
	ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);
	
    m_min_edge_length = 2;
    m_max_edge_length = 10;
    m_max_vertices = 550;
    m_max_path_length = 1000;
    m_min_path_pairwise_distance = 1;

    m_time_candidate_paths = 20;
    m_time_edges_and_vertices = 3;

    m_num_candidate_paths = 10;
    m_low_boundary = -20;
    m_high_boundary = 10;
    m_range_box = 5;

    m_k_diverse_short_alg = EPPSTEIN;
    m_path_distance_measure_alg = LEVENSHTEIN;

    //this two parameters don't in use in original code (Without the xml loader)
    m_radius_factor = 0;
    m_avoid_method = AM::UNKNOWN;

    visualization_msgs::Marker::_scale_type vertices_scale;
    vertices_scale.x = 0.2;
    vertices_scale.y = 0.2;

    visualization_msgs::Marker::_color_type vertices_color;
    // points are red
    vertices_color.r = 1;
    vertices_color.g = 0;
    vertices_color.b = 0;
    vertices_color.a = 1.0;

    visualization_msgs::Marker::_scale_type edges_scale;
    edges_scale.x = 0.1;
    edges_scale.y = 0.1;

    visualization_msgs::Marker::_color_type edges_color;
    // lines are olive green
    edges_color.r = 85  / 255.0;
    edges_color.g = 107 / 255.0;
    edges_color.b = 47  / 255.0;
    edges_color.a = 1.0;

    visualization_msgs::Marker::_scale_type bounds_scale;
    bounds_scale.x = 0.1;
    bounds_scale.y = 0.1;

    visualization_msgs::Marker::_color_type bounds_color;
    // points are black
    bounds_color.r = 0;
    bounds_color.g = 0;
    bounds_color.b = 0;
    bounds_color.a = 1.0;

    visualization_msgs::Marker::_scale_type start_goal_scale;
    start_goal_scale.x = 0.5;
    start_goal_scale.y = 0.5;

    visualization_msgs::Marker::_color_type start_goal_color;
    // points are dark blue
    start_goal_color.r = 0;
    start_goal_color.g = 0;
    start_goal_color.b = 139 / 255.0;
    start_goal_color.a = 1.0;

    visualization_msgs::Marker::_scale_type candidate_paths_scale;
    candidate_paths_scale.x = 0.1;
    candidate_paths_scale.y = 0.1;

//    init("map",
//         vertices_scale,
//         vertices_color,
//         edges_scale,
//         edges_color,
//         bounds_scale,
//         bounds_color,
//         start_goal_scale,
//         start_goal_color,
//         candidate_paths_scale
//    );
}

GenerateActionsPRM::GenerateActionsPRM(const Perception *const p_perception, const Goals *const p_goals, const Visualization *const vis, ConfigLoader& loader) :
        GenerateActions(p_perception, p_goals, loader, vis),
        m_min_edge_length(loadValueAs(loader, m_min_edge_length)),
        m_max_edge_length(loadValueAs(loader, m_max_edge_length)),
        m_max_vertices(loadValueAs(loader, m_max_vertices)),
        m_max_path_length(loadValueAs(loader, m_max_path_length)),
        m_min_path_pairwise_distance(loadValueAs(loader, m_min_path_pairwise_distance)),
        m_num_candidate_paths(loadValueAs(loader, m_num_candidate_paths)),
        m_low_boundary(loadValueAs(loader, m_low_boundary)),
        m_high_boundary(loadValueAs(loader, m_high_boundary)),
        m_range_box(loadValueAs(loader, m_range_box)),
        m_radius_factor(loadValueAs(loader, m_radius_factor))
{
	ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);

    std::string ns = "GenerateActionsPRM";

    m_vis_vertices.ns       = ns;
    m_vis_edges.ns          = ns;
    m_vis_bounds.ns         = ns;
    m_vis_start_goal.ns     = ns;
    m_vis_vertices_text.ns  = ns;

    loadKDS(loader);
    loadPDM(loader);
    loadAM(loader);
}

void GenerateActionsPRM::setup(ob::PlannerTerminationConditionFn &conditionFn) {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);

    ob::StateSpacePtr space(new ob::SE2StateSpace());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(m_low_boundary);
    bounds.setHigh(m_high_boundary);

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup simple_setup(space);

    Waypoint pose = m_p_perception->getRobotLocation();
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(pose.x());
    start->setY(pose.y());
    start->setYaw(pose.rotation().yaw());

    // create a goal state
    Waypoint nextGoal = m_p_goals->front().getWaypoint();
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(nextGoal.x());
    goal->setY(nextGoal.y());
    goal->setYaw(nextGoal.rotation().yaw());

    // set the start and goal states
    simple_setup.setStartAndGoalStates(start, goal);

    m_planner = ob::PlannerPtr(new og::PRM(simple_setup.getSpaceInformation()));

    og::PRM::ConnectionFilter connection_filter = prmFilterStrategy();
    m_planner->as<og::PRM>()->setConnectionFilter(connection_filter);

    simple_setup.setPlanner(m_planner);

    conditionFn = [this]()
    {
        ob::PlannerData data(m_planner->getSpaceInformation());
        m_planner->getPlannerData(data);
        unsigned long int num_vertices = data.numVertices();
        bool to_stop =  num_vertices >= m_max_vertices;
        return to_stop;
    };

    // set state validity checking for this space
    simple_setup.setStateValidityChecker(boost::bind(&GenerateActionsPRM::isStateValidFCL, this, _1));
    double resolution = m_p_perception->getResolution();
    simple_setup.getSpaceInformation()->setStateValidityCheckingResolution(resolution);

    simple_setup.setup();
    ob::PlannerTerminationConditionFn stopConditionFn = [this]()
    {
        return true;
    };
    simple_setup.solve(stopConditionFn);
}

og::PRM::ConnectionFilter GenerateActionsPRM::prmFilterStrategy() {
	ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);

    og::PRM::ConnectionFilter connection_filter = [this](const og::PRM::Vertex &v1, const og::PRM::Vertex &v2) {
        //from: http://www.boost.org/doc/libs/1_38_0/libs/graph/doc/using_adjacency_list.html
        og::PRM::Graph g = m_planner->as<og::PRM>()->getRoadmap();
        boost::property_map<og::PRM::Graph, og::PRM::vertex_state_t>::type
                states = boost::get(og::PRM::vertex_state_t(), g);
        ob::State* state1 = states[v1];
        ob::State* state2 = states[v2];
        gtsam::Pose3 waypoint1(Conversion<gtsam::Pose2>::as(state1));
        gtsam::Pose3 waypoint2(Conversion<gtsam::Pose2>::as(state2));

        double range = waypoint1.range(waypoint2);

        Globals::string_stream << std::endl << "v1: "    << v1
                               << std::endl << "v2: "    << v2
                               << std::endl << "g[v1]: " << waypoint1.translation()
                               << std::endl << "g[v2]: " << waypoint2.translation()
                               << std::endl << "range: " << range;
        anplLogMessage(info, 2, Globals::string_stream, LogTag::generate_action);

        if (range <= m_max_edge_length && range >= m_min_edge_length) {
            return !m_p_perception->isObstacleAheadFCL(waypoint1, waypoint2);
        }
        return false;
    };
    return connection_filter;
}

bool GenerateActionsPRM::isStateValid(const ob::State *state) const {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);

    Waypoint waypoint(Conversion<gtsam::Pose2>::as(state));
    return !m_p_perception->isStateOccupied(waypoint);
}

bool GenerateActionsPRM::isStateValidFCL(const ob::State *state) const {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);

    Waypoint waypoint(Conversion<gtsam::Pose2>::as(state));
    Waypoint pose = m_p_perception->getRobotLocation();
    if (waypoint.x() == pose.x() && waypoint.y() == pose.y() && pose.rotation().yaw() == waypoint.rotation().yaw()) {
        return true;
    }
    return !m_p_perception->isStateOccupiedFCL(waypoint);
}

ActionsPaths GenerateActionsPRM::generateActions() {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);

    const bool is_vis_edges_and_vertices    = true;
    unsigned int id_vis                     = m_id_start + 1;


    m_vis_array.markers.push_back(m_vis_clear_all);
    m_marker_array_pub.publish(m_vis_array);
    m_vis_array.markers.clear();


    // Setup the planner: m_planner member and stop condition (maximum number of vertices)
    ob::PlannerTerminationConditionFn conditionFn;
    setup(conditionFn);

    setMarkerTime(m_vis_start_goal);
    setMarkerTime(m_vis_vertices);
    setMarkerTime(m_vis_edges);
    setMarkerTime(m_vis_bounds);


    ob::PlannerData data(m_planner->getSpaceInformation());
    m_planner->getPlannerData(data);
    m_vis_start_goal.points.push_back(getPointFromVertex<gtsam::Pose2>(data.getVertex(data.getStartIndex(0))));
    m_vis_start_goal.points.push_back(getPointFromVertex<gtsam::Pose2>(data.getVertex(data.getGoalIndex(0))));
    setMarkerBounds(m_vis_bounds, m_low_boundary, m_high_boundary);

    m_vis_array.markers.push_back(m_vis_start_goal);
    m_vis_array.markers.push_back(m_vis_bounds);
    m_marker_array_pub.publish(m_vis_array);
    m_vis_array.markers.clear();

    m_planner->as<og::PRM>()->growRoadmap(conditionFn);
    m_planner->getPlannerData(data);

    Globals::string_stream << "########## OMPL planning ###########";
    anplLogMessage(info, 1, Globals::string_stream, LogTag::generate_action);

    Globals::string_stream << "data.numVertices(): " << data.numVertices();
    anplLogMessage(info, 1, Globals::string_stream, LogTag::generate_action);

    if(is_vis_edges_and_vertices) {
        for (unsigned int v = 0; v < data.numVertices(); ++v, ++id_vis) {
            geometry_msgs::Point point_v = getPointFromVertex<gtsam::Pose2>(data.getVertex(v));
            std::vector<unsigned int> edgeList;
            data.getEdges(v, edgeList);
            for (unsigned int v_edge : edgeList) {
                geometry_msgs::Point point_e = getPointFromVertex<gtsam::Pose2>(data.getVertex(v_edge));
                m_vis_edges.points.push_back(point_v);
                m_vis_edges.points.push_back(point_e);
            }
            m_vis_vertices.points.push_back(point_v);

            setMarkerTime(m_vis_vertices_text);
            m_vis_vertices_text.id = id_vis;
            m_vis_vertices_text.pose.position = point_v;
            m_vis_vertices_text.pose.position.z = 1;
            m_vis_vertices_text.text = std::to_string(v);
            m_vis_array.markers.push_back(m_vis_vertices_text);
        }

        m_vis_array.markers.push_back(m_vis_vertices);
        m_vis_array.markers.push_back(m_vis_edges);
        m_marker_array_pub.publish(m_vis_array);
        m_vis_array.markers.clear();
    }

    std::stringstream string_stream;
    data.printGraphML(string_stream);

    //from: https://bitbucket.org/caleb_voss/diverse_short_paths
    //from: http://calebvoss.com/wp-content/uploads/voss-2015-diverse_short_paths.pdf
    TestData *testData = new TestData(string_stream, m_num_candidate_paths, m_max_path_length, m_min_path_pairwise_distance);
    PathDistanceMeasure *distanceMeasure = nullptr;
    switch(m_path_distance_measure_alg) {
        case LEVENSHTEIN:
            distanceMeasure = new Levenshtein();
            break;
        case FRECHET:
            distanceMeasure = new Frechet();
            break;
        case HAUSDORFF:
            distanceMeasure = new Hausdorff();
    }

    Neighborhood::AvoidMethod avoidMethod;
    switch(m_avoid_method) {
        case AM::CSPACE:
            avoidMethod = Neighborhood::CSPACE;
            break;
        case AM::GRAPH:
            avoidMethod = Neighborhood::GRAPH;
            break;
        case AM::UNKNOWN:
            avoidMethod = Neighborhood::UNKNOWN;
    }
    KDiverseShort *kDiverseShort;
    switch(m_k_diverse_short_alg) {
        case EPPSTEIN:
            kDiverseShort = new Eppstein(testData, distanceMeasure);
            break;
        case VOSS:
            kDiverseShort = new Voss(testData, distanceMeasure, m_radius_factor, avoidMethod);
    }

    PathsVertices paths_vertices;
    kDiverseShort->timedRun();
    kDiverseShort->print(Globals::string_stream);
    anplLogMessage(info, 1, Globals::string_stream, LogTag::generate_action);
    kDiverseShort->saveNodes(paths_vertices);

    delete kDiverseShort;
    delete distanceMeasure;
    delete testData;

    //actions for all paths generate from actions generator;
    ActionsPaths paths_actions;
    //waypoints for all paths in robot perspective
    std::vector<PathPose3> paths_waypoints;

    for (int path_number = 0; path_number < paths_vertices.size(); ++path_number) {
        auto &path_vertices = paths_vertices[path_number];
        PathPose3 path_waypoints;
        PathPose3 path_actions;
        setWaypointsPathFromVerticesPath<gtsam::Pose2>(data, path_vertices, data.getGoalIndex(0), path_waypoints);
        setActionsPathFromWaypointsPath(path_waypoints, path_actions);
        paths_waypoints.push_back(path_waypoints);
        paths_actions.push_back(path_actions);
    }

    drawActionsPaths(paths_actions);

    return paths_actions;
}

void GenerateActionsPRM::loadPDM(ConfigLoader& loader) {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);

    std::string $ = loader.getValue<std::string>(nameOf(m_path_distance_measure_alg));

    if (strcasecmp($.c_str(), "f") == 0 || strcasecmp($.c_str(), "frechet") == 0) {
        m_path_distance_measure_alg = PDM::FRECHET;
        return;
    }
    if (strcasecmp($.c_str(), "l") == 0 || strcasecmp($.c_str(), "levenshtein") == 0) {
        m_path_distance_measure_alg = PDM::LEVENSHTEIN;
        return;
}
    if (strcasecmp($.c_str(), "h") == 0 || strcasecmp($.c_str(), "hausdorff") == 0) {
        m_path_distance_measure_alg = PDM::HAUSDORFF;
        return;
    }

    throw std::invalid_argument(std::string("loading error: unknown path distance measure algorithm: ") + $);
}

void GenerateActionsPRM::loadKDS(ConfigLoader& loader) {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);

    std::string $ = loader.getValue<std::string>(nameOf(m_k_diverse_short_alg));

    if (strcasecmp($.c_str(), "e") == 0 || strcasecmp($.c_str(), "eppstein") == 0) {
        m_k_diverse_short_alg = KDS::EPPSTEIN;
        return;
    }
    if (strcasecmp($.c_str(), "v") == 0 || strcasecmp($.c_str(), "voss") == 0) {
        m_k_diverse_short_alg = KDS::VOSS;
        return;
    }

    throw std::invalid_argument(std::string("loading error: unknown k diverse short algorithm: ") + $);
}

void GenerateActionsPRM::loadAM(ConfigLoader& loader) {
    ADD_FIRST_LEVEL_INFO_LOG(LogTag::generate_action);

    std::string $ = loader.getValue<std::string>(nameOf(m_avoid_method));

    if (strcasecmp($.c_str(), "c") == 0 || strcasecmp($.c_str(), "cspace") == 0) {
        m_avoid_method = AM::CSPACE;
        return;
    }
    if (strcasecmp($.c_str(), "g") == 0 || strcasecmp($.c_str(), "graph") == 0) {
        m_avoid_method = AM::GRAPH;
        return;
    }

    m_avoid_method = AM::UNKNOWN;
}