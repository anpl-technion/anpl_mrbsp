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
 * @file: action_generator.hpp
 * @brief: service which generates a set of valid actions given a planning request
 * @author: Andrej Kitanov
 *
 */

#ifndef ACTION_GENERATOR_NODE_ACTION_GENERATOR_H
#define ACTION_GENERATOR_NODE_ACTION_GENERATOR_H

#include <ros/ros.h>
#include <mrbsp_ros_msgs/Actions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrbsp_ros_msgs/GenerateActions.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#define PATH_Z_OFFSET 0.5
#include "ompl_planners.h"
#include <diverse_short_paths/TestData.h>
#include <diverse_short_paths/Levenshtein.h>
#include <diverse_short_paths/Neighborhood.h>
#include <diverse_short_paths/Eppstein.h>
#include <diverse_short_paths/Frechet.h>
#include <diverse_short_paths/Hausdorff.h>
#include <diverse_short_paths/Voss.h>

#include <tf/transform_listener.h>
#include <signal.h>
bool __TERMINATE_ALL_THREADS;
#define VAR_X 0.25
#define VAR_Y 0.25
typedef geometry_msgs::PoseStamped NavGoal;
#define TIMER_TIMEOUT (0.5F) // in sec

void mySigintHandler(int sig)
{
	// Do some custom action.
	// For example, publish a stop message to some other nodes.
	__TERMINATE_ALL_THREADS = true;

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

double distanceBetweenPoints(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
	return sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) + (p2.z-p1.z)*(p2.z-p1.z));
}

double distanceBetween2DPoints(const ob::State *state1, const ob::State *state2) {
    // cast the abstract state type to the type we expect
    const ob::SE2StateSpace::StateType *se2state1 = state1->as<ob::SE2StateSpace::StateType>();
    const ob::SE2StateSpace::StateType *se2state2 = state2->as<ob::SE2StateSpace::StateType>();


    return sqrt((se2state2->getX()-se2state1->getX())*(se2state2->getX()-se2state1->getX())
                + (se2state2->getY()-se2state1->getY())*(se2state2->getY()-se2state1->getY()));
}

/**
 * ActionGenerator class implements the service which generates a set of valid actions
 * for a given planning request
 */
class ActionGenerator
{

private:

	ros::NodeHandle nh;

	ros::ServiceServer generateActions_service_;

	/// subscribers to rviz interactive inputs to obtain intermediate and goal path points
	ros::Subscriber int_sub, goal_sub;
	/// publisher to mark selected robot
	ros::Publisher sel_pub;
	/// publisher of the current action
	ros::Publisher path_curr_pub;
    /// publisher of previous paths
	ros::Publisher path_pub;


    tf::Transform relative_pose;

	mrbsp_ros_msgs::Actions control;
	bool all_actions_generated;
	bool single_action_generated;
	geometry_msgs::PointStamped last_point_clicked;
	geometry_msgs::PoseWithCovarianceStamped initialpose;
	geometry_msgs::Point initialpoint;
	unsigned int idx_pose;
	unsigned int idx_path;
	nav_msgs::Path path_selected;
	nav_msgs::Path path_previous;
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker sel_path_marker;
	visualization_msgs::Marker goal_marker;
	boost::mutex mutex;
	bool serviceBusy;


    std::shared_ptr<octomap::OcTree> m_p_octree_map;
    std::shared_ptr<fcl::Box>        m_robot_box;


    // a pointer to a planner
    ob::PlannerPtr planner;

public:


    enum KDS {EPPSTEIN = 'e', VOSS = 'v'};
    enum PDM {LEVENSHTEIN = 'l', FRECHET = 'f', HAUSDORFF = 'h'};
    enum AM {CSPACE, GRAPH, UNKNOWN};

	ActionGenerator(ros::NodeHandle& n):
		nh(n),
		all_actions_generated(false), single_action_generated(false),
		idx_pose(0), idx_path(0),
        m_map_boundary(NULL)
	{
		// Read local parameters
		ros::NodeHandle local_nh("~");

		// Override the default ros sigint handler.
		// This must be set after the first NodeHandle is created.
		signal(SIGINT, mySigintHandler);

		int_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &ActionGenerator::setIntermediatePoints, this);
		goal_sub = nh.subscribe<NavGoal>("/move_base_simple/goal", 1, &ActionGenerator::setNavGoal, this);
		sel_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
		m_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>( "ompl_markers", 1 );
		//path_curr_pub = nh.advertise<visualization_msgs::Marker>( "path_marker", 10 );
		path_curr_pub = nh.advertise<nav_msgs::Path>("path_msg", 1);
		path_pub = nh.advertise<nav_msgs::Path>("paths", 1);
		//po_timer = nh.createTimer( ros::Duration( TIMER_TIMEOUT ), boost::bind( &ActionGenerator::onTimer,  this ) );


        std::string method;
        bool correctOctomapFormat = true;

        if (!local_nh.getParam("method", method) || method == "interactive") {
            method = "interactive";
            ROS_INFO("Interactive method");
            generateActions_service_ = nh.advertiseService("generate_actions", &ActionGenerator::generateActionsInteractive, this);

        } else {
            if (method == "ompl") {

                ROS_INFO("OMPL method");

                // box robot model for collision checking
                m_robot_box = (std::shared_ptr<fcl::Box>) new fcl::Box(ROBOT_BOX_SIZE_X, ROBOT_BOX_SIZE_Y, ROBOT_BOX_SIZE_Z);

                m_map_boundary = new ob::RealVectorBounds(2);
                std::vector<double> map_boundary(4);

                std::string world_octomap;
                if (local_nh.getParam("world_octomap", world_octomap)) {
                    ROS_WARN_STREAM("Using world " << world_octomap << " for actions generation.");


                    if (world_octomap.substr(world_octomap.length()-3, 3) == ".bt") {
                        //Read the binary OcTree (full Octree converted to the maximum likelihood estimate and pruned for maximum compression)
                        m_p_octree_map = (std::shared_ptr<octomap::OcTree>) new octomap::OcTree(world_octomap);

                    } else if (world_octomap.substr(world_octomap.length()-3, 3) == ".ot") {
                        // Read the full Octree (serialized)
                        octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(world_octomap);
                        m_p_octree_map = (std::shared_ptr<octomap::OcTree>) dynamic_cast<octomap::OcTree *>(tree);
                    } else {

                        ROS_WARN("Octomap should be given in .bt or .ot format.");
                        correctOctomapFormat = false;
                        ROS_WARN("Map not loaded. Using empty map in actions generation.");
                        m_p_octree_map = (std::shared_ptr<octomap::OcTree>) new octomap::OcTree(MAP_RESOLUTION);
                    }


                    double x_min, y_min, z_min, x_max, y_max, z_max;
                    m_p_octree_map->getMetricMin(x_min, y_min, z_min);
                    m_p_octree_map->getMetricMax(x_max, y_max, z_max);
                    m_sensor_height = z_min + (z_max-z_min)/2; // set the sensor plane z = m_sensor_height

                    if (local_nh.getParam("ompl/map_boundary", map_boundary)) {

                        ROS_WARN_STREAM("MAP BOUNDARY FOR PLANNING SET TO [" << map_boundary[0] << "," <<
                                                                map_boundary[1] << "," <<
                                                                map_boundary[2] << "," <<
                                                                map_boundary[3] << "]");

                    } else if (correctOctomapFormat) {
                        ROS_WARN("Whole octomap will be used in planning. ");

                        ROS_WARN_STREAM("BX: [" << x_min << ", "<< x_max << "]");
                        ROS_WARN_STREAM("BY: [" << y_min << ", "<< y_max << "]");
                        ROS_WARN_STREAM("BZ: [" << z_min << ", "<< z_max << "]");
                        map_boundary[0] = x_min; map_boundary[1] = x_max;
                        map_boundary[2] = y_min; map_boundary[3] = y_max;
                    } else {
                        ROS_WARN("NO MAP BOUNDARY!");
                        map_boundary[0] = -INFINITY;
                        map_boundary[1] = INFINITY;
                        map_boundary[2] = -INFINITY;
                        map_boundary[3] = INFINITY;
                    }

                } else {
                    ROS_WARN("Map not specified. Using empty map in actions generation.");
                    m_p_octree_map = (std::shared_ptr<octomap::OcTree>) new octomap::OcTree(MAP_RESOLUTION);
                    m_sensor_height = 0;

                    if (local_nh.getParam("ompl/map_boundary", map_boundary)) {

                        ROS_WARN_STREAM("MAP BOUNDARY FOR PLANNING SET TO [" << map_boundary[0] << "," <<
                                                                             map_boundary[1] << "," <<
                                                                             map_boundary[2] << "," <<
                                                                             map_boundary[3] << "]");
                    } else {
                        ROS_WARN("NO MAP BOUNDARY!");
                        map_boundary[0] = -INFINITY;
                        map_boundary[1] = INFINITY;
                        map_boundary[2] = -INFINITY;
                        map_boundary[3] = INFINITY;
                    }
                }

                // set the bounds for the R^2 part of SE(2)
                m_map_boundary->setLow(0, map_boundary[0]);
                m_map_boundary->setHigh(0, map_boundary[1]);
                m_map_boundary->setLow(1, map_boundary[2]);
                m_map_boundary->setHigh(1, map_boundary[3]);


                generateActions_service_ = nh.advertiseService("generate_actions", &ActionGenerator::generateActionsOMPL, this);

            } else {
                // TODO motion primitives
                ROS_INFO("Motion primitives method");
            }
        }


		__TERMINATE_ALL_THREADS = false;
		serviceBusy = false;



        // parameters of prm and kds algorithm
        local_nh.param("ompl/min_vertices", m_min_vertices, 20);
        local_nh.param("ompl/max_planning_time", m_max_planning_time, 5.0);
        local_nh.param("ompl/resolution", m_resolution, 0.01);
        local_nh.param("ompl/min_edge_length", m_min_edge_length, 0.5);
        local_nh.param("ompl/max_edge_length", m_max_edge_length, 8.0);
        local_nh.param("ompl/waypoint_density", m_waypointDensity, 1.0);

        local_nh.param("kds/num_candidate_paths", m_num_candidate_paths, 3);
        local_nh.param("kds/min_path_pairwise_distance", m_min_path_pairwise_distance, 1.0);
        local_nh.param("kds/max_path_length", m_max_path_length, 1000.0);
        int opt;
        std::string str_opt;
        local_nh.param<std::string>("kds/k_diverse_short_alg", str_opt, "e");
        m_k_diverse_short_alg = static_cast<KDS>(str_opt.c_str()[0]);
        local_nh.param<std::string>("kds/path_distance_measure_alg", str_opt, "l");
        m_path_distance_measure_alg = static_cast<PDM>(str_opt.c_str()[0]);
        local_nh.param("kds/m_radius_factor", m_radius_factor, 0.3);
        local_nh.param<int>("kds/avoid_method", opt, 0);
        m_avoid_method = static_cast<AM>(opt);


        initializeMarkers();

        ROS_INFO("Action generator listening for planning requests...");

    }

    virtual ~ActionGenerator() {

        if (m_map_boundary)
            delete m_map_boundary;

    }

    void Draw()
	{
		sel_pub.publish(initialpose);
		//path_curr_pub.publish( sel_path_marker );
		path_curr_pub.publish(path_selected);
	}

	void setNavGoal(const NavGoal::ConstPtr& input) {

		if (serviceBusy) {
			ROS_INFO("Adding goal of action %d", idx_path);
			path_selected.poses.push_back(*input);


			goal_marker.header.stamp = ros::Time::now();
			//goal_marker.id = idx_path;
			//goal_marker.type = visualization_msgs::Marker::ARROW;
			goal_marker.points.push_back(last_point_clicked.point);
			geometry_msgs::Point goalPoint;
			goalPoint.x = input->pose.position.x; goalPoint.y = input->pose.position.y; goalPoint.z = input->pose.position.z;
			goal_marker.points.push_back(goalPoint);


			control.actions.push_back(path_selected);
			markers.markers.push_back(goal_marker);
			//a_vis_pub.publish(markers);
			path_previous = path_selected;
			path_pub.publish(path_previous);
			idx_path++;

			// we are done with this non-myopic action
			// add last non-empty action to the set of actions
			std::cout << "Action of length L = " << path_selected.poses.size()-1 <<
					" added. Total # of actions |A| =" << control.actions.size() << std::endl;
			single_action_generated = true;
			path_selected.poses.clear();
			setPathStart();
			goal_marker.points.clear();
		}

	}

	void initializeMarkers() {
		sel_path_marker.points.clear();
		idx_pose = 0;
		sel_path_marker.header.frame_id = "world";
		sel_path_marker.header.stamp = ros::Time::now();
		//sel_path_marker.ns = "";
		sel_path_marker.id = 0;
		sel_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
		sel_path_marker.action = visualization_msgs::Marker::ADD;
		sel_path_marker.points.push_back(initialpoint);
		sel_path_marker.pose.orientation.w = 1;
		sel_path_marker.scale.x = 0.2;
		sel_path_marker.scale.y = 0.1;
		sel_path_marker.scale.z = 0.1;
		sel_path_marker.color.a = 1.0; // Don't forget to set the alpha!
		sel_path_marker.color.r = 1.0;
		sel_path_marker.color.g = 1.0;
		sel_path_marker.color.b = 0.0;


		goal_marker.points.clear();
		goal_marker.header.frame_id = "world";
		goal_marker.id = 1;
		goal_marker.type = visualization_msgs::Marker::ARROW;
		goal_marker.action = visualization_msgs::Marker::ADD;
		goal_marker.pose.orientation.w = 1;
		goal_marker.scale.x = 0.1;
		goal_marker.scale.y = 0.2;
		goal_marker.scale.z = 0.05;
		goal_marker.color.a = 1.0; // Don't forget to set the alpha!
		goal_marker.color.r = 1.0;
		goal_marker.color.g = 1.0;
		goal_marker.color.b = 0.0;


        m_vis_vertices.type = visualization_msgs::Marker::SPHERE_LIST;
        m_vis_vertices.action = visualization_msgs::Marker::ADD;
        m_vis_vertices.header.frame_id = "world";
        m_vis_vertices.pose.orientation.w = 1;
        m_vis_vertices.ns = "prm_vertices";

        visualization_msgs::Marker::_scale_type vertices_scale;
        vertices_scale.x = 0.2;
        vertices_scale.y = 0.2;
        vertices_scale.z = 0.2;
        m_vis_vertices.scale = vertices_scale;

        visualization_msgs::Marker::_color_type vertices_color;
        // points are red
        vertices_color.r = 1;
        vertices_color.g = 0;
        vertices_color.b = 0;
        vertices_color.a = 1.0;
        m_vis_vertices.color = vertices_color;

        m_vis_edges.type = visualization_msgs::Marker::LINE_LIST;
        m_vis_edges.action = visualization_msgs::Marker::ADD;
        m_vis_edges.header.frame_id = "world";
        m_vis_edges.pose.orientation.w = 1;
        m_vis_edges.ns = "prm_edges";

        visualization_msgs::Marker::_scale_type edges_scale;
        edges_scale.x = 0.05;
        edges_scale.y = 0.05;
        edges_scale.z = 0.05;
        m_vis_edges.scale = edges_scale;

        visualization_msgs::Marker::_color_type edges_color;
        // lines are olive green
        edges_color.r = 85  / 255.0;
        edges_color.g = 107 / 255.0;
        edges_color.b = 47  / 255.0;
        edges_color.a = 0.3;
        m_vis_edges.color = edges_color;

        m_vis_bounds.header.frame_id = "world";
        visualization_msgs::Marker::_scale_type bounds_scale;
        bounds_scale.x = 0.1;
        bounds_scale.y = 0.1;
        bounds_scale.z = 0.1;
        m_vis_bounds.scale = bounds_scale;

        visualization_msgs::Marker::_color_type bounds_color;
        // points are black
        bounds_color.r = 0;
        bounds_color.g = 0;
        bounds_color.b = 0;
        bounds_color.a = 1.0;
        m_vis_bounds.color = bounds_color;


        // TODO
        m_vis_start_goal.type = visualization_msgs::Marker::MESH_RESOURCE;
        m_vis_start_goal.action = visualization_msgs::Marker::ADD;
        m_vis_start_goal.header.frame_id = "world";
        m_vis_start_goal.pose.orientation.w = 1;
        m_vis_start_goal.pose.position.z = PATH_Z_OFFSET;
        m_vis_start_goal.ns = "start";
        m_vis_start_goal.mesh_resource = "package://action_generator_node/meshes/start.stl";


        visualization_msgs::Marker::_scale_type start_goal_scale;
        start_goal_scale.x = 2.5;
        start_goal_scale.y = 2.5;
        start_goal_scale.z = 2.0;
        m_vis_start_goal.scale = start_goal_scale;

        visualization_msgs::Marker::_color_type start_goal_color;
        // points are dark blue
        start_goal_color.r = 1.0;
        start_goal_color.g = 1.0;
        start_goal_color.b = 0;
        start_goal_color.a = 1.0;
        m_vis_start_goal.color = start_goal_color;



        m_vis_candidate_path.type = visualization_msgs::Marker::LINE_STRIP;
        m_vis_candidate_path.action = visualization_msgs::Marker::ADD;
        m_vis_candidate_path.header.frame_id = "world";
        m_vis_candidate_path.pose.orientation.w = 1;
        m_vis_candidate_path.ns = "candidate_paths";

        visualization_msgs::Marker::_scale_type candidate_paths_scale;
        candidate_paths_scale.x = 0.2; // line width
        //candidate_paths_scale.y = 0.2; // not used
        //candidate_paths_scale.z = 0.2; // not used
        m_vis_candidate_path.scale = candidate_paths_scale;

        visualization_msgs::Marker::_color_type cand_paths_color;
        // first candidate path is yellow
        cand_paths_color.r = 1.0;
        cand_paths_color.g = 1.0;
        cand_paths_color.b = 0.0;
        cand_paths_color.a = 0.3;
        m_vis_candidate_path.color = cand_paths_color;
        // color of all other cand. paths will be random
        srand (time(NULL));

    }

	void setIntermediatePoints(const geometry_msgs::PointStamped::ConstPtr& input) {

		ROS_INFO("Point (%5.2f, %5.2f, %5.2f) selected", input->point.x, input->point.y, input->point.z);
		geometry_msgs::PoseStamped inputPose;

		// intermediate points
		//path_selected.header = input->header;
		inputPose.header = input->header;
		inputPose.pose.position.x = input->point.x;
		inputPose.pose.position.y = input->point.y;
		inputPose.pose.position.z = input->point.z;

		if (distanceBetweenPoints(input->point, last_point_clicked.point) < std::min(VAR_X, VAR_Y)) {

			char answer;
			path_selected.poses.clear();
			std::cout << "Last action canceled.";
			//std::cout << "Last action canceled. Continue [y/n]: ";
			//std::scanf("%c", &answer); // problematic with multiple threads
			//if (answer == 'n') {
				// we are done
				single_action_generated = true;
				all_actions_generated = true;
				ROS_INFO("ENDING actions generation.");
			//} else { // back to start
			//	setPathStart();
			//}
			//sel_path_marker.points.pop_back();

		} else {
			//keep adding poses to the last action
			if (serviceBusy) {
				path_selected.poses.push_back(inputPose);
				ROS_INFO("Intermediate point %d added", path_selected.poses.size()-1);
			}
		}



		//sel_path_marker.header.stamp = ros::Time::now();
		//sel_path_marker.ns = "";
		//sel_path_marker.id = idx_path;
		//sel_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
		//sel_path_marker.action = visualization_msgs::Marker::ADD;
		//sel_path_marker.pose = inputPose.pose;
		// sel_path_marker.points.push_back(input->point); // add input point to markers
		//if (!markers.markers.empty()) markers.markers.pop_back();
		//markers.markers.push_back(sel_path_marker);
		//a_vis_pub.publish(markers);
		//path_curr_pub.publish(sel_path_marker);

		last_point_clicked.point = input->point;
		return;

	}

	void setPathStart() {
		// push the start pose to every action
		geometry_msgs::PoseStamped tempPoseStamped;
		tempPoseStamped.pose.position = initialpose.pose.pose.position;
		tempPoseStamped.pose.orientation = initialpose.pose.pose.orientation;
		tempPoseStamped.header = initialpose.header;
		path_selected.poses.push_back(tempPoseStamped);
		path_selected.header.frame_id = "world";
		path_selected.header.stamp = tempPoseStamped.header.stamp;

		// set last_point_clicked to starting pose of the robot, that is where all actions start from
		last_point_clicked.point.x = initialpose.pose.pose.position.x;
		last_point_clicked.point.y = initialpose.pose.pose.position.y;
		last_point_clicked.point.z = initialpose.pose.pose.position.z;
		initialpoint = last_point_clicked.point;
	}

    // TODO load motion primitives
	mrbsp_ros_msgs::Actions loadActionsFromFile(std::string filename) {
		return mrbsp_ros_msgs::Actions();
	}

    // TODO save manually generated actions as motion primitives
	bool saveActionsToFile(std::string filename) {
		//control.actions.at(0).poses.at(0).pose.position.x

		return true;
	}

protected:


    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds* m_map_boundary;

    // OMPL parameters
    int m_min_vertices;
    double m_max_planning_time;
    double m_resolution;
    double m_min_edge_length;
    double m_max_edge_length;
    double m_sensor_height;
    double m_waypointDensity;

    // parameters of the k-Diverse shortest path alg.
    int m_num_candidate_paths;
    double m_max_path_length;
    double m_min_path_pairwise_distance;
    KDS m_k_diverse_short_alg;
    PDM m_path_distance_measure_alg;
    double m_radius_factor;
    AM m_avoid_method;

    /// vector of all markers for clearing for next plan
    std::vector<visualization_msgs::Marker*> m_vis_marker_vector;

    /// marker for visualizing vertices of PRM
    visualization_msgs::Marker m_vis_vertices;

    /// marker for visualizing edges of PRM
    visualization_msgs::Marker m_vis_edges;

    /// marker for visualizing robot location and current goal
    visualization_msgs::Marker m_vis_start_goal;

    /// marker for visualizing bounds of the PRM
    visualization_msgs::Marker m_vis_bounds;

    /// marker for visualizing PRM vertex number
    visualization_msgs::Marker m_vis_vertices_text;

    /// marker for visualizing candidate path number
    visualization_msgs::Marker m_vis_text_path;

    /// marker for visualizing candidate path
    visualization_msgs::Marker m_vis_candidate_path;

    /// marker for clearing all visualizations
    visualization_msgs::Marker m_vis_clear_all;

    /// marker for clearing specific marker
    visualization_msgs::Marker m_vis_clear_marker;

    /// marker array to publish all markers.
    visualization_msgs::MarkerArray m_vis_array;

    /// publisher of array marker
    ros::Publisher      m_marker_array_pub;


    /// ID number to start counting the new markers
    unsigned int        m_id_start;


	bool generateActionsInteractive(mrbsp_ros_msgs::GenerateActions::Request& req, mrbsp_ros_msgs::GenerateActions::Response& res)
	{


		//boost::unique_lock<boost::mutex> lock(mutex);
		ROS_INFO("Service called. Busy %s", (serviceBusy?"YES":"NO"));
		//if (!serviceBusy) {

		// mark the robot for which actions are planned
		// publish to /initialpose topic
		initialpose.pose.pose = req.start;
		//initialpose.pose.pose.orientation.w = 1; // mitigate the bug in req not setting orientation correctly
		initialpose.pose.pose.position.z += 0.05; // 5 cm off the ground for better rendering
		initialpose.header.frame_id = "world";
		initialpose.pose.covariance.elems[0] = VAR_X;
		initialpose.pose.covariance.elems[7] = VAR_Y;
		initialpose.pose.covariance.elems[14] = 0;
		initialpose.header.stamp = ros::Time::now();


		// clear previous actions
		control.actions.clear();
		path_selected.poses.clear();
		setPathStart();
		initializeMarkers();
		idx_path = 0;
		all_actions_generated = false;
		single_action_generated = false;
		serviceBusy = true;

		std::cout << "----------------------------------------------------------------------------------------"
				<< std::endl;
		std::cout << "Actions set empty. To generate actions for the marked robot use this commands:\n\n" <<
				"\t'Publish point' RVIZ tool to select intermediate path points\n" <<
				"\tPress 'g' to set the navigation goal for a single action\n" <<
				"\tPress 'm' to move the camera\n" <<
				"\tPress 'i' to interact with the map\n" <<
				"\tTo END generating actions for this robot, press on 'Publish point' button, \n" <<
				"\tand then double-click anywhere in the map\n";
		std::cout << "----------------------------------------------------------------------------------------"
				<< std::endl;
		//}
		while (!all_actions_generated && !__TERMINATE_ALL_THREADS) { // double-click outside the workspace

			while (!single_action_generated && !__TERMINATE_ALL_THREADS) { // single-click outside the workspace

				//sel_pub.publish(initialpose);
				//sel_pub.publish(initialpose);
				Draw();

				ros::spinOnce();

				ros::Duration(0.1).sleep(); // sleep for 0.1 s
			}


			single_action_generated = false;
			initializeMarkers();
			ros::Duration(0.1).sleep(); // sleep for 0.1 s
		}

		/*
            if (!all_actions_generated) {
                if (single_action_generated) {
                    // add last non-empty action to the set of actions
                    std::cout << "Action added. Total # of actions |A| = " << control.actions.size() << std::endl;

                    single_action_generated = false;
                    initializeMarkers();
                }
                return false;
            } else {
                res.actions = control;
                ROS_INFO("ACTIONS generated");
                serviceBusy = false;
            }*/

		res.actions = control;
		serviceBusy = false;
		ROS_INFO("ACTIONS generated");
		std::cout << "Total # of actions |A| = " << control.actions.size() << std::endl;
		return true;
	}

    bool generateActionsOMPL(mrbsp_ros_msgs::GenerateActions::Request& req, mrbsp_ros_msgs::GenerateActions::Response& res)
    {

        // clear previous actions
        control.actions.clear();
        m_vis_array.markers.clear();

        // construct the state space we are planning in
        ob::StateSpacePtr space(new ob::SE2StateSpace());
        if (m_map_boundary)
            space->as<ob::SE2StateSpace>()->setBounds(*m_map_boundary);


        ob::ScopedState<ob::SE2StateSpace> start(space);
        start->setX(req.start.position.x);
        start->setY(req.start.position.y);
        start->setYaw(2*atan2(req.start.orientation.z, req.start.orientation.w));
        start.enforceBounds(); // otherwise error (state out of bounds for PI approx. with higher number of decimals)
        start.print();
        //start->setZ(req.start.position.z);
        //start->rotation().setAxisAngle(req.start.orientation.x, req.start.orientation.y, req.start.orientation.z, req.start.orientation.w); // Set the quaternion from axis-angle representation.
        //start->rotation().setIdentity(); // Set the state to identity – no rotation.
        //start.random();
        //start.print();
        m_vis_start_goal.header.stamp = ros::Time::now();
        m_vis_start_goal.ns = "start";
        m_vis_start_goal.mesh_resource = "package://action_generator_node/meshes/start.stl";
        m_vis_start_goal.pose.position.x = start->getX();
        m_vis_start_goal.pose.position.y = start->getY();
        m_vis_start_goal.pose.orientation = req.start.orientation;
        m_vis_array.markers.push_back(m_vis_start_goal);

        ROS_WARN("Ckecking start");
        if(NULL != m_p_octree_map) {
            octomap::point3d point3d(start->getX(), start->getY(), m_sensor_height);//se3State->getX(),se3State->getY(),se3State->getZ()
            octomap::OcTreeNode *octree_node = m_p_octree_map->search(point3d);
            if (NULL != octree_node) {
                if (m_p_octree_map->isNodeOccupied(octree_node)) {
                    ROS_WARN("Start in collision!");
                } else
                    ROS_WARN("Start OK.");
            }
        }

        std::cout << "isStateValid(START) = " << isStateValid((const ob::State *)&start) << std::endl;

        // create a goal state
        ob::ScopedState<ob::SE2StateSpace> goal(space);
        //goal->setX(27.0); // for testing
        //goal->setY(-8.0);
        goal->setX(req.goal.position.x);
        goal->setY(req.goal.position.y);
        goal->setYaw(2*atan2(req.goal.orientation.z, req.goal.orientation.w));
        //goal->setZ(req.goal.position.z);
        //goal->rotation().setAxisAngle(req.goal.orientation.x, req.goal.orientation.y, req.goal.orientation.z, req.goal.orientation.w); // Set the quaternion from axis-angle representation.
        goal.enforceBounds();
        goal.print();
        m_vis_start_goal.ns = "goal";
        m_vis_start_goal.mesh_resource = "package://action_generator_node/meshes/goal.stl";
        m_vis_start_goal.pose.position.x = goal->getX();
        m_vis_start_goal.pose.position.y = goal->getY();
        m_vis_start_goal.pose.orientation = req.goal.orientation;
        m_vis_array.markers.push_back(m_vis_start_goal);

        // multi-query planner
        //planWithSimpleSetup(space, start, goal);
        plan(space, start, goal, true);

        res.actions = control;
        return true;
    }


    bool isStateValid(const ob::State *state)
    {
        // cast the abstract state type to the type we expect
        const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO2StateSpace::StateType *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

        // check validity of state defined by pos & rot

        // NaN or Inf
        if (std::isnan(se2state->getX())  || std::isnan(se2state->getY()) ||
                std::isinf(se2state->getX())  || std::isinf(se2state->getY()))
            return false;


        if(NULL != m_p_octree_map) {
            //ROS_WARN_STREAM("Search " << se3state->getX() << ", " << se3state->getY() );
            octomap::point3d point3d(se2state->getX(),se2state->getY(), m_sensor_height); //z = height of the sensor plane
            octomap::OcTreeNode *octree_node = m_p_octree_map->search(point3d); // at full resolution, max. tree depth

            if (NULL != octree_node) {
                if (m_p_octree_map->isNodeOccupied(octree_node)) {

                    /*ROS_ERROR_STREAM("(" << point3d.x() << "," << point3d.y() << ", " << point3d.z() << ")" <<
                    " OCC " << octree_node->getOccupancy());*/
                    return false;
                }/* else {

                    ROS_WARN_STREAM("(" << point3d.x() << "," << point3d.y() << ", " << point3d.z() << ")" <<
                                         " FREE " << octree_node->getOccupancy());
                }*/
            }
        }

        return true;

        // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
        //return (const void*)rot != (const void*)pos;
    }


    og::PRM::ConnectionFilter prmFilterStrategy() {
        // @author Tal Regev
        // @author: Asaf Feniger

        og::PRM::ConnectionFilter connection_filter = [this](const og::PRM::Vertex &v1, const og::PRM::Vertex &v2) {
            //from: http://www.boost.org/doc/libs/1_38_0/libs/graph/doc/using_adjacency_list.html
            og::PRM::Graph g = planner->as<og::PRM>()->getRoadmap();
            boost::property_map<og::PRM::Graph, og::PRM::vertex_state_t>::type
                    states = boost::get(og::PRM::vertex_state_t(), g);
            ob::State* state1 = states[v1];
            ob::State* state2 = states[v2];

            double range = distanceBetween2DPoints(state1, state2);


            if (range <= m_max_edge_length && range >= m_min_edge_length) {
                return true;//!m_p_perception->isObstacleAheadFCL(waypoint1, waypoint2);
            }
            return false;
        };
        return connection_filter;
    }
/*
    bool PerceptionMapOctomap::isObstacleAhead(const Waypoint &waypoint_origin, const Waypoint &waypoint_end) const {
        ADD_FIRST_LEVEL_INFO_LOG(LogTag::map);

        if (NULL != m_p_octree_map) {
            octomap::point3d origin = Conversion<octomap::point3d>::as(waypoint_origin.translation());
            octomap::point3d end    = Conversion<octomap::point3d>::as(waypoint_end.translation());
            octomap::KeyRay key_ray;

            m_p_octree_map->computeRayKeys(origin, end, key_ray);
            for (octomap::OcTreeKey key : key_ray)
            {
                octomap::OcTreeNode *octree_node = m_p_octree_map->search(key);
                if (NULL != octree_node) {
                    if (m_p_octree_map->isNodeOccupied(octree_node)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    bool PerceptionMapOctomap::isObstacleAheadFCL(const Waypoint &waypoint_origin, const Waypoint &waypoint_end) const {
        ADD_FIRST_LEVEL_INFO_LOG(LogTag::map);

        //https://github.com/kuri-kustar/laser_collision_detection/blob/master/src/laser_obstacle_detect.cpp#L135-L228
        const static ros::Duration duration(0.01);

        fcl::Transform3f tf_orig = Conversion<fcl::Transform3f>::as(waypoint_origin);
        fcl::CollisionObject robot(m_robot_box, tf_orig);

        fcl::Transform3f tf_goal = Conversion<fcl::Transform3f>::as(waypoint_end);

        for (auto &&box : m_boxes) {
            fcl::ContinuousCollisionResult result;
            //http://ompl.kavrakilab.org/FCLMethodWrapper_8h_source.html
            //fcl::ContinuousCollisionRequest request(10, 0.0001, fcl::CCDM_SCREW, fcl::GST_LIBCCD, fcl::CCDC_CONSERVATIVE_ADVANCEMENT);
            fcl::ContinuousCollisionRequest request(15);
            fcl::Transform3f tf_box(box.getAABB().center());
            fcl::continuousCollide(&robot, tf_goal, &box, tf_box, request, result);
            if(result.is_collide) {
                if(DEBUG(3)) {
                    debugPrintRay(waypoint_origin, waypoint_end, true);
                    debugPrintRay(waypoint_origin, waypoint_end, false, true);
                    duration.sleep();
                    debugVisClear();
                }
                return true;
            }
        }
        if(DEBUG(3)) {
            debugPrintRay(waypoint_origin, waypoint_end);
            debugPrintRay(waypoint_origin, waypoint_end, false, true);
            duration.sleep();
            debugVisClear();
        }
        return isObstacleAhead(waypoint_origin, waypoint_end);
    }
*/



    void planWithSimpleSetup(ob::StateSpacePtr space, ob::ScopedState<ob::SE2StateSpace> start, ob::ScopedState<ob::SE2StateSpace> goal)
    {

        // define a simple setup class
        og::SimpleSetup ss(space);

        // set state validity checking for this space
        ss.setStateValidityChecker(boost::bind(&ActionGenerator::isStateValid, this, _1));


        // set the start and goal states
        ss.setStartAndGoalStates(start, goal);


        // change planner
//    cout << "==== change planner ====" << endl;
        ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
        ss.setPlanner(planner);

        // print params
//    cout << "==== change range ====" << endl;
        double range = 10;
        ompl::base::ParamSet& params = planner->params();
        if (params.hasParam(std::string("range"))) {
            params.setParam(std::string("range"), boost::lexical_cast<std::string>(range));
        }
//  cout << "==== print params ====" << endl;
        // this call is optional, but we put it in to get more output information
        ss.setup();
        ss.print();
        ss.getPlanner()->printSettings(std::cout);
        //exit(0);

        // attempt to find an exact solution within five seconds
        if (ss.solve(5.0) == ob::PlannerStatus::EXACT_SOLUTION)
        {
            og::PathGeometric slnPath = ss.getSolutionPath();

            std::cout << std::endl;
            std::cout << "Found solution with " << slnPath.getStateCount() << " states and length " << slnPath.length() << std::endl;
            // print the path to screen
            //slnPath.print(std::cout);


            ob::PlannerData data(ss.getSpaceInformation());
            ss.getPlannerData(data);
            drawRoadMap(data);


            /*
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

      */

            //std::cout << "Writing PlannerData to file './myPlannerData'" << std::endl;
            //ob::PlannerDataStorage dataStorage;
            //dataStorage.store(data, "myPlannerData");

            //slnPath.print(std::cout);
            slnPath.printAsMatrix(std::cout);
            //omplPathToGtsamGraph(slnPath);
        }
        else
            std::cout << "No solution found" << std::endl;
    }


    void plan(ob::StateSpacePtr space, ob::ScopedState<ob::SE2StateSpace> start, ob::ScopedState<ob::SE2StateSpace> goal, bool multiQueryPlanning = true) {

        //planner->clearQuery(); // TODO move planner initialization into constructor and clear query before replanning. This keeps the roadmap, and clears problem def.

        // construct an instance of  space information from this state space
        ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

        // set state validity checking for this space
        si->setStateValidityChecker(boost::bind(&ActionGenerator::isStateValid, this, _1));
        si->setStateValidityCheckingResolution(m_resolution);

        // check to see if the motion of the robot between two states is valid
        // motion is discretized to some resolution and states are checked for validity at that resolution
        // using StateValidator
        //si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
        // TODO define better motion validator that does not discretize paths

        si->setMotionValidator(std::make_shared<rayCastingMotionValidator>(si, si->getStateValidityChecker(), m_p_octree_map));


        // create a problem instance
        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

        // set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        // create a planner for the defined space
        //ob::PlannerPtr planner(new og::PRM(si));
        planner = ob::PlannerPtr(new og::PRM(si));
        //m_p_planner = planner;

        // set the problem we are trying to solve for the planner
        planner->setProblemDefinition(pdef);

        /* Set the function that can reject a milestone connection.
        The given function is called immediately before a connection is checked for collision
        and added to the roadmap. Other neighbors may have already been connected before this function is called.
        This allows certain heuristics that use the structure of the roadmap
        (like connected components or useful cycles) to be implemented by changing this function. */
        //og::PRM::ConnectionFilter connection_filter = prmFilterStrategy();
        //planner->as<og::PRM>()->setConnectionFilter(connection_filter);

        //Set the maximum length of a motion to be added to the roadmap.
        //planner->as<og::LazyPRM>()->setRange(2.0);

        // perform setup steps for the planner
        planner->setup();

        ROS_INFO_STREAM("Longest valid segment fraction = " << space->getLongestValidSegmentFraction());
        ROS_INFO_STREAM("Longest valid segment length = " << space->getLongestValidSegmentLength());

        // print the settings for this space
        si->printSettings(std::cout);

        // print the problem settings
        pdef->print(std::cout);


        // grow roadmap so that it containg at least N vertices
        ob::PlannerTerminationConditionFn roadmapTerminationConditionFn = [this]()
        {
            //ROS_WARN("Growing roadmap...");
            ob::PlannerData data(planner->getSpaceInformation());
            planner->getPlannerData(data);
            unsigned long int num_vertices = data.numVertices();
            return  num_vertices >= m_min_vertices;
        };
        // the roadmap is improved until a given condition is true
        planner->as<og::PRM>()->growRoadmap(roadmapTerminationConditionFn);

        // attempt to solve the problem within n seconds of planning time
        ob::PlannerStatus solved = planner->solve(m_max_planning_time);

        nav_msgs::Path candidatePath;
        geometry_msgs::PoseStamped waypoint;
        geometry_msgs::Point point;
        waypoint.header.stamp = ros::Time::now();
        waypoint.header.frame_id = "world";
        candidatePath.header.stamp = waypoint.header.stamp;
        candidatePath.header.frame_id = "world";

        m_vis_candidate_path.header = candidatePath.header;

        if (!multiQueryPlanning) {

            if (solved) {
                // get the goal representation from the problem definition (not the same as the goal state)
                // and inquire about the found path
                ob::PathPtr path = pdef->getSolutionPath();
                std::cout << "Found solution:" << std::endl;

                // print the path to screen
                path->print(std::cout);

                og::PathGeometric *pPathGeometric = path->as<og::PathGeometric>();
                std::size_t numberOfWayPts = pPathGeometric->getStateCount();
                double pathLength = pPathGeometric->length();
                // Insert a number of states in a path so that the path is made up of exactly given count of states.
                // States are inserted uniformly (more states on longer segments). Changes are performed only if a path has less than given count of states.
                pPathGeometric->interpolate(ceil(m_waypointDensity * pathLength));
                std::vector<ob::State *> SolutionStatesPtr = pPathGeometric->getStates();
                std::cout << "Interpolated path length " << SolutionStatesPtr.size() << std::endl;


                for (std::size_t i = 0; i < SolutionStatesPtr.size(); i++) {

                    const ob::State *state = SolutionStatesPtr.at(i);
                    const ob::SE2StateSpace::StateType *se2State = state->as<ob::SE2StateSpace::StateType>();

                    waypoint.pose.position.x = se2State->getX();
                    waypoint.pose.position.y = se2State->getY();
                    waypoint.pose.position.z = 0.0;
                    waypoint.pose.orientation.w = cos(se2State->getYaw() / 2);
                    waypoint.pose.orientation.x = 0.0;
                    waypoint.pose.orientation.y = 0.0;
                    waypoint.pose.orientation.z = sin(se2State->getYaw() / 2);

                    candidatePath.poses.push_back(waypoint);
                    // to visualize candidate path as a line strip
                    point.x = waypoint.pose.position.x;
                    point.y = waypoint.pose.position.y;
                    point.z = waypoint.pose.position.z;
                    m_vis_candidate_path.points.push_back(point);
                }


                control.actions.push_back(candidatePath);
                m_vis_candidate_path.id = 0;
                m_vis_array.markers.push_back(m_vis_candidate_path);

                candidatePath.poses.clear();
                m_vis_candidate_path.points.clear();

                ob::PlannerData data(si);
                planner->getPlannerData(data);
                drawRoadMap(data);


            } else
                std::cout << "No solution found" << std::endl;

        } else { // multi-query planning

            ob::PlannerData data(si);
            //planner->as<og::PRM>()->growRoadmap(conditionFn);
            //planner->as<og::PRM>()->growRoadmap(5.0);
            planner->getPlannerData(data);
            drawRoadMap(data);

            std::stringstream string_stream;
            data.printGraphML(string_stream);

            /* Diverse Short Paths, Caleb Voss see ICRA2015 paper
             *
             * Usage: diverse <graphml> <paths> <algorithm> [ -l <maxLength> | -d <minDistance> ] [-s]
            <graphml> a file in *.graphml format
            <paths> is the number of paths to find
            <algorithm> is of the form <name>:<pathDistance>[:<neighborhoodDistance>:<neighborhoodRadius>]
                <name> is 'eppstein' or 'randomavoidance'
                <pathDistance> is 'levenshtein', 'frechet', or 'hausdorff'
                <neighborhoodDistance> is 'cspace' or 'graph'; it is required by 'randomavoidance'
                                                                                                                                                                                                                                                                                <maxLength>,<minDistance> are floats specifying constraints on returned paths
            -s flag enables saving the path set in a file called paths.txt

            Example: diverse resources/grid2.graphml 10 e:l -d 5.7
             */

            //from: https://bitbucket.org/caleb_voss/diverse_short_paths
            //from: http://calebvoss.com/wp-content/uploads/voss-2015-diverse_short_paths.pdf
            TestData *testData = new TestData(string_stream, m_num_candidate_paths, m_max_path_length, m_min_path_pairwise_distance);
            //TestData *testData = new TestData(string_stream, 10, 1000, 1);
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
            // algorithm to find k diverse, short paths in a graph.
            KDiverseShort *kDiverseShort;
            switch(m_k_diverse_short_alg) {
                case EPPSTEIN:
                    kDiverseShort = new Eppstein(testData, distanceMeasure);
                    break;
                case VOSS:
                    kDiverseShort = new Voss(testData, distanceMeasure, m_radius_factor, avoidMethod);
            }

            kDiverseShort->timedRun();
            kDiverseShort->print();
            // Write all paths to the file "~/.ros/paths.txt" in OMPL App format. Interpolate paths with extra 4 points on each line segment. Goal point is not saved!
            //kDiverseShort->saveSet();


            Path* pSolutionPaths = kDiverseShort->getSolutionPaths();
            if (pSolutionPaths) {

                ompl::base::State *inner_state = pSolutionPaths->getSpaceInformationPtr()->allocState();
                std::vector<double> reals;

                for (std::size_t i = 0; i < kDiverseShort->numPathsFound(); i++) {
                    std::ofstream ofs (std::to_string(i) + "._interpolated_path.txt", std::ofstream::out);
                    //pSolutionPaths[i].saveOMPLFormat(ofs);
                    const std::vector<const ompl::base::State *> states = pSolutionPaths[i].getStates();
                    for (std::size_t j = 1; j < states.size(); j++) {

                        const ompl::base::State * prev_state = states.at(j-1);
                        const ompl::base::State * state = states.at(j);


                        /*const ob::SE2StateSpace::StateType *se2State = state->as<ob::SE2StateSpace::StateType>();
                        waypoint.pose.position.x = se2State->getX();
                        waypoint.pose.position.y = se2State->getY();
                        waypoint.pose.position.z = 0.0;
                        waypoint.pose.orientation.w = cos(se2State->getYaw() / 2);
                        waypoint.pose.orientation.x = 0.0;
                        waypoint.pose.orientation.y = 0.0;
                        waypoint.pose.orientation.z = sin(se2State->getYaw() / 2);*/


                        // Interpolate line segments on the path to contain poses in which observations will be taken.
                        // This should match informative condition in DA in ideal case when the path is followed exactly or at least very accurately.
                        // This is the first part of the topology prediction model, second part predicts loop closings and multi-robot pose contraints
                        // and is given in the planner node.

                        double pathSegmentLength = distanceBetween2DPoints(prev_state, state);
                        double delta_t = 1/(m_waypointDensity * pathSegmentLength);
                        for (double t = 0; t < 1; t += delta_t) {
                            pSolutionPaths[i].getSpaceInformationPtr()->getStateSpace()->interpolate(prev_state, state, t, inner_state);
                            pSolutionPaths[i].getSpaceInformationPtr()->getStateSpace()->copyToReals(reals, inner_state);

                            addWaypointToCandPath(waypoint, reals, candidatePath);

                            // write waypoint to a file
                            for (std::size_t k = 0; k < reals.size(); k++)
                            {
                                if (k > 0)
                                    ofs << " ";
                                ofs << reals[k];
                            }
                            ofs << "\n";

                        }
                    }

                    // add last state of the path
                    pSolutionPaths[i].getSpaceInformationPtr()->getStateSpace()->copyToReals(reals, states.at(states.size()-1));
                    addWaypointToCandPath(waypoint, reals, candidatePath);
                    // write endpoint to a file
                    for (std::size_t k = 0; k < reals.size(); k++)
                    {
                        if (k > 0)
                            ofs << " ";
                        ofs << reals[k];
                    }
                    ofs.close();

                    control.actions.push_back(candidatePath);
                    candidatePath.poses.clear();

                    m_vis_candidate_path.id = i;
                    m_vis_array.markers.push_back(m_vis_candidate_path);

                    // new path
                    m_vis_candidate_path.points.clear();
                    m_vis_candidate_path.color.r = (float)rand()/RAND_MAX;
                    m_vis_candidate_path.color.g = (float)rand()/RAND_MAX;
                    m_vis_candidate_path.color.b = (float)rand()/RAND_MAX;

                }

                pSolutionPaths->getSpaceInformationPtr()->freeState(inner_state);


            } else std::cout << "No solution found" << std::endl;

            delete kDiverseShort;
            delete distanceMeasure;
            delete testData;

        }
        // visualize markers
        m_marker_array_pub.publish(m_vis_array);
        m_vis_array.markers.clear();
    }

    void drawRoadMap(ob::PlannerData& data) {

        /*std::ofstream verticesFile;
        verticesFile.open("/home/andrej/Desktop/vertices.txt");
        ROS_ERROR("Opened file vertices.txt");*/

        for (unsigned int v = 0; v < data.numVertices(); ++v, ++m_id_start) {
            ob::PlannerDataVertex vertex = data.getVertex(v);
            const ob::State* state = vertex.getState();
            const ob::SE3StateSpace::StateType* se3State = state->as<ob::SE3StateSpace::StateType>();
            geometry_msgs::Point point_v;
            point_v.x = se3State->getX();
            point_v.y = se3State->getY();
            point_v.z = PATH_Z_OFFSET; // little bit off the ground for drawing a 2D motion plan in RVIZ X-Y plane

            //verticesFile << point_v << std::endl;

            std::vector<unsigned int> edgeList;
            data.getEdges(v, edgeList);
            geometry_msgs::Point point_e;
            for (unsigned int v_edge : edgeList) {
                point_e.x = data.getVertex(v_edge).getState()->as<ob::SE3StateSpace::StateType>()->getX();
                point_e.y = data.getVertex(v_edge).getState()->as<ob::SE3StateSpace::StateType>()->getY();
                point_e.z = PATH_Z_OFFSET;

                m_vis_edges.points.push_back(point_v);
                m_vis_edges.points.push_back(point_e);
            }
            m_vis_vertices.points.push_back(point_v);
            //std::cout << "Vertex: " << point_v << std::endl;

            //setMarkerTime(m_vis_vertices_text);
            /*m_vis_vertices_text.id = m_id_start;
            m_vis_vertices_text.pose.position = point_v;
            m_vis_vertices_text.pose.position.z = 1;
            m_vis_vertices_text.text = std::to_string(v);
            m_vis_array.markers.push_back(m_vis_vertices_text);*/
        }

        m_vis_vertices.header.stamp = ros::Time::now();
        m_vis_array.markers.push_back(m_vis_vertices);
        m_vis_array.markers.push_back(m_vis_edges);

        m_marker_array_pub.publish(m_vis_array);
        m_vis_array.markers.clear();

        //verticesFile.close();

    }

    // initialize waypoint from real array and add it to a path and RVIZ marker
    void addWaypointToCandPath(geometry_msgs::PoseStamped& wp, std::vector<double>& reals, nav_msgs::Path& path) {

        geometry_msgs::Point point;

        wp.pose.position.x = reals[0];
        wp.pose.position.y = reals[1];
        wp.pose.position.z = 0.0;
        wp.pose.orientation.w = cos(reals[2] / 2);
        wp.pose.orientation.x = 0.0;
        wp.pose.orientation.y = 0.0;
        wp.pose.orientation.z = sin(reals[2] / 2);

        path.poses.push_back(wp);

        // to visualize candidate path as a line strip
        point.x = wp.pose.position.x;
        point.y = wp.pose.position.y;
        point.z = wp.pose.position.z;
        m_vis_candidate_path.points.push_back(point);
    }
};


#endif //ACTION_GENERATOR_NODE_ACTION_GENERATOR_H
