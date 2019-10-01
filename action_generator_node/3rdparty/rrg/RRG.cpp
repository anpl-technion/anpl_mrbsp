/* Authors: Luis G. Torres */

#include "RRG.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/Goal.h"

#include <boost/utility.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <map>
#include <boost/timer/timer.hpp>

ompl::geometric::RRG::RRG(const base::SpaceInformationPtr &si) :
    base::Planner(si, "RRG")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    maxDistance_ = 0.0;
    ballRadiusMax_ = 0.0;
    ballRadiusConst_ = 0.0;
    numCollisionChecks_ = 0;

    Planner::declareParam<double>("range", this, &RRG::setRange, &RRG::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("ball_radius_constant", this, &RRG::setBallRadiusConstant, &RRG::getBallRadiusConstant);
    Planner::declareParam<double>("max_ball_radius", this, &RRG::setMaxBallRadius, &RRG::getMaxBallRadius);    
}

ompl::geometric::RRG::~RRG(void)
{
    freeMemory();
}

void ompl::geometric::RRG::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (ballRadiusMax_ == 0.0)
        ballRadiusMax_ = maxDistance_ * sqrt((double)si_->getStateSpace()->getDimension());
    if (ballRadiusConst_ == 0.0)
        ballRadiusConst_ = si_->getMaximumExtent();

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<DistItem>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&RRG::distanceFunction, this, _1, _2));


    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_->hasOptimizationObjective())
        opt_ = pdef_->getOptimizationObjective();
    else
    {
        OMPL_INFORM("Defaulting to optimizing path length.");
        opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
}

void ompl::geometric::RRG::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    numCollisionChecks_ = 0;
}

void ompl::geometric::RRG::preprocess(unsigned numIterations)
{
    this->initPreprocess();

    base::State *state = si_->allocState();
    boost::timer::cpu_timer timer;
    for (unsigned i = 0; i < numIterations; ++i)
    {
        sampler_->sampleUniform(state);

        Node newNode, nearNode;
        if (this->extend(state, &newNode, &nearNode))
        {
            this->connectNear(newNode, nearNode);

            DistItem newDistItem;
            newDistItem.state = graph_[newNode];
            newDistItem.node = newNode;
            nn_->add(newDistItem);
        }
    }
    OMPL_INFORM("RRG preprocess took %s seconds.", timer.format(6,"%w").c_str());
    si_->freeState(state);

    predMap_.resize(boost::num_vertices(graph_));
    vertexCostMap_.resize(boost::num_vertices(graph_));
}

ompl::base::PlannerStatus ompl::geometric::RRG::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    // Initialize start node
    Node startNode = boost::graph_traits<Graph>::null_vertex();
    while (const base::State *st = pis_.nextStart())
    {
        // given the start state, get the vertex in the roadmap
        // closest to the start state: this will be our start node
        //
        // TODO: make this less horrible
        DistItem queryItem;
        queryItem.state = st;
        queryItem.node = boost::graph_traits<Graph>::null_vertex();
        DistItem nearDistItem = nn_->nearest(queryItem);
        startNode = nearDistItem.node;
    }
    if (startNode == boost::graph_traits<Graph>::null_vertex())
    {
        OMPL_ERROR("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    // Initialize goal
    base::GoalPtr goal = pdef_->getGoal();
    if (!goal)
    {
        OMPL_ERROR("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Find path and add it to problem definition
    //
    // TODO: figure out how to deal with approximate solutions and
    // such
    boost::timer::cpu_timer timer;
    base::PathPtr path = getBestPath(startNode, goal);
    timer.stop();

    pdef_->addSolutionPath(path);
    return base::PlannerStatus(true, false);
}

void ompl::geometric::RRG::initPreprocess(void)
{
    while (const base::State *st = pis_.nextStart())
    {
        Node n = boost::add_vertex(graph_);
        graph_[n] = si_->allocState();
        si_->copyState(graph_[n], st);
        DistItem d;
        d.state = graph_[n];
        d.node = n;
        nn_->add(d);
    }

    // TODO: throw an exception which solve() catches so that solve()
    // can return the proper PlannerStatus
    if (nn_->size() == 0)
    {
        OMPL_ERROR("There are no valid initial states!");
        // return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // OMPL_INFORM("Starting with %u states", boost::num_vertices(graph_));
}

bool ompl::geometric::RRG::extend(const base::State *s, Node* newNode, Node* nearNode)
{
    // Find closest state in graph
    DistItem queryItem;
    queryItem.state = s;

    DistItem nearDistItem = nn_->nearest(queryItem);
    *nearNode = nearDistItem.node;
    const base::State *nearState = nearDistItem.state;

    // If distance between nearest state and sampled state is greater
    // than maxDistance_, find state maxDistance_ away from near in
    // direction of sampled
    base::State* newState = 0;
    double dist = si_->distance(nearState, s);
    if (dist > maxDistance_)
    {
        newState = si_->allocState();
        si_->getStateSpace()->interpolate(nearState, s, maxDistance_ / dist, newState);
    }

    // If motion between near and new state is collision-free, add new
    // state and edge to graph. We don't add this new state to the
    // near-neighbor structure until after we call connectNear() in
    // preprocess().
    ++numCollisionChecks_;
    bool collisionFree = si_->checkMotion(nearState, s);
    if (collisionFree)
    {
        *newNode = boost::add_vertex(graph_);
        if (!newState)
        {
            newState = si_->allocState();
            si_->copyState(newState, s);
        }
        graph_[*newNode] = newState;

        Edge newEdge;
        bool success;
        boost::tie(newEdge, success) = boost::add_edge(*nearNode, *newNode, graph_);
        graph_[newEdge] = opt_->motionCost(nearState, newState);

        // If in a fully symmetric space, we go and add the reverse edge too
        if (si_->getStateSpace()->hasSymmetricDistance() &&
            si_->getStateSpace()->hasSymmetricInterpolate() &&
            opt_->isSymmetric())
        {
            Edge revEdge;
            boost::tie(revEdge, success) = boost::add_edge(*newNode, *nearNode, graph_);
            graph_[revEdge] = graph_[newEdge];
        }
    }
    else if (newState)
        si_->freeState(newState);

    return collisionFree;
}

// We make connections within a certain radius from other states to
// the new state, and then vice versa.
void ompl::geometric::RRG::connectNear(Node node, Node nearNode)
{
    bool symDist = si_->getStateSpace()->hasSymmetricDistance();
    bool symInterp = si_->getStateSpace()->hasSymmetricInterpolate();
    bool symCost = opt_->isSymmetric();

    // If this state space doesn't have a symmetric distance function,
    // we have to set the distance function to be FROM other states TO
    // the new state
    if (!symDist)
        nn_->setDistanceFunction(boost::bind(&RRG::distanceFunction, this, _1, _2));

    // Get nearest neighbors (FROM other states TO this state;
    // important distinction for asymmetric distance metric)
    //
    // TODO: make neighbors vector static so that we're not
    // constructing/destructing so much crap all the time
    std::vector<DistItem> neighbors;
    DistItem queryItem;
    queryItem.state = graph_[node];
    queryItem.node = node;
    double neighborRadius = this->getNeighborhoodRadius();
    nn_->nearestR(queryItem, neighborRadius, neighbors);

    // Try to connect to every node (except the one node this new node
    // is already connected to, from the extension step)
    for (std::vector<DistItem>::const_iterator nbhr = neighbors.begin();
         nbhr != neighbors.end();
         ++nbhr)
    {
        if (nbhr->node != nearNode)
        {
            ++numCollisionChecks_;
            if (si_->checkMotion(graph_[nbhr->node], graph_[node]))
            {
                Edge newEdge;
                bool success;
                boost::tie(newEdge, success) = boost::add_edge(nbhr->node, node, graph_);
                graph_[newEdge] = opt_->motionCost(graph_[nbhr->node], graph_[node]);

                // Add reverse edge if this space is completely
                // symmetric (effectively create an undirected graph)
                if (symDist && symInterp && symCost)
                {
                    base::Cost cost = graph_[newEdge];
                    boost::tie(newEdge, success) = boost::add_edge(node, nbhr->node, graph_);
                    graph_[newEdge] = cost;
                }
            }
        }
    }

    // If our distance function is asymmetric, we have to run another
    // near neighbors query going FROM our new node TO every other
    // node
    if (!symDist)
    {
        nn_->setDistanceFunction(boost::bind(&RRG::distanceFunction, this, _2, _1));
        nn_->nearestR(queryItem, neighborRadius, neighbors);
    }

    // We have to iterate back over our neighbors if:
    //
    // a) our distance metric is asymmetric (different set of
    // neighbors)
    //
    // b) interpolation between states is asymmetric [(s1,s2) might be
    // valid but (s2,s1) might not]
    //
    // c) cost function between states is asymmetric
    if (!symDist || !symInterp || !symCost)
    {
        for (std::vector<DistItem>::const_iterator nbhr = neighbors.begin();
             nbhr != neighbors.end();
             ++nbhr)
        {
            // If distance or interpolation are asymmetric, we need to
            // do another collision check from the new node to the
            // neighbor. If the above are symmetric and there's no
            // edge already between them, it means there's a collision
            // so this edge is invalid.
            //
            // TODO: see if it's more efficient to cache the collision
            // check queries, instead of checking for edge existence.
            bool validConnection = true;
            Edge reverseEdge;
            bool reverseEdgeExists = false;
            if (!symDist || !symInterp)
            {
                ++numCollisionChecks_;
                validConnection = si_->checkMotion(graph_[node], graph_[nbhr->node]);
            }
            else
            {
                boost::tie(reverseEdge, reverseEdgeExists) = boost::edge(nbhr->node,node,graph_);
                if (!reverseEdgeExists)
                    validConnection = false;
            }

            if (validConnection)
            {
                Edge newEdge;
                bool success;
                boost::tie(newEdge, success) = boost::add_edge(node, nbhr->node, graph_);

                // If our cost function is symmetric and we have an
                // edge in the reverse direction, we just copy over
                // that cost. If not, we directly compute the cost.
                if (symCost && reverseEdgeExists)
                    graph_[newEdge] = graph_[reverseEdge];
                else
                    graph_[newEdge] = opt_->motionCost(graph_[node], graph_[nbhr->node]);    
            }
        } // end for each neighbor
    } // end if asymmetric
}

// We iterate through each vertex and free their states, and then
// through each edge and free their costs, then clear the graph
// structure
void ompl::geometric::RRG::freeMemory(void)
{
    // Free state at each vertex
    boost::graph_traits<Graph>::vertex_iterator vi, vLast;
    for (boost::tie(vi,vLast) = boost::vertices(graph_);
         vi != vLast;
         ++vi)
    {
        si_->freeState(graph_[*vi]);
    }

    // clear graph
    graph_.clear();
}

ompl::base::PathPtr
ompl::geometric::RRG::getBestPath(Node startNode, const base::GoalPtr& goal)
{
    // Initialized structures used for Astar
    double nearestGoalDist = std::numeric_limits<double>::infinity();
    Node nearestToGoal = boost::graph_traits<Graph>::null_vertex();
    astar_goal_visitor astarVisitor(boost::dynamic_pointer_cast<base::GoalRegion>(goal).get(),
                                    &nearestGoalDist,
                                    &nearestToGoal);

    Node goalNode = boost::graph_traits<Graph>::null_vertex();
    try
    {
        boost::astar_search(graph_, startNode,
                            boost::bind(&geometric::RRG::costToGo, this, _1, goal.get()),
                            weight_map(boost::get(boost::edge_bundle, graph_)).
                            predecessor_map(&(predMap_[0])).
                            distance_map(&(vertexCostMap_[0])).
                            distance_compare(boost::bind(&base::OptimizationObjective::
                                                         isCostBetterThan, opt_.get(), _1, _2)).
                            distance_combine(boost::bind(&base::OptimizationObjective::
                                                         combineCosts, opt_.get(), _1, _2)).
                            distance_inf(opt_->infiniteCost()).
                            distance_zero(opt_->identityCost()).
                            visitor(astarVisitor));
    }
    catch (found_goal& e)
    {
        goalNode = e.goalNode;
    }

    // If no node was found that actually satisfied the goal, we'll
    // just use the node closest to the goal
    //
    // BTW this is gonna be super specific to heuristic = distanceGoal
    // so sorry mother
    if (goalNode == boost::graph_traits<Graph>::null_vertex())
        goalNode = *(astarVisitor.nearestToGoal);

    // Populate path to goal. Simply populate a vector backwards
    // using the predecessor map and the minimum-cost goal node,
    // then populate the actual path structure that we'll return.
    std::vector<base::State*> rPath;
    Node currentNode = boost::graph_traits<Graph>::null_vertex();
    Node nextNode = goalNode;
    do
    {
        currentNode = nextNode;
        rPath.push_back(graph_[currentNode]);
        nextNode = predMap_[currentNode];
    } while (currentNode != nextNode);
    PathGeometric *geomPath = new PathGeometric(si_);
    for (std::vector<base::State*>::const_reverse_iterator i = rPath.rbegin();
         i != rPath.rend();
         ++i)
    {
        geomPath->append(*i);
    }

    return base::PathPtr(geomPath);
}

ompl::base::Cost ompl::geometric::RRG::costToGo(Node v, const base::Goal* goal) const
{
    return opt_->costToGo(graph_[v], goal);
}

void ompl::geometric::RRG::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Add vertices and edges simultaneously
    boost::graph_traits<Graph>::edge_iterator ei, eLast;
    for (boost::tie(ei,eLast) = boost::edges(graph_);
         ei != eLast;
         ++ei)
    {
        Node src = boost::source(*ei, graph_);
        Node trg = boost::target(*ei, graph_);
        data.addEdge(base::PlannerDataVertex(graph_[src]),
                     base::PlannerDataVertex(graph_[trg]));
    }
}
