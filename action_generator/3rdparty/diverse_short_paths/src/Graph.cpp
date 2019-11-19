/*
 * Graph.cpp
 */

#include "Graph.h"

#include "Path.h"
#include "Neighborhood.h"

// Constructors, destructors

Graph::Graph (const ompl::base::SpaceInformationPtr &si, std::istream &graphml)
: boost_graph(), si(si), apsp(nullptr)
{
    // Read in vertex coordinates and edge weights from graphml format
    std::map<Vertex, std::string> coordStrings;
    std::map<Edge, double> weights;
    boost::associative_property_map<std::map<Vertex, std::string> > coords_map(coordStrings);
    boost::associative_property_map<std::map<Vertex, std::string> > id_map(ids);
    boost::associative_property_map<std::map<Edge, double> > weight_map(weights);
    boost::dynamic_properties dyn_prop;
    dyn_prop.property("coords", coords_map);
    dyn_prop.property("id", id_map);
    dyn_prop.property("weight", weight_map);
    boost::read_graphml(graphml, *this, dyn_prop);
    
    // Assign weights
    foreachEdge([&] (Edge e) -> void
    {
        boost::put(boost::edge_weight, *this, e, weight_map[e]);
    });
    
    // Populate state information
    foreachVertex([&] (Vertex v) -> void
    {
        ompl::base::State *state = si->allocState();
        std::istringstream coords(coordStrings[v]);
        std::string coord;
        std::vector<double> reals;
        while (!coords.eof())
        {
            std::getline(coords, coord, ',');
            reals.push_back(std::atof(coord.c_str()));
        }
        si->getStateSpace()->copyFromReals(state, reals);
        si->getStateSpace()->enforceBounds(state);
        boost::put(boost::vertex_prop, *this, v, state);
    });
    
    std::cout << "Num vertices: " << getNumVertices() << "\nNum edges: " << boost::num_edges(*this) << "\n";
    double deg = 0;
    foreachVertex([&] (Vertex v) -> void
    {
        deg += boost::out_degree(v, *this);
    });
    std::cout << "Avg degree: " << deg/getNumVertices() << "\n";
}

Graph::~Graph ()
{
    // Free state of each vertex
    foreachVertex([&] (Vertex v) -> void
    {
        boost::get(boost::vertex_prop, *this, v).freeState(si);
    });
}

// Public methods

const ompl::base::SpaceInformationPtr Graph::getSpaceInfo () const
{
    return si;
}

std::size_t Graph::getNumVertices () const
{
    return boost::num_vertices(*this);
}

Edge Graph::getEdge (Vertex u, Vertex v) const
{
    // No guarantee this edge exists
    return boost::edge(u, v, *this).first;
}

double Graph::getEdgeWeight (Edge e) const
{
    return boost::get(boost::edge_weight, *this, e);
}

double Graph::getEdgeWeight (Vertex u, Vertex v) const
{
    if (u == v)
        return 0;
    std::pair<Edge, bool> res = boost::edge(u, v, *this);
    if (res.second)
        return getEdgeWeight(res.first);
    return std::numeric_limits<double>::infinity();
}

const ompl::base::State *Graph::getVertexState (Vertex v) const
{
    return boost::get(boost::vertex_prop, *this, v).getState();
}

std::tuple<Vertex,Vertex> Graph::getVertices (Edge e) const
{
    return std::make_tuple(boost::source(e, *this), boost::target(e, *this));
}

std::string Graph::getVertexID (Vertex v) const
{
    if (ids[v] == "")
    {
        std::stringstream id;
        id << v;
        return id.str();
    }
    
    return ids[v];
}

void Graph::foreachEdge (std::function<void (Edge)> applyMe) const
{
    BOOST_FOREACH (const Edge e, boost::edges(*this))
    {
        applyMe(e);
    }
}

void Graph::foreachVertex (std::function<void (Vertex)> applyMe) const
{
    BOOST_FOREACH (const Vertex v, boost::vertices(*this))
    {
        applyMe(v);
    }
}

void Graph::midpoint (const ompl::base::State *s1, const ompl::base::State *s2, ompl::base::State *mid) const
{
    si->getStateSpace()->interpolate(s1, s2, 0.5, mid);
}

void Graph::allPairsShortestPaths () const
{
    // Allocate a 2D array for the distances
    if (apsp == nullptr)
    {
        const std::size_t n = getNumVertices();
        apsp = new double*[n];
        for (std::size_t i = 0; i < n; i++)
            apsp[i] = new double[n];
    }
    
    // Run the Floyd-Warshall algorithm
    boost::floyd_warshall_all_pairs_shortest_paths(*this, apsp);
}

double Graph::graphDistance (Vertex u, Vertex v) const
{
    // Pre-compute all graph distances if we haven't already
    if (apsp == nullptr)
        allPairsShortestPaths();
    
    return apsp[u][v];
}
