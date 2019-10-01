/*
 * Voss.cpp
 */

#include "Voss.h"

#include "Graph.h"
#include "Neighborhood.h"
#include "Path.h"
#include "TestData.h"

// Constructors, destructors

Voss::Voss (const TestData *data, PathDistanceMeasure *pDist, double radiusFactor, Neighborhood::AvoidMethod avoid)
: KDiverseShort(data, pDist), radiusFactor(radiusFactor), avoidance(avoid)
{
    if (radiusFactor < 0)
    {
        std::cerr << "Please specify a positive neighborhood radius.\n";
        std::exit(-1);
    }
}

// Private methods

void Voss::run ()
{
    description << "Random Avoidance (in " << avoidString() << "), radius factor " << radiusFactor;
    
    // Set up neighborhoods
    const Graph &g = testData->getGraph();
    Neighborhood::constructSharedResources(avoidance, &g);
    
    if (!needMore())
        return;
    
    std::vector<std::vector<Neighborhood> > resultAvoids;   // Avoided neighborhoods each path in resultsPaths was made with
    std::set<Path> unfilteredPathSet;                       // Unfiltered set of paths found
    std::size_t frontier = 0;                               // The next path to analyze
    
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<Neighborhood> alreadyAvoiding;
    Path referencePath = getShortestPathUnderAvoidance(alreadyAvoiding);
    if (referencePath.empty())
        return;
    considerPath(referencePath);
    if (tooLong())
        return;
    resultAvoids.push_back(alreadyAvoiding);
    unfilteredPathSet.insert(referencePath);
    
    // Work through the queue until we have enough
    while (unfilteredPathSet.size() && !tooLong() && needMore())
    {
        referencePath = *unfilteredPathSet.begin();
        unfilteredPathSet.erase(unfilteredPathSet.begin());
        alreadyAvoiding = resultAvoids[frontier];
        frontier++;
        
        // Make attempts at imposing a new neighborhood to avoid on the graph
        double radius = radiusFactor * referencePath.getLength();
        for (std::size_t i = 0; i < 2 && needMore(); i++)
        {
            std::vector<Neighborhood> avoid = alreadyAvoiding;
            ompl::base::State *sample = g.getSpaceInfo()->allocState();
            Edge sampledEdge = referencePath.sampleUniform(sample, radius);
            avoid.push_back(Neighborhood(sample, sampledEdge, radius));
            
            // Get the shortest path under these constraints
            Path path = getShortestPathUnderAvoidance(avoid);
            if (path.empty())
                continue;
            
            // Can we keep it?
            considerPath(path);
            
            // But we'll need it regardless for later iterations
            unfilteredPathSet.insert(path);
            resultAvoids.push_back(avoid);
        }
    }
    
    Neighborhood::destroySharedResources();
    
    return;
}

Path Voss::getShortestPathUnderAvoidance (const std::vector<Neighborhood> &avoid) const
{
    // Run the A* search
    const Graph &g = testData->getGraph();
    const Vertex end = testData->getEnd();
    Vertex *pred = new Vertex[g.getNumVertices()];
    std::vector<Vertex> pathtmp;
    Path path((Graph *)&g);
    try
    {
        boost::astar_search(g, testData->getStart(), heuristic(g, end),
                            boost::weight_map(edgeWeightMap(g, avoid)).
                            predecessor_map(pred).
                            visitor(visitor(end)));
    }
    catch (foundGoalException e)
    {
        // Trace back the shortest path
        Vertex v;
        for (v = end; v != pred[v]; v = pred[v])
            pathtmp.push_back(v);
        if (v != end)
            pathtmp.push_back(v);
        
        BOOST_REVERSE_FOREACH(Vertex v, pathtmp)
        {
            path.push_back(v);
        }
    }
    
    delete [] pred;
    
    return path;
}

std::string Voss::avoidString () const
{
    switch (avoidance)
    {
    case Neighborhood::CSPACE:
        return "C-space";
    case Neighborhood::GRAPH:
        return "graph distance";
    default:
        return "UNKNOWN";
    }
}
