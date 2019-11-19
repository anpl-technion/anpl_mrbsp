/*
 * Eppstein.cpp
 */

#include "Eppstein.h"

#include "graehl/kbest.h"

#include "Graph.h"
#include "Path.h"
#include "TestData.h"

// Constructors, destructors

Eppstein::Eppstein (const TestData *data, PathDistanceMeasure *pDist)
: KDiverseShort(data, pDist)
{
    graehl_kernel = nullptr;
    clear();
}

Eppstein::~Eppstein ()
{
    delete graehl_kernel;
}

// Private static methods

std::stringstream &Eppstein::makeGraehlGraph (std::stringstream &out, const Graph &g)
{
    // Write the number of vertices
    out << g.getNumVertices();
    
    // Write each edge with distance
    g.foreachEdge([&] (const Edge e) -> void
    {
        Vertex u, v;
        std::tie(u, v) = g.getVertices(e);
        out << "(" << u << " " << v << " " << g.getEdgeWeight(e) << ")";
    });
    
    return out;
}

graehl::Graehl *Eppstein::initGraehl (std::stringstream &input, const std::size_t start, const std::size_t end)
{
    // Graehl was written for ints, so make sure our numbers aren't too big
    assert(start < INT_MAX);
    assert(end < INT_MAX);
    graehl::Graehl *g = new graehl::Graehl(input, (int)start, (int)end);
    return g;
}

// Private methods

void Eppstein::run ()
{
    std::stringstream ss;
    graehl::Graehl * graehl_kernel = initGraehl(makeGraehlGraph(ss, testData->getGraph()), testData->getStart(), testData->getEnd());
    
    static std::size_t c = 0;
    description << "Eppstein's Shortest Paths";
    
    // Until we have enough paths, or they get too long...
    while (!tooLong() && needMore())
    {
        // There is a hard-coded limit on the number of paths we can evaluate
        if (++c > MAXPATHS)
        {
            std::cerr << "Error: Need to increase MAXPATHS!\n";
            delete graehl_kernel;
            return;
        }
        
        // Get the next path from Graehl
        graehl::List<graehl::GraphArc *> *raw = graehl_kernel->getNextPath();
        if (raw == nullptr)
            break;
        
        // Convert it to our format
        Path path((Graph *)&testData->getGraph());
        graehl::GraphArc *edge = nullptr;
        while (raw->notEmpty())
        {
            edge = raw->top();
            path.push_back(edge->source);
            raw->pop();
        }
        path.push_back(edge->dest);
        
        // Can we keep it?
        considerPath(path);
    }
    
    delete graehl_kernel;
    return;
}
