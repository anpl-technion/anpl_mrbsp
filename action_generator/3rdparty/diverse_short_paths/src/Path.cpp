/*
 * Path.cpp
 */

#include "Path.h"

#include "Graph.h"
#include "Neighborhood.h"

#define TOOMANYTRIES 1

// Constructors, destructors

Path::Path ()
:  std::vector<Vertex>(), g(nullptr)
{
}

Path::Path (const Path &path)
:  std::vector<Vertex>(path), g(path.getGraph()), parametrization(path.getPartialEdgeSums())
{
}

Path::Path (std::vector<Vertex> &path, const Graph *graph)
:  std::vector<Vertex>(path), g(graph)
{
    parametrization.reserve(size());
    parametrization.push_back(0);
    for (std::size_t i = 1; i < size(); i++)
    {
        parametrization.push_back(parametrization[i-1] + g->getEdgeWeight((*this)[i-1], (*this)[i]));
    }
}

Path::Path (const Graph *graph)
:  std::vector<Vertex>(), g(graph)
{
}

// Static methods

void Path::setDistanceFunction (std::function<PathDistanceMeasure::FunctionType> func)
{
    distanceFunc = func;
}

double Path::distance (const Path &p1, const Path &p2)
{
    if (p1.getGraph() == nullptr || p2.getGraph() == nullptr)
    {
        std::cerr << "Error: Cannot measure distance with a blank path!\n";
        std::exit(-1);
    }
    else if (p1.getGraph() != p2.getGraph())
    {
        std::cerr << "Error: Cannot measure distance between paths in different graphs!\n";
        std::exit(-1);
    }
    
    if (distanceFunc == nullptr)
    {
        std::cerr << "Error: Path distance function not set!\n";
        std::exit(-1);
    }
    
    return distanceFunc(p1, p2);
}

// Public methods

bool Path::operator< (const Path &rhs) const
{
    // Compare sizes first
    std::size_t s1 = size();
    std::size_t s2 = rhs.size();
    if (s1 != s2)
        return s1 < s2;
    
    // Find first mismatched vertex
    for (std::size_t i = 0; i < s1; i++)
    {
        if ((*this)[i] != rhs[i])
            return (*this)[i] < rhs[i];
    }
    
    // They are equal
    return false;
}

void Path::saveOMPLFormat(std::ostream &out) const
{
    ompl::base::State *state = g->getSpaceInfo()->allocState();
    for (std::size_t i = 1; i < size(); i++)
    {
        for (double interp = 0; interp < 1; interp += 0.2)
        {
            const ompl::base::State *s1 = g->getVertexState((*this)[i-1]);
            const ompl::base::State *s2 = g->getVertexState((*this)[i]);
            g->getSpaceInfo()->getStateSpace()->interpolate(s1, s2, interp, state);
            std::vector<double> reals;
            g->getSpaceInfo()->getStateSpace()->copyToReals(reals, state);
            for (std::size_t j = 0; j < reals.size(); j++)
            {
                if (j > 0)
                    out << " ";
                out << reals[j];
            }
            out << "\n";
        }
    }
    g->getSpaceInfo()->freeState(state);
}

ompl::base::SpaceInformationPtr Path::getSpaceInformationPtr() {
    ompl::base::SpaceInformationPtr si = g->getSpaceInfo();

    return si;
}

void Path::saveGephiFormat(std::ostream &out) const
{
    for (std::size_t i = 0; i < size(); i++)
    {
        out << g->getVertexID((*this)[i]) << " ";
    }
}

void Path::print () const
{
    for (std::size_t i = 0; i < size(); i++)
    {
        std::cout << (*this)[i] << " ";
    }
    std::cout << ": " << (empty() ? 0 : getLength()) << "\n";
}

double Path::getLength () const
{
    return parametrization[size()-1];
}

const Graph *Path::getGraph () const
{
    return g;
}

void Path::push_back (const Vertex &v)
{
    std::vector<Vertex>::push_back(v);
    if (size() == 1)
        parametrization.push_back(0);
    else
        parametrization.push_back(parametrization[size()-2]
            + g->getEdgeWeight((*this)[size()-2], (*this)[size()-1]));
}

Edge Path::sampleUniform (ompl::base::State *sample, const double r) const
{
    // Reject if too close to goal
    const Vertex start = front();
    const Vertex goal = back();
    const ompl::base::State *startState = g->getVertexState(start);
    const ompl::base::State *goalState = g->getVertexState(goal);
    std::size_t i;
    double par;
    std::size_t count = 0;
    do
    {
        // Sample between [r,length] to stay away from start
        par = r + (getLength() - r) * ((double)rand() / (double)RAND_MAX);
        
        // Find vertices to interpolate between
        i = 0;
        while (!(parametrization[i] <= par && par <= parametrization[i+1]))
            i++;
        
        par = (par-parametrization[i]) / getLength();
        
        // Interpolate
        g->getSpaceInfo()->getStateSpace()->interpolate(g->getVertexState((*this)[i]), g->getVertexState((*this)[i+1]), par, sample);
    }
    while (count++ < TOOMANYTRIES &&
        (Neighborhood::distance(sample, (*this)[i], (*this)[i+1], par, goalState, goal, goal, 0) < r ||
         Neighborhood::distance(sample, (*this)[i], (*this)[i+1], par, startState, start, start, 0) < r));
    
    return g->getEdge((*this)[i], (*this)[i+1]);
}

const std::vector<double> &Path::getWeights () const
{
    if (!isWeightCached())
        cacheWeights();
    
    return weights;
}

std::vector<double> Path::getPartialEdgeSums () const
{
    return parametrization;
}

// Private methods

bool Path::isWeightCached () const
{
    return weights.size() > 0;
}

void Path::cacheWeights () const
{
    Vertex prev = (*this)[0];
    BOOST_FOREACH(Vertex v, *this)
    {
        if (prev == v)
            continue;
        weights.push_back(g->getEdgeWeight(prev, v));
        prev = v;
    }
}

const std::vector<const ompl::base::State *> &Path::getStates () const
{
    if (!isStateCached())
        cacheStates();
    
    return states;
}

bool Path::isStateCached () const
{
    return states.size() > 0;
}

void Path::cacheStates () const
{
    BOOST_FOREACH(Vertex v, *this)
    {
        states.push_back(g->getVertexState(v));
    }
}

// Static members

std::function<PathDistanceMeasure::FunctionType> Path::distanceFunc = nullptr;
