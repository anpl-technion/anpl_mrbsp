/*
 * Path.h
 */

#ifndef __PATH_H
#define __PATH_H

#include "pch.h"

#include "_graph_detail.h"
#include "PathDistanceMeasure.h"

/**
 * A sequence of adjacent edges, represented by the vertices in between.
 */
class Path : public std::vector<Vertex>
{
private:
    
    static std::function<PathDistanceMeasure::FunctionType> distanceFunc;   // Function to compute path distance
    
    const Graph *g;                                         // Graph the path lies in
    mutable std::vector<const ompl::base::State *> states;  // State of each vertex
    mutable std::vector<double> weights;                    // Weight of each edge
    std::vector<double> parametrization;                    // Partial sums of edge weights
    
public:
    
    /** Default constructor. */
    Path ();
    
    /**
     * Copy constructor.
     * @param path  existing path to copy
     */
    Path (const Path &path);
    
    /**
     * Construct path from vertex list.
     * @param path  list of vertices in \a g in order
     * @param g     graph this path lies in
     */
    Path (std::vector<Vertex> &path, const Graph *g);
    
    /**
     * Construct an empty path.
     * @param g graph this path lies in
     */
    Path (const Graph *g);
    
    /**
     * Set the function to compute path distance.
     * @param func  function accepting two paths and returning a double as their distance
     */
    static void setDistanceFunction (std::function<PathDistanceMeasure::FunctionType> func);
    
    /**
     * Compute the distance between two paths.
     * @param p1    first path
     * @param p2    second path
     * @return distance between \a p1 and \a p2 according to some measure
     * @warning Terminates program if \a p1 and \a p2 are in different graphs.
     */
    static double distance (const Path &p1, const Path &p2);
    
    /**
     * Compare a path with this one.
     * @param rhs   another path
     * @return true (false) if size of \a *this is < (>) size of \a rhs; otherwise,
     *  u < v where u,v are the first vertices in \a *this, \a rhs which do not match
     */
    bool operator< (const Path &rhs) const;
    
    /**
     * Write this path in format used by OMPL App.
     * @param out   stream to write to
     */
    void saveOMPLFormat(std::ostream &out) const;

    ompl::base::SpaceInformationPtr getSpaceInformationPtr();
    
    /**
     * Write this path in format used by our osm2gephi.py script.
     * @param out   stream to write to
     */
    void saveGephiFormat(std::ostream &out) const;
    
    /** Print this path as a list of vertices and total length. */
    void print () const;
    
    /**
     * Get the length of this path (sum of edge weights).
     * @return path length
     */
    double getLength () const;
    
    /**
     * Get the graph this path lies in.
     * @return our graph \a g
     */
    const Graph *getGraph () const;
    
    /**
     * Append a vertex.
     * @param v vertex in our graph \a g
     */
    void push_back (const Vertex &v);
    
    /**
     * Uniformly sample a state along the path avoiding some margin.
     * @param sample[out]   allocated state to put sample in
     * @param r             margin at the start and end of the path to avoid
     * @return edge sampled state lies in
     */
    Edge sampleUniform (ompl::base::State *sample, const double r) const;
    
    /**
     * Get the weight of each edge, in order.
     * @return our \a weights vector
     */
    const std::vector<double> &getWeights () const;
    
    /**
     * Get the state of each vertex, in order.
     * @return out \a states vector
     */
    const std::vector<const ompl::base::State *> &getStates () const;
    
    /**
     * Get the partial sums of the edge weights, in order.
     * @return our \a parameterization vector
     */
    std::vector<double> getPartialEdgeSums () const;
    
private:
    
    /**
     * Have the weights been stored?
     * @return true if \a weights has been initialized; false otherwise
     */
    bool isWeightCached () const;
    
    /** Populate \a weights for random access. */
    void cacheWeights () const;
    
    /**
     * Have the states been stored?
     * @return true if \a states has been initialized; false otherwise
     */
    bool isStateCached () const;
    
    /** Populate \a weights for random access. */
    void cacheStates () const;
};

#endif
