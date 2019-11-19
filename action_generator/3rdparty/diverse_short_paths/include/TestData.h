/*
 * TestData.h
 */

#ifndef __TEST_DATA_H
#define __TEST_DATA_H

#include "pch.h"

#include "_graph_detail.h"
#include "Path.h"

/**
 * Data and parameters to test an algorithm on.
 */
class TestData
{
private:
    
    Vertex start;                           // Vertex to start search from
    Vertex end;                             // Vertex to end search at
    const std::size_t k;                    // Number of paths to try to find
    const double maxLength;                 // Maximum allowable length of a returned path
    const double minDistance;               // Minimum allowable distance between any two returned paths
    const Graph *graph;                     // Graph to search in
    
public:

    TestData (const std::stringstream &string_stream, std::size_t numPaths,
                    double maxPathLength, double minPathPairwiseDistance);
    
    /**
     * Construct data set.
     * @param graphFileName             name of the *.graphml file containing the graph
     * @param numPaths                  number of paths to try to find
     * @param maxPathLength             maximum length of any returned path
     * @param minPathPairwiseDistance   minimum distance between any two returned paths
     */
    TestData (const std::string &graphFileName, std::size_t numPaths,
        double maxPathLength, double minPathPairwiseDistance);
    
    /** Destructor. */
    ~TestData ();
    
    /**
     * Get the start node.
     * @return our \a start vertex
     */
    Vertex getStart () const;
    
    /**
     * Get the end node.
     * @return our \a end vertex
     */
    Vertex getEnd () const;
    
    /**
     * Get the number of paths to find.
     * @return our \a k
     */
    std::size_t getK () const;
    
    /**
     * Get the maximum allowable path length.
     * @return our \a maxLength
     */
    double getMaxLength () const;
    
    /**
     * Get the minimum allowable path closeness.
     * @return our \a minDistance
     */
    double getMinDistance () const;
    
    /**
     * Get the graph.
     * @return our \a graph
     */
    const Graph &getGraph () const;
};

#endif
