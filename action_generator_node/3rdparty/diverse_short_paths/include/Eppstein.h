/*
 * Eppstein.h
 */

#ifndef __EPPSTEIN_H
#define __EPPSTEIN_H

#include "pch.h"

#include "KDiverseShort.h"

/**
 * Maximum number of paths Graehl implementation is prepared to handle.
 */
#define MAXPATHS 20000000

/**
 * Eppstein's algorithm for k shortest paths.
 */
class Eppstein : public KDiverseShort
{
private:
    
    graehl::Graehl *graehl_kernel;  // Encapsulation of Graehl's implementation
    
public:
    
    /**
     * Construct an instance of this algorithm.
     * @param data  data set to run on
     * @param pDist path distance measure to use
     */
    Eppstein (const TestData *data, PathDistanceMeasure *pDist);
    
    /** Destructor. */
    ~Eppstein ();
    
private:
    
    /**
     * Convert a Graph to the Graehl graph format.
     * @param out   stream to write graph to
     * @param g     Graph to convert
     * @return \a out
     */
    static std::stringstream &makeGraehlGraph (std::stringstream &out, const Graph &g);
    
    /**
     * Set up the Graehl code for the graph.
     * @param input stream to read graph from
     * @param start vertex number to start at
     * @param end   vertex number to end at
     * @return new instance of Graehl for this Graph
     */
    static graehl::Graehl *initGraehl (std::stringstream &input, const std::size_t start, const std::size_t end);
    
    /**
     * Execute the algorithm and write a string to \a description.
     */
    void run ();
};

#endif
