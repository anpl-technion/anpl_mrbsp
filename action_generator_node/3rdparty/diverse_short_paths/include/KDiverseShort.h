/*
 * KDiverseShort.h
 */

#ifndef __K_DIVERSE_SHORT_H
#define __K_DIVERSE_SHORT_H

#include "pch.h"

#include "Path.h"

/**
 * Abstract algorithm to find k diverse, short paths in a graph.
 */
class KDiverseShort
{
private:
    
    bool too_long;              // Whether the last considered path was too long
    Path *pathArray;            // Storage for result paths
    std::size_t i;              // Next index to store a path
    std::size_t c;              // Total paths looked at
    std::string pDistName;      // Name of the path distance measure
    double seconds;             // Execution time of the algorithm
    
protected:
    
    /** Nearest-neighbor data structure of paths found. */
    ompl::NearestNeighbors<Path> *pathNN;
    
    /** Data to run the algorithm on. */
    const TestData *testData;
    
    /** Textual description of the algorithm. */
    std::stringstream description;
    
public:
    
    /**
     * Construct the algorithm to run on the given data set.
     * @param data  data to run on
     * @param pDist path distance measure to use
     */
    KDiverseShort (const TestData *data, PathDistanceMeasure *pDist);
    
    /** Destructor. */
    virtual ~KDiverseShort ();
    
    /** Time a run of the algorithm. */
    void timedRun ();
    
    /**
     * Clear internal data structures to prepare for future use.
     * @note Overriding implementations should call \c KDiverseShort::clear() in addition
     *  to performing their own clean up.
     */
    virtual void clear ();
    
    /** Write all paths to the same file in OMPL App format. */
    void saveSet () const;
    
    /** Write all paths to the same file in Gephi format. */
    void saveNodes () const;
    
    /** Print a summary of these results. */
    void print () const;

    /** Get a pointer to paths */
    inline Path* getSolutionPaths() {
        return pathArray;
    };

    inline std::size_t numPathsFound () {
        return i;
    }

    
    /**
     * Get the length of the shortest of our paths.
     * @return length of shortest path in our \a paths set
     * @warning Assumes this is actually just the first path due to underlying algorithms.
     */
    double findShortestLength () const;
    
    /**
     * Get the length of the longest of our paths.
     * @return length of longest path in our \a paths set
     */
    double findLongestLength () const;
    
    /**
     * Get the minimum distance between any pair of paths.
     * @return min of distances between every pair of distance
     *  paths in our \a paths set
     */
    double minNearestPathDistance () const;
    
    /**
     * Get the average distance between any pair of paths.
     * @return mean of distances between every path and its nearest
     *  neighboring path in our \a paths set
     */
    double meanNearestPathDistance () const;
    
    /**
     * Get the distance to the nearest neighbor of some path.
     * @param which index of path to find nearest neighbor for
     * @return distance to nearest neighbor of path \a which
     * @warning Does not check bounds on \a which.
     */
    double nearestPathDistance (const std::size_t which) const;
    
protected:
    
    /**
     * Was the last considered path too long?
     * @return true if \c considerPath() rejected path because it was too long; false otherwise
     */
    bool tooLong () const;
    
    /**
     * Do we still need more paths?
     * @return true if we have fewer paths than requested by \a testData; false otherwise
     */
    bool needMore () const;
    
    /**
     * Evaluate a path against the set of paths found so far, saving it if it meets the criteria.
     * @param path  path to evaluate
     * @return true if \a path is saved; false otherwise
     * @throw plannerTerminationException when \a stop returns true
     */
    bool considerPath(const Path &path);
    
    /**
     * Execute the algorithm and write a string to \a description.
     */
    virtual void run () = 0;
};

#endif
