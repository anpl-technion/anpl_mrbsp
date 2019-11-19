/*
 * PathDistanceMeasure.h
 */

#ifndef __PATH_DISTANCE_MEASURE_H
#define __PATH_DISTANCE_MEASURE_H

#include "pch.h"

class PathDistanceMeasure
{
public:
    
    /** Destructor. */
    virtual ~PathDistanceMeasure ();
    
    /** Type of function that computes path distance. */
    typedef double FunctionType(const Path &, const Path &);
    
    /**
     * Get the name of this measure.
     * @return string naming this class's distance measure
     */
    virtual std::string getName () = 0;
    
    /**
     * Compute the distance between two paths.
     * @param p1    first path
     * @param p2    second path
     * @return distance between \a p1 and \p p2 according to some measure
     * @note Implementation may assume \a p1 and \a p2 are in the same graph.
     */
    virtual double distance (const Path &p1, const Path &p2) = 0;
};

#endif
