/*
 * Voss.h
 */

#ifndef __VOSS_H
#define __VOSS_H

#include "pch.h"

#include "KDiverseShort.h"
#include "Neighborhood.h"

/**
 * My algorithm for k short, diverse paths by random avoidance.
 */
class Voss : public KDiverseShort
{
private:
    
    const double radiusFactor;                  // Size of avoided regions as a factor of shortest path length
    const Neighborhood::AvoidMethod avoidance;  // Method used to measure distance to center of neighborhood
    
public:
    
    /**
     * Construct an instance of this algorithm.
     * @param data          data set to run on
     * @param pDist         path distance measure to use
     * @param radiusFactor  radius of avoided regions as a factor of shortest path length
     * @param avoid         method used to measure distance to center of neighborhood
     * @warning Terminates program if \a radiusFactor is non-positive.
     */
    Voss (const TestData *data, PathDistanceMeasure *pDist, double radiusFactor, Neighborhood::AvoidMethod avoid);
    
private:
    
    /**
     * Execute the algorithm and write a string to \a description.
     */
    void run ();
    
    /**
     * Compute the shortest path under avoidance constraints.
     * @param avoidThese    set of regions paths are now forbidden from entering
     * @return shortest path through the graph that does not enter a forbidden region,
     *  or an empty Path object if no such path exists
     */
    Path getShortestPathUnderAvoidance (const std::vector<Neighborhood> &avoidThese) const;
    
    /**
     * Return the name of the avoid method used.
     * @return string name our \a avoidance setting
     */
    std::string avoidString () const;
};

#endif
