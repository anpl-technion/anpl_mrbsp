/*
 * Frechet.h
 */

#ifndef __FRECHET_H
#define __FRECHET_H

#include "pch.h"

#include "PathDistanceMeasure.h"

/**
 * Container for the discrete Frechet distance algorithm.
 */
class Frechet : public PathDistanceMeasure
{
public:
    
    /**
     * Get the name of this measure.
     * @return string naming this class's distance measure
     */
    std::string getName();
    
    /**
     * Compute the distance between two paths according to the discrete Frechet algorithm.
     * @param path1 first path
     * @param path2 second path
     * @return distance between \a path1 and \a path2
     * @note Does not check if paths are in the same graph.
     */
    double distance (const Path &path1, const Path &path2);
};

#endif
