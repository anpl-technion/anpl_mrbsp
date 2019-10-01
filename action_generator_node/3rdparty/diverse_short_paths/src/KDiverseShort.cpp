/*
 * KDiverseShort.cpp
 */

#include "KDiverseShort.h"

#include "Graph.h"
#include "Path.h"
#include "TestData.h"

// Constructors, destructors

KDiverseShort::KDiverseShort (const TestData *data, PathDistanceMeasure *pDist)
: too_long(false), i(0), c(0), testData(data)
{
    // Set up path storage and nearest neighbors
    Path::setDistanceFunction(boost::bind(&PathDistanceMeasure::distance, pDist, _1, _2));
    pDistName = pDist->getName();
    pathArray = new Path[testData->getK()];
    pathNN = new ompl::NearestNeighborsGNAT<Path>();
    pathNN->setDistanceFunction(&Path::distance);
    seconds = 0;
}

KDiverseShort::~KDiverseShort ()
{
    delete [] pathArray;
    delete pathNN;
}

// Public methods

void KDiverseShort::timedRun ()
{
    // Time the execution
    clock_t start = clock();
    run();
    clock_t end = clock();
    seconds = ((double)(end-start))/CLOCKS_PER_SEC;
    
    // Label the results
    const double d = testData->getMinDistance();
    if (d <= std::numeric_limits<double>::epsilon())
    {
        const double l = testData->getMaxLength();
        if (l == std::numeric_limits<double>::infinity())
            description << ", no filtering";
        else
            description << ", filtering for maximum length " << l;
    }
    else
        description << ", filtering for pairwise distance " << d;
    description << " (" << pDistName << ")";
}

void KDiverseShort::clear ()
{
    // Prepare for future runs
    too_long = false;
    i = 0;
    c = 0;
    pathNN->clear();
    description.str("");
}

void KDiverseShort::saveSet () const
{
    // Open a file and save each path in turn
    std::ofstream fout("paths.txt");
    for (std::size_t j = 0; j < i; j++)
    {
        pathArray[j].saveOMPLFormat(fout);
    }
}

void KDiverseShort::saveNodes () const
{
    // Open a file and save each path in turn
    std::ofstream fout("paths.txt");
    for (std::size_t j = 0; j < i; j++)
    {
        pathArray[j].saveGephiFormat(fout);
        fout << "\n";
    }
}

void KDiverseShort::print () const
{
    std::cout << "Description: " << description.str() << "\n";
    std::cout << " Found " << i << " of " << testData->getK() << " requested paths.\n";
    const double shortest = findShortestLength();
    std::cout << "\tshortest path length: " << shortest << "\n";
    const double longest = findLongestLength();
    std::cout << "\tlongest path length:  " << longest << " (" << longest/shortest << " times as long)\n";
    std::cout << "\tmin distance to nearest neighbor:  " << minNearestPathDistance() << "\n";
    std::cout << "\tmean distance to nearest neighbor: " << meanNearestPathDistance() << "\n";
    std::cout << " Completed in " << seconds << " seconds\n\n\n";
}
double KDiverseShort::findShortestLength () const
{
    // Assume it's the first one
    if (i > 0)
        return pathArray[0].getLength();
    else
        return std::numeric_limits<double>::quiet_NaN();
}

double KDiverseShort::findLongestLength () const
{
    double longest = 0;
    for (std::size_t j = 0; j < i; j++)
    {
        if (pathArray[j].getLength() > longest)
            longest = pathArray[j].getLength();
    }
    return longest;
}

double KDiverseShort::minNearestPathDistance () const
{
    double min = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < i; j++)
    {
        double d = nearestPathDistance(j);
        if (d < min)
            min = d;
    }
    return min;
}

double KDiverseShort::meanNearestPathDistance () const
{
    double sum = 0;
    for (std::size_t j = 0; j < i; j++)
    {
        sum += nearestPathDistance(j);
    }
    return sum/i;
}

double KDiverseShort::nearestPathDistance (const std::size_t which) const
{
    std::vector<Path> ret(2);
    pathNN->nearestK(pathArray[which], 2, ret);
    if (ret[1].getGraph() == nullptr)
        return std::numeric_limits<double>::infinity();
    return Path::distance(pathArray[which], ret[1]);
}

// Protected methods

bool KDiverseShort::tooLong () const
{
    return too_long;
}

bool KDiverseShort::needMore () const
{
    return i < testData->getK();
}

bool KDiverseShort::considerPath(const Path &path)
{
    // Update user on progress during long runs
    if (++c % 10000 == 0)
        std::cout << "Success rate: " << i << "/" << c << "\n";
    
    // Reject path if it is too long
    if (path.getLength() > testData->getMaxLength())
    {
        too_long = true;
        return false;
    }
    
    // Reject path if it is too close to others
    if (i > 0)
    {
        const Path &nearest = pathNN->nearest(path);
        if (Path::distance(path, nearest) < testData->getMinDistance())
            return false;
    }
    
    // Path meets criteria
    pathArray[i++] = path;
    pathNN->add(pathArray[i-1]);
    return true;
}
