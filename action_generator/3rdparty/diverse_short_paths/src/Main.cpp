/*
 * Main.cpp
 */


#include "pch.h"

#include "Eppstein.h"
#include "Frechet.h"
#include "Hausdorff.h"
#include "Levenshtein.h"
#include "Neighborhood.h"
#include "TestData.h"
#include "Voss.h"

/**
 * Print a usage message and exit.
 * @warning Terminates the program.
 */
void usage ()
{
    std::cerr << "Usage: diverse <graphml> <paths> <algorithm> [ -l <maxLength> | -d <minDistance> ] [-s]\n";
    std::cerr << "    <graphml> a file in *.graphml format\n";
    std::cerr << "    <paths> is the number of paths to find\n";
    std::cerr << "    <algorithm> is of the form <name>:<pathDistance>[:<neighborhoodDistance>:<neighborhoodRadius>]\n";
    std::cerr << "        <name> is 'eppstein' or 'randomavoidance'\n";
    std::cerr << "        <pathDistance> is 'levenshtein', 'frechet', or 'hausdorff'\n";
    std::cerr << "        <neighborhoodDistance> is 'cspace' or 'graph'; it is required by 'randomavoidance'\n";
    std::cerr << "        <neighborhoodRadius> is a float in (0,1); it is required by 'randomavoidance'\n";
    std::cerr << "      * algorithm specifications may be abbreviated using e,r,l,f,h,c,g\n";
    std::cerr << "    <maxLength>,<minDistance> are floats specifying constraints on returned paths\n";
    std::cerr << "    -s flag enables saving the path set in a file called paths.txt\n\n";
    std::cerr << "Example: diverse resources/grid2.graphml 10 e:l -d 5.7\n";
    std::exit(-1);
}
/*
 * Run tests on k diverse short path algorithms.
 */
int main (int argc, char *argv[])
{
    srand(time(NULL) + getpid());
    
    // Parse command line args
    int maxargs = 7;
    if (argc > maxargs || argc < maxargs-3)
        usage();
    int arg = 1;
    const char *graphFile = argv[arg++];
    int paths = std::atoi(argv[arg++]);
    if (paths < 0)
    {
        std::cerr << "Number of paths should not be negative!\n";
        std::exit(-1);
    }
    
    // Algorithm specifications
    char *parser = argv[arg++];
    char *s;
    const char *algorithm(strsep(&parser, ":"));
    const char *pathDistance((s = strsep(&parser, ":")) ? s : "?");
    const char *neighborhoodDistance((s = strsep(&parser, ":")) ? s : "?");
    const char *neighborhoodRadius((s = strsep(&parser, ":")) ? s : "-1");
    const double radiusFactor = std::atof(neighborhoodRadius);
    
    double maxPathLength = std::numeric_limits<double>::infinity();
    double minPathPairwiseDistance = std::numeric_limits<double>::epsilon();
    if (arg < argc && std::strcmp("-l", argv[arg]) == 0)
    {
        maxPathLength = std::atof(argv[arg+1]);
        arg += 2;
    }
    else if (arg < argc && std::strcmp("-d", argv[arg]) == 0)
    {
        minPathPairwiseDistance = std::atof(argv[arg+1]);
        arg += 2;
    }
    const bool save = arg < argc && (std::strcmp("-s", argv[arg]) == 0);
    const bool saveGephi = arg < argc && (std::strcmp("-g", argv[arg]) == 0);
    
    // Build graph to test on
    TestData data(graphFile, paths, maxPathLength, minPathPairwiseDistance);
    
    // Choose distance function
    PathDistanceMeasure *distanceMeasure = nullptr;
    if (strcasecmp(pathDistance, "f") == 0 || strcasecmp(pathDistance, "frechet") == 0)
        distanceMeasure = new Frechet();
    else if (strcasecmp(pathDistance, "l") == 0 || strcasecmp(pathDistance, "levenshtein") == 0)
        distanceMeasure = new Levenshtein();
    else if (strcasecmp(pathDistance, "h") == 0 || strcasecmp(pathDistance, "hausdorff") == 0)
        distanceMeasure = new Hausdorff();
    else
    {
        std::cout << "Error: Unknown distance measure '" << pathDistance << "'\n";
        std::exit(-1);
    }
    
    // Choose neighborhood avoidance method (if any)
    Neighborhood::AvoidMethod avoidMethod = Neighborhood::UNKNOWN;
    if (strcasecmp(neighborhoodDistance, "c") == 0 || strcasecmp(neighborhoodDistance, "cspace") == 0)
        avoidMethod = Neighborhood::CSPACE;
    else if (strcasecmp(neighborhoodDistance, "g") == 0 || strcasecmp(neighborhoodDistance, "graph") == 0)
        avoidMethod = Neighborhood::GRAPH;
    
    // Choose algorithm
    KDiverseShort *kDiverseShort = nullptr;
    if (strcasecmp(algorithm, "e") == 0 || strcasecmp(algorithm, "eppstein") == 0)
        kDiverseShort = new Eppstein(&data, distanceMeasure);
    else if (strcasecmp(algorithm, "r") == 0 || strcasecmp(algorithm, "randomavoidance") == 0)
        kDiverseShort = new Voss(&data, distanceMeasure, radiusFactor, avoidMethod);
    else
    {
        std::cout << "Error: Unknown algorithm '" << algorithm << "'\n";
        std::exit(-1);
    }
    
    
    std::cout << "\n";
    kDiverseShort->timedRun();
    kDiverseShort->print();
    if (save)
        kDiverseShort->saveSet();
    else if (saveGephi)
        kDiverseShort->saveNodes();
    delete kDiverseShort;
    delete distanceMeasure;
    
    return 0;
}
