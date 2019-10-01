/*
 * pch.h
 */

#ifndef __PCH_H
#define __PCH_H

// Standard Library

#include <algorithm>
#include <climits>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// POSIX

#include <unistd.h>

// BOOST

#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

// OMPL

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

// Class Prototypes

class Eppstein;
class Frechet;
class Graph;
class Hausdorff;
class KDiverseShort;
class Levenshtein;
class Neighborhood;
class Path;
class PathDistanceMeasure;
class Results;
class TestData;
class Voss;
namespace graehl { class Graehl; }

#endif
