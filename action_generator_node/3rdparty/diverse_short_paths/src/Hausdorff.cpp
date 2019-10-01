/*
 * Hausdorff.cpp
 */

#include "Hausdorff.h"

#include "Graph.h"
#include "Path.h"

// Some convenient macros
#define DYN(I,J)    (distarray[(I)+(J)*(rowLength+1)])
#define DIST(a,b)   (si->distance(a, b))

// Public methods

std::string Hausdorff::getName ()
{
    return "Hausdorff";
}

double Hausdorff::distance (const Path &path1, const Path &path2)
{
  const ompl::base::SpaceInformationPtr si = path1.getGraph()->getSpaceInfo();
  const std::size_t rowLength = path1.size();
  const std::size_t colLength = path2.size();
  double *const distarray = new double[(rowLength+1)*(colLength+1)];
  const std::vector<const ompl::base::State *> states1 = path1.getStates();
  const std::vector<const ompl::base::State *> states2 = path2.getStates();

  // Initialize the min records.
  for (std::size_t i = 0; i < rowLength; i++)
    DYN(i,colLength) = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < colLength; j++)
    DYN(rowLength,j) = std::numeric_limits<double>::infinity();

  // Fill in the distance matrix.
  for (std::size_t i = 0; i < rowLength; i++)
  {
    for (std::size_t j = 0; j < colLength; j++)
    {
      DYN(i,j) = DIST(states1[i], states2[j]);

      // Update the min records.
      DYN(i,colLength) = std::min(DYN(i,colLength), DYN(i,j));
      DYN(rowLength,j) = std::min(DYN(rowLength,j), DYN(i,j));
    }
  }

  // Compute the max over both min records.
  double max = 0;
  for (std::size_t i = 0; i < rowLength; i++)
    max = std::max(max, DYN(i,colLength));
  for (std::size_t j = 0; j < colLength; j++)
    max = std::max(max, DYN(rowLength,j));

  delete [] distarray;

  return max;
}

#undef DYN
#undef DIST
