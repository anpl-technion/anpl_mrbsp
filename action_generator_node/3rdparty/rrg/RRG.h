/* Authors: Luis G. Torres */

#ifndef OMPL_CONTRIB_RRT_STAR_RRG_
#define OMPL_CONTRIB_RRT_STAR_RRG_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/goals/GoalRegion.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

namespace ompl
{
    namespace geometric
    {
        class RRG : public base::Planner
        {
        public:
            RRG(const base::SpaceInformationPtr &si);

            virtual ~RRG(void);

            void preprocess(unsigned numIterations);

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange(void) const
            {
                return maxDistance_;
            }

            /** \brief When the planner attempts to refine the graph,
                it does so by looking at some of the neighbors within
                a computed radius. The computation of that radius
                depends on the multiplicative factor set here.
                Set this parameter should be set at least to the side
                length of the (bounded) state space. E.g., if the state
                space is a box with side length L, then this parameter
                should be set to at least L for rapid and efficient
                convergence in trajectory space. */
            void setBallRadiusConstant(double ballRadiusConstant)
            {
                ballRadiusConst_ = ballRadiusConstant;
            }

            /** \brief Get the multiplicative factor used in the
                computation of the radius whithin which tree rewiring
                is done. */
            double getBallRadiusConstant(void) const
            {
                return ballRadiusConst_;
            }

            /** \brief When the planner attempts to refine the graph,
                it does so by looking at some of the neighbors within
                a computed radius. That radius is bounded by the value
                set here. This parameter should ideally be equal longest
                straight line from the initial state to anywhere in the
                state space. In other words, this parameter should be
                "sqrt(d) L", where d is the dimensionality of space
                and L is the side length of a box containing the obstacle free space. */
            void setMaxBallRadius(double maxBallRadius)
            {
                ballRadiusMax_ = maxBallRadius;
            }

            /** \brief Get the maximum radius the planner uses in the
                tree rewiring step */
            double getMaxBallRadius(void) const
            {
                return ballRadiusMax_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<DistItem>());
            }

            virtual void setup(void);

            unsigned getNumCollisionChecks(void) const
            {
                return numCollisionChecks_;
            }

        protected:

            typedef boost::adjacency_list<boost::vecS, 
                                          boost::vecS, 
                                          boost::directedS, 
                                          base::State*, base::Cost> Graph;

            typedef boost::graph_traits<Graph>::vertex_descriptor Node;
            typedef boost::graph_traits<Graph>::edge_descriptor Edge;

            struct DistItem
            {
                const base::State *state;
                Node node;
                bool operator==(const DistItem& rhs) const
                {
                    return (this->state == rhs.state) && (this->node == rhs.node);
                }
                bool operator!=(const DistItem& rhs) const
                {
                    return !(*this == rhs);
                }
            };

            void initPreprocess(void);

            void initSolve(void);

            bool extend(const base::State *s, Node* newNode, Node* nearNode);

            void connectNear(Node node, Node nearNode);

            double getNeighborhoodRadius() const
            {
                return std::min(ballRadiusConst_ * 
                                pow(log((double)(1 + nn_->size())) / (double)(nn_->size()), 
                                    1.0 / (double)si_->getStateSpace()->getDimension()),
                                ballRadiusMax_);
            }

            base::PathPtr getBestPath(Node startNode, const base::GoalPtr& goal);

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);        

            /** \brief Compute distance between nodes (actually distance between contained states) */
            double distanceFunction(DistItem a, DistItem b) const
            {
                return si_->distance(a.state, b.state);
            }

            /** \brief Graph representing the roadmap */
            Graph graph_;

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the graph nodes */
            boost::shared_ptr< NearestNeighbors<DistItem> > nn_;

            /** \brief The maximum length of a motion to be added to the graph */
            double                                         maxDistance_;

            /** \brief Shrink rate of radius the planner uses to find near neighbors */
            double                                         ballRadiusConst_;

            /** \brief Maximum radius the planner uses to find near neighbors */
            double                                         ballRadiusMax_;

            /** \brief Total number of calls to checkMotion() during execution */
            unsigned                                       numCollisionChecks_;

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            base::Cost costToGo(Node v, const base::Goal* goal) const;

            struct found_goal
            {
                found_goal(Node u) : goalNode(u) {}
                Node goalNode;
            };            

            // TOOOOTALLY ASSUMES THAT WE'RE WORKING WITH A GOAL REGION
            class astar_goal_visitor : public boost::default_astar_visitor
            {
            public:
                astar_goal_visitor(const base::GoalRegion* goal,
                                   double* nearestGoalDist,
                                   Node* nearestToGoal) :
                    goal(goal),
                    nearestGoalDist(nearestGoalDist),
                    nearestToGoal(nearestToGoal)
                {
                    *nearestGoalDist = std::numeric_limits<double>::infinity();
                    *nearestToGoal = boost::graph_traits<Graph>::null_vertex();
                }

                void examine_vertex(Node u, const Graph& g)
                {
                    double dist;
                    if (goal->isSatisfied(g[u], &dist))
                    {
                        throw found_goal(u);
                    }
                    else if (dist < *nearestGoalDist)
                    {
                        *nearestGoalDist = dist;
                        *nearestToGoal = u;
                    }
                }

                const base::GoalRegion* goal;
                double* nearestGoalDist;
                Node* nearestToGoal;
            };

            // persistent vectors for use in A*
            std::vector<Node> predMap_;
            std::vector<base::Cost> vertexCostMap_;
        };
    }
}

#endif
