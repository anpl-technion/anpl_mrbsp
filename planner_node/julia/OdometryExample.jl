# Odometry example from GTSAM written in Julia
# This example is using GTSAM 4.0 Python modules
# @author: Andrej Kitanov
# ANPL, Technion

using PyCall
#@pyimport __future__.print_function
@pyimport gtsam
@pyimport gtsam_utils
@pyimport numpy as np

# keys(o::PyObject) # returns an array of the available attribute symbols.

fg = gtsam.NonlinearFactorGraph()
initial = gtsam.Values()

x0 = gtsam.Pose2(0.0, 0.0, 0.0)
#priorNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
priorNoise = gtsam.noiseModel[:Diagonal][:Sigmas](np.array([0.3, 0.3, 0.1])) # python
#priorNoise = gtsam.noiseModel_Diagonal[:Sigmas](np.array([0.3, 0.3, 0.1])) # cython
fg[:add](gtsam.PriorFactorPose2(0, x0, priorNoise))

# Add odometry factors
odometry = gtsam.Pose2(2.0, 0.0, 0.0)
odometryNoise = gtsam.noiseModel[:Diagonal][:Sigmas](np.array([0.2, 0.2, 0.1])) # python
#odometryNoise = gtsam.noiseModel_Diagonal[:Sigmas](np.array([0.2, 0.2, 0.1])) # cython
# Create odometry (Between) factors between consecutive poses
fg[:add](gtsam.BetweenFactorPose2(0, 1, odometry, odometryNoise))
fg[:add](gtsam.BetweenFactorPose2(1, 2, odometry, odometryNoise))
fg[:print]() # python
#fg[:print_]("") # cython

initial[:insert](0, gtsam.Pose2(0.5, 0.0, 0.2))
initial[:insert](1, gtsam.Pose2(2.3, 0.1, -0.2))
initial[:insert](2, gtsam.Pose2(4.1, 0.1, 0.1))
initial[:print]("\nInitial Estimate:\n")
#initial[:print_]("\nInitial Estimate:\n")

params = gtsam.LevenbergMarquardtParams()
lm = gtsam.LevenbergMarquardtOptimizer(fg, initial, params)
result = lm[:optimize]()
result[:print]("\nFinal Result:\n")
#result[:print_]("\nFinal Result:\n")

# show more rows on screen
# show(IOContext(STDOUT, limit=true, displaysize=(100,4)), "text/plain", zeros(100,1))
# show(IOContext(STDOUT, limit=false), "text/plain", keys(fg))
#gtsam.writeG2o(fg, initial, "odometry_graph.g2o")
#retrieved = gtsam.readG2o("odometry_graph.g2o")
#retrieved[1][:print_]("\nRetrieved graph from file\n")
#retrieved[2][:print_]("\nRetrieved values from file\n")

# test serialization
str = gtsam.serializeGraph(fg)
println(str)
fg2 = gtsam.deserializeGraph(str)
fg2[:print]("Deserialized factor graph")
