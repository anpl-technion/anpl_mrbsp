using PyCall
@pyimport pygtsam
@pyimport numpy as np


noise=pygtsam.Diagonal[:Sigmas](np.array([0.2, 0.2, 0.1]))

x = pygtsam.Pose2(3,2,1)

f = pygtsam.PriorFactorPose2(0, x, noise)

fg = pygtsam.NonlinearFactorGraph()

fg[:add](f)
s = fg[:serialize]()
fg2 = pygtsam.deserializeGraph(s)
fg[:printf]("Factor graph before serialization")
println("-------------------")
fg2[:printf]("After serialization")


println()
v = pygtsam.Values()

v[:insert](0, x)

s = v[:serialize]()
v2 = pygtsam.deserializeValues(s)
println("Values before serialization")
v[:printf]()
println("-------------------")
println("Values after serialization")
v2[:printf]()
