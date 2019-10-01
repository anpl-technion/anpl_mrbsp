using PyCall
@pyimport pygtsam as gtsam


#file = open("/home/andrej/factor_graph.txt")
file = open("/home/andrej/anpl/code/debugging/ORB_SLAM2_SERIALIZED_GRAPH.txt")

global graph_str, val_str
graph_str = "";
for line in eachline(file)
    #println(line)
    graph_str *= line;
    graph_str *= "\n";
end
close(file)

file = open("/home/andrej/anpl/code/debugging/ORB_SLAM2_SERIALIZED_VALUES.txt")
val_str = "";
for line in eachline(file)
    #println(line)
    val_str *= line;
    val_str *= "\n";
end
close(file)


fg = gtsam.deserializeGraph(graph_str)
fg[:printf]()
#show(IOContext(STDOUT, limit=false), "text/plain", keys(fg))

# get first factor
f0 = fg[:__getitem__](0)

#relPose=gtsam.BetweenFactorPose2;
#relPose = gtsam.getNonlinearFactor(fg, 1)
#z = relPose[:measured]()
#z[:x](), z[:y](), z[:theta]()

initialEst = gtsam.deserializeValues(val_str)
initialEst[:printf]()

# symbol 'x0'
s = gtsam.Symbol( UInt8('x'),0)

robot_poses = gtsam.extractPose3(initialEst)
arr = collect(robot_poses)
# extract the first pose in array, not neccesarily 'x0'
s = arr[1][1]
pose3 = arr[1][2]
s[:print]("")
s[:chr]()
s[:index]()
p = robot_poses[s]
t = p[:translation]()[:vector]()
R = p[:rotation]()[:matrix]()

landmark_pos = gtsam.extractPoint3(initialEst)

# TODO No to_python (by-value) converter found for C++ type: gtsam::Value'
#k = gtsam.extractKeys(initialEst)
#initialEst[:at](k[1])

isam = gtsam.ISAM2()
#isam[:update](fg, initialEst)
