% add nonlinear equality constraint on pose
value = isam2_values.at(gtsam.symbol('A',0))
fixed_pose = gtsam.Pose3(value.rotation, value.translation)
neq3 = gtsam.NonlinearEqualityPose3(gtsam.symbol('A',0), fixed_pose);
isam2_graph.add(neq3)
isam2_new=gtsam.ISAM2;
isam2_new.update(isam2_graph, isam2_values);
result=isam2_new.calculateBestEstimate;
result
lfg=isam2_graph.linearize(isam2_values);
A=lfg.jacobian;
cond(A)

Ab=lfg.augmentedJacobian;
% rhs
b=Ab(:,end)'

% problem in the factor f2
f2=isam2_graph.at(1);
f2.linearize(isam2_values)
% b = [      nan      nan      nan  -19.496 -25.8766       -0 ]