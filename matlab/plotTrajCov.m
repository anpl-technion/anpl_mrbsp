function plotTrajCov(robot_id, fg, result)
import gtsam.*

marginals = Marginals(fg, result);

for i=0:result.size()-1
    
    key_i = gtsam.symbol(robot_id, i);
    pose_i = result.at(key_i);
    P = marginals.marginalCovariance(key_i);
    H = pose_i.matrix;
    euler_ang = rotm2eul(H(1:3,1:3));
    % convert to 2D pose
    pose_i = gtsam.Pose2(pose_i.x, pose_i.y, euler_ang(1));
    cov = P(4:6, 4:6); % position only
    plotPose2(pose_i,'b',cov)
    text(pose_i.x, pose_i.y, num2str(i), 'Color', 'red')
end