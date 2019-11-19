function [J, t, result] = mrpln40_CalcInfoTheoreticObjFunc(fg, initial)

import gtsam.*

plotting_cov = false;
plotting_traj = false;
global StateSpace
load robots.mat % for compatibility reason between Matlab and Gazebo simulation

%% Plot Initial Estimate
%cla
if plotting_traj || plotting_cov
    figure(333)
    subplot121 = subplot(1,2,1); hold on
    xlabel('X [m]','FontWeight','bold');
    ylabel('Y [m]','FontWeight','bold');
    title('Metric space','FontSize',12);
    % % Set the remaining axes properties
    set(subplot121,'Color',...
        [1 1 1],'DataAspectRatio',...
        [1 1 1],'FontWeight','bold','PlotBoxAspectRatio',...
        [187.409090909091 143.288372093023 35.8220930232558]);
    %set(gca, 'Color', 0.5*[1 1 1]);
    
    if plotting_traj && strcmp(StateSpace, 'SE(2)')
        
        %plot2DTrajectory(initial, 'g-*'); %axis equal
    end
end

%% Gauss-Newton optimization
%   
% % print initial values
% initials.print('\nInitial Values:\n'); 
% 
% 
% % Use Gauss-Newton method optimizes the initial values
% parameters = GaussNewtonParams;
% 
% % print per iteration
% parameters.setVerbosity('ERROR');
%   
% % optimize!
% optimizer = GaussNewtonOptimizer(graph, initials, parameters);
% results = optimizer.optimizeSafely();
%   
% % print final values
% results.print('Final Result:\n');
% 
% 
% % Calculate marginal covariances for all poses
% marginals = Marginals(graph, results);


%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
parameters = gtsam.LevenbergMarquardtParams;
% print per iteration
%parameters.setVerbosity('ERROR')

t1 = tic;
optimizer = LevenbergMarquardtOptimizer(fg, initial, parameters);
%tic
result = optimizer.optimizeSafely;
%toc

%tic
marginals = Marginals(fg, result);
%toc

tplot1 = tic;
hold on; subplot(1,2,1), 
if plotting_traj && strcmp(StateSpace, 'SE(2)')
    plot2DTrajectory(result, 'b.-');
end
%drawnow limitrate


%% Plot Covariance Ellipses
if plotting_cov  
    robot_id = robots{1}.id;
    for i=1:2:result.size()-1 % skip pose 0, 2, 4, ...
        
        key_i = gtsam.symbol(robot_id, i);
        pose_i = result.at(key_i);
        P = marginals.marginalCovariance(key_i);
        H = pose_i.matrix;
        euler_ang = rotm2eul(H(1:3,1:3));
        
        if strcmp(StateSpace, 'SE(3)') % show position cov only
            % convert to 2D pose
            pose_i = gtsam.Pose2(pose_i.x, pose_i.y, euler_ang(1));
            cov = P(4:6, 4:6); 
        else
            cov = P(1:2, 1:2);
        end
        plotPose2(pose_i,'b',cov) % 1.5 sigma confidence region
    end
    view(2)
    axis tight; axis equal;
    drawnow
    % fprintf(1,'%.5f %.5f %.5f\n',P{99})
end
dtplot = toc(tplot1);
%%
keys = result.keys; % KeyList
keyVector = gtsam.KeyVector; % KeyVector

keys.pop_front; % exempt the anchor node (for which the state is fixed) from the estimated states covariance

for i=1:keys.size 
%    disp([num2str(i) '. var --------------------------------------'])
    k = keys.front();
%    vk = result.at(k);
%    disp('x y | theta')
%    disp([num2str([vk.x vk.y]) ' | ' num2str(vk.rotation.theta)]);
%    disp(' ')
    keyVector.push_back(k);
    keys.pop_front;
    
end
%marginals.marginalCovariance(values_all.keys) % ne radi sa Keylist args
%cov = marginals.jointMarginalCovariance(keyVector).fullMatrix;
Lambda = sparse(marginals.jointMarginalInformation(keyVector).fullMatrix);

% subplot222 = subplot(2,2,2), spy(Lambda)
% title('Information matrix sparsity pattern')
% xlabel('nz = 1468','FontWeight','bold');
% box(subplot222,'on');
% axis(subplot222,'ij');
% % Set the remaining axes properties
% set(subplot222,'Color',...
%     [0.941176474094391 0.941176474094391 0.941176474094391],'FontSize',12,...
%     'FontWeight','bold','GridLineStyle','none','PlotBoxAspectRatio',[151 151 1]);
% %waitfor(subplot222, 'Color', [0.941176474094391 0.941176474094391 0.941176474094391])
%drawnow
%J = 1/2*(sum(log(eigs(cov, size(cov,1))))+ (log(2*pi)+1)*size(cov,1)); % global entropy
J = 1/2*(-sum(log(eigs(Lambda, size(Lambda,1))))+ (log(2*pi)+1)*size(Lambda,1)); % global entropy
t = toc(t1)-dtplot;