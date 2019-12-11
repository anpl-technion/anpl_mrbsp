close all


%% ROS bag
%bagfilename = '/media/andrej/Seagate Backup Plus Drive/ANPL_infrastructure/withobstables.bag'
bagfilename = '/media/andrej/Seagate Backup Plus Drive/ANPL_infrastructure/DA_withobstacles.bag'
if(length(bagfilename) == 0)
    disp('No bag files to read');
    return;
end
bag = rosbag(bagfilename)
%scans_bag = select(bag, 'Topic', '/Robot_A/scan');
%scan_msgs = readMessages(scans_bag);

keyframes_bag = select(bag, 'Topic', '/Odometry/keyframe_init');
keyframes_msgs = readMessages(keyframes_bag);


% where are the logs from belief_node?
% If not available, they can be recontructed from the given bag by executing:
% roslaunch topological_mr_active inference_test.launch
% roslaunch topological_mr_active playing_rosbag.launch
%path_to_results = '/media/andrej/Seagate Backup Plus Drive/ANPL_infrastructure/reconstructed_logs/withobstacles';
path_to_results = '/home/andrej/ANPL/infrastructure/planning_ws/src/mrbsp_ros/scenarios/topological_mr_active/results/Andrej_Kitanov_2018-05-31_02-58-26';
filesep = '/';

%% draw posterior Beliefs

robot_A_idx = 0 : 38;
%robot_A_idx = 0 : 27;

figure(1)
path = {};
path{1} = [];
cov1 = [];
color = ['b', 'y', 'g', 'k'];
cov_plot_rate = 1;
warning OFF BACKTRACE

for i = robot_A_idx(1) : robot_A_idx(end)
    
    filename = sprintf('belief_A%d', i);
    
    warning('*************************** Begin %s ***********************************************', filename);
    
    values = importdata([path_to_results filesep 'matlab' filesep filename '_values.txt']);
    factors = importdata([path_to_results filesep 'matlab' filesep filename '_factors.txt']);
    
    j = 0;
    while (true)
        j = j + 1;
        id = values.textdata(j,1);
        if (j == length(values.textdata) && strcmp(id{1},'A'))
            break;
        elseif (~strcmp(id{1},'A'))
            j = j - 1;
            break;
        end
    end
    
    % draw results from specific belief
    %j=i+2;

    % arrange data
    id = values.textdata(j,1);
    index = values.data(j,1);
    t = [values.data(j,2); values.data(j,3); values.data(j,4)];
    R = [values.data(j,5:7);
         values.data(j,8:10);
         values.data(j,11:13)];
    cov = [values.data(j,14:19);
           values.data(j,20:25);
           values.data(j,26:31);
           values.data(j,32:37);
           values.data(j,38:43);
           values.data(j,44:49)];
       
    pose3 = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(t));
    yaw = pose3.rotation.yaw;
    pose2 = gtsam.Pose2(t(1),t(2),yaw);
    
    path{1} = [path{1}; t(1),t(2)];
    cov1 = [cov1; {cov}];
    
        % plot
    if (i == 1 || mod(i,cov_plot_rate) == 0)
        hold on;
        axis equal
        grid on
        gtsam.plotPose2(pose2,color(3),cov(4:6,4:6),0.1);
        title('Robot A - 2D Incremental Results - w MR, w LC - Nearest Neighbors Threshold = 0.80', 'FontSize', 16);
        %scatter(path{1}(:,1), path{1}(:,2), color(1));
        p = plot(path{1}(:,1), path{1}(:,2), color(1), 'LineWidth', 2);
        text(t(1), t(2), num2str(i));
        drawnow;
        
        % factor graph
        full_fname = [path_to_results filesep 'serialized_files' filesep filename '_ser_factors.txt'];
        cppgraphstr = load_serialized_gtsam_objects(full_fname);
        
        % replace serialization::archive 12 with serialization::archive 11
        cppgraphstr = replace(cppgraphstr,'serialization::archive 12', 'serialization::archive 11');
        
        factor_graph = gtsam.NonlinearFactorGraph.string_deserialize(cppgraphstr{1})
        
        
        % current estimate
        full_fname = [path_to_results filesep 'serialized_files' filesep filename '_ser_values.txt'];
        cppvaluesstr = load_serialized_gtsam_objects(full_fname);
        
        % replace serialization::archive 12 with serialization::archive 11
        cppvaluesstr = replace(cppvaluesstr,'serialization::archive 12', 'serialization::archive 11');
        
        values = gtsam.Values.string_deserialize(cppvaluesstr{1})

    end
    
    warning('*************************** End %s ***********************************************', filename);
end

%plot3DTrajectory(values, 'r-*', true, 0.3);
%drawnow;
%plotTrajCov('A', factor_graph, values)



%% check point cloud registration
%close all
disp('--- Check point cloud registration. Enter scan indexes to check. To exit enter the same scan twice. ---')
method = 'icp'; % 'icp' | 'ndt' | 'gpc' | 'go_icp'
%i = 25;
%j = 14;
% [2018-05-08 17:06:32.603] [da_laser] [info] DA: Search for loop closure, Robot A indexes: 25, 14
% [2018-05-08 17:06:32.603] [da_laser] [info] DA: temp_pose: X = 2.60569, Y = -4.09238, Z = 0
% [2018-05-08 17:06:32.603] [da_laser] [info] DA: current_pose: X = 1.0981, Y = -3.39311, Z = 0
% [2018-05-08 17:06:32.603] [da_laser] [info] DA: temp_odom: X = -0.266946, Y = -1.64029, Z = 0
% [2018-05-08 17:06:32.603] [da_laser] [info] DA: range: 1.66187, angle difference: : 3.10876
% [2018-05-08 17:06:32.603] [da_laser] [info] PLANNAR_ICP: Matching scans between indexes: A25,A14
% [2018-05-08 17:06:32.655] [da_laser] [info] PLANNAR_ICP:
% delta: X = 3.4234, Y = -0.507714, Yaw = -2.81029
% Number of iterations: 50
% Stoping threshold: 0.000619761
% Min status score: 0.865465
% [2018-05-08 17:06:32.655] [da_laser] [warning] DA: loop closure : Big difference between initial guess and icp results, temp index 14



fReg = [];

while (1)
    
    i = input('First scan ID > ');
    j = input('Second scan ID > ');
    %keyboard  
    if isempty(i) || isempty(j)
        break
    end
    if  i == j break; end
    if ~isempty(fReg) clf(fReg); end
    clc
    
    key_i = gtsam.symbol('A', i);
    p_i = values.at(key_i);
    key_j = gtsam.symbol('A', j);
    p_j = values.at(key_j);
    
    
    delta = p_j.between(p_i);
    H = delta.matrix;
    rotVec = rotm2eul(H(1:3,1:3));
    
    parameters.initial_2d_pose = [H(1,4), H(2,4), rotVec(1)];
    parameters.initial_transform = affine3d(H');
    parameters.icp.max_iter = 50;
    parameters.icp.inlier_ratio = 0.8;
    
    fReg = figure(2);
    % Matlab indicies start from 1
    [relpose, stats, plots] = registerScans(keyframes_msgs{i+1}.LaserScan, keyframes_msgs{j+1}.LaserScan, method, parameters);
    switch method
        case 'icp'
            disp('Initial relative pose')
            disp(parameters.initial_transform.T')
            disp('Est. relative pose')
            disp(relpose.T')
            disp('RMSE')
            disp(stats.rmse)
        case 'ndt'
            disp('Initial relative pose')
            disp(parameters.initial_2d_pose)
            disp('Est. relative pose')
            relpose
            display(stats.score)
        case 'gpc'
            disp('Initial relative pose')
            disp(parameters.initial_2d_pose)
            fprintf('GPC estimated pose: %s\n', sprintf('%f ', relpose));
        case 'go_icp'
            % Paper https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7368945
    end
    legend(plots.h1, ['Scan ' num2str(i)], ['Scan ' num2str(j)], 'Location','northeastoutside')
    legend(plots.h2, ['Scan ' num2str(i)], ['Scan ' num2str(j)], 'Initial guess', 'Location','northeastoutside')

end

