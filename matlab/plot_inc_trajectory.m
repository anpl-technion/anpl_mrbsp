%clear;
%close all;
clc;
clearvars -except 'Robot_A_inc_trajectories' 'Robot_B_inc_trajectories' 'Robot_A_inc_cov' 'Robot_B_inc_cov';

scenario = 'mr_centralized_laser';
rsults_folder = 'no_lc_no_mr';
path_to_results = ['..' filesep 'mrbsp_scenarios' filesep 'scenarios' filesep scenario filesep...
    'results' filesep rsults_folder filesep 'matlab'];
robot_A_idx = 0 : 69;
robot_B_idx = 0 : 68;

color = ['b', 'y', 'g', 'k'];
cov_plot_rate = 3;
draw_map = true;

if draw_map
    bag_name = 'projected_map.bag';
    path_to_bag = [getenv('HOME') filesep 'ANPL' filesep 'code' filesep 'mrbsp_ws' filesep 'src' ...
        filesep 'mrbsp_ros' filesep 'mrbsp_scenarios' filesep 'scenarios' filesep scenario ...
        filesep 'results' filesep rsults_folder filesep bag_name];
    
    bag = rosbag(path_to_bag);
    occ_grid_msg = readMessages(bag);

    occ_grid = readBinaryOccupancyGrid(occ_grid_msg{1});
    %p_grid = show(occ_grid);
    
    axis equal
    grid on
end

path = {};

figure(1)
show(occ_grid);
hold on;
path{1} = [];
cov1 = [];
for i = robot_A_idx(1) : robot_A_idx(end)
    filename = sprintf('belief_A%d', 69);
    values = importdata([path_to_results filesep filename '_values.txt']);
    factors = importdata([path_to_results filesep filename '_factors.txt']);
    
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
        p = plot(path{1}(:,1), path{1}(:,2), color(1));
        drawnow;
    end
end

hold off
legend(p, 'Robot A incremental trajectory', 'Location','southwest')

figure(2)
p_grid = show(occ_grid);
path{2} = [];
cov2 = [];
for i = robot_B_idx(1) : robot_B_idx(end)
    filename = sprintf('belief_B%d', i);
    values = importdata([path_to_results filesep filename '_values.txt']);
    factors = importdata([path_to_results filesep filename '_factors.txt']);
    
    j = 0;
    
    % arrange data
    id = values.textdata(end,1);
    index = values.data(end,1);
    t = [values.data(end,2); values.data(end,3); values.data(end,4)];
    R = [values.data(end,5:7);
         values.data(end,8:10);
         values.data(end,11:13)];
    cov = [values.data(end,14:19);
           values.data(end,20:25);
           values.data(end,26:31);
           values.data(end,32:37);
           values.data(end,38:43);
           values.data(end,44:49)];
       
    pose3 = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(t));
    yaw = pose3.rotation.yaw;
    pose2 = gtsam.Pose2(t(1),t(2),yaw);
    
    path{2} = [path{2}; t(1),t(2)];
    cov2 = [cov2; {cov}];
        % plot
    if (i == 1 || mod(i,cov_plot_rate) == 0)
        figure(2)
        hold on;
        axis equal
        grid on
        gtsam.plotPose2(pose2,color(4),cov(4:6,4:6),0.1);
        title('Robot B - 2D Incremental Results - w MR, w LC - Nearest Neighbors Threshold = 0.80', 'FontSize', 16);
        %scatter(path{2}(:,1), path{2}(:,2), color(2));
        plot(path{2}(:,1), path{2}(:,2), color(2));
        drawnow;
    end
end

%Robot_A_inc_trajectories{1} = path{1};
%Robot_B_inc_trajectories{1} = path{2};
%Robot_A_inc_cov{1} = cov1;
%Robot_B_inc_cov{1} = cov2;
