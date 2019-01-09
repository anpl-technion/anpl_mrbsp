clear
close all
clc

scenario = 'mr_centralized_laser';
results_folder = 'mr_lc_nn_80';

path_to_bags = strings;

bag_name_01 = ('Robot_A_keyframe_bag.bag');
path_to_bags(1) = [getenv('HOME') filesep 'ANPL' filesep 'code' filesep 'mrbsp_ws' filesep 'src'...
    filesep 'mrbsp_ros' filesep 'mrbsp_scenarios' filesep 'scenarios' filesep scenario...
    filesep 'results' filesep results_folder filesep bag_name_01];

bag_name_02 = ('Robot_B_keyframe_bag.bag');
path_to_bags(2) = [getenv('HOME') filesep 'ANPL' filesep 'code' filesep 'mrbsp_ws' filesep 'src'...
    filesep 'mrbsp_ros' filesep 'mrbsp_scenarios' filesep 'scenarios' filesep scenario...
    filesep 'results' filesep results_folder filesep bag_name_02];

%% Load robot A data
% bag file
bag_A = rosbag(path_to_bags{1});
% trajectory data
load('Robot_A_final_trajectories.mat');
load('Robot_A_inc_trajectories.mat');
load('occ_grid.mat');

%% Ground truth data
bag_A_gt = select(bag_A, 'Topic', '/ground_truth_pose');
bag_A_gt_msg = readMessages(bag_A_gt);

robot_A_gt_pose = [];
robot_A_init_pose = bag_A_gt_msg{2}.Pose.Position;
calibration_pose = Robot_A_final_path{1}(2,:);

X_correction = -calibration_pose(2) - robot_A_init_pose.X;
Y_correction =  calibration_pose(1) - robot_A_init_pose.Y;

for i = 1 : length(bag_A_gt_msg)
    point_msg = bag_A_gt_msg{i}.Pose.Position;
    % rotate frame by 90 deg and fix init pose
    gt_x = point_msg.X + X_correction;
    gt_y = point_msg.Y + Y_correction;
    gt_z = point_msg.Z;
    
    robot_A_gt_pose = [robot_A_gt_pose; gt_y -gt_x gt_z];
end
clear 'bag_A_gt' 'bag_A_gt_msg'

%% Odometry estimation data
bag_A_odom_est = select(bag_A, 'Topic', '/estimated_pose');
bag_A_odom_est_msg = readMessages(bag_A_odom_est);

robot_A_odom_est_pose = [];
for i = 1 : length(bag_A_odom_est_msg)
    point_msg = bag_A_odom_est_msg{i}.Pose.Position;
    robot_A_odom_est_pose = [robot_A_odom_est_pose; point_msg.X point_msg.Y point_msg.Z];
end
clear 'bag_A_odom_est' 'bag_A_odom_est_msg'

%% Plot robot A start
figure
hold on
grid on

first_idx = 2;
last_idx = 7;
p_grid = show(occ_grid);
p1 = plot(robot_A_gt_pose(first_idx:last_idx,1), robot_A_gt_pose(first_idx:last_idx,2));
p2 = plot(Robot_A_final_path{1}(first_idx:last_idx,1), Robot_A_final_path{1}(first_idx:last_idx,2),'LineWidth',2);
p3 = plot(Robot_A_final_path{2}(first_idx:last_idx,1), Robot_A_final_path{2}(first_idx:last_idx,2),'LineWidth',2);
p4 = plot(Robot_A_final_path{3}(first_idx:last_idx,1), Robot_A_final_path{3}(first_idx:last_idx,2),'LineWidth',2);
p5 = plot(Robot_A_inc_trajectories{1}(first_idx:last_idx,1), Robot_A_inc_trajectories{1}(first_idx:last_idx,2),'LineWidth',2);
p6 = plot(robot_A_odom_est_pose(first_idx:last_idx,1), robot_A_odom_est_pose(first_idx:last_idx,2));

title('Robot A start', 'FontSize', 16)
legend([p1 p2 p3 p4 p5 p6],...
    {'ground truth', 'final w LC, w MR', 'final w LC, w/o MR', 'final w/o LC, w/o MR', 'incremental w LC, w MR', 'odometry'},...
    'FontSize', 12, 'Location','southeast')

% robot A start error
figure
hold on
grid on
p1 = plot(first_idx:last_idx, sqrt((Robot_A_final_path{1}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_final_path{1}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p2 = plot(first_idx:last_idx, sqrt((Robot_A_final_path{2}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_final_path{2}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p3 = plot(first_idx:last_idx, sqrt((Robot_A_final_path{3}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_final_path{3}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p4 = plot(first_idx:last_idx, sqrt((Robot_A_inc_trajectories{1}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_inc_trajectories{1}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p5 = plot(first_idx:last_idx, sqrt((robot_A_odom_est_pose(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (robot_A_odom_est_pose(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);

title('Robot A RMS Error - Start of Trajectory', 'FontSize', 16)
legend([p1 p2 p3 p4 p5],...
    {'final w LC, w MR', 'final w LC, w/o MR', 'final w/o LC, w/o MR', 'incremental w LC, w MR', 'odometry'},...
    'FontSize', 12, 'Location','northwest')
xlabel('keyframe index')
ylabel('RMS error [m]')

%% Plot robot A end - Final trajectory
figure
hold on
grid on

first_idx = 64;
last_idx = 70;
p_grid = show(occ_grid);
p1 = plot(Robot_A_final_path{1}(first_idx:last_idx,1), Robot_A_final_path{1}(first_idx:last_idx,2),'LineWidth',2);
p2 = plot(Robot_A_final_path{2}(first_idx:last_idx,1), Robot_A_final_path{2}(first_idx:last_idx,2),'LineWidth',2);
p3 = plot(Robot_A_final_path{3}(first_idx:last_idx,1), Robot_A_final_path{3}(first_idx:last_idx,2),'LineWidth',2);
p4 = plot(robot_A_gt_pose(first_idx:last_idx,1), robot_A_gt_pose(first_idx:last_idx,2),'LineWidth',2,'Color','g');

title('Robot A - Final trajectory with ground truth (where available)', 'FontSize', 16)
legend([p1 p2 p3 p4],...
    {'final w LC, w MR', 'final w LC, w/o MR', 'final w/o LC, w/o MR', 'ground truth'},...
    'FontSize', 12, 'Location','southeast')

% robot A end error
figure
hold on
grid on
p1 = plot(first_idx:last_idx, sqrt((Robot_A_final_path{1}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_final_path{1}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p2 = plot(first_idx:last_idx, sqrt((Robot_A_final_path{2}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_final_path{2}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p3 = plot(first_idx:last_idx, sqrt((Robot_A_final_path{3}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_final_path{3}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);


title('Robot A - Final trajectory RMS Error', 'FontSize', 16)
legend([p1 p2 p3],...
    {'final w LC, w MR', 'final w LC, w/o MR', 'final w/o LC, w/o MR'},...
    'FontSize', 12, 'Location','northwest')
xlabel('keyframe index')
ylabel('RMS error [m]')

%% Plot robot A end - Inc trajectory
figure
hold on
grid on

first_idx = 64;
last_idx = 70;
p_grid = show(occ_grid);
p1 = plot(Robot_A_inc_trajectories{1}(first_idx:last_idx,1), Robot_A_inc_trajectories{1}(first_idx:last_idx,2),'LineWidth',2);
p2 = plot(Robot_A_inc_trajectories{2}(first_idx:last_idx,1), Robot_A_inc_trajectories{2}(first_idx:last_idx,2),'LineWidth',2);
p3 = plot(Robot_A_inc_trajectories{3}(first_idx:last_idx,1), Robot_A_inc_trajectories{3}(first_idx:last_idx,2),'LineWidth',2);
p4 = plot(robot_A_odom_est_pose(first_idx:last_idx,1), robot_A_odom_est_pose(first_idx:last_idx,2),'LineWidth',2);
p5 = plot(robot_A_gt_pose(first_idx:last_idx,1), robot_A_gt_pose(first_idx:last_idx,2),'LineWidth',2,'Color','g');

title('Robot A - Incremental trajectory with ground truth (where available)', 'FontSize', 16)
legend([p1 p2 p3 p4 p5],...
    {'incremental w LC, w MR', 'incremental w LC, w/o MR', 'incremental w/o LC, w/o MR', 'odometry', 'ground truth'},...
    'FontSize', 12, 'Location','southeast')

% robot A end error
figure
hold on
grid on
p1 = plot(first_idx:last_idx, sqrt((Robot_A_final_path{1}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_inc_trajectories{1}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p2 = plot(first_idx:last_idx, sqrt((Robot_A_final_path{2}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_inc_trajectories{2}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p3 = plot(first_idx:last_idx, sqrt((Robot_A_final_path{3}(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_A_inc_trajectories{3}(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p4 = plot(first_idx:last_idx, sqrt((robot_A_odom_est_pose(first_idx:last_idx,1) - robot_A_gt_pose(first_idx:last_idx,1)).^2 +...
          (robot_A_odom_est_pose(first_idx:last_idx,2) - robot_A_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);

title('Robot A - Incremental trajectory RMS Error', 'FontSize', 16)
legend([p1 p2 p3 p4],...
    {'incremental w LC, w MR', 'incremental w LC, w/o MR', 'incremental w/o LC, w/o MR', 'odometry'},...
    'FontSize', 12, 'Location','northwest')
xlabel('keyframe index')
ylabel('RMS error [m]')

%% Load robot B data
% bag file
bag_B = rosbag(path_to_bags{2});
% trajectory data
load('Robot_B_final_trajectories.mat');
load('Robot_B_inc_trajectories.mat');

%% Ground truth data
bag_B_gt = select(bag_B, 'Topic', '/ground_truth_pose');
bag_B_gt_msg = readMessages(bag_B_gt);

robot_B_gt_pose = [];
%robot_A_init_pose = bag_A_gt_msg{2}.Pose.Position;
%calibration_pose = Robot_A_final_path{1}(2,:);

%X_correction = -calibration_pose(2) - robot_A_init_pose.X;
%Y_correction =  calibration_pose(1) - robot_A_init_pose.Y;

for i = 1 : length(bag_B_gt_msg)
    point_msg = bag_B_gt_msg{i}.Pose.Position;
    % rotate frame by 90 deg and fix init pose
    gt_x = point_msg.X + X_correction;
    gt_y = point_msg.Y + Y_correction;
    gt_z = point_msg.Z;
    
    robot_B_gt_pose = [robot_B_gt_pose; gt_y -gt_x gt_z];
end
clear 'bag_B_gt' 'bag_B_gt_msg'

%% Odometry estimation data
bag_B_odom_est = select(bag_B, 'Topic', '/estimated_pose');
bag_B_odom_est_msg = readMessages(bag_B_odom_est);

robot_B_odom_est_pose = [];
for i = 1 : length(bag_B_odom_est_msg)
    point_msg = bag_B_odom_est_msg{i}.Pose.Position;
    robot_B_odom_est_pose = [robot_B_odom_est_pose; point_msg.X point_msg.Y point_msg.Z];
end
clear 'bag_B_odom_est' 'bag_B_odom_est_msg'

%% Plot robot B - Final trajectory
figure
hold on
grid on

first_idx = 39;
last_idx = 50;
p_grid = show(occ_grid);
p1 = plot(Robot_B_final_path{1}(first_idx:last_idx,1), Robot_B_final_path{1}(first_idx:last_idx,2),'LineWidth',2);
p2 = plot(Robot_B_final_path{2}(first_idx:last_idx,1), Robot_B_final_path{2}(first_idx:last_idx,2),'LineWidth',2);
p3 = plot(Robot_B_final_path{3}(first_idx:last_idx,1), Robot_B_final_path{3}(first_idx:last_idx,2),'LineWidth',2);
p4 = plot(robot_B_gt_pose(first_idx:last_idx,1), robot_B_gt_pose(first_idx:last_idx,2),'LineWidth',2,'Color','g');

title('Robot B - Final trajectory with ground truth (where available)', 'FontSize', 16)
legend([p1 p2 p3 p4],...
    {'final w LC, w MR', 'final w LC, w/o MR', 'final w/o LC, w/o MR', 'ground truth'},...
    'FontSize', 12, 'Location','southeast')

% robot B end error
figure
hold on
grid on
p1 = plot(first_idx:last_idx, sqrt((Robot_B_final_path{1}(first_idx:last_idx,1) - robot_B_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_B_final_path{1}(first_idx:last_idx,2) - robot_B_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p2 = plot(first_idx:last_idx, sqrt((Robot_B_final_path{2}(first_idx:last_idx,1) - robot_B_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_B_final_path{2}(first_idx:last_idx,2) - robot_B_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p3 = plot(first_idx:last_idx, sqrt((Robot_B_final_path{3}(first_idx:last_idx,1) - robot_B_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_B_final_path{3}(first_idx:last_idx,2) - robot_B_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);


title('Robot B - Final trajectory RMS Error', 'FontSize', 16)
legend([p1 p2 p3],...
    {'final w LC, w MR', 'final w LC, w/o MR', 'final w/o LC, w/o MR'},...
    'FontSize', 12, 'Location','northwest')
xlabel('keyframe index')
ylabel('RMS error [m]')

%% Plot robot B - Inc trajectory
figure
hold on
grid on

first_idx = 39;
last_idx = 50;
p_grid = show(occ_grid);
p1 = plot(Robot_B_inc_trajectories{1}(first_idx:last_idx,1), Robot_B_inc_trajectories{1}(first_idx:last_idx,2),'LineWidth',2);
p2 = plot(Robot_B_inc_trajectories{2}(first_idx:last_idx,1), Robot_B_inc_trajectories{2}(first_idx:last_idx,2),'LineWidth',2);
p3 = plot(Robot_B_inc_trajectories{3}(first_idx:last_idx,1), Robot_B_inc_trajectories{3}(first_idx:last_idx,2),'LineWidth',2);
p4 = plot(robot_B_odom_est_pose(first_idx:last_idx,1), robot_B_odom_est_pose(first_idx:last_idx,2),'LineWidth',2);
p5 = plot(robot_B_gt_pose(first_idx:last_idx,1), robot_B_gt_pose(first_idx:last_idx,2),'LineWidth',2,'Color','g');

title('Robot B - Incremental trajectory with ground truth (where available)', 'FontSize', 16)
legend([p1 p2 p3 p4 p5],...
    {'incremental w LC, w MR', 'incremental w LC, w/o MR', 'incremental w/o LC, w/o MR', 'odometry', 'ground truth'},...
    'FontSize', 12, 'Location','southeast')

% robot B end error
figure
hold on
grid on
p1 = plot(first_idx:last_idx, sqrt((Robot_B_final_path{1}(first_idx:last_idx,1) - robot_B_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_B_inc_trajectories{1}(first_idx:last_idx,2) - robot_B_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p2 = plot(first_idx:last_idx, sqrt((Robot_B_final_path{2}(first_idx:last_idx,1) - robot_B_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_B_inc_trajectories{2}(first_idx:last_idx,2) - robot_B_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p3 = plot(first_idx:last_idx, sqrt((Robot_B_final_path{3}(first_idx:last_idx,1) - robot_B_gt_pose(first_idx:last_idx,1)).^2 +...
          (Robot_B_inc_trajectories{3}(first_idx:last_idx,2) - robot_B_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);
p4 = plot(first_idx:last_idx, sqrt((robot_B_odom_est_pose(first_idx:last_idx,1) - robot_B_gt_pose(first_idx:last_idx,1)).^2 +...
          (robot_B_odom_est_pose(first_idx:last_idx,2) - robot_B_gt_pose(first_idx:last_idx,2)).^2),'LineWidth',2);

title('Robot B - Incremental trajectory RMS Error', 'FontSize', 16)
legend([p1 p2 p3 p4],...
    {'incremental w LC, w MR', 'incremental w LC, w/o MR', 'incremental w/o LC, w/o MR', 'odometry'},...
    'FontSize', 12, 'Location','northwest')
xlabel('keyframe index')
ylabel('RMS error [m]')