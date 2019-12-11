clear
close all
clc

load('Robot_A_inc_trajectories.mat');
load('Robot_B_inc_trajectories.mat');
load('Robot_A_inc_cov.mat');
load('Robot_B_inc_cov.mat');
load('Robot_A_final_trajectories.mat');
load('Robot_B_final_trajectories.mat');
load('Robot_A_final_cov.mat');
load('Robot_B_final_cov.mat');
load('occ_grid.mat');

%% Robot A Final Trajectory
figure
hold on
grid on

p_grid = show(occ_grid);
p1 = plot(Robot_A_final_path{3}(:,1), Robot_A_final_path{3}(:,2),'LineWidth',2);
p2 = plot(Robot_A_final_path{2}(:,1), Robot_A_final_path{2}(:,2),'LineWidth',2);
p3 = plot(Robot_A_final_path{1}(:,1), Robot_A_final_path{1}(:,2),'LineWidth',2);

title('Robot A Final Trajectory', 'FontSize', 16)
legend([p1 p2 p3], {'w/o LC, w/o MR', 'w LC, w/o MR', 'w LC, w MR'}, 'FontSize', 12, 'Location','southwest')

%% Robot B Final Trajectory
figure
hold on
grid on

p_grid = show(occ_grid);
p1 = plot(Robot_B_final_path{3}(:,1), Robot_B_final_path{3}(:,2),'LineWidth',2);
p2 = plot(Robot_B_final_path{2}(:,1), Robot_B_final_path{2}(:,2),'LineWidth',2);
p3 = plot(Robot_B_final_path{1}(:,1), Robot_B_final_path{1}(:,2),'LineWidth',2);

title('Robot B Final Trajectory', 'FontSize', 16)
legend([p1 p2 p3], {'w/o LC, w/o MR', 'w LC, w/o MR', 'w LC, w MR'}, 'FontSize', 12, 'Location','southwest')

%% Robot A Final Trajectory Error
figure
hold on
grid on

p1 = plot(sqrt((Robot_A_final_path{1}(:,1) - Robot_A_final_path{3}(:,1)).^2 +...
          (Robot_A_final_path{1}(:,2) - Robot_A_final_path{3}(:,2)).^2),'LineWidth',2);
p2 = plot(sqrt((Robot_A_final_path{1}(:,1) - Robot_A_final_path{2}(:,1)).^2 +...
          (Robot_A_final_path{1}(:,2) - Robot_A_final_path{2}(:,2)).^2),'LineWidth',2);

title('Robot A Final Trajectory Error (Comparing to trajectory w LC and w MR)', 'FontSize', 16)
legend([p1 p2], {'w/o LC, w/o MR', 'w LC, w/o MR'}, 'FontSize', 12, 'Location','northwest')

%% Robot B Final Trajectory Error
figure
hold on
grid on

p1 = plot(sqrt((Robot_B_final_path{1}(:,1) - Robot_B_final_path{3}(:,1)).^2 +...
          (Robot_B_final_path{1}(:,2) - Robot_B_final_path{3}(:,2)).^2),'LineWidth',2);
p2 = plot(sqrt((Robot_B_final_path{1}(:,1) - Robot_B_final_path{2}(:,1)).^2 +...
          (Robot_B_final_path{1}(:,2) - Robot_B_final_path{2}(:,2)).^2),'LineWidth',2);

title('Robot B Final Trajectory Error (Comparing to trajectory w LC and w MR)', 'FontSize', 16)
legend([p1 p2], {'w/o LC, w/o MR', 'w LC, w/o MR'}, 'FontSize', 12, 'Location','northwest')

%% Robot A - Covariance Final Trajectory
figure
hold on
grid on

for i = 1 : length(Robot_A_final_cov)
    for j = 1 : length(Robot_A_final_cov{i})
        d = diag(Robot_A_final_cov{i}{j});
        final_cov_norm_A(i,j) =  sqrt(norm(d(4:5))); %% only x & y
    end
    plot(0:j-1, final_cov_norm_A(i,:))
end
xlabel('Keyframe Index')
ylabel('sqrt(norm(diag))) [m]')
title('Robot A - Final Trajectory Covariance', 'FontSize', 16)
legend('w MR, w LC', 'w/o MR, w LC', 'w/o MR, w/o LC', 'Location','northwest')

%% Robot A - Covariance Incremental Trajectory
figure
hold on
grid on

for i = 1 : length(Robot_A_inc_cov)
    for j = 1 : length(Robot_A_inc_cov{i})
        d = diag(Robot_A_inc_cov{i}{j});
        inc_cov_norm_A(i,j) =  sqrt(norm(d(4:5))); %% only x & y
    end
    plot(0:j-1, inc_cov_norm_A(i,:))
end
xlabel('Keyframe Index')
ylabel('sqrt(norm(diag))) [m]')
title('Robot A - Incremental Trajectory Covariance', 'FontSize', 16)
legend('w MR, w LC', 'w/o MR, w LC', 'w/o MR, w/o LC', 'Location','northwest')

%% Robot B - Covariance Final Trajectory
figure
hold on
grid on

for i = 1 : length(Robot_B_final_cov)
    for j = 1 : length(Robot_B_final_cov{i})
        d = diag(Robot_B_final_cov{i}{j});
        final_cov_norm_B(i,j) =  sqrt(norm(d(4:5))); %% only x & y
    end
    plot(0:j-1, final_cov_norm_B(i,:))
end
xlabel('Keyframe Index')
ylabel('sqrt(norm(diag))) [m]')
title('Robot B - Final Trajectory Covariance', 'FontSize', 16)
legend('w MR, w LC', 'w/o MR, w LC', 'w/o MR, w/o LC', 'Location','northwest')

%% Robot B - Covariance Incremental Trajectory
figure
hold on
grid on

for i = 1 : length(Robot_B_inc_cov)
    for j = 1 : length(Robot_B_inc_cov{i})
        d = diag(Robot_B_inc_cov{i}{j});
        inc_cov_norm_B(i,j) =  sqrt(norm(d(4:5))); %% only x & y
    end
    plot(0:j-1, inc_cov_norm_B(i,:))
end
xlabel('Keyframe Index')
ylabel('sqrt(norm(diag))) [m]')
title('Robot B - Incremental Trajectory Covariance', 'FontSize', 16)
legend('w MR, w LC', 'w/o MR, w LC', 'w/o MR, w/o LC', 'Location','northwest')

%% Robot A Final Trajectory Vs Incremental Trajectory Without MR With LC
close all
figure
hold on
grid on

p_grid = show(occ_grid);
p1 = plot(Robot_A_final_path{2}(:,1), Robot_A_final_path{2}(:,2),'LineWidth',2);
p2 = plot(Robot_A_inc_trajectories{2}(:,1), Robot_A_inc_trajectories{2}(:,2),'LineWidth',2);

title('Robot A Final Trajectory Vs Incremental Trajectory Without MR With LC', 'FontSize', 16)
legend([p1 p2], {'Final trajectory', 'Incremental trajectory'},...
       'FontSize', 12, 'Location','southwest')
   
%% Robot A Final Trajectory Vs Incremental Trajectory With MR With LC
figure
hold on
grid on

p_grid = show(occ_grid);
p1 = plot(Robot_A_final_path{1}(:,1), Robot_A_final_path{1}(:,2),'LineWidth',2);
p2 = plot(Robot_A_inc_trajectories{1}(:,1), Robot_A_inc_trajectories{1}(:,2),'LineWidth',2);

title('Robot A Final Trajectory Vs Incremental Trajectory With MR With LC', 'FontSize', 16)
legend([p1 p2], {'Final trajectory', 'Incremental trajectory'},...
       'FontSize', 12, 'Location','southwest')
   
   %% Robot B Final Trajectory Vs Incremental Trajectory Without MR With LC
figure
hold on
grid on

p_grid = show(occ_grid);
p1 = plot(Robot_B_final_path{2}(:,1), Robot_B_final_path{2}(:,2),'LineWidth',2);
p2 = plot(Robot_B_inc_trajectories{2}(:,1), Robot_B_inc_trajectories{2}(:,2),'LineWidth',2);

title('Robot B Final Trajectory Vs Incremental Trajectory Without MR With LC', 'FontSize', 16)
legend([p1 p2], {'Final trajectory', 'Incremental trajectory'},...
       'FontSize', 12, 'Location','southwest')
   
%% Robot B Final Trajectory Vs Incremental Trajectory With MR With LC
figure
grid on
hold on

p_grid = show(occ_grid);
p1 = plot(Robot_B_final_path{1}(:,1), Robot_B_final_path{1}(:,2),'LineWidth',2);
p2 = plot(Robot_B_inc_trajectories{1}(:,1), Robot_B_inc_trajectories{1}(:,2),'LineWidth',2);

title('Robot B Final Trajectory Vs Incremental Trajectory With MR With LC', 'FontSize', 16)
legend([p1 p2], {'Final trajectory', 'Incremental trajectory'},...
       'FontSize', 12, 'Location','southwest')