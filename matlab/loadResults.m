%clear;
close all;
clc;
clearvars -except 'Robot_A_final_trajectories' 'Robot_B_final_trajectories' 'Robot_A_final_cov' 'Robot_B_final_cov';

scenario = 'anpl_mr_passive_rosbag';
rsults_folder = 'Tal_Regev/anpl_mr_passive_rosbag_Tal_Regev_without_mr_without_lc';
path_to_results = [getenv('HOME') filesep 'ANPL' filesep 'infrastructure' filesep 'mrbsp_ws' filesep 'src' ...
        filesep 'open_mrbsp' filesep 'scenarios' filesep scenario filesep...
    'results' filesep rsults_folder filesep 'matlab'];

filename = 'belief_A274';
file_str = [path_to_results filesep filename '_values.txt'];
if ~ isfile(file_str)
    filename = 'belief_A215';
end

values = importdata([path_to_results filesep filename '_values.txt']);
factors = importdata([path_to_results filesep filename '_factors.txt']);

path = {};
cov = {};
cov_plot_rate = 1;
color = ['b', 'g', 'y', 'k'];
robot_id = ['Robot A', 'Robot B', 'Robot C', 'Robot D'];
robot_num = 0;
map = containers.Map;
draw_map = false;
draw_text = false;
draw_ground_truth=true;
draw_odom=false;
figure(1);
if draw_odom
    path_to_bag = '/home/talregev/ANPL/data/bags/MR_laser/pioneer_laser_01.bag';
    bag = rosbag(path_to_bag);
    bagSelect = select(bag,'Topic','/pioneer1/odom');
    array = 1:1000:bagSelect.NumMessages;
    X=zeros(1,numel(array));
    Y=zeros(1,numel(array));   
    for i = 1 : numel(array)
        msg = readMessages(bagSelect,array(i));       
        X(i)= msg{1}.Pose.Pose.Position.X;
        Y(i)= msg{1}.Pose.Pose.Position.Y;   
        if(mod(i,10) == 0)
            delete_array = repmat('\b',1,33);
            fprintf(delete_array);          
            fprintf('load odom laser_01.bag: %.2f%%', i/numel(array)*100);
        end        
    end 
    fprintf('\n');
    hold on
    plot(X,Y);
end

if draw_ground_truth
    path_to_bag = '/home/talregev/ANPL/data/bags/MR_laser/pioneer_laser_01.bag';
    bag = rosbag(path_to_bag);
    bagSelect = select(bag,'Topic','/gazebo/model_states');
    array = 1:1000:bagSelect.NumMessages;
    X=zeros(1,numel(array));
    Y=zeros(1,numel(array));   
    for i = 1 : numel(array)
        msg = readMessages(bagSelect,array(i));
        [~,~,position] = msg{1}.Pose.Position;
        X(i)= position.X;
        Y(i)= position.Y;   
        if(mod(i,10) == 0)
            delete_array = repmat('\b',1,33);
            fprintf(delete_array);
            fprintf('load pioneer_laser_01.bag: %.2f%%', i/numel(array)*100);
        end
        
    end   
    fprintf('\n');
    hold on
    plot(X,Y);
end

if draw_ground_truth
    path_to_bag = '/home/talregev/ANPL/data/bags/MR_laser/pioneer_laser_02.bag';
    bag = rosbag(path_to_bag);
    bagSelect = select(bag,'Topic','/gazebo/model_states');
    array = 1:1000:bagSelect.NumMessages;
    X=zeros(1,numel(array));
    Y=zeros(1,numel(array));   
    for i = 1 : numel(array)
        msg = readMessages(bagSelect,array(i));
        [~,~,position] = msg{1}.Pose.Position;
        X(i)= position.X;
        Y(i)= position.Y;   
        if(mod(i,10) == 0)
            delete_array = repmat('\b',1,33);
            fprintf(delete_array);
            fprintf('load pioneer_laser_02.bag: %.2f%%', i/numel(array)*100);
        end
        
    end   
    fprintf('\n');
    hold on
    plot(X,Y);
end

if draw_map
    %map_draw = importdata('map.txt'); % manualy map
    %offset = [-1.952, 2.051]; % initial position in the map
    %hold on;
    %plot(map_draw(:,1) + offset(1), map_draw(:,2) + offset(2), 'LineWidth', 2)
    bag_name = 'projected_map.bag';
    path_to_bag = [current_run_folder filesep bag_name];
    
    bag = rosbag(path_to_bag);
    occ_grid_msg = readMessages(bag);

    occ_grid = readBinaryOccupancyGrid(occ_grid_msg{1});
    p_grid = show(occ_grid);
    
    
    axis equal
    grid on
end


for i = 1 : size(values.data, 1)
    % arrange data
    id = values.textdata(i,1);
    index = values.data(i,1);
    t = [values.data(i,2); values.data(i,3); values.data(i,4)];
    R = [values.data(i,5:7);
         values.data(i,8:10);
         values.data(i,11:13)];
    curr_cov = [values.data(i,14:19);
                values.data(i,20:25);
                values.data(i,26:31);
                values.data(i,32:37);
                values.data(i,38:43);
                values.data(i,44:49)];

    new_map = containers.Map(sprintf('%c%d',id{1},index), [t(1); t(2)]);
    map = [map; new_map];

    pose3 = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(t));
    yaw = pose3.rotation.yaw;
    pose2 = gtsam.Pose2(t(1),t(2),yaw);
    
    if(draw_text)
        index_txt = sprintf('%c%d', id{1},index);
        txt = ['\leftarrow ' index_txt];
        text(t(1),t(2),txt)
    end

    if(index == 0)
        robot_num = robot_num + 1;
        path{robot_num} = [];
        cov{robot_num} = [];
    end

    % save pose to path
    path{robot_num} = [path{robot_num}; t(1),t(2)];
    cov{robot_num} = [cov{robot_num}; {curr_cov}];

    % plot
    if (i == 1 || mod(i,cov_plot_rate) == 0)
        figure(1)
        hold on;
        axis equal
        grid on
        %gtsam.plotPose2(pose2,color(robot_num),cov(4:6,4:6),0.1);
        title('2D Factor Graph With Map - w MR, w LC - Nearest Neighbors Threshold = 0.80', 'FontSize', 16);
        scatter(path{robot_num}(:,1), path{robot_num}(:,2), color(robot_num));
        p(robot_num) = plot(path{robot_num}(:,1), path{robot_num}(:,2), color(robot_num), 'LineWidth', 3);
        drawnow;
    end
end

for j = 1 : size(factors.data, 1)
    factor_type = factors.textdata(j,1);
    first_key_id = factors.textdata(j,2);
    first_key_index = factors.textdata(j,3);
    first_key = sprintf('%c%c',first_key_id{1},first_key_index{1});
    second_key_id = factors.textdata(j,4);
    second_key = sprintf('%c%d',second_key_id{1},factors.data(j,1));
    
    if(strcmp(factor_type{1},'Prior'))
    else
        point1 = map(first_key);
        point2 = map(second_key);
        
        if(strcmp(factor_type{1},'Between'))
            %line([point1(1),point2(1)],[point1(2),point2(2)],'Color','red');
        elseif(strcmp(factor_type{1},'Between_lc'))
            p(3) = line([point1(1),point2(1)],[point1(2),point2(2)],'Color','red');
        elseif(strcmp(factor_type{1},'Between_mr'))
            p(4) = line([point1(1),point2(1)],[point1(2),point2(2)],'Color','cyan');
        end
    end
end

hold off
if(numel(p) == 2)
    legend([p(1) p(2)],'Robot A','Robot B', 'Location','southwest')
end
if(numel(p) == 3)
    legend([p(1) p(2) p(3)],'Robot A','Robot B', 'Loop Closures', 'Location','southwest')
end
if(numel(p) == 4)
    legend([p(1) p(2) p(3) p(4)],'Robot A','Robot B', 'Loop Closures', 'Multi Robot Factors', 'Location','southwest')
end

