function [A, hG] = mrpln33_graphAdjacencyMatrix(fg, values, values_priors, robots, varargin)

% Construct adjacency matrix of the given factor graph of the joint
% multi-robot state and return its corresponding topological graph.
% Joint state X is defined as {Xr1, Xr2, ...} where Xri denotes the set of
% poses of the robot ri. Currently, only pose graph is considered.

global Debug
gazebo_simulation = true; % TODO add as argument
global SPATIAL_SEARCH_RADIUS

% path_cell{r}.keys --->  values{r}.keys
% e.g. values{1}.keys(path_cell{1}.keys(1)) value of the 1st pose of the 1st robot 
nrRobots = length(robots);

if Debug > 1
    values_all = gtsam.Values; 
    values_all.insert(values_priors);
    for r=1:nrRobots
        values_all.insert(values{r});
    end
    
    plot_flags.factor_color = [0.1 0.3 0.5];
    plot_flags.factor_line_width = 2;
    plot_flags.factor_linestyle = '-';
    
    figure(333);
    subplot121 = subplot(1,2,1); hold on
    
    mrpln02b_PlotPairwiseFactorsInFG(fg, values_all, plot_flags);
    
    % plot mr factors
    if nargin > 4 && varargin{1}.size > 0
        fg_mr = varargin{1};
        plot_flags.factor_color = 'cyan';
        plot_flags.factor_line_width = 2;
        plot_flags.factor_linestyle = '-';
        mrpln02b_PlotPairwiseFactorsInFG(fg_mr, values_all, plot_flags);
    end

end

% this vector will contain [size(Xr1), size(Xr2),...]
robot_state_dims = zeros(nrRobots,1);
robot_ids = zeros(nrRobots,1);
robot_offsets = zeros(nrRobots,1);

for r=1:nrRobots
    robot_state_dims(r) = values{r}.size;
    robot_ids(r) = robots{r}.id;
    % Matlab simulation
    % robot_offsets(r) = 1000*r; % check this assumption!!!
    % Gazebo simulation
    robot_offsets(r) = 0;
end

n = sum(robot_state_dims);
A = sparse(n,n); % an n-by-n all zero sparse matrix
% connect local factor graphs of robot A and B due to prior correlation
if (nrRobots == 2)
    A(1,robot_state_dims(1)+1) = 1;
    A(robot_state_dims(1)+1, 1) = 1;
end

f_indx = 0;
while f_indx < fg.size()
    if ~fg.exists(f_indx)
        f_indx = f_indx + 1;
        continue;
    end
    
    factor = fg.at(f_indx);
    keys   = factor.keys();
    
    f_indx = f_indx + 1;
    
    % non-binary factor, skip
    if keys.size() ~= 2, continue; end
    
    % ignore factors outside of SPATIAL_SEARCH_RADIUS
    if norm(factor.measured.translation.vector) > SPATIAL_SEARCH_RADIUS, continue; end
    
    %str = ['Binary factor '  mrpln16_key2text(keys.front()) '---'  mrpln16_key2text(keys.back()) ' found '];
     %   disp(str)
    
    Chr1 = gtsam.mrsymbolChr(keys.front());
    Chr2 = gtsam.mrsymbolChr(keys.back());
    
    ki = find(Chr1 == robot_ids);
    kj = find(Chr2 == robot_ids);
    
    
    % discard factors including variables outside the scope defined by
    % robot_offsets
    i0 = gtsam.mrsymbolIndex(keys.front())-robot_offsets(ki);
    if ~gazebo_simulation
        if i0 <= 0   % skip prior factors A0-A1001, B0-B1001 because this is identity, the same variable
            continue;
        else
            i = i0+sum(robot_state_dims(1:ki-1));
        end
        j0 = gtsam.mrsymbolIndex(keys.back())-robot_offsets(kj);
        if j0 <= 0
            continue;
        else
            j = j0+sum(robot_state_dims(1:kj-1));
        end
        
    else % Gazebo simulation, different variable indexing
        if i0 < 0
            warning('something is wrong in indexing vars')
            continue;
        else
            i = i0+1+sum(robot_state_dims(1:ki-1));
        end
        j0 = gtsam.mrsymbolIndex(keys.back())-robot_offsets(kj);
        if j0 < 0
            warning('something is wrong in indexing vars')
            continue;
        else
            j = j0+1+sum(robot_state_dims(1:kj-1));
        end
    end
    
    
    
    % add bi-directional edge {i,j}
    if (i ~= 0 && j~= 0)
%         str = ['Adding factor '  mrpln16_key2text(keys.front()) '---'  mrpln16_key2text(keys.back()) ' as '];
%         disp(str)
%         edge = [i, j]
%         if Debug & (Chr1 ~= Chr2)
%             disp('which is MR contraint')
%         end
        if nargin > 5 && strcmp(varargin{2}, 'RelativeTopologicalDistance')
            aij = abs(double(i0)-double(j0));
            A(i,j) = aij; % topological distance
            A(j,i) = aij;
        else
            A(i,j) = 1;
            A(j,i) = 1;
        end
        
    end
    
%     % Extract values for keys
%     value1 = values{r}.at(keys.at(0));
%     value2 = values{r}.at(keys.at(1));
%     
%     x1 = value1.x;
%     y1 = value1.y;
%     
%     x2 = value2.x;
%     y2 = value2.y;
%     
%     line([x1; x2], [y1; y2], 'Color',color_curr, 'LineWidth',line_width); %,'LineStyle',linestyle);
%    
end

if Debug > 1
    G = graph(A);
    %figure(333)
    
    figure(333);
    set(gcf, 'Color', [1 1 1], 'Position', [1924 522 1000 487]);
    subplot122 = subplot(1,2,2);
    hG=plot(G)
    % Set the remaining axes properties
    set(subplot122,'Color',...
        [1 1 1],'XColor',...
        [1 1 1],'YColor',...
        [1 1 1],'ZColor',...
        [1 1 1]);
    
%     hG.MarkerSize = 8;
%     hG.NodeColor = [1 0 0];
%     hG.EdgeColor = [0 0 1];
%     hG.LineWidth = 3;
    title('Topological space','FontSize',12);
    %      pause
%      clf(333)
%      clf(1)
end

 % Algo 4 with flag
% % for i1=1:length(path_struct_cell_r1)
% %     for i=1: numel(path_struct_cell_r1{i1}.keys)
% %         key = path_struct_cell_r1{i1}.keys(i);
% %         r_num = 2;
% %         robot_id = robots{r_num}.id;
% %         key0 = gtsam.symbol(robot_id, 0);
% %         RelKeys_mat = mrpln13c_GetInvolvedKeysPerFactorInGraph(output{i1,i2_curr}.graph_upd, key0);
% %         graph = mrpln13d_SelectAllFactors(output{i1,i2_curr}.graph_upd, output{i1,i2_curr}.values, key, [], RelKeys_mat, key0);
% %         optimizer = gtsam.LevenbergMarquardtOptimizer(graph, output{i1,i2_curr}.values);
% %         result = optimizer.optimizeSafely();
% %         marginals = gtsam.Marginals(graph, result);
% %         keyVector = gtsam.KeyVector;
% %         keyVector.push_back(key);
% %         cov = marginals.jointMarginalCovariance(keyVector);
% %     end
% % end


