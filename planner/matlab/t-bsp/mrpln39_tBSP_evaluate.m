function [s_VN_2_exact, t_VN_exact, J_exact, t_exact, s_VN_2, t_VN, s_ST, t_ST, s_VN_incr, t_VN_incr, bounds_Inf, bounds_J, LB_time] = mrpln39_tBSP_evaluate(string_array, session_num, save_plots, base_dir)

global Debug
Debug = 2;
global StateSpace
StateSpace = 'SE(2)'; % SE(3), topological BSP bounds currently support only SE(2), i.e. planar pose SLAM
load robots.mat % for compatibility reason between Matlab and Gazebo simulation
global SPATIAL_SEARCH_RADIUS
SPATIAL_SEARCH_RADIUS = Inf;
% sparsify ON = removing low informative edges, e.g. if a node = 5 and its neighbors
% are [1 2 4 6 30 31 32], after sparsification [1 4 6 32] will be left
sparsify = false;
NUM_RUNS = 10; % number of times to run the stopwatch (for precise timing increase it)

%if save_plots
    base_dir = [base_dir 'session_' sprintf('%02d', session_num) '/'];
    if ~exist([base_dir 'images'], 'dir') mkdir(base_dir, 'images'); end
    if ~exist([base_dir 'beliefs'], 'dir') mkdir(base_dir, 'beliefs'); end
%end

message_code   = str2num(string_array{1});
nr_beliefs     = str2num(string_array{2});
% extract posterior pdfs resulting from candidate actions
if message_code == 1,
    num_cand_actions = nr_beliefs-1;
     % prior VN entropy
    s_VN_incr_ = str2double(string_array{3}); % all actions share the same prior graph
    
    start = 2;
    head_offset = 4;
    % prior topological graph, needed for incremental t-bsp
    isam2_graph         = gtsam.NonlinearFactorGraph.string_deserialize(string_array{head_offset});
    isam2_values        = gtsam.Values.string_deserialize(string_array{head_offset+1});
    A_ = mrpln33_graphAdjacencyMatrix(isam2_graph, {isam2_values}, gtsam.Values, {robots{1}}); % single robot, Gazebo simulation
    G_ = graph(A_);
   
else
    num_cand_actions = nr_beliefs;
    start = 0;
    head_offset = 3;
end

s_VN_2_exact = zeros(num_cand_actions, 1);
s_VN_2 = zeros(num_cand_actions, 1);
s_ST = zeros(num_cand_actions, 1);
s_VN_incr = zeros(num_cand_actions, 1);
t_VN_exact = zeros(num_cand_actions, 1);
t_VN = zeros(num_cand_actions, 1);
t_ST = zeros(num_cand_actions, 1);
t_VN_incr = zeros(num_cand_actions, 1);
J_exact = zeros(num_cand_actions, 1);
t_exact = zeros(num_cand_actions, 1);
bounds_Inf = zeros(num_cand_actions, 5);
bounds_J = zeros(num_cand_actions, 5);
LB_time = zeros(num_cand_actions, 3);
node_dist2 = [];

for i=start:2:2*nr_beliefs-1
    disp(['*** Belief ' num2str(i/2)]);
    isam2_graph         = gtsam.NonlinearFactorGraph.string_deserialize(string_array{i+head_offset})
    isam2_values        = gtsam.Values.string_deserialize(string_array{i+head_offset+1})
    disp(['*** End of belief ' num2str(i/2)])
    
    %mrpln21_PlotFactors(isam2_graph, isam2_values)
    
    [A, hG] = mrpln33_graphAdjacencyMatrix(isam2_graph, {isam2_values}, gtsam.Values, {robots{1}}); % single robot, Gazebo simulation
    
    %pause, clf
    
    if ~sparsify
        G = graph(A)
        d = degree(G);
    else
        G = mrpln41_SparsifyGraph(graph(A))
        A = G.adjacency;
        d = degree(G);
    end
    
    %sigmas = [0.085; 0.085; 0.085; 0.1; 0.1; 0.1];
    sigmas = diag(isam2_graph.at(1).get_noiseModel.R^-1); % get the pairwise factor noise variances from the Gazebo simulation
    %sigmas = [0.035; 0.035; 0.035; 0.1; 0.1; 0.1]; % use this noise model instead
    %sigmas = [0.01; 0.01; 0.01; 0.1; 0.1; 0.1]; % use this noise model instead
    %sigmas = [0.001; 0.001; 0.001; 0.1; 0.1; 0.1]; % use this noise model instead
    %sigmas = [0.01; 0.01; 0.01; 1; 1; 1]; % use this noise model instead
    
    if strcmp(StateSpace, 'SE(2)')
        sigmas = sigmas([4,5,1]); % take only var_x, var_y, and var_theta
    end
    Omega = diag(sigmas)^-2;
    
    n = G.numnodes;
    act = i/2;
    
    t_total = 0;
    for nr = 1:NUM_RUNS
        t1 = tic;
        [tau, Lred] = LogNumSpanningTrees(A,d,n);
        % normalize signature
        s_ST(act) = 3/2*tau;% + (n-1)/2*(log(det(Omega))+log(2*pi*exp(1)));
        correction = (n-1)/2*(log(det(Omega))-size(Omega,1)*log(2*pi*exp(1))); % n-1 states are estimated, 1 is fixed, i.e. anchored
        % correction = 0;
        s_ST(act) = s_ST(act) + correction;
        t_total = t_total + toc(t1);
    end
    t_ST(act) = t_total/NUM_RUNS;
    
    
    % s_VN_2 calculated directly, from scratch
    t_total = 0;
    for nr = 1:NUM_RUNS
        t1 = tic;
        d = degree(G);
        [k,j] = find(triu(A));
%         n = size(A,1);
%         q = 0;
%                 for m = 1:nnz(A)/2
%                     q = q + 1/(d(k(m))*d(j(m)));
%                 end
%         
        q = sum(1./( d(k(1:nnz(A)/2)) .* d(j(1:nnz(A)/2)) ));
        %q = sum(T.edge_weights);
        
        % normalization of VN signature
        correction = 6*(n-1)/2*(log(det(Omega))-size(Omega,1)*log(2*pi*exp(1)))/(7*size(Omega,1));
        % turn off normalization of VN signature
        %correction = 0;
        
        % VN signature first order approx.
        % s_VN = 1 - 1/n - 1/n^2 * q; % bad approximation because argument of ln (lambda(k)/n) far
        % from linearization point x = 1
        
        % better first order approximation, closer to linearization point
        % s_VN = -sum(lambda(1:n-1)/n.*(log(lambda(1:n-1)/2)+log(2/n)));
        % aprox above knowing that lambda is in [0,2] and ln(x) ~ x - 1, around x =
        % 1, while for small x, x ln(x) goes to zero
        %s_VN(act) = log(n) - 1/n * q; % 1/2 - log(2/n) - 1/(2*n) * q;
        %s_VN(act) = s_VN(act) + correction;
        % 2. defininition s_VN = -sum(lambda(k)/2 * log(lambda(k)/2))
        s_VN_2(act) = n/2*log(2) - q + correction; % undirected edge considered only once in q, i.e. (i,j) not (j,i) so we don't divide by 2 
        t_total = t_total + toc(t1);
    end
    t_VN(act) = t_total/NUM_RUNS; % average time
    
    
    % VN signature exact
    t_total = 0;
    scale = diag(d)^(-1/2);
    for nr = 1:NUM_RUNS
        t1 = tic;
        %d = degree(G);
        %scale = diag(d)^(-1/2);
        L_norm = scale * laplacian(G) * scale;
        lambda = sort(eigs(L_norm, n));
        %    s_VN_exact(act) = 0;
        %     for k = 2:n % the first eigenvalue is zero and by convention 0*log 0 = 0
        %
        %         s_VN_exact(act) = s_VN_exact(act) - lambda(k)/n*log(lambda(k)/n);
        %     end
        % this for loop is equivalent to
        %s_VN_exact(act) = -sum(lambda(2:n)/n.*log(lambda(2:n)/n));
        %s_VN_exact(act) = s_VN_exact(act)+ correction;
        
        % 2. defininition s_VN = -sum(lambda(k)/2 * log(lambda(k)/2))
        s_VN_2_exact(act) = -sum(lambda(2:n)/2.*log(lambda(2:n)/2)) + correction;
        % relation between 1. and 2. definition of VN entropy is
        % s_VN_exact = 2/n * S_VN_2_exact - log(2/n)
        % i.e. scaling and adding a constant
        
        t_total = t_total + toc(t1);
        
    end
    t_VN_exact(act) = t_total/NUM_RUNS; % average time
    
    
    
    % s_VN_2 calculated incrementally
    
    % not measuring time for finding new edges Delta_E, calculating graph node
    % degrees and set of involved variables because this is input (Delta_E)
    % or incrementally updated (edge weights, node degrees, incidence matrices)
    % and does not affect O() complexity.
    % Here we have to recalculate it form scratch because of the way
    % information is sent to the Matlab from planner ROS side
    n_ = G_.numnodes;
    % expanded prior graph, added new states only
    G_0 = addnode(G_, n - n_);
    G_aux = graph(G.adjacency-G_0.adjacency);
    % Delta_E = new edges 
    Delta_E = G_aux.Edges.EndNodes;   
    T = updateTopologicalDataStructures(G, G_, Delta_E);
    
    t_total = 0;
    
    edges_updated = sparse([], [], [], 1, T.m);
    for nr = 1:NUM_RUNS
            
        t1 = tic;
        for v = 1:length(T.V_I)
            edges_touched = find(T.Inc(T.V_I(v),:));
            edges_updated(edges_touched) = true;
        end
        
        % Markov blanket MB is determined by edges_updated == true that are in G_
        idxMBedges = find(edges_updated(1:T.m_));
        
        dq = sum(T.edge_weights(idxMBedges) - T.edge_weights_(idxMBedges)) + sum(T.edge_weights(T.m_+1:end));
        % s_VN(G) = T.n_*log(2)/2 - sum(edge_weights_) + (T.n-T.n_)*log(2)/2-q
        %         = T.n*log(2)/2-sum(edge_weights)
        %T.n_*log(2)/2 - sum(T.edge_weights_) + (T.n-T.n_)*log(2)/2-q
        %T.n*log(2)/2-sum(T.edge_weights)
        
        % here correction is done assuming unnormalized s_VN_incr_, otherwise correction term should be (n-n_)*(log(det(Omega))-size(Omega,1)*log(2*pi*exp(1)))/2*6/(7 kappa)
        s_VN_incr(act) = s_VN_incr_ + (T.n-T.n_)*log(2)/2-dq + correction; 
        
        t_total = t_total + toc(t1);
    end
    t_VN_incr(act) = t_total/NUM_RUNS;
    
    %[k,j] = find(MB);
    highlight(hG,T.V_I,'NodeColor','r')
    [k,j] = find(T.Inc(:,idxMBedges));
    highlight(hG,k(1:2:end),k(2:2:end),'EdgeColor','r'); % column-wise storage of sparse matrix!
    highlight(hG,Delta_E(:,1),Delta_E(:,2),'EdgeColor','g'); % new edges
    
    % standard BSP
    
    
    % MLE estimator
    % remove prior on the starting pose and anchor the first pose
    isam2_graph_ML = gtsam.NonlinearFactorGraph;
    key_0 = gtsam.symbol(robots{1}.id, 0);
    first = isam2_values.at(key_0);
    
    if strcmp(StateSpace, 'SE(3)')
        
        isam2_graph_ML.add(gtsam.NonlinearEqualityPose3(key_0, first));
        
        % small prior on the first pose, MAP
        %     priorMean = first;
        %     priorNoise = gtsam.noiseModel.Diagonal.Sigmas(1e-1*[0.085; 0.085; 0.085; 0.1; 0.1; 0.1]);
        %     isam2_graph_ML.add(gtsam.PriorFactorPose3(key_0, priorMean, priorNoise)); % add directly to graph
    else % 'SE(2)'
        
        R = first.rotation.matrix;
        fi = rotm2eul(R);
        yaw = fi(1);
        t = first.translation;
        
        isam2_graph_ML.add(gtsam.NonlinearEqualityPose2(key_0, gtsam.Pose2(t.x, t.y, yaw)));
        
%         % small prior on the first pose, MAP
%         priorMean = gtsam.Pose2(t.x, t.y, yaw);
%         priorNoise = gtsam.noiseModel.Diagonal.Sigmas(1e-2*[0.1; 0.1; 0.085]);
%         isam2_graph_ML.add(gtsam.PriorFactorPose2(key_0, priorMean, priorNoise)); % add directly to graph
        
        pairwiseFactorsNoiseModel = gtsam.noiseModel.Diagonal.Sigmas(sigmas); %  x, y, yaw
    end
    % copy rest of the relative pose factors
    for fid = 0:isam2_graph.size-1
        
        if isa(class(isam2_graph.at(fid)), 'gtsam.PriorFactorPose3')
            continue; % skip prior factor
        end
        relpose = isam2_graph.at(fid);
        
        % if the factor exist in sparsified graph, add it to the new FG
        if A(gtsam.mrsymbolIndex(relpose.keys.front)+1, gtsam.mrsymbolIndex(relpose.keys.back)+1) == 1
               
            if strcmp(StateSpace, 'SE(3)')
                isam2_graph_ML.add(relpose);
            else
                
                R = relpose.measured.rotation.matrix;
                fi = rotm2eul(R);
                yaw = fi(1);
                t = relpose.measured.translation;
                disp(['A' num2str(gtsam.mrsymbolIndex(relpose.keys.front)) '- A' num2str(gtsam.mrsymbolIndex(relpose.keys.back))])
                disp([norm(t.vector) abs(yaw)])
                %factor_keys_str = mrpln02_getStringFromFactor( relpose )
                isam2_graph_ML.add(gtsam.BetweenFactorPose2(relpose.keys.front, relpose.keys.back, gtsam.Pose2(t.x, t.y, yaw), pairwiseFactorsNoiseModel));
            end
        end

    end
    
    if strcmp(StateSpace, 'SE(2)')
        isam2_values_ML = gtsam.Values;
        for vid=0:isam2_values.size-1
            key = gtsam.symbol(robots{1}.id, vid);
            pose = isam2_values.at(key);
            R = pose.rotation.matrix;
            fi = rotm2eul(R);
            yaw = fi(1);
            t = pose.translation;
            isam2_values_ML.insert(key, gtsam.Pose2(t.x, t.y, yaw));
            %subplot(1,2,1), quiver(t.x,t.y,cos(yaw),sin(yaw), 'g', 'LineWidth', 2)
        end
        isam2_values = isam2_values_ML;
    end
    
    % MAP estimator by default if this line is commented
    isam2_graph = isam2_graph_ML;
    
 
    
    %t1 = tic;
    t_total = 0;
    for nr = 1:NUM_RUNS
        [J_exact(act) t_exact(act) X_est] = mrpln40_CalcInfoTheoreticObjFunc(isam2_graph, isam2_values);
        t_total = t_total + t_exact(act);
    end
    %t_exact(act) = toc(t1)
    t_exact(act) = t_total/NUM_RUNS;
    %pause
    
    gfg = isam2_graph.linearize(X_est);
    %m = isam2_graph.size -1; % skip the prior factor, and 0-1
    H = gfg.jacobian;
    %A = A(13:end, :);
%     Sigma = []; whitening already included in A
%     for i=3:isam2_graph.size
%         Sigma = blkdiag(Sigma, Omega^-1);
%     end

    % FIM at estimated state
    Lambda = H'*H;
    %logDetLambdaML =  log(det(Lambda));  % causes numerical problems
    logDetLambdaML = sum(log(eigs(Lambda, size(Lambda,1))));
    
    
    
    
    % determine Psi scale from node distances
    psi_scale = 0;
    for n1=2:n
        n_i = G.neighbors(n1);
        dist2 = 0;
        key_i = gtsam.symbol(robots{1}.id, n1-1); % -1 because of C-style indexing of vars that start from 0
        pose_i = isam2_values.at(key_i);
        for n2=1:length(n_i)
            key_j = gtsam.symbol(robots{1}.id, n_i(n2)-1);
            pose_j = isam2_values.at(key_j);
            %z_ij_est = [pose_j.x-pose_i.x pose_j.y-pose_i.y pose_j.z-pose_i.z];
            z_ij_est = [pose_j.x-pose_i.x pose_j.y-pose_i.y]; % assumption: ML observations
            dist2 = dist2 + norm(z_ij_est)^2;
        end
        node_dist2 = [node_dist2; n1 dist2];
        psi_scale = max(psi_scale, dist2);
    end
    
    %psi_scale = 10^2*max(G.degree); % consider spatial search radius in LC, informative condition
    
    % TODO calculate FIM at true value when ground truth will be available
    
    logDetInfGTSAMMarginals = -2*J_exact(act)+(n-1)*size(Omega,1)*log(2*pi*exp(1));
    bounds_Inf(act,:) = [BoundsOfLogDetInformationMatrix(Lred, tau, sigmas, psi_scale) logDetInfGTSAMMarginals logDetLambdaML];
    [bounds, lb_time] = BoundsOfEntropy(Lred, tau, sigmas, psi_scale);
    bounds_J(act,:) = [bounds J_exact(act)]
    LB_time(act, :) = lb_time;
    
    filename = [base_dir '/beliefs/graph_' sprintf('%02d', act)];
    figure(333);
    %set(gcf, 'Position', [2855 388 745 331]);
    set(gcf, 'Color', [1 1 1], 'Position', [1924 522 1000 487]);
    gcf; hs = subplot(1,2,1); hold on, plotStartAndGoals(session_num)
    %axis([16.5 35 -6 15])
    axis tight; axis equal;
    xl=xlim; yl = ylim;
%     xlim([xl(1) xl(2)+1]) % space for text labels
    ylim([yl(1) yl(2)+0.5]) % space for the title
    if save_plots, 
        savefig(gcf, filename);
        F(act) = getframe(gcf);
    end
    clf(333);
    %clf(1)
    %clf(2)

end

%movie(F,1,1) % 1x, 1 fps
close(333)


h3=figure(3), title('Joint entropy of posterior beliefs and its bounds')
%bounds_bar = bar(bounds_Inf(:,1:end-1),'DisplayName','log(det(Inf))');
%legend('lower bound, topological metric', 'upper bound exact', 'upper bound Hadamard', 'log(det(I(X_{est})))')
[~, I] = sort(bounds_J(:,end));
bounds_J_sorted = bounds_J(I,:)
%bounds_bar = bar(bounds_J_sorted,'DisplayName','J');
%set(bounds_bar(2), 'FaceColor',[0.749019622802734 0.749019622802734 0]);
plot(bounds_J_sorted(:,1),'-.', 'Linewidth', 2), hold on
%plot(bounds_J_sorted(:,2),'-.', 'Linewidth', 2)
plot(bounds_J_sorted(:,3),'-.', 'Linewidth', 2)
plot(bounds_J_sorted(:,4),'-.', 'Linewidth', 2)
plot(bounds_J_sorted(:,5),'go-', 'Linewidth', 2)
%legend('lower bound exact', 'lower bound Hadamard', 'upper bound, topological metric', 'J_{est}')
J_sel = bounds_J_sorted(1,4);
J_min_LB = min(bounds_J(:,3));
plot([0.5 3.5], [J_sel J_sel], 'k-', 'LineWidth', 2)
plot([0.5 3.5], [J_min_LB J_min_LB], 'k-', 'LineWidth', 2)
% Create doublearrow
[X1 Y1] = ds2nfu(gca, 3.3, J_sel);
[X2 Y2] = ds2nfu(gca, 3.3, J_min_LB);
annotation(gcf,'doublearrow',[X1 X2],...
    [Y1 Y2], 'HeadSize', 6);
text(3.5, min(bounds_J(:,3))+(bounds_J_sorted(1,4)-min(bounds_J(:,3)))/2, '\DeltaJ_{max}', 'FontSize',12)
xlabel({'actions'});
ylabel({'joint entropy [nats]'})
legend('lower bound exact', 'lower bound Hadamard', 'upper bound, topological metric', 'J_{est}')
%legend('lower bound exact', 'lower bound SymmPoly', 'lower bound Hadamard', 'upper bound, neg. topological metric', 'J_{est}', 'Location', 'Northwest')
if sparsify, SPARS ='ON', else SPARS ='OFF', end;
title(['\sigma_{\theta}/\sigma_{p} = ' num2str(sigmas(3)/sigmas(1)) ',    SPARS:' SPARS])

if save_plots
        set(h3,'PaperPositionMode','auto')
        set(h3, 'Position',  [2782 312 572 802])
        filename = [base_dir 'images/entropy_bounds_' sprintf('%02d', session_num)]
        %print(filename, '-dpng','-r0')
        print(h3, '-painters','-dpdf',filename)
        system(['pdfcrop ' filename]);
        print(h3, '-painters','-depsc',filename)
        savefig(h3, filename)
        
        vidObj = VideoWriter([base_dir 'metric_top_' sprintf('%02d', session_num)]);
        vidObj.FrameRate = 1;
        open(vidObj);
        writeVideo(vidObj,F);
        close(vidObj);
end

% tBSP vs info-theor. cost
h1 = figure(1);
[J_e,I] = sort(J_exact);
yyaxis left
% 1.def
%plot(J_e, s_VN_exact(I), 'b.--', 'MarkerSize', 15), hold on
%plot(J_e, s_VN(I), 'g.--', 'MarkerSize', 15)
% 2. def.
plot(J_e, s_VN_2_exact(I), 'b.--', 'MarkerSize', 15), hold on
plot(J_e, s_VN_2(I), 'g.--', 'MarkerSize', 15)
xlabel('joint entropy [nats]')
ylabel('VN topological metrics')
yyaxis right
plot(J_e, s_ST(I), '.--', 'MarkerSize', 15, 'Color',   [0.850,  0.325,  0.098])
 %plot(J_e, s_VN_2(I), 'y.--', 'MarkerSize', 20)
 %plot(J_e, s_VN_2_exact(I), 'c.--', 'MarkerSize', 20)
legend('s_{VNexact}', 's_{VN}', 's_{ST}')
set(legend,'FontSize',12);
ylabel('ST topological metric')
title('tBSP vs info-theor. cost')
if save_plots
        set(h1,'PaperPositionMode','auto')
        set(h1, 'Position',  [2782 312 460 509])
        filename = [base_dir 'images/corr_session_' sprintf('%02d', session_num)]
        %print(filename, '-dpng','-r0')
        print(h1, '-painters','-dpdf',filename)
        system(['pdfcrop ' filename]);
        print(h1, '-painters','-depsc',filename)
        savefig(h1, filename)
        %MovieFrames(i) = getframe(gcf);
end




% time complexity of tBSP
h2 = figure(2);
actions = [1:num_cand_actions];
plot(actions, 1e3*t_VN_exact, 'b.--', ...
    actions, 1e3*t_VN, 'g.--', ...
    actions, 1e3*t_exact/10, 'm.--', 'MarkerSize', 20), hold on;
plot(actions, 1e3*t_ST, '.--', 'Color',   [0.850,  0.325,  0.098], 'MarkerSize', 20)
plot(actions, 1e3*t_VN_incr, '.--', 'Color',   [0, 1, 1], 'MarkerSize', 20, 'LineWidth', 2)
legend('t_{VNexact}', 't_{VN}', 't_{standard}/10', 't_{ST}', 't_{VN}_{incr}')
set(legend,'FontSize',12);
xlabel('actions')
ylabel('time [ms]')
title('time complexity of tBSP vs standard BSP')
if save_plots
        set(h2,'PaperPositionMode','auto')
        filename = [base_dir 'images/time_session_' sprintf('%02d', session_num)]
        %print(filename, '-dpng','-r0')
        print(h2, '-painters','-dpdf',filename)
        system(['pdfcrop ' filename]);
        print(h2, '-painters','-depsc',filename)
        savefig(h2, filename)
        %MovieFrames(i) = getframe(gcf);


    %save([base_dir 'planning_session_' sprintf('%02d', session_num) '.mat'])
end

%save([base_dir 'planning_session_' sprintf('%02d', session_num) '.mat']) %recording

% syms x
% f = log(x);
% f1 = x-1;
% f2 = x-1-(x-1)^2;
%figure(33); fplot(f, [0,2]), hold on;
%fplot(f1, [0,2])
%fplot(f2, [0,2])
%fplot(x*f, [0,2])
%legend


%save(['./icra19/' 'session_' sprintf('%02d', session_num) 'Th:' num2str(sigmas(3)/sigmas(1)) 'Spars:' num2str(sparsify) '.mat'])


end


% *************************************************************************
% auxiliary functions
% *************************************************************************

function [s, L] = LogNumSpanningTrees(A, d, n)

% K = Kirchoff matrix, Laplacian or admittance matrix
% D = diag(vertex degrees). The degree of a graph vertex v of a graph is the number of graph edges which touch v
% A = adjacency matrix
D = sparse(1:n, 1:n, d, n, n);
K = D - A;

% By anchoring arbitary vertex, we define reduced Laplacian L
% or Dirichlet
L = K(2:end, 2:end); % let the anchor be the first vertex

% The matrix tree theorem, also called Kirchhoff's matrix-tree theorem,
% states that the number of nonidentical spanning trees of a graph G, t(G), is equal to any cofactor
% of its Laplacian matrix.
% t(G) = det(L) = 1/n prod{i=2 to n} Lambda_i(K), where
% Lambda_i(K) is i-th eigenvalue of K such that
% Lambda_1 <=  Lambda_2 <= ... <= Lambda_n

%s = sum(log(eigs(L, size(L,1))));
R = chol(L); % Matlab built-in function which calls C implementation from
% CHOLMOD library

s = 2*sum(log(diag(R)));
end

function bounds = BoundsOfLogDetInformationMatrix(Lred, tau, sigmas, scale)

n_1 = size(Lred,1); % number of estimated states = n-1, n = |V|
Omega = diag(sigmas)^-2; % process noise
% Psi = (sigmas(1)/sigmas(4))^2*scale; % scale should be calculated from distances between nodes
% 2D SLAM
Psi = (sigmas(3)/sigmas(1))^2*scale; % scale should be calculated from distances between nodes

bounds(1) = 3*tau + n_1*log(det(Omega)); % lower bound
bounds(2) = bounds(1) + log(det(eye(size(Lred))+Lred^-1*Psi)); % upper bound exact
bounds(3) = bounds(1) + log(prod(diag(Lred + Psi*eye(size(Lred)))))-tau; % upper bound Hadamard

end

function [bounds, time] = BoundsOfEntropy(Lred, tau, sigmas, scale)

n_1 = size(Lred,1); % number of estimated states = n-1, n = |V|
Omega = diag(sigmas)^-2; % process noise
% Psi = (sigmas(1)/sigmas(4))^2*scale; % scale should be calculated from distances between nodes
% 2D SLAM
Psi = (sigmas(3)/sigmas(1))^2*scale; % scale should be calculated from distances between nodes

bounds(4) = n_1/2*(size(Omega,1)*log(2*pi*exp(1))-log(det(Omega))) - 3/2*tau; % upper bound, topological metric
%t0 = tic; bounds(1) = bounds(4) - 1/2*sum(log(eig(eye(size(Lred))+Lred^-1*Psi))); time(1)= toc(t0)% lower bound exact
t0 = tic; bounds(1) = bounds(4) - 1/2*( sum(log(eig(Lred + Psi*eye(size(Lred))))) - tau); time(1)= toc(t0)% lower bound exact
%bounds(3) = bounds(4) - 1/2*( sum(log(diag(Lred + Psi*eye(size(Lred)))))-tau ); % lower bound Hadamard
t0 = tic; bounds(3) = bounds(4) - 1/2*( sum(log(diag(Lred) + Psi))-tau ); time(3)= toc(t0) % lower bound Hadamard
% bound based on elementary symmetric polynomials
% t0 = tic;
% polysum = 0;
% degrees = diag(Lred);
% degSorted = sort(degrees, 'descend');
% for k=1:n_1
%     %polysum = polysum + nchoosek(n_1,n_1-k)*Psi^k*sum(prod(nchoosek(degrees,n_1-k),2)/tau)*tau^(n_1-k)/exp(tau); % TODO precalculate sym. polynomials
%     polysum = polysum + nchoosek(n_1, n_1-k)*Psi^k*prod(degSorted(1:n_1-k)/tau) * tau^(n_1-k)/exp(tau); % soon out of machine numerical range
% end
% % 
% bounds(2) = bounds(4)-1/2*log(1+polysum); % lower bound symmetric poly
% %bounds(2) = bounds(3)-1/2*log(1+exp(n_1*(Psi+max(diag(Lred)))-tau));
% time(2) = toc(t0);
end


function plotStartAndGoals(session_num)


try
    % Java YAML parser for MATLAB: https://github.com/ewiger/yamlmatlab
    config_dir = evalin('base', 'config_dir');
    %start = yaml.ReadYaml([config_dir 'pioneer_robot_A.yaml']);
    start = yaml.ReadYaml([config_dir 'gazebo_robot_A.yaml']);
    start = struct2array(start.init_pose.position);
    %goals = yaml.ReadYaml([config_dir 'pioneer_robot_A_goals.yaml']);
    goals = yaml.ReadYaml([config_dir 'gazebo_Robot_A_goals.yaml']);
    goals = reshape(cell2mat(struct2array(goals.goals)), 3,[])';
catch
    % for older simulations or if parser not avail. input manualy start and goals
    warning('Start and goals are predefined.')
    start = [19.9, -0.1, 0];
    goals = [18.0, 5.0, 0.0;
        26.0, 8.0, 0.0;
        25.0, -2.95, 0.0];
end


%figure(fh)
plot(start(1), start(2), 'r<', 'MarkerFaceColor', 'red', 'MarkerSize', 10)
text(start(1), start(2)-1, ['START'], 'FontWeight','bold', 'FontSize', 8)

for i = 1:min(session_num, size(goals,1))
    plot(goals(i,1), goals(i,2), 'gh', 'MarkerFaceColor', 'green', 'MarkerSize', 10)
    text(goals(i,1)+1, goals(i,2), ['GOAL' num2str(i)], 'FontWeight','bold', 'Color', 'yellow', 'BackgroundColor', 'black', 'FontSize', 8)
end


end

