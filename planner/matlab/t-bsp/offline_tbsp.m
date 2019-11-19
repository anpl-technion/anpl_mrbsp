anpl_mrbsp_dir = '/home/andrej/ANPL/infrastructure/opensource_ws/src/anpl_mrbsp';

config_dir = [anpl_mrbsp_dir '/scenarios/topological_mr_active/config_files/'];
exp_base_folder = [anpl_mrbsp_dir '/scenarios/topological_mr_active/results/hyundai_demo/pioneer_experiment/t-bsp/'];

t_per_act = [];


for session_num = 1:3
    
    mat_filename = [exp_base_folder 'session_' sprintf('%02d', session_num) '/planning_session_' sprintf('%02d', session_num) '.mat']
    
    load(mat_filename);
    save_plots = 0;
    
    [s_VN_exact, t_VN_exact, J_exact, t_exact, s_VN, t_VN, s_ST, t_ST, s_VN_incr, t_VN_incr, ~, bounds_J] = mrpln39_tBSP_evaluate(string_array, session_num, save_plots, exp_base_folder);
    
    t_per_act = [t_per_act; mean([t_VN t_VN_incr t_ST t_VN_exact t_exact/10])];
    
    [~,idx_opt] = min(J_exact); % standard bsp
    %[~,idx_sel] = max(s_ST); % topological bsp ST
    [~,idx_sel] = max(s_VN); % topological bsp VN
    
    % action consistency
    if (idx_opt == idx_sel)
        act_consistent(session_num) = 1;
    else
        act_consistent(session_num) = 0;
    end
    
    % action elimination based on topological bounds
    %     session{session_num}.bounds_J = bounds_J; % LB 0 HB UB J
    potential_subset = find(bounds_J(:,3) < bounds_J(idx_sel,4));
    percent_elim(session_num) = 1 - (length(potential_subset)-1)/length(s_VN);
    
    
    % action diversity
    figure(456), subplot(3,1,3)
    title('Normalized entropy'),
    xlabel('planning sessions')
    ylabel('J/max(|J|)'), hold on
    plot(repmat(session_num, 1, length(J_exact)), J_exact/max(abs(J_exact)), '.', 'MarkerSize', 10)
    
    
    disp('Press ENTER to continue...')
    pause
    close ([1:3])
end

figure
tbar = bar(1e3*t_per_act, 'stacked'), legend({'t_{VN}', 't_{VNincr}', 't_{ST}', 't_{VNexact}', 't_{standard}/10'})
set(tbar(1),'DisplayName','t_{VN}',...
    'FaceColor',[0.301960784313725 0.749019607843137 0.929411764705882]);
set(tbar(2),'DisplayName','t_{VNincr}','FaceColor',[1 1 0],'BarWidth',0.4,...
    'BarLayout','stacked');
set(tbar(3),'DisplayName','t_{ST}');
set(tbar(4),'DisplayName','t_{VNexact}');
set(tbar(5),'DisplayName','t_{standard}/10');
xlabel('planning sessions')
ylabel('time [ms]')

figure(456)
subplot(3,1,1), plot(act_consistent, 'r')
xlabel('planning sessions')
title('Action consistency')

subplot(3,1,2), plot(percent_elim)
xlabel('planning sessions')
title('Percent of suboptimal actions detected by t-bsp Hadamard bound LB_{HAD}')

%save([exp_base_folder 'processed.mat'], 't_per_act', 'act_consistent', 'percent_elim')


