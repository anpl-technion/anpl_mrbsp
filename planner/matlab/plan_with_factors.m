function [ index_to_send ] = plan_with_factors(string_array, error_dynamic_percentage)
% @author: Tal Regev

    paremater_size = 9;

    robot_location      = gtsam.Pose3.string_deserialize(string_array{2});
    next_goal           = gtsam.Pose3.string_deserialize(string_array{3});
    paths_actions       = gtsam.ActionsPaths.string_deserialize(string_array{4});
    isam2_values        = gtsam.Values.string_deserialize(string_array{5});
    isam2_graph         = gtsam.NonlinearFactorGraph.string_deserialize(string_array{6});
    current_index       = str2double(string_array{7});
    robot_id            = string_array{8};
    is_along_the_path   = str2double(string_array{9});

belief = zeros(1,paths_actions.size);

if (is_along_the_path)
    sum = paremater_size;   
    for path_number=0:(paths_actions.size-1)
        belief_size = (paths_actions.at(path_number).size);          
        index_of_last_isam = sum + belief_size*2 - 1;
        
        isam2_values        = gtsam.Values.string_deserialize(string_array{index_of_last_isam});
        isam2_graph         = gtsam.NonlinearFactorGraph.string_deserialize(string_array{index_of_last_isam + 1});
        
        isam2 = gtsam.ISAM2;
        isam2.update(isam2_graph, isam2_values);
        
        current_key  = gtsam.symbol(robot_id, current_index - 1 + belief_size);
        d = det(isam2.marginalCovariance(current_key));
        belief(path_number + 1) = d;
        sum = sum + belief_size*2;
        
    end
else
    for path_number=0:(paths_actions.size-1)
        belief_size = (paths_actions.at(path_number).size);
        index_of_isam = paremater_size + 1 + path_number*2;
               
        isam2_values        = gtsam.Values.string_deserialize(string_array{index_of_isam});
        isam2_graph         = gtsam.NonlinearFactorGraph.string_deserialize(string_array{index_of_isam + 1});
        
        isam2 = gtsam.ISAM2;
        isam2.update(isam2_graph, isam2_values);
        
        current_key  = gtsam.symbol(robot_id, current_index - 1 + belief_size);
        d = det(isam2.marginalCovariance(current_key));
        belief(path_number + 1) = d;       
	end
end

    index_to_send = -1;
    [~, index_to_send] = min(belief);
    index_to_send = index_to_send - 1;

end