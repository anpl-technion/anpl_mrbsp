function [ index_to_send ] = plan( string_array, error_dynamic_percentage )
% @author: Tal Regev

    robot_location  = gtsam.Pose3.string_deserialize(string_array{2});
    next_goal       = gtsam.Pose3.string_deserialize(string_array{3});
    paths_actions   = gtsam.ActionsPaths.string_deserialize(string_array{4});
    isam2_values    = gtsam.Values.string_deserialize(string_array{5});
    isam2_graph     = gtsam.NonlinearFactorGraph.string_deserialize(string_array{6});
    current_index   = str2double(string_array{7});
    robot_id        = string_array{8};
    
    isam2 = gtsam.ISAM2;
    isam2.update(isam2_graph, isam2_values);
    
    belief = zeros(1,paths_actions.size);
    
    for path_number=0:(paths_actions.size-1)
        new_factors = gtsam.NonlinearFactorGraph;
        new_values  = gtsam.Values;
        previous_idx = current_index - 1;
        for action_number=0:(paths_actions.at(path_number).size-1)
            action      = paths_actions.at(path_number).at(action_number);
            translation = action.translation;
            rotation    = action.rotation;
            roll        = rotation.roll  * error_dynamic_percentage;
            pitch       = rotation.pitch * error_dynamic_percentage;
            yaw         = rotation.yaw   * error_dynamic_percentage;
            x           = translation.x  * error_dynamic_percentage;
            y           = translation.y  * error_dynamic_percentage;
            z           = translation.z  * error_dynamic_percentage;
            
            noise_model  = gtsam.noiseModel.Diagonal.Sigmas([roll pitch yaw x y z]');
            previous_key = gtsam.symbol(robot_id, previous_idx);
            current_key  = gtsam.symbol(robot_id, previous_idx + 1);
            factor       = gtsam.BetweenFactorPose3(previous_key, current_key, action, noise_model);
            new_factors.push_back(factor);
            new_values.insert(current_key, action);
            previous_idx  = previous_idx + 1;
        end
        new_isam = gtsam.ISAM2(isam2);
        new_isam.update(new_factors, new_values);
        d = det(new_isam.marginalCovariance(current_key));
        belief(path_number + 1) = d;
    end
    
    index_to_send = -1;
    [~, index_to_send] = min(belief);
    index_to_send = index_to_send - 1;
end

