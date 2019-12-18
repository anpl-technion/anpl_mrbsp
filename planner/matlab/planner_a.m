% @author: Andrej Kitanov
% @author: Evgeny Koretsky
clear 
close all
clc

rosshutdown;
rosinit

clear('matlab_planner');

delimiter = ',';
m_matlab_service_request_topic  = 'matlab_request';
m_matlab_service_response_topic = 'matlab_response';

matlab_planner    = robotics.ros.Node('matlab_planner');
matlab_sub      = robotics.ros.Subscriber(matlab_planner, m_matlab_service_request_topic,	'std_msgs/String');
matlab_pub      = robotics.ros.Publisher (matlab_planner, m_matlab_service_response_topic,'std_msgs/UInt32');

session_num = 1;
save_results = false;
if has(rosparam, '/logger/loggerPath') % output results go here
    base_dir = [rosparam('get','/logger/loggerPath') '/t-bsp/'];
else
    base_dir = ['~/.ros/t-bsp/' datestr(now) '/']; 
end
if has(rosparam, '/scenario_folder') % configs are read from here
    config_dir = [rosparam('get','/scenario_folder') '/config_files/'];
else
    config_dir = './';
end


while true
    disp ('planner waiting');
    str             = receive(matlab_sub);
    close all
    
    string_array    = strsplit(str.Data, delimiter);
    % Matlab message codes defined in planner config.h
    % enum MATLAB_MSG_CODES {MATLAB_SEND_POSTERIOR, MATLAB_SEND_PRIOR_AND_POSTERIOR};
    message_code   = str2num(string_array{1});
    size         = str2num(string_array{2});
    if message_code == 1
        %s_VN_incr = str2num(string_array{3});
        head_offset = 4;
    else
        head_offset = 3;
    end
    switch message_code
        case 0
            disp (['planner received: ' string_array{2} ' posterior beliefs resulting from different candiate actions.']);
        case 1
            disp (['planner received: ' string_array{2} ' beliefs. Belief 0 is MR prior p[Xk|Hk]. The rest are posteriors resulting from different candiate actions.']);
        otherwise
            warning (['planner received: custom message, code ' string_array{1} 'Check parsing.'])
    end
    
    for i=0:2:2*size-1
        disp(['*** Belief ' num2str(i/2)]); 
        isam2_graph         = gtsam.NonlinearFactorGraph.string_deserialize(string_array{i+head_offset})
        isam2_values        = gtsam.Values.string_deserialize(string_array{i+head_offset+1})
        disp(['*** End of belief ' num2str(i/2)])
        
%         if (message_code == 1 && i == 0)
%             continue; % skip prior
%         end
%         
        
    end

    % run your matlab code here...
    %
    % and send back the result

    index_to_send = input('Action Number (1,2,...): ');
%     index_to_send = 1;
    msg = rosmessage(matlab_pub);
    msg.Data = int32(index_to_send)-1; % first maximum, index in C-style
    matlab_pub.send(msg);

    disp (['Chosen Path: ' num2str(index_to_send)]);
    
    session_num = session_num + 1
        
end