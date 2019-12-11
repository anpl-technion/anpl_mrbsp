% @author: Tal Regev

rosshutdown;
rosinit

clear('node_planner');

error_dynamic_percentage = 0.1;
delimiter = ',';
m_matlab_service_request_topic  = 'matlab_request';
m_matlab_service_response_topic = 'matlab_response';
%m_cpp_service_request_topic     = 'cpp_request';
%m_cpp_service_response_topic    = 'cpp_response';

node_planner    = robotics.ros.Node('node_planner');
matlab_sub      = robotics.ros.Subscriber(node_planner, m_matlab_service_request_topic,	'std_msgs/String');
matlab_pub      = robotics.ros.Publisher (node_planner, m_matlab_service_response_topic,'std_msgs/String');
%cpp_sub         = robotics.ros.Subscriber(node_planner, m_cpp_service_response_topic,	'std_msgs/String');
%cpp_pub         = robotics.ros.Publisher (node_planner, m_cpp_service_request_topic,    'std_msgs/String');

while true
    disp ('planner waiting');
    str             = receive(matlab_sub);

    string_array    = strsplit(str.Data, delimiter);

    request         = string_array{1};
    disp (['planner processing request: ' request]);

    switch request
        case 'plan'
            index_to_send = plan(string_array, error_dynamic_percentage);
        case 'plan_with_factors'
            index_to_send = plan_with_factors(string_array, error_dynamic_percentage);
    end
    msg = rosmessage(matlab_pub);
    msg.Data = num2str(index_to_send);
    matlab_pub.send(msg);

    disp (['Chosen Path: ' num2str(index_to_send)]);
end