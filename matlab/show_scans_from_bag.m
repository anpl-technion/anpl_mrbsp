function [] = show_scans_from_bag(path_to_bag, msg_type, msg_topic)
%SHOW_SCANS_FROM_BAG Summary of this function goes here
%   Detailed explanation goes here

bag = rosbag(path_to_bag);
grid on;

switch(msg_type)
    case 'scan'
        bag_select = select(bag, 'Topic', msg_topic);
        scans_msgs = readMessages(bag_select);
        
        for i = 1 : length(scans_msgs)
            scan = scans_msgs{i};
            scan.RangeMax = 10;
            plot(scan);
            title_str = sprintf('Scan #%d, Msg seq #%d', i, scan.Header.Seq);
            title(title_str);
            pause(0.1);
        end
    case 'image'
    otherwise
        fprint('Unrecognized or empty message type...');
end

end

