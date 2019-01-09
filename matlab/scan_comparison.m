function [] = scan_comparison(path_to_bags, mathcing_indexes)
%SCANS_MATCHING_EXAMING Summary of this function goes here
%   Detailed explanation goes here

if(length(path_to_bags) == 0)
    disp('No bag files to read');
    return;
end

if(length(mathcing_indexes) ~= 2)
    disp('2 indexes are needed');
    return;
end

% Read first scan
bag = rosbag(path_to_bags{1});
scans_bag = select(bag, 'Topic', '/scan');
scan_msgs = readMessages(scans_bag);
scan_01 = scan_msgs{mathcing_indexes(1) + 1};
scan_01.RangeMax = 10;
readMessages(scans_bag);

index_bag = select(bag, 'Topic', '/index');
index_msgs = readMessages(index_bag);
index_01 = index_msgs{mathcing_indexes(1) + 1};

image_bag = select(bag, 'Topic', '/image');
image_msgs = readMessages(image_bag);
image_01 = image_msgs{mathcing_indexes(1) + 1};

% Read second scan
if(length(path_to_bags) > 1)
    % Read 2nd bagfile
    clear 'bag' 'scans_bag' 'scan_msgs' 'image_bag' 'image_msgs';
    bag = rosbag(path_to_bags{2});
    scans_bag = select(bag, 'Topic', '/scan');
    scan_msgs = readMessages(scans_bag);
    index_bag = select(bag, 'Topic', '/index');
    index_msgs = readMessages(index_bag);
    image_bag = select(bag, 'Topic', '/image');
    image_msgs = readMessages(image_bag);
end

scan_02 = scan_msgs{mathcing_indexes(2) + 1};
scan_02.RangeMax = 10;

index_02 = index_msgs{mathcing_indexes(2) + 1};
image_02 = image_msgs{mathcing_indexes(2) + 1};


figure
subplot(2,2,1);
plot(scan_01)
scan_01_title = sprintf('Scan %s', index_01.Data);
title(scan_01_title);

subplot(2,2,2); 
plot(scan_02)
scan_02_title = sprintf('Scan %s', index_02.Data);
title(scan_02_title);

subplot(2,2,3);
imshow(readImage(image_01))
image_01_title = sprintf('Image %s', index_01.Data);
title(image_01_title);

subplot(2,2,4);
imshow(readImage(image_02))
image_02_title = sprintf('Image %s', index_02.Data);
title(image_02_title);

lidarScan_01 = lidarScan(scan_01);
lidarScan_02 = lidarScan(scan_02);
[pose, stats] = matchScans(lidarScan_01, lidarScan_02)

end

