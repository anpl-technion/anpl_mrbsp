close all
clear all
clc


path_to_dataset = '~/ANPL/infrastructure/mr_laser_rosbag/build/scans';
addpath(path_to_dataset);

for i = 0 : 134
    scan_index = ['A',int2str(i)];
    loadScan(scan_index);
    pause(1);
end
close all;