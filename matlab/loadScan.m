function [ output_args ] = loadScan( scan_index)
%LOADSCAN Summary of this function goes here
%   Detailed explanation goes here

data = importdata(['scan_', scan_index, '.txt']);
figure('Name', scan_index)
scatter(data(:,1),data(:,2),'.')
axis equal
end

