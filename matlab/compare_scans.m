clear;
%close all;
clc;

scenario = 'mr_centralized_laser';
results_folder = 'no_lc_no_mr';

path_to_bags = strings;

bag_name_01 = ('Robot_A_keyframe_bag.bag');
path_to_bags(1) = [getenv('HOME') filesep 'ANPL' filesep 'code' filesep 'mrbsp_ws' filesep 'src'...
    filesep 'mrbsp_ros' filesep 'mrbsp_scenarios' filesep 'scenarios' filesep scenario...
    filesep 'results' filesep results_folder filesep bag_name_01];
%{
bag_name_02 = 'Robot_B_keyframe_bag.bag';
path_to_bags(2) = [getenv('HOME') filesep 'ANPL' filesep 'code' filesep 'mrbsp_ws' filesep 'src' ...
    filesep 'mrbsp_ros' filesep 'mrbsp_scenarios' filesep 'scenarios' filesep scenario ...
    filesep 'results' filesep results_folder filesep bag_name_02];
%}
mathcing_indexes = [37,38];

scan_comparison(path_to_bags, mathcing_indexes)