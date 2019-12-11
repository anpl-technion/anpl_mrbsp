% @author: Evgeny Koretsky
%% Clearing and rosinit
clear all
close all
clc
rosshutdown;
rosinit
clear('matlab_planner');
addpath('/home/evgenykoretsky/Desktop/ORB_Analysis/Functions/');
%% The main string array will be parsed by the delimeter
delimiter = ',';

%% Initializing matlab topic, subsriciber and publisher
m_matlab_service_request_topic  = 'matlab_request';
m_matlab_service_response_topic = 'matlab_response';

matlab_planner    = robotics.ros.Node('matlab_planner');
matlab_sub      = robotics.ros.Subscriber(matlab_planner, m_matlab_service_request_topic,	'std_msgs/String');
matlab_pub      = robotics.ros.Publisher (matlab_planner, m_matlab_service_response_topic,'std_msgs/UInt32');

%% Main loop
session_num = 1;
while true
    %rosparam set /waypoint TRUE
    
    disp ('planner waiting');
    str             = receive(matlab_sub);
    string_array    = strsplit(str.Data, delimiter, 'CollapseDelimiters', false);
    string_array = string_array'; save('string_array.mat','string_array');
    % 1 message_code
    
    %     load('string_array.mat');
%%    
    % 1 number of cells in message  including this one
%     message_code = str2double(string_array{1});
    % 2 all Fac
    msg.all.all_graph = gtsam.NonlinearFactorGraph.string_deserialize(string_array{2});
    % 3 all Val
    msg.all.all_values = gtsam.Values.string_deserialize(string_array{3});
    % 4 del states
    msg.s_del = gtsam.KeyList.string_deserialize(string_array{4});
    % 5 add state
    msg.s_add = gtsam.KeyList.string_deserialize(string_array{5});
    % 6 vec fac_rem_ind
    rem_fac_indx_ser = string_array{6};
    rem_fac = [];
    parserLoc = find(rem_fac_indx_ser == '-');
    parserLoc = [0 parserLoc];
    for i = 1: length(parserLoc)-1
        rem_fac(end+1) = str2num(rem_fac_indx_ser(parserLoc(i)+1:(parserLoc(i+1)-1)));
    end
    msg.rem_fac = rem_fac;
    clear('parserLoc','rem_fac','rem_fac_indx_ser');
    % 7 inc Fac
    msg.inc.inc_graph = gtsam.NonlinearFactorGraph.string_deserialize(string_array{7});
    % 8 inc Val
    msg.inc.inc_values = gtsam.Values.string_deserialize(string_array{8});
    % 9 goal str
    msg.goalPose3 = gtsam.Pose3.string_deserialize(string_array{9});
    % 10 str nonCollisionMat
    nonCollisionMat_ser = string_array{10}; % [column length - decimal rep of binary matrix]
    parserLoc = find(nonCollisionMat_ser == '-');
    columnLength = str2num(nonCollisionMat_ser(1:parserLoc-1));
    % decimal rep
    %         decimalRep = str2num(nonCollisionMat_ser(parserLoc+1:end));
    %         spreadMat = de2bi(decimalRep,'left-msb');
    % no decimal rep
    spreadMat = str2double(num2cell(convertStringsToChars(nonCollisionMat_ser(parserLoc+1:end))));
    msg.nonCollisionMat = vec2mat(spreadMat,columnLength);
    clear('spreadMat','nonCollisionMat_ser','parserLoc','columnLength')
    % 11 Primitive Dictionary
    msg.PrimDictionary = gtsam.Values.string_deserialize(string_array{11});
    % 12 num of candidate paths
    msg.candi.candPathNum = str2num(string_array{12});
    candidatePaths = cell(1,msg.candi.candPathNum);
    % 12 till the end, candidate paths
    for i = 13 : (12 + msg.candi.candPathNum)
        path = gtsam.Values.string_deserialize(string_array{i});
        if path.size >0
            msg.candi.candidatePaths{i-12} = path;
        end
    end
    clear('path','nonCollisionMat_ser','candidatePaths');
    
    disp(['planner received: Prior + Changes + actions']);
    % String to plot interesting LineStyle
    LineStyle = {'-','--',':','-.'};
    MarkerType = ['+','o','*','x','s','d','p','h'];
    Color = 'kbrm'; %rgbmykw
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Ploting prior paths trajectory %%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    f = figure(session_num);hold on; grid on;
    gtsam.plot3DTrajectory(msg.all.all_values,'c^-');
    keys = gtsam.KeyVector(msg.all.all_values.keys);
    for i=0:msg.all.all_values.size-1
        if char(gtsam.symbolChr(keys.at(i))) == 'x'
            gtsam.plotPose3(msg.all.all_values.at(keys.at(i)), [], 0.3);
            
            lastpose = msg.all.all_values.at(keys.at(i));

        end
    end
    axis equal;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Ploting candidate paths trajectory %%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    title(['Currently there are ',num2str(msg.candi.candPathNum), ' future paths of session ',num2str(session_num)])
    for i=1:msg.candi.candPathNum
%         val = gtsam.Values(msg.candi.candidatePaths{i});
        val_strc = Values2pointsANDrotmatrix(msg.candi.candidatePaths{i});
        ls = [Color(randi(length(Color))),...
            char(LineStyle(randi(length(LineStyle)))),...
            MarkerType(randi(length(MarkerType)))];
%         gtsam.plot3DTrajectory(val,ls);
        lp = [lastpose.translation.x lastpose.translation.y lastpose.translation.z]';
        val_strc.Way = [lp val_strc.Way(:,:)];
        p(i) = plot(val_strc.Way(1,:),val_strc.Way(2,:),ls);
        p(i).DisplayName = ["path " + num2str(i)]; 
                
    end
    legend(p);
    hold off;
    axis equal;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    % run your matlab code here...
    % and send back the result
    
    disp(['planner received: Prior + actions']);
    
    prompt = 'What is index_to_send value? (choose "0" for defalut)';
    index_to_send = input(prompt);
%     index_to_send = 0; % only for testing, choose allways the first path
    clear msg lp ls lineStyle MarkerType p prompt val_strc str string_array;
   
    msgg = rosmessage(matlab_pub);
    msgg.Data = index_to_send; % index in C-style
    matlab_pub.send(msgg);
    
    disp (['Chosen Path: ' num2str(index_to_send)]);
    disp (['++++++++++++++++++++++++++++++++++++++++++++++']);
    session_num = session_num + 1;
    
    
end

