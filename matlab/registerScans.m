function [pose, stats, plots] = registerScans(scan_01, scan_02, method, parameters)

plots.h1 = subplot(2,1,1);
plot(scan_01, 'MaximumRange', 7); hold on
s2 = plot(scan_02, 'MaximumRange', 7);

lidarScan_01 = lidarScan(scan_01);
lidarScan_02 = lidarScan(scan_02);
axis equal
xlim([0 7])
ylim([-7 7])

plots.h2 = subplot(2,1,2);
hold on
        
switch method
    case 'ndt'
        disp('NDT')
        % Normal Distributions Transform (NDT)
        [pose, stats.score] = matchScans(lidarScan_01, lidarScan_02, 'InitialPose', parameters.initial_2d_pose);
        
        transformedScan = transformScan(lidarScan_01, pose);       
        plot(transformedScan)
        plot(lidarScan_02)
        transformedScanInitial = transformScan(lidarScan_01, parameters.initial_2d_pose);
        plot(transformedScanInitial)

        
        %xlim([0 7])
        xlim([-7 7])
        ylim([-7 7])
        
        
    case 'icp'
        disp('ICP')
        ptCloud1 = pointCloud([lidarScan_01.Cartesian zeros(lidarScan_01.Count, 1)]);
        ptCloud2 = pointCloud([lidarScan_02.Cartesian zeros(lidarScan_02.Count, 1)]);
        
        ptCloudTformedInitial = pctransform(ptCloud1, parameters.initial_transform);
        
        % ICP
        [pose, ptCloudTformed, stats.rmse] = pcregrigid(ptCloud1, ptCloud2, 'InitialTransform', parameters.initial_transform, ...
                                                                       'MaxIterations', parameters.icp.max_iter, ...
                                                                       'InlierRatio', parameters.icp.inlier_ratio);
        
        xlim('manual')
        plot(ptCloudTformed.Location(:,2), ptCloudTformed.Location(:,1), '.'); % registered
        plot(ptCloud2.Location(:,2), ptCloud2.Location(:,1), '.'); % referent
        plot(ptCloudTformedInitial.Location(:,2), ptCloudTformedInitial.Location(:,1), '.'); % initial
        xlabel('Y'), ylabel('X')
        set(plots.h2, 'XDir','reverse')
        grid on
        axis equal
        xlim([-7 7])
        %ylim([0 7])
        ylim([-7 7])
        
        
    case 'gpc'
        
        disp('General 2D point correspodence')
        transformedScanInitial = transformScan(lidarScan_01, parameters.initial_2d_pose);
        dt = delaunayTriangulation(lidarScan_02.Cartesian);
        xi = nearestNeighbor(dt, transformedScanInitial.Cartesian);
        xnn = lidarScan_02.Cartesian(xi,:);
        
        % finding correspodences in laser scans needs further consideration
        j = 1;
        for i=1:lidarScan_02.Count
            corr{j}.p = transformedScanInitial.Cartesian(i,:)';
            corr{j}.q = xnn(i,:)'; % nearest neighbour in the referent scan of the point in initial transformed query scan
            %corr{i}.C = vers(alpha(i))*vers(alpha(i))'; % point - line metric C_i = n_i * n_i', n_i line segment normal
            corr{j}.C = eye(2); % point - point metric
            
            if norm(corr{j}.p - corr{j}.q) > 1
                continue
            else
                %             plot(p(1,i), p(2,i), '.', 'MarkerSize', 15), hold on
                %             plot(corr{i}.q(1), corr{i}.q(2), 'r.', 'MarkerSize', 15)
                %line([corr{j}.p(1) corr{j}.q(1)]',[corr{j}.p(2) corr{j}.q(2)]', 'Color', 'green')
                j = j+1;
            end
            

        end
        
        
        delta_pose = gpc(corr);
        H_delta = [eul2rotm([delta_pose.x(3) 0 0]) [delta_pose.x(1:2); 0]; 0 0 0 1];
        H = parameters.initial_transform.T' * H_delta; % initial_pose + delta_pose;
        theta = rotm2eul(H(1:3,1:3));
        pose = [H(1,4), H(2,4), theta(1)]; 
        stats = [];

        transformedScan = transformScan(lidarScan_01, pose);       
        plot(transformedScan)
        plot(lidarScan_02)
        transformedScanInitial = transformScan(lidarScan_01, parameters.initial_2d_pose);
        plot(transformedScanInitial)

        
        %xlim([0 7])
        xlim([-7 7])
        ylim([-7 7])
        
        
        
    case 'go_icp'
        
        disp('GoICP')
        
        ptCloud1_ = pointCloud([lidarScan_01.Cartesian zeros(lidarScan_01.Count, 1)]);
        ptCloud2_ = pointCloud([lidarScan_02.Cartesian zeros(lidarScan_02.Count, 1)]);
        
        roi = [-30,30;-30,30;0,inf];
        indices = findPointsInROI(ptCloud1_, roi);
        ptCloud1 = select(ptCloud1_,indices);
        indices = findPointsInROI(ptCloud2_, roi);
        ptCloud2 = select(ptCloud2_,indices);
        
        ptCloudTformedInitial = pctransform(ptCloud1, parameters.initial_transform);
        
        % Globally optimal ICP
        if ispc()
            cmd = '/home/andrej/ANPL/code/3rdparty/GoICP_V1.3/bin/GoICP_vc2012.exe';
        else
            cmd = '/home/andrej/ANPL/code/3rdparty/GoICP_V1.3/bin/GoICP';
        end
        model = ptCloud2.Location';
        data = ptCloud1.Location';
        writepoints(model, 'icp_model.txt')
        writepoints(data, 'icp_data.txt')
        
        cmd = [cmd ' icp_model.txt icp_data.txt 721 GoICPconfig.txt output.txt'];
        
        system(cmd);
        
        file = fopen('output.txt', 'r');
        t = fscanf(file, '%f', 1);
        R = fscanf(file, '%f', [3,3])';
        T = fscanf(file, '%f', [3,1]);
        fclose(file);
        
        figure;
        subplot(1,2,1);
        plot(model(1,:), model(2,:), '.r');
        %plot3(model(1,:), model(2,:), model(3,:), '.r');
        hold on;
        plot(data(1,:),  data(2,:), '.b');
        %plot3(data(1,:),  data(2,:),  data(3,:), '.b');
        hold off; axis equal; title('Initial Pose');
        subplot(1,2,2);
        data_ = bsxfun(@plus, R*data, T);
        plot(model(1,:), model(2,:), '.r');
        %plot3(model(1,:), model(2,:), model(3,:), '.r');
        hold on;
        plot(data_(1,:), data_(2,:), '.b');
        %plot3(data_(1,:), data_(2,:), data_(3,:), '.b');
        hold off; axis equal;  title('Result');
        pose.R = R;
        pose.T = T;
        
        
%         xlim('manual')
%         plot(ptCloudTformed.Location(:,2), ptCloudTformed.Location(:,1), '.'); % registered
%         plot(ptCloud2.Location(:,2), ptCloud2.Location(:,1), '.'); % referent
%         plot(ptCloudTformedInitial.Location(:,2), ptCloudTformedInitial.Location(:,1), '.'); % initial
%         xlabel('Y'), ylabel('X')
%         set(plots.h2, 'XDir','reverse')
%         grid on
%         axis equal
%         xlim([-7 7])
%         %ylim([0 7])
%         ylim([-7 7])
        
        
    otherwise
        disp('unknown method')
        
end

title(['Scans aligned by ' upper(method) ' method'])


