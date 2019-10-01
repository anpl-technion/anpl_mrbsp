 % Initialize ROS master and node
 rosinit;
 
 
 figureHandle = figure('Name', 'Map');
 axesHandle = axes('Parent', figureHandle);
 
 mapSub = rossubscriber('octomap_binary');
 
 % Receive a new map reading
 mapMsg = receive(mapSub);
 map = readOccupancyMap3D(mapMsg); % Introduced in R2018a
 
 mapHandle = show(map, 'Parent', axesHandle);
 title(axesHandle, 'OccupancyMap: World Map');
 
 
 % Shutdown ROS
 rosshutdown;
 
 % Plot interpolated candidate paths
 P = load('~/.ros/0._interpolated_path.txt')
 hold on
 quiver(P(:,1), P(:,2), cos(P(:,3)), sin(P(:,3)), 0.21)