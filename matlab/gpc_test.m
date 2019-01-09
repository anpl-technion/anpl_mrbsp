function gpc_test
% Tests the gpc code

	theta = deg2rad(4);
	t = [0.3; -0.2];
	real_x = [t(1); t(2); theta];

	p = [1 0; 0 1; -1 0; 2 1; 4 2]';
    % line normals
	alpha = [deg2rad(0);deg2rad(10);deg2rad(20);deg2rad(50);deg2rad(-20);];

	noise = 0
    figure(3), clf
	for i=1:size(p,2)
		corr{i}.p = p(:,i);
		corr{i}.q = rot(theta)*p(:,i)+t + randn(2,1)*noise;
		corr{i}.C = vers(alpha(i))*vers(alpha(i))'; % point - line metric C_i = n_i * n_i', n_i line segment normal
        %	corr{i}.C = eye(2); % point - point metric
        
%         plot(p(1,i), p(2,i), '.', 'MarkerSize', 15), hold on
%         plot(corr{i}.q(1), corr{i}.q(2), 'r.', 'MarkerSize', 15)
%         line([corr{i}.p(1) corr{i}.q(1)]',[corr{i}.p(2) corr{i}.q(2)]')
%         quiver(corr{i}.p(1), corr{i}.p(2), cos(alpha(i)), sin(alpha(i)))
	end

	res = gpc(corr);

	fprintf('Real     : %s\n', sprintf('%f ', real_x));
	fprintf('Estimated: %s\n', sprintf('%f ', res.x));


function R = rot(theta)
	R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
	
function v = vers(theta)
	v = [cos(theta); sin(theta)];