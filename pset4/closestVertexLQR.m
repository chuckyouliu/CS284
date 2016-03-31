function [index, min_vertex, K] = closestVertexLQR( rrt_verts, xy )
% use lqr to find closest vertex

% constants
g = 9.81;
b = 0.1;

% set up return variables and cost tracking variable
min_vertex = [0;0];
min_cost = flintmax;
index = -1;

% set up lqr
A = [0 1; -g*cos(xy(1)) -b];
B = [0;1];
Q = eye(2);
R = 0.1;
[K,S] = lqr(A,B,Q,R);

for k=1:size(rrt_verts, 2)
    % calculate difference from xy taking into account wrapping of theta
    % then update cost
    diff = rrt_verts(:, k) - xy;
    while diff(1) < -pi/2
        diff(1) = diff(1) + 2*pi;
    end
    while diff(1) > 3*pi/2
        diff(1) = diff(1) - 2*pi;
    end
    cost = diff'*S*diff;
    if cost < min_cost
        min_cost = cost;
        min_vertex = rrt_verts(:, k);
        index = k;
    end
end

end

