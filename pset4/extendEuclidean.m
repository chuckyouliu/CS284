function new_vert = extendEuclidean( closest_vert, xy )
% constants
g = 9.81;
b = 0.1;

% discretize the u input
u = -5:.5:5;

% vector to store simulations from each u
sim_res = zeros(2, length(u));

% for each u input, simulate from closest_vert for .1 seconds
for k = 1:size(u,2)
    [t,y] = ode45(@(t,y) [y(2); u(k) - g*sin(y(1))-b*y(2)],[0 .1],closest_vert);
    sim_vert = y(size(y,1), :)';
    % normalize theta to within [-pi/2, 3*pi/2]
    while sim_vert(1) > 3*pi/2
        sim_vert(1) = sim_vert(1) - 2*pi;
    end
    while sim_vert(1) < -pi/2
        sim_vert(1) = sim_vert(1) + 2*pi;
    end
    sim_res(:, k) = sim_vert;
end

% now get the closest vertex to xy from the simulated u's
new_vert = closestVertexEuclidean(sim_res, xy);
end

