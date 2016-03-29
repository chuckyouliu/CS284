N_track = zeros(10,1);
for i = 1:10
%% Code setup (Do not modify, but please read) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacles. Each obstacle is a cell in Obs. An obstacle is
% represented as a convex hull of a number of points. These points are
% stored in the cells of Obs.
% First row is x, second is y (position of vertices)
w = 0.5;
Obs{1} = [0 0;5 0;5 w;0 w]';
Obs{2} = [0 0;2*w 0;w 10;0 10]';
Obs{3} = [0 10-w;5 10;5 10+w;0 10+w]';
Obs{4} = [5-w 0;5+w 0;5+w 5;5 5]';
Obs{5} = [5-w 10+w;5+w 10+w;5+w 6.25;5 6.25]';
Obs{6} = [4 5;5+w 5;5+w 5+w;4 5+w]';
Obs{7} = [4 6.25;5+w 6.25;5+w 6.25+w;4 6.25+w]';

% Bounds on world
world_bounds_x = [-8,10];
world_bounds_y = [-4,14];


% Draw obstacles
figure(1); clf; hold on;
axis([world_bounds_x world_bounds_y]);

for k = 1:length(Obs)
    patch(Obs{k}(1,:),Obs{k}(2,:),'r');
end

% Start and goal positions
xy_start = [4;1]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
xy_goal = [-4;6]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10);

% Initialize RRT. The RRT will be represented as a 2 x N list of points. So
% each column represents a vertex of the tree.
% two trees for bidirectional
i_verts = zeros(2,1000);
i_verts(:,1) = xy_start;

g_verts = zeros(2, 1000);
g_verts(:, 1) = xy_goal;

% keep track of nodes for both i and g
N_i = 1;
N_g = 1;

% max iterations and iteration counter
iter = 0;
max_iter = 1000;

% keep track of which tree we're modifying - 0 = i, 1 = g
curr_tree = 0;

nearGoal = false; % This will be set to true if goal has been reached
minDistGoal = 0.25; % This is the convergence criterion. We will declare
                    % success when the tree reaches within 0.25 in distance
                    % from the goal. DO NOT MODIFY.

% Extension parameter
d = 0.5; % This controls how far the RRT extends in each step. DO NOT MODIFY.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RRT algorithm
while iter < max_iter
    % Figure out which tree we'll be adding a node to
    if N_i < N_g
        curr_tree = 0;
    else
        curr_tree = 1;
    end
    
    % Sample point
    rnd = rand(1);
    % With probability 0.05, sample the goal. This promotes movement to the
    % goal.
    if rnd < 0.05
        % In this bi-directional case the goal can mean either the goal or
        % starting point depending on what tree we're adding to
        if curr_tree == 0
            xy = xy_goal;
        else
            xy = xy_start;
        end
    else
        %% FILL ME IN
        % Sample (uniformly) from space (with probability 0.95). The space is defined
        % with the bounds world_bounds_x and world_bounds_y defined above.
        % So, the x coordinate should be sampled in the interval
        % world_bounds_x and the y coordinate from world_bounds_y.
        xy = [world_bounds_x(1) + (world_bounds_x(2)-world_bounds_x(1))*rand(1)
              world_bounds_y(1) + (world_bounds_y(2)-world_bounds_y(1))*rand(1)]; % Should be a 2 x 1 vector
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    % Check if sample is collision free
    collFree = isCollisionFree(Obs,xy);
    
    if ~collFree
        continue;
    end
    
    % If it is collision free, find closest point in corresponding tree. 
    % The points in the existing tree are i_verts(:,1:N)
    if curr_tree == 0
        closest_vert = closestVertex(i_verts(:,1:N_i),xy);
    else
        closest_vert = closestVertex(g_verts(:,1:N_g),xy);
    end
        
    % Extend tree towards xy from closest_vert. Use the extension parameter
    % d defined above as your step size. In other words, the Euclidean
    % distance between new_vert and closest_vert should be d (do not modify
    % d. It should be 0.5).
    a = (xy(2) - closest_vert(2))/(xy(1) - closest_vert(1));
    b = xy(2) - xy(1)*a;
    
    % check if norm is less than d already, in which case can just use xy
    % otherwise new_vert needs to be extended as long as possible
    if norm(xy-closest_vert) <= d
        new_vert = xy;
    else
        new_x = fsolve(@(x) sqrt((a*x+b - closest_vert(2))^2 + (x - closest_vert(1))^2) - d, xy(1));
        new_vert = [new_x; a*new_x+b];
    end
    
    % Check if new_vert is collision free
    collFree = isCollisionFree(Obs,new_vert); % Same function you wrote before.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    % If it is not collision free, continue with loop
     if ~collFree
        continue;
     end
     
    % Plot extension (Comment the next 3 lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
    %figure(1)
    %plot(new_vert(1),new_vert(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    %line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
     
    % If it is collision free, add it to tree
    if curr_tree == 0
        N_i = N_i+1;
        if N_i > size(i_verts,2)
            i_verts = [i_verts zeros(size(i_verts))];
        end
        i_verts(:, N_i) = new_vert;
    else
        N_g = N_g+1;
        if N_g > size(g_verts,2)
            g_verts = [g_verts zeros(size(g_verts))];
        end
        g_verts(:, N_g) = new_vert;
    end
    
    % now find the nearest node in the other tree to new_vert
    if curr_tree == 0
        closest_vert = closestVertex(g_verts(:,1:N_g),new_vert);
    else
        closest_vert = closestVertex(i_verts(:,1:N_i),new_vert);
    end
    
    % if closest_vert is within d of new_vert, see if there's some
    % collision in the middle. If there isn't, then we've connected the two
    % trees. Otherwise discard. If not within d, extend the other tree with
    % a node d length away
    if norm(closest_vert - new_vert) <= d
        midpoint = (closest_vert+new_vert)/2;
        if isCollisionFree(Obs, midpoint)
            N_track(i) = N_i + N_g;
            break
        end
    else
        a = (new_vert(2) - closest_vert(2))/(new_vert(1) - closest_vert(1));
        b = new_vert(2) - new_vert(1)*a;
        new_x = fsolve(@(x) sqrt((a*x+b - closest_vert(2))^2 + (x - closest_vert(1))^2) - d, new_vert(1));
        new_vert = [new_x; a*new_x+b];
        % Check if new_vert is collision free
        if isCollisionFree(Obs,new_vert)
            if curr_tree == 1
                N_i = N_i+1;
                if N_i > size(i_verts,2)
                    i_verts = [i_verts zeros(size(i_verts))];
                end
                i_verts(:, N_i) = new_vert;
            else
                N_g = N_g+1;
                if N_g > size(g_verts,2)
                    g_verts = [g_verts zeros(size(g_verts))];
                end
                g_verts(:, N_g) = new_vert;
            end
        end
    end
    
    % increment iterations, though this should end before max_iter hit
    iter = iter + 1;
end

% Plot vertices in RRT
plot(i_verts(1,:),i_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
plot(g_verts(1,:),g_verts(2,:),'go','MarkerFaceColor','g','MarkerSize',5);
end
N_track
