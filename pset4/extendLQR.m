function new_vert = extendLQR( closest_vert,xy,K )
% apply lqr policy for .1 seconds and return resulting vertex
g = 9.81;
b = 0.1;

% get current difference, again noting that theta can wrap around
diff = closest_vert - xy;
diff(1) = diff(1) - 2*pi*ceil((diff(1) - pi)/(2*pi));
u_opt = -K*diff;

% apply optimal u
u_opt = min(max(u_opt, -5), 5);
[t,y] = ode45(@(t,y) [y(2); u_opt - g*sin(y(1))-b*y(2)],[0 .1],closest_vert);

new_vert = y(size(y,1), :)';
% normalize to between -pi/2 to 3pi/2
new_vert(1) = new_vert(1) - 2*pi*ceil((new_vert(1) - 3*pi/2)/(2*pi));
end

