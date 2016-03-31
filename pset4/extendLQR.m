function new_vert = extendLQR( closest_vert,xy,K )
% apply lqr policy for .1 seconds and return resulting vertex
g = 9.81;
b = 0.1;

% get current difference
diff = closest_vert - xy;
while diff(1) > 3*pi/2
    diff(1) = diff(1) - 2*pi;
end
while diff(1) < -pi/2
    diff(1) = diff(1) + 2*pi;
end

% apply optimal u
u_opt = min(max(-K*diff, -5), 5);
[t,y] = ode45(@(t,y) [y(2); u_opt - g*sin(y(1))-b*y(2)],[0 .1],closest_vert);

new_vert = y(size(y,1), :)';
while new_vert(1) > 3*pi/2
    new_vert(1) = new_vert(1) - 2*pi;
end
while new_vert(1) < -pi/2
    new_vert(1) = new_vert(1) + 2*pi;
end
end

