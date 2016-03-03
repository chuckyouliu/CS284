function [g, dg] = colconstraint(h,b,x,u)
% calculate u_c values
u_c = (u(1:9) + u(2:10))/2;

% calculate accelerations at points and add to x
ddot_theta = u' - 10*sin(x(1,:)) - b*x(2,:);
x = [x;ddot_theta];

% calculate dotx_c from cubic formula 1.5*h*(x1-x0) - .25*(dotx0 + dotx1)
dotx_c = 1.5*h*(x(1:2, 2:10) - x(1:2, 1:9)) - .25*(x(2:3, 1:9) + x(2:3, 2:10));

% calculate x_c values
% x_c = .5*(x_0 + x_1) + h*(dotx0 - dotx1)/8
x_c = .5*(x(1:2, 1:9) + x(1:2, 2:10)) + h*(x(2:3, 1:9) - x(2:3, 2:10))/8;

% calculate accelerations at x_c
ddot_theta_c = u_c' - 10*sin(x_c(1,:)) - b*x_c(2,:);

% g is difference between dotx_c and [x_c(2, :); ddot_theta_c]
g = dotx_c - [x_c(2, :);ddot_theta_c];

% reshape g to be 18 entry vector
g = reshape(g, [18,1]);

% initialize dg
dg = zeros(18, 30);

% for the last 10 columns corresponding to dg/du_i
% looking at the variable names that make up g:
% d(dotx_c)/d(u_i) = 0 - .25*([0;1]) if i is one of the knot pts for c
% d(x_c(2,:))/d(u_i) = h/8 if i is left knot pt for c, -h/8 if right knot pt
% d(ddot_theta_c)/d(u_i) = 1/2-b*h/8 if left knot pt, 1/2+b*h/8 if right knot
%fill in values first for left knot then right knot
for n = 21:29
    %left knot index
    index = 2*(n-20);
    dg(index-1:index, n) = -[0;.25] - [h/8;.5-b*h/8];
end
for n = 22:30
    %right knot index
    index = 2*(n-21);
    dg(index-1:index, n) =  dg(index-1:index, n) -[0;.25] - [-h/8;.5+b*h/8];
end

% for the columns corresponding to dg/dtheta_i
% d(dotx_c)/d(theta_i) = [-1.5h;2.5cos(theta_i)] if left knot pt, [1.5h;2.5cos(theta_i)] if right
% d(x_c(2,:))/d(theta_i) = -10h/8*cos(theta_i) left knot, 10h/8cos(theta_i) right
% d(ddot_theta_c)/d(theta_i) = -5cos(theta_c) + 5bh/4(cos(theta_i)) if left, -5cos(theta_c) - 5bh/4(cos(theta_i)) if right 
collocation_pt = 1;
for n = 1:2:18
    theta_i = x(1, collocation_pt);
    theta_c = x_c(1, collocation_pt);
    dg(n:n+1, n) = [-1.5*h;2.5*cos(theta_i)] - [-5*h/4*cos(theta_i); -5*cos(theta_c)+5*b*h/4*cos(theta_i)];
    collocation_pt = collocation_pt + 1;
end
collocation_pt = 1;
for n = 3:2:20
    theta_i = x(1, collocation_pt+1);
    theta_c = x_c(1, collocation_pt);
    dg(n-2:n-1, n) = dg(n-2:n-1, n) + [1.5*h;2.5*cos(theta_i)] - [5*h/4*cos(theta_i); -5*cos(theta_c)-5*b*h/4*cos(theta_i)];
    collocation_pt = collocation_pt + 1;
end

% for the columns corresponding to dg/d(theta*)_i
% d(dotx_c)/d(theta*_i) = [-.25;-1.5h+.25b] if left knot [-.25;1.5h+.25b] if right knot
% d(x_c(2,:))/d(theta*_i) = .5-hb/8 if left knot, .5+hb/8 if right knot
% d(ddot_theta_c)/d(theta*_i) = -5h/4cos(theta_c) - b/2 + b^2*h/8 if left,
% 5h/4cos(theta_c) - b/2 - b^2*h/8 if right
collocation_pt = 1;
for n = 2:2:19
    theta_c = x_c(1, collocation_pt);
    dg(n-1:n, n) = [-.25;-1.5*h+.25*b] - [.5-h*b/8; -5*h/4*cos(theta_c)-b/2+b^2*h/8];
    collocation_pt = collocation_pt + 1;
end
collocation_pt = 1;
for n = 4:2:20
    theta_c = x_c(1, collocation_pt);
    dg(n-3:n-2, n) = dg(n-3:n-2, n) + [-.25;1.5*h+.25*b] - [.5+h*b/8; 5*h/4*cos(theta_c)-b/2-b^2*h/8];
    collocation_pt = collocation_pt + 1;
end
end