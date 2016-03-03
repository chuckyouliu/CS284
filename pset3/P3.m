h = .1;
b = .1+rand;

% decision variables
z = 10*randn(30, 1);

% x is 2x10, where (x:,k) is [theta_k;\dot \theta_k]
x = reshape(z(1:20), 2, []);

% u is 1x10, where u(k) is u_k
u = reshape(z(21:30), 1, [])';

%all code in colconstraint file, testing approximation below

%error for 
[g,dg] = colconstraint(h,b,x,u);
diff = .001;

% theta_1
x_diff = zeros(2,10);
x_diff(1,1) = diff;
g_new = colconstraint(h,b,x+x_diff,u);
error = (g_new - g)/diff-dg(:, 1);
error(1:2)

% theta*_1
x_diff = zeros(2,10);
x_diff(2,1) = diff;
g_new = colconstraint(h,b,x+x_diff,u);
error = (g_new - g)/diff-dg(:, 2);
error(1:2)

% theta_10
x_diff = zeros(2,10);
x_diff(1,10) = diff;
g_new = colconstraint(h,b,x+x_diff,u);
error = (g_new - g)/diff-dg(:, 19);
error(17:18)

% theta*_10
x_diff = zeros(2,10);
x_diff(2,10) = diff;
g_new = colconstraint(h,b,x+x_diff,u);
error = (g_new - g)/diff-dg(:, 20);
error(17:18)

% u_1
u_diff = zeros(10,1);
u_diff(1) = diff;
g_new = colconstraint(h,b,x,u+u_diff);
error = (g_new - g)/diff-dg(:, 21);
error(1:2)

% u_10
u_diff = zeros(10,1);
u_diff(10) = diff;
g_new = colconstraint(h,b,x,u+u_diff);
error = (g_new - g)/diff-dg(:, 30);
error(1:2)
