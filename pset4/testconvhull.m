xx = -1:.5:1; yy = abs(sqrt(xx));
[x,y] = pol2cart(xx,yy);
k = convhull(x,y);
hold on
plot(x(k),y(k),'r-',x,y,'b+')
x_test = [rand(1)]
y_test = [rand(1)]
IN = inConvHull([x_test;y_test], [x;y]);
plot(x_test(IN),y_test(IN),'g*',x_test(~IN),y_test(~IN),'y*')
hold off