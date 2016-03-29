function inArr = inConvHull(p, s)
% return array of 0/1 for points p in conv hull of s
x = s(1, :);
y = s(2, :);
k = convhull(x,y);
inArr = inpolygon(p(1, :),p(2, :),x(k),y(k));
end
