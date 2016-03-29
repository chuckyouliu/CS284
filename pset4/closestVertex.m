function vertex = closestVertex( rrt_verts, xy )
% find closest vertex based off euclidean distance
distances = sqrt((rrt_verts(1, :) - xy(1)) .^ 2 + (rrt_verts(2, :) - xy(2)) .^ 2);
[M,I] = min(distances);
vertex = rrt_verts(:, I);
end
