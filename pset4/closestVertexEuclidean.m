function vertex = closestVertexEuclidean( rrt_verts, xy )
% find closest vertex based off euclidean distance
% because theta wraps around, need to check if wrapping around will make it
% closer
distances = sqrt((rrt_verts(1, :) - xy(1)) .^ 2 + (rrt_verts(2, :) - xy(2)) .^ 2);
distances_plus2pi = sqrt((rrt_verts(1, :) + 2*pi - xy(1)) .^ 2 + (rrt_verts(2, :) - xy(2)) .^ 2);
distances_minus2pi = sqrt((rrt_verts(1, :) - 2*pi - xy(1)) .^ 2 + (rrt_verts(2, :) - xy(2)) .^ 2);
distances = min(min(distances, distances_plus2pi), distances_minus2pi);
[M,I] = min(distances);
vertex = rrt_verts(:, I);
end

