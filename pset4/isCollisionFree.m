function isColFree = isCollisionFree( Obs, xy )
% check if there's a collision for point xy in Obs
for k = 1:length(Obs)
    in = inConvHull(xy, Obs{k});
    isColFree = (in == 0);
    if ~isColFree
        return
    end
end
end
