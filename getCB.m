function [CB] = getCB(Robot, Obstacle, vRobot, vObstacle)
%GETCB find the C-space obstacle using the Lozano Perez algorithm mentioned in Latombe ch. 3
% The idea is to scan the vectors -vA_i(theta) and vB_j in cc order
% Then construct all (nA + nB) vertices. 

    CB = [NaN, NaN]; % just to init... will be removed later
    ExRobot = [Robot(end, :); Robot; Robot(1, :)]; % to access i-1 and i+1 points extend the matrix 
    ExObstacle = [Obstacle(end, :); Obstacle; Obstacle(1, :)]; % to access j-1 and j+1 points extend the matrix

    for i = 1:4
        for j = 1:4
            % ignore in the case of contact of faces
            if -vRobot(i, :)==vObstacle(j, :)
                continue
            end 
            if AAPL("A", vRobot, Obstacle, i, j)
                % so (bj-ai) and (bj-ai+1) are vertices of CB
                CB = union(CB, Obstacle(j, :)-Robot(i, :), 'rows'); % union: add to CB only points that are not there yet, over rows axis
                CB = union(CB, Obstacle(j, :)-ExRobot(i+2, :), 'rows');
            end
            if AAPL("B", vObstacle, Robot, i, j)
                % so (bj-ai) and (bj+1-ai) are vertices of CB
                CB = union(CB, Obstacle(j, :) - Robot(i, :), 'rows');
                CB = union(CB, ExObstacle(j+2, :) - Robot(i, :), 'rows');
            end
        end 
    end 
    CB = CB(~any(isnan(CB), 2), :); % clean the NaN values used to init.
    k = convhull(CB(:, 1), CB(:, 2)); % use convex hull to reorder points 
    CB = CB(k, :) + repmat(Robot(1,:), [length(k),1]); % get the CB to origin with the boundary
end

