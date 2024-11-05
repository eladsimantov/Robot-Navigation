function path_plan = online_astar_pathfinding(space_grid, start, goal, maxIterations)
    % space_grid: 3D grid representing the space (x, y, theta_index)
    % start: [x_start, y_start, theta_index_start]
    % goal: [x_goal, y_goal, theta_index_goal]
    % maxIterations: Set maximum iterations. 1000 by default.
    % ------------------------------------------- %
    % Based on Latombe Appendix C - Basic Methods - A* Algorithm
    % Modified for Online A* navigation

    if nargin < 4
        maxIterations = 10000; % default
    end

    visited = false(size(space_grid));             % All nodes are initially marked as unvisited
    path_plan = [];                                % Initiallize the plan to empty vector.
    current_node = start; % set IC
    directions = [
        -1, 0, 0;   % Left
        1, 0, 0;    % Right
        0, -1, 0;   % Down
        0, 1, 0;    % Up
        0, 0, -1;   % Rotate CW
        0, 0, 1;    % Rotate CCW
    ];

    while ~isequal(current_node, goal) && length(path_plan) < maxIterations
        % Scan neighbors from all directions
        f_costs = inf([6,1]);
        valid_node_indeces = [];
        for i = 1:size(directions, 1)
            neighbor_node = current_node + directions(i, :);
            neighbor_node(3) = mod(neighbor_node(3) - 1, size(space_grid, 3)) + 1; % Wrap theta
            
            if is_valid(neighbor_node, space_grid) 
                % Find neighbor cost 
                % f_costs(i) = length(path_plan) + 1 + heuristic(neighbor_node, goal); % Total cost (f) is the length of path + one step + heuristic for neighboring node.
                f_costs(i) = heuristic(neighbor_node, goal); % Total cost (f) is the length of path + one step + heuristic for neighboring node.
                valid_node_indeces = [valid_node_indeces, i];
            end

            if visited(neighbor_node(1), neighbor_node(2), neighbor_node(3))
                f_costs(i) = 5 * f_costs(i); % to avoid loops
            end
            
        end
        
        % Find neighbor with lowest cost and update path
        [~, min_node_index] = min(f_costs);
        
        best_node = current_node + directions(min_node_index, :);
        best_node(3) = mod(best_node(3) - 1, size(space_grid, 3)) + 1; % Wrap theta

        % add random choice if keeps repeating best node
        if visited(best_node(1), best_node(2), best_node(3)) == true
            rand_node_index = valid_node_indeces(randi(length(valid_node_indeces)));
            best_node = current_node + directions(rand_node_index, :);
            best_node(3) = mod(best_node(3) - 1, size(space_grid, 3)) + 1; % Wrap theta
        end
        path_plan = [path_plan; {current_node}];  % update the path plan to include the current node
        visited(current_node(1), current_node(2), current_node(3)) = true;      % Mark the current node as visited
        current_node = best_node;               % update the current node for next step to be the best node
        % disp(current_node); disp(f_costs); disp(valid_node_indeces)

        if isequal(current_node, goal)
            path_plan = [path_plan; {goal}]; % if goal is reached update plan and terminate
        end
    end


end
       


function h = heuristic(node, goal)
    % Heuristic function: 2D distance to estimtate the cost of the
    % path between the given node and the goal. We assume that the angle theta (the index in this case) 
    % is not part of the target and therefore it is not included in the heuristic function
    h = sqrt((goal(1) - node(1))^2 + (goal(2) - node(2))^2);
end


function valid = is_valid(node, space_grid)
    % Check if the position is within bounds and not an obstacle
    x = node(1); y = node(2); theta = node(3);
    valid = x > 0 && x <= size(space_grid, 1) && ...
            y > 0 && y <= size(space_grid, 2) && ...
            space_grid(x, y, theta) == 0;
end
