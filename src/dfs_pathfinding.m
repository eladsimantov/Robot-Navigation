function path_plan = dfs_pathfinding(space_grid, start, goal)
    % space_grid: 3D grid representing the space (x, y, theta_index)
    % start: [x_start, y_start, theta_index_start]
    % goal: [x_goal, y_goal, theta_index_goal]
    % ------------------------------------------- %
    % Based on Latombe Appendix C - Basic Methods - detpth-first search

    stack = {start};                                % Init a stack with the start position
    visited = false(size(space_grid));              % All nodes are initially marked as unvisited
    parent_nodes = cell(size(space_grid));          % Init a parent node map to reconstruct the path when goal is found.
    visited(start(1), start(2), start(3)) = true;   % Set the initial node as visited
    path_plan = [];

    % Directions for moving in the grid. We assume "Tetris" like movements
    % which are descretized to only one coordintate in every step.
    directions = [
        -1, 0, 0;   % Left
        1, 0, 0;    % Right
        0, -1, 0;   % Down
        0, 1, 0;    % Up
        0, 0, -1;   % Rotate CW
        0, 0, 1;    % Rotate CCW
    ];
    
    % Operate until all nodes that can be reached have been visited.
    % The stack will be updated with the neighboring nodes that are found
    % by looking at all directions with respect to the current node. If the
    % neighboring node is "valid" which means that it is not in an
    % obstacle, it will be added to the stack.
    while ~isempty(stack)
        current_node = stack{end}; % Remove current node from stack
        stack(end) = [];
        
        if isequal(current_node, goal)
            % Reconstruct the path from goal to start
            path_plan = reconstruct_path(parent_nodes, start, goal);
            disp("Found path!")
            disp("LENGTH: " + length(path_plan))
            return;
        end
        
        % Explore neighboring nodes
        for i = 1:size(directions, 1)
            neighbor_node = current_node + directions(i, :); 
            neighbor_node(3) = mod(neighbor_node(3) - 1, size(space_grid, 3)) + 1; % let the angle be continuous (theta = 0 is also equal to 2pi)
            
            % Check if the neighboring node not in an obstacle and has not
            % been visited before.
            if is_valid(neighbor_node, space_grid) && ~visited(neighbor_node(1), neighbor_node(2), neighbor_node(3))
                visited(neighbor_node(1), neighbor_node(2), neighbor_node(3)) = true;
                stack{end+1} = neighbor_node;
                parent_nodes{neighbor_node(1), neighbor_node(2), neighbor_node(3)} = current_node;
            end
        end
    end
    if isempty(path_plan)
        disp("No path found\n")
    end
end

    function valid = is_valid(node, space_grid)
    % Check if the position is within bounds and not an obstacle
    x = node(1); y = node(2); theta = node(3);
    valid = x > 0 && x <= size(space_grid, 1) && ...
            y > 0 && y <= size(space_grid, 2) && ...
            space_grid(x, y, theta) == 0;
    end

function reconstructed_path = reconstruct_path(parents, start, goal)
    % Reconstruct the path from goal to start using the parent map
    reconstructed_path = {goal};
    current = goal;
    while ~isequal(current, start)
        current = parents{current(1), current(2), current(3)};
        reconstructed_path = [{current}, reconstructed_path];
    end
end
