function path_plan = astar_pathfinding(space_grid, start, goal, stop_when_found_path)
    % space_grid: 3D grid representing the space (x, y, theta_index)
    % start: [x_start, y_start, theta_index_start]
    % goal: [x_goal, y_goal, theta_index_goal]
    % stop_when_found_path: a T/F variable to stop the search if path
    % found. Set to true by default.
    % ------------------------------------------- %
    % Based on Latombe Appendix C - Basic Methods - A* Algorithm
    
    if nargin < 4
        stop_when_found_path = true;
    end

    OPEN = {start};                                % Priority queue of nodes to be explored, init with start node
    visited = false(size(space_grid));             % All nodes are initially marked as unvisited
    parent_nodes = cell(size(space_grid));         % Parent map for path reconstruction
    g_costs = inf(size(space_grid));               % Cost from start to each node
    g_costs(start(1), start(2), start(3)) = 0;     % Start node has zero cost
    f_costs = inf(size(space_grid));               % Estimated total cost (g + h)
    f_costs(start(1), start(2), start(3)) = heuristic(start, goal);
    closed_nodes = [];                             % Init to store closed nodes to show ellipse property
    closed_nodes_costs = [];
    path_plan = [];                                % Initiallize the plan to empty vector.
    directions = [
        -1, 0, 0;   % Left
        1, 0, 0;    % Right
        0, -1, 0;   % Down
        0, 1, 0;    % Up
        0, 0, -1;   % Rotate CW
        0, 0, 1;    % Rotate CCW
    ];

    while ~isempty(OPEN)
        % Get the node in OPEN with the lowest f_cost
        [~, idx] = min(cellfun(@(node) f_costs(node(1), node(2), node(3)), OPEN));
        current_node = OPEN{idx};
        OPEN(idx) = []; % Remove from OPEN

        % Add to closed nodes for ellipsoid property plottings
        closed_nodes = [closed_nodes; current_node];
        closed_nodes_costs = [closed_nodes_costs, f_costs(current_node(1), current_node(2),current_node(3))]; 


        % Check if the goal is reached
        if isequal(current_node, goal)
            path_plan = reconstruct_path(parent_nodes, start, goal);
            disp("Found path!")
            disp("Path length: " + length(path_plan))
            if stop_when_found_path 
                save("A_STAR_out", "closed_nodes_costs", "closed_nodes");
                return;
            end
        end

        % Explore neighbors
        for i = 1:size(directions, 1)
            neighbor_node = current_node + directions(i, :);
            neighbor_node(3) = mod(neighbor_node(3) - 1, size(space_grid, 3)) + 1; % Wrap theta

            % Skip if the neighbor is invalid or already visited
            if ~is_valid(neighbor_node, space_grid) || visited(neighbor_node(1), neighbor_node(2), neighbor_node(3))
                continue;
            end

            tentative_g_cost = g_costs(current_node(1), current_node(2), current_node(3)) + 1; % Assume unit cost

            % If this path to neighbor is better, update costs and parent map
            if tentative_g_cost < g_costs(neighbor_node(1), neighbor_node(2), neighbor_node(3))
                g_costs(neighbor_node(1), neighbor_node(2), neighbor_node(3)) = tentative_g_cost;
                f_costs(neighbor_node(1), neighbor_node(2), neighbor_node(3)) = tentative_g_cost + heuristic(neighbor_node, goal);
                parent_nodes{neighbor_node(1), neighbor_node(2), neighbor_node(3)} = current_node;

                % Add neighbor to OPEN if not already there
                if ~any(cellfun(@(node) isequal(node, neighbor_node), OPEN))
                    OPEN{end + 1} = neighbor_node;
                end
            end
        end

        % Mark the current node as visited
        visited(current_node(1), current_node(2), current_node(3)) = true;
    end
    % If we reached here it means all nodes were closed and we did not stop
    % when path was found.
    save("A_STAR_out", "closed_nodes_costs", "closed_nodes"); % for plotting later
    
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

function reconstructed_path = reconstruct_path(parent, start, goal)
    % Reconstruct the path from goal to start using the parent map
    reconstructed_path = {goal};
    current = goal;
    while ~isequal(current, start)
        current = parent{current(1), current(2), current(3)};
        reconstructed_path = [{current}, reconstructed_path];
    end
end
