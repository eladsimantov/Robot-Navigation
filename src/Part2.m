% ---------------------------------------------- %
clearvars;          clc;            close all;
% ----------------- INPUTS --------------------- %
xy_resolution       =   32;
theta_resolution    =   64;
B1 = [0 18; 10 18; 10 19; 0 19];
B2 = [17 17; 18 17; 18 29; 17 29];
B3 = [25 18; 32 18; 32 19; 25 19];
B4 = [0 14; 19 14; 19 15; 0 15];
B5 = [24 13; 32 13; 32 15; 24 15];
B6 = [10 19; 12 19; 12 20; 10 20];
B7 = [23 19; 25 19; 25 20; 23 20];
B01 = [0 29; 32 29; 32 30; 0 30];
B02 = [0 0; 1 0; 1 30; 0 30];
B03 = [0 0; 32 0; 32 1; 0 1];
B04 = [31 0; 32 0; 32 30; 31 30];

% redefine robot and his frame
FrameA  = repmat([4 24], [4, 1]);
A_org   = FrameA + [0, 0; 8, 0; 8, 1; 0, 1]; % (x, y) coords, nA = 4 vertices

% Create a cell to include all boundary in single datastructure
B_All = {B1; B2; B3; B4; B5; B6; B7; B01; B02; B03; B04};
% B_All = {B3}; RUN THIS FOR SHOWING MODIFIED TASK FOR ELLIPSOID SECTION
v_org = [0 -1; 1 0; 0 1; -1 0]; % all polygons have same noramal edge vectors
CB_Combined = num2cell(zeros(length(B_All), 32)); % init by length of B_all x 32 slices

% Loop over all obstacles over all slices
thetas = linspace(0, 2*pi - 2*pi/theta_resolution, theta_resolution); % Define vector of thetas 
for obst_inx=1:length(B_All)
    for slice=1:theta_resolution
        theta=thetas(slice);
        R3d = axang2rotm([0, 0, 1, theta]); % like a bowass
        R = R3d(1:2, 1:2); % go to 2D
        vRobot = transpose(R*v_org'); % update/override v_org to rotated vRobot 
        Robot = FrameA + transpose(R*(A_org - FrameA)'); % update/override A to rotated A
        CB = getCB(Robot, B_All{obst_inx}(:,:), vRobot, v_org);
        CB_Combined{obst_inx, slice} = CB; 
    end
end

% % Plot all obstacles for selected slices
% obst_inx_to_cobst = @(n)(n*(n<=7)+(0.1)*(n==8)+0.2*(n==9)+0.3*(n==10)+0.4*(n==11)); % function to convert index of obstacles to their real names
% for slice=[1, 8, 16, 32]
%     createStandardPlot(slice, thetas(slice)); % custom function for standard plotting
%     for obst_inx=1:length(B_All)
%         CB = CB_Combined{obst_inx, slice};
%         cobst = replace(string(obst_inx_to_cobst(obst_inx)), ".", ""); % name CB01 instead of CB0.1
%         fill(B_All{obst_inx}(:,1), B_All{obst_inx}(:,2), 'black', 'FaceAlpha', 0.4, 'HandleVisibility','off') % 'DisplayName', 'B' + string(obst_inx)); % Polygon B
%         plot(CB(:,1), CB(:,2), 'DisplayName', '$CB$'+ cobst, 'LineWidth', 1, 'LineStyle','-.'); % CB 
%     end
%     axis equal; grid minor; legend("Location","eastoutside")
% end

space_grid = zeros(xy_resolution,xy_resolution,theta_resolution); % init the grid

test_points_indeces = table2array(combinations(linspace(1, xy_resolution, xy_resolution), linspace(1, xy_resolution, xy_resolution)));
test_points = test_points_indeces*32/xy_resolution;

for slice=1:theta_resolution
    % fill the grid with ones for obstacles and zeros are by default.
    for obst_inx=1:length(B_All)
        % compute for every boundary within given slice all the points in cb
        CB = CB_Combined{obst_inx, slice};
        [IN, ON] = inpolygon(test_points(:,1), test_points(:,2), CB(:,1), CB(:, 2));        
        xy_in_cb = test_points(IN , :);
        xy_on_cb = test_points(ON , :);
        
        for x_index=1:xy_resolution
            for y_index=1:xy_resolution
                x_value = x_index * 32 / xy_resolution;
                y_value = y_index * 32 / xy_resolution;
                if space_grid(x_index, y_index, slice)==0
                    not_on_CB = ~ismember([x_value, y_value], xy_on_cb, "rows");
                    in_the_CB = ismember([x_value, y_value], xy_in_cb, "rows");
                    space_grid(x_index, y_index, slice) = in_the_CB * not_on_CB;
                end
            end
        end
    end
end

% Mannual override
space_grid(:, 31, :) = 1; % because it is out of the wall for all thetas.
space_grid(:, 1, :) = 1; % wall
space_grid(33, :, :) = 1; % wall
space_grid(1, :, :) = 1; % wall

% % Plotting
% for slice_to_plot = [1, 8, 16, 32]
%     % add boundaries to exsiting figures
%     createStandardPlot(slice_to_plot, thetas(slice_to_plot)) 
%     [x_indeces,y_indeces] = find(space_grid(:,:,slice_to_plot) == 1);
%     x_values = x_indeces * 32 / xy_resolution;
%     y_values = y_indeces * 32 / xy_resolution;
%     plot(x_values, y_values ,'r+', 'HandleVisibility','off')
%     axis equal; grid on; % legend("off") if in separate plots
% end

%% Navigation 
% We chose to use the A* algorithm, after running both BFS and DFS.
clc; close all;
start   = [4, 24, 1];
goal    = [4, 8, 1];

path_plan = astar_pathfinding(space_grid, start, goal)';
% path_plan = astar_pathfinding(space_grid, start, goal, false)'; % To show the ellipsoid we want to continue closing nodes and not finish when path is found..
% path_plan = bfs_pathfinding(space_grid, start, goal)';
% path_plan = dfs_pathfinding(space_grid, start, goal)';

%% Plot
load("A_STAR_out.mat"); clc; close all; 
figure(999); hold on; box on; axis equal; legend('off'); grid minor;
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(gcf, "Color", "w")
% Draw all obstacles in 2D grid
for obst_inx=1:length(B_All)
    fill(B_All{obst_inx}(:,1), B_All{obst_inx}(:,2), 'black', 'FaceAlpha', 0.4, 'HandleVisibility','off') % 'DisplayName', 'B' + string(obst_inx)); % Polygon B
end

for idx_point=1:length(path_plan)
    point = path_plan(idx_point);
    % Find 4 points of robot and plot fill
    xp = point{1}(1) * 32 / xy_resolution; 
    yp = point{1}(2) * 32 / xy_resolution; 
    theta = thetas(point{1}(3));

    R3d = axang2rotm([0, 0, 1, theta]);     % like a bowass
    R = R3d(1:2, 1:2);                      % go to 2D
    A = repmat([xp, yp], [4,1]) + transpose(R*(A_org - FrameA)');

    % Add robot to drawing
    fill(A(:,1), A(:,2), 'k-', 'FaceAlpha', 0, 'DisplayName', 'Robot $A$'); % Polygon A
    drawnow
    pause(0.002)
end

%% Plot A* ellipse section
% -------------- NOTICE -------------- %
% (1) Make sure to rerun the program with the 3rd boundary only for a clear path.
% (2) Make sure to input to the astar_pathfinding not to stop when path is found.
numPoints = 27253; % This will work only if you disabled the boundaries and kept the 3rd one only.
plot_closed_nodes(start, goal, closed_nodes(1:numPoints,:), closed_nodes_costs(1:numPoints));
set(gcf, "Color", 'w');
figure; plot(closed_nodes_costs(1:numPoints)); xlabel('Close episode'); ylabel('Total Cost (f=g+h)');
title('Closed nodes costs in A* Search'); grid on; set(gcf, "Color", 'w');


%% Offline vs Online A* section
% Housekeeping
clc; close all; clearvars -except A_org B_All path_plan CB_Combined FrameA goal start v_org xy_resolution theta_resolution space_grid thetas

% conduct an online path planner
online_path_plan = online_astar_pathfinding(space_grid, start, goal);

%% Plot online pathplan

figure(998); hold on; box on; axis equal; legend('off'); grid minor;
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(gcf, "Color", "w")
% Draw all obstacles in 2D grid
for obst_inx=1:length(B_All)
    fill(B_All{obst_inx}(:,1), B_All{obst_inx}(:,2), 'black', 'FaceAlpha', 0.4, 'HandleVisibility','off') % 'DisplayName', 'B' + string(obst_inx)); % Polygon B
end

for idx_point=1:length(online_path_plan)
    point = online_path_plan(idx_point);
    % Find 4 points of robot and plot fill
    xp = point{1}(1) * 32 / xy_resolution; 
    yp = point{1}(2) * 32 / xy_resolution; 
    theta = thetas(point{1}(3));

    R3d = axang2rotm([0, 0, 1, theta]);     % like a bowass
    R = R3d(1:2, 1:2);                      % go to 2D
    A = repmat([xp, yp], [4,1]) + transpose(R*(A_org - FrameA)');

    % Add robot to drawing
    fill(A(:,1), A(:,2), 'k-', 'FaceAlpha', 0, 'DisplayName', 'Robot $A$'); % Polygon A
    drawnow
    pause(0.002)
end


%% Functions

function plot_closed_nodes(start, goal, closed_nodes, closed_nodes_costs)
    % Plot the closed nodes in the search to visualize the ellipsoid shape
    figure;
    scatter3(start(1), start(2), start(3), 'filled', 'o'); hold on;
    scatter3(goal(1), goal(2), goal(3), 'filled', 'o');
    % [C, IA, IC] = unique(closed_nodes_costs); % unique values to show in each plot.
    
    % % loop over number of unique values
    % for i=1:length(C)-1
    %     start_index = IA(i);
    %     end_index = IA(i+1) - 1;
    %     numPoints = end_index - start_index + 1;
    %     plot3(closed_nodes(start_index:end_index, 1), closed_nodes(start_index:end_index, 2), ones(1,numPoints)*C(i));
    %     drawnow;
    %     pause(0.1);
    % end
    scatter3(closed_nodes(:, 1), closed_nodes(:, 2), closed_nodes_costs, "black", 'Marker','.');
    xlabel('X');
    ylabel('Y');
    zlabel('Costs');
    title('Closed Nodes in A* Search');
    grid on;
end


