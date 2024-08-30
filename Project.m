%% Q1
% ---------------------------------------------------------------------- %
clearvars; clc; close all;
% ---------------------------------------------------------------------- %

% Define polygons via vertices in cc order (original == without rotation)
FrameA  = repmat([4 24], [4, 1]);
FrameB1 = repmat([0 18], [4, 1]);
A_org   = FrameA + [0, 0; 8, 0; 8, 1; 0, 1]; % (x, y) coords, nA = 4 vertices
B1  = FrameB1 + [0, 0; 10, 0; 10, 1; 0, 1]; % nB1 = 4 vertices

% Compute the normal edges of A and B1
vA_org  = [0 -1; 1 0; 0 1; -1 0]; % original normal edges of A -> [vA1; vA2; vA3; vA4], nA = 4 vertices
vB1 = [0 -1; 1 0; 0 1; -1 0]; % normal edges of B1 -> [vB1_1; vB1_2; vB1_3; vB1_4], 

% apply a rotation of theta only to A because the Boundary is fixed.
thetas = linspace(0, 2*pi - 2*pi/32, 32);
for slice=1:32
    theta=thetas(slice);
    R3d = axang2rotm([0, 0, 1, theta]); % like a bowass
    R = R3d(1:2, 1:2); % go to 2D
    vA = transpose(R*vA_org'); % update/override vA to rotated vA 
    A = FrameA + transpose(R*(A_org - FrameA)'); % update/override A to rotated A
    CB = getCB(A, B1, vA, vB1);

    % Plot selected slices
    if (slice==1)||(slice==8)||(slice==16)||(slice==32)
        createStandardPlot(slice, theta); % custom function for standard plotting

        % draw the rotated robot on one of the CB points
        FrameFeature = repmat([CB(1,1), CB(1,2)], [4 1]);
        A_featured = A - FrameA + FrameFeature; % move robot origin to some point on CB
        Ax_ax = R*[1; 0]; Ay_ax = R*[0; 1];
        fill(A_featured(:,1), A_featured(:,2), 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Robot $A$'); % Polygon A
        fill(B1(:,1), B1(:,2), 'black', 'FaceAlpha', 0.5, 'DisplayName', '$B1$'); % Polygon B
        plot(CB(:,1), CB(:,2), 'r-.', 'DisplayName', '$CB1$', 'LineWidth', 1); % CB 
        quiver(A_featured(1,1), A_featured(1,2), Ax_ax(1), Ax_ax(2), 'c', 'filled', 'LineWidth', 1.5, ...
            "DisplayName", "$\hat {x}_A$", "MaxHeadSize", 1)
        quiver(A_featured(1,1), A_featured(1,2), Ay_ax(1), Ay_ax(2), 'm', 'filled', 'LineWidth', 1.5, ...
            "DisplayName", "$\hat {y}_A$", "MaxHeadSize", 1)
        maxx = max([CB(:,1); B1(:,1); A_featured(:,1)]); minx = min([CB(:,1); B1(:,1); A_featured(:,1)]); 
        xlim([minx-1, maxx+1]);
        axis equal; grid on; legend("Location","eastoutside")
    end
end


%% Q2
% ---------------------------------------------------------------------- %
clearvars; clc; close all;
% ---------------------------------------------------------------------- %

% redefine obstacles in world frame
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
v_org = [0 -1; 1 0; 0 1; -1 0]; % all polygons have same noramal edge vectors

% Find all obstacle CB
CB_Combined = num2cell(zeros(length(B_All), 32)); % init by length of B_all x 32 slices

% Loop over all obstacles over all slices
thetas = linspace(0, 2*pi - 2*pi/32, 32);
for obst_inx=1:length(B_All)
    for slice=1:32
        theta=thetas(slice);
        R3d = axang2rotm([0, 0, 1, theta]); % like a bowass
        R = R3d(1:2, 1:2); % go to 2D
        vRobot = transpose(R*v_org'); % update/override v_org to rotated vRobot 
        Robot = FrameA + transpose(R*(A_org - FrameA)'); % update/override A to rotated A
        CB = getCB(Robot, B_All{obst_inx}(:,:), vRobot, v_org);
        CB_Combined{obst_inx, slice} = CB; 
    end
end

% Plot all obstacles for selected slices
% function to convert index of obstacles to their real names
obst_inx_to_cobst = @(n)(n*(n<=7)+(0.1)*(n==8)+0.2*(n==9)+0.3*(n==10)+0.4*(n==11));
for slice=[1, 8, 16, 32]
    createStandardPlot(slice, thetas(slice)); % custom function for standard plotting
    for obst_inx=1:length(B_All)
        CB = CB_Combined{obst_inx, slice};
        cobst = replace(string(obst_inx_to_cobst(obst_inx)), ".", ""); % name CB01 instead of CB0.1
        fill(B_All{obst_inx}(:,1), B_All{obst_inx}(:,2), 'black', 'FaceAlpha', 0.4, 'HandleVisibility','off') % 'DisplayName', 'B' + string(obst_inx)); % Polygon B
        plot(CB(:,1), CB(:,2), 'DisplayName', '$CB$'+ cobst, 'LineWidth', 1, 'LineStyle','-.'); % CB 
    end
    axis equal; grid minor; legend("Location","eastoutside")
end

% % add the robot to the plot in some configuration
% CB_Featured = CB_Combined{6, slice};
% FrameFeature = repmat([CB_Featured(1,1), CB_Featured(1,2)], [4 1]);
% Robot_featured = Robot - FrameA + FrameFeature; % move robot origin to some point on CB
% Robotx_ax = R*[1; 0]; Roboty_ax = R*[0; 1];
% fill(Robot_featured(:,1), Robot_featured(:,2), 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Robot $A$'); % Polygon A
% quiver(Robot_featured(1,1), Robot_featured(1,2), Robotx_ax(1), Robotx_ax(2), 'c', 'filled', 'LineWidth', 1.5, ...
%     "DisplayName", "$\hat {x}_A$", "MaxHeadSize", 1, 'HandleVisibility','off')
% quiver(Robot_featured(1,1), Robot_featured(1,2), Roboty_ax(1), Roboty_ax(2), 'm', 'filled', 'LineWidth', 1.5, ...
%     "DisplayName", "$\hat {y}_A$", "MaxHeadSize", 1, 'HandleVisibility','off')

%% Q3
% ---------------------------------------------------------------------- %
clc; % close all; % to show both Q2 and Q3 on same plots for selected slices
% ---------------------------------------------------------------------- %
space_grid = zeros(32,32,32);

% create a set of points to test for in each CB
test_points = table2array(combinations(linspace(1, 32, 32), linspace(1, 32, 32)));
% ------------------ NOTICE -------------------- %
% the test points start at the (1,1) location and end at the (32,32)
% location. Only these points will be tested, so if a finer resolution is
% needed this is an issu,e because it was assumed that the test_points
% indeces are also the (x, y) locations, so one does not simply change the
% linspace because then it will not be a valid index. 
% -------------------------------------------- %
for slice=1:32
    % fill the grid with ones for obstacles and zeros are by default.
    for obst_inx=1:length(B_All)
        % compute for every boundary within given slice all the points in cb
        CB = CB_Combined{obst_inx, slice};
        inx_in_cb = inhull(test_points, CB, convhulln(CB), 0); % find the points contained in the CB convex hull of a polygon.
        xy_in_cb = test_points(inx_in_cb , :);
        for x=1:32
            for y=1:32
                if space_grid(x, y, slice)==0
                   space_grid(x, y, slice) = 1 * ismember([x, y], xy_in_cb, "rows");
                end
            end
        end

    end
end

% Mannual override 
space_grid(:, 31:32, :) = 1; % because it is out of the wall for all thetas.
space_grid(:, 1, :) = 1; % wall
space_grid(:, 30, :) = 1; % wall 
space_grid(1, :, :) = 1; % wall
space_grid(32, :, :) = 1; % wall

% Plotting
for slice_to_plot = [1 8 16 32]
    % add boundaries to exsiting figures
    createStandardPlot(slice_to_plot, thetas(slice_to_plot)) 
    [x,y] = find(space_grid(:,:,slice_to_plot) == 1);
    plot(x,y,'r+', 'HandleVisibility','off')
    axis equal; grid on; % legend("off") if in separate plots
end
