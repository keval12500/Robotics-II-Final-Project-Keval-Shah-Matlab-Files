clc; clear; close all;

%% Parameters
room_width = 130;
room_height = 135;
max_speed = 6;
dt = 0.1;
step = max_speed * dt;
robot_radius = 5;  % cleaning footprint radius (inches)
sim_time_minutes = 10;
max_iterations = (sim_time_minutes * 60) / dt;

%% Obstacles
% Table 1
table1_x = 100;
table1_y = 123;
table1_width = 130 - 100;  % 30
table1_height = 135 - 123; % 12

% Table 2
table2_x = 115;
table2_y = 0;
table2_width = 130 - 115;  % 15
table2_height = 22;        % 22

% Bed
bed_x = 0;
bed_y = 18;
bed_width = 79;
bed_height = 75 - 18;      % 57

%% Coverage Grid
grid_resolution = 1;
grid_rows = room_height + 1;
grid_cols = room_width + 1;
coverage = false(grid_rows, grid_cols);

%% Initial Conditions
robot_pos = [10, 10];
current_direction = 0; % radians
robot_path = robot_pos;

% Setup initial coverage at starting position
ix = min(grid_cols, max(1, floor(robot_pos(1)/grid_resolution) + 1));
iy = min(grid_rows, max(1, floor(robot_pos(2)/grid_resolution) + 1));
coverage(iy, ix) = true;

%% Plot Setup (Final Plot Only)
% (The figure is created after simulation completes)

%% Collision Check Function
check_collision = @(pos) ( pos(1) < 0 || pos(1) > room_width || ...
                           pos(2) < 0 || pos(2) > room_height || ...
                          ((pos(1) >= table1_x) && (pos(1) <= (table1_x + table1_width)) && ...
                           (pos(2) >= table1_y) && (pos(2) <= (table1_y + table1_height))) || ...
                          ((pos(1) >= table2_x) && (pos(1) <= (table2_x + table2_width)) && ...
                           (pos(2) >= table2_y) && (pos(2) <= (table2_y + table2_height))) || ...
                          ((pos(1) >= bed_x) && (pos(1) <= (bed_x + bed_width)) && ...
                           (pos(2) >= bed_y) && (pos(2) <= (bed_y + bed_height))) );

%% Simulation Loop (Offline Calculation)
for iter = 1:max_iterations
    % Calculate the next position in the current direction
    next_pos = robot_pos + step * [cos(current_direction), sin(current_direction)];
    
    % If next step results in collision, find a new random direction
    if check_collision(next_pos)
        safe_direction_found = false;
        for k = 1:100
            current_direction = 2*pi * rand();
            next_pos = robot_pos + step * [cos(current_direction), sin(current_direction)];
            if ~check_collision(next_pos)
                safe_direction_found = true;
                break;
            end
        end
        if ~safe_direction_found
            continue;  % Stay in place if no safe direction found
        end
    end
    
    % Move robot and record path
    robot_pos = next_pos;
    robot_path = [robot_path; robot_pos];
    
    % Update coverage grid for the robot's cleaning footprint
    ix_center = min(grid_cols, max(1, floor(robot_pos(1)/grid_resolution) + 1));
    iy_center = min(grid_rows, max(1, floor(robot_pos(2)/grid_resolution) + 1));
    ix_range = max(1, ix_center - ceil(robot_radius / grid_resolution)) : min(grid_cols, ix_center + ceil(robot_radius / grid_resolution));
    iy_range = max(1, iy_center - ceil(robot_radius / grid_resolution)) : min(grid_rows, iy_center + ceil(robot_radius / grid_resolution));
    for ix = ix_range
        for iy = iy_range
            cell_x = (ix - 1) * grid_resolution;
            cell_y = (iy - 1) * grid_resolution;
            if norm([cell_x, cell_y] - robot_pos) <= robot_radius
                coverage(iy, ix) = true;
            end
        end
    end
end

covered_cells = sum(coverage(:));
total_cells = grid_rows * grid_cols;
coverage_percent = (covered_cells / total_cells) * 100;

%% Final Plot
figure; hold on; axis equal;
xlim([0 room_width]); ylim([0 room_height]);
xlabel('X (inches)'); ylabel('Y (inches)');
title('Roomba Simulation (Random Direction, 10 min Sim) Final Result');
rectangle('Position', [0, 0, room_width, room_height], 'EdgeColor', 'k', 'LineWidth', 2);
rectangle('Position', [table1_x, table1_y, table1_width, table1_height], 'FaceColor', [0.5 0.7 1], 'EdgeColor', 'b', 'LineWidth', 2);
text(table1_x + table1_width/2, table1_y + table1_height/2, 'Table 1', 'HorizontalAlignment', 'center');
rectangle('Position', [table2_x, table2_y, table2_width, table2_height], 'FaceColor', [0.7 1 0.7], 'EdgeColor', 'g', 'LineWidth', 2);
text(table2_x + table2_width/2, table2_y + table2_height/2, 'Table 2', 'HorizontalAlignment', 'center');
rectangle('Position', [bed_x, bed_y, bed_width, bed_height], 'FaceColor', [1 0.7 0.7], 'EdgeColor', 'r', 'LineWidth', 2);
text(bed_x + bed_width/2, bed_y + bed_height/2, 'Bed', 'HorizontalAlignment', 'center');
plot(robot_path(:,1), robot_path(:,2), 'b-', 'LineWidth', 2);
plot(robot_path(end,1), robot_path(end,2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
theta = linspace(0, 2*pi, 100);
plot(robot_path(end,1) + robot_radius * cos(theta), robot_path(end,2) + robot_radius * sin(theta), 'b--');
text(10, room_height - 10, sprintf('Coverage: %.1f%%', coverage_percent), 'FontSize', 12, 'Color', 'm');
