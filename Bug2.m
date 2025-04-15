clc; clear; close all;

%% Parameters and Setup
room_width = 130;
room_height = 135;
max_speed = 10;
dt = 0.1;
step = max_speed * dt;
robot_radius = 5;  
sim_time_minutes = 10;
max_iterations = (sim_time_minutes * 60) / dt;

% Obstacles
table1_x = 100; table1_y = 123; table1_width = 30; table1_height = 12;
table2_x = 115; table2_y = 0;   table2_width = 15; table2_height = 22;
bed_x = 0;      bed_y = 18;       bed_width = 79;  bed_height = 57;

grid_resolution = 1;
grid_rows = room_height + 1; grid_cols = room_width + 1;
coverage = false(grid_rows, grid_cols);

% Goal Points (I chose these)
goal_points = [ ...
    0     ,   0;
    114   ,   11;
    115   ,   0;
    0     ,   18;
    39    ,   17;
    115   ,   18;
    79    ,   20;
    115   ,   22;
    123   ,   23;
    79    ,   46;
    80    ,   46;
    130   ,  67.5;
    130   ,  100;
    0     ,   80;
    39    ,   76;
    80    ,   76;
    100   ,  125;
    115   ,  122;
    99    ,  129;
    0     ,  135;
    65    ,  135;
    100   ,  135;
    0     ,  100;
];

% Generate 20 extra free-space points
num_extra_points = 11;
extra_points = [];
while size(extra_points,1) < num_extra_points
    candidate = [room_width * rand(), room_height * rand()];
    if (candidate(1) >= table1_x && candidate(1) <= (table1_x + table1_width) && ...
        candidate(2) >= table1_y && candidate(2) <= (table1_y + table1_height))
        continue;
    end
    if (candidate(1) >= table2_x && candidate(1) <= (table2_x + table2_width) && ...
        candidate(2) >= table2_y && candidate(2) <= (table2_y + table2_height))
        continue;
    end
    if (candidate(1) >= bed_x && candidate(1) <= (bed_x + bed_width) && ...
        candidate(2) >= bed_y && candidate(2) <= (bed_y + bed_height))
        continue;
    end
    extra_points = [extra_points; candidate];
end

goal_points = [goal_points; extra_points];

goal_index = 2;
current_goal = goal_points(goal_index, :);
robot_pos = goal_points(1, :);
current_direction = 0;
robot_path = robot_pos;

ix = min(grid_cols, max(1, floor(robot_pos(1)/grid_resolution) + 1));
iy = min(grid_rows, max(1, floor(robot_pos(2)/grid_resolution) + 1));
coverage(iy, ix) = true;

% Collision check function
check_collision = @(pos) ( pos(1) < 0 || pos(1) > room_width || ...
                           pos(2) < 0 || pos(2) > room_height || ...
                          ((pos(1) >= table1_x) && (pos(1) <= (table1_x + table1_width)) && ...
                           (pos(2) >= table1_y) && (pos(2) <= (table1_y + table1_height))) || ...
                          ((pos(1) >= table2_x) && (pos(1) <= (table2_x + table2_width)) && ...
                           (pos(2) >= table2_y) && (pos(2) <= (table2_y + table2_height))) || ...
                          ((pos(1) >= bed_x) && (pos(1) <= (bed_x + bed_width)) && ...
                           (pos(2) >= bed_y) && (pos(2) <= (bed_y + bed_height))) );

% Bug2 State Setup
mode = 0;  % 0: GO-TO-GOAL, 1: FOLLOW-BOUNDARY
dtheta = pi/36;
tolerance = 2;
hit_point = [];
hit_distance = Inf;

%% Simulation Loop (Calculation Only)
for iter = 1:max_iterations
    if mode == 0  % GO-TO-GOAL
        desired_direction = atan2(current_goal(2) - robot_pos(2), current_goal(1) - robot_pos(1));
        next_pos = robot_pos + step * [cos(desired_direction), sin(desired_direction)];
        if ~check_collision(next_pos)
            robot_pos = next_pos;
            current_direction = desired_direction;
        else
            mode = 1;
            hit_point = robot_pos;
            hit_distance = norm(robot_pos - current_goal);
        end
        if norm(robot_pos - current_goal) < tolerance
            goal_index = goal_index + 1;
            if goal_index > size(goal_points,1)
                break;
            else
                current_goal = goal_points(goal_index, :);
                mode = 0;
            end
        end
    elseif mode == 1  % FOLLOW-BOUNDARY
        found_direction = false;
        new_direction = current_direction;
        for angle_adjust = 0:dtheta:(2*pi)
            candidate_direction = current_direction - angle_adjust;
            candidate_next = robot_pos + step * [cos(candidate_direction), sin(candidate_direction)];
            if ~check_collision(candidate_next)
                found_direction = true;
                new_direction = candidate_direction;
                break;
            end
        end
        if found_direction
            current_direction = new_direction;
            robot_pos = robot_pos + step * [cos(current_direction), sin(current_direction)];
        end
        if ~isempty(hit_point)
            A = hit_point; B = current_goal; P = robot_pos;
            if norm(B - A) > 0
                distance_to_line = abs(det([B-A; P-A])) / norm(B - A);
            else
                distance_to_line = norm(P - A);
            end
            if (distance_to_line < 1) && (norm(P - current_goal) < hit_distance)
                mode = 0;
                hit_point = [];
            end
        end
    end
    robot_path = [robot_path; robot_pos];
    ix_center = min(grid_cols, max(1, floor(robot_pos(1) / grid_resolution) + 1));
    iy_center = min(grid_rows, max(1, floor(robot_pos(2) / grid_resolution) + 1));
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
title('Roomba Bug2 Simulation Final Result');
rectangle('Position', [0, 0, room_width, room_height], 'EdgeColor', 'k', 'LineWidth', 2);
rectangle('Position', [table1_x, table1_y, table1_width, table1_height], 'FaceColor', [0.5 0.7 1], 'EdgeColor', 'b', 'LineWidth', 2);
text(table1_x + table1_width/2, table1_y + table1_height/2, 'Table 1', 'HorizontalAlignment', 'center');
rectangle('Position', [table2_x, table2_y, table2_width, table2_height], 'FaceColor', [0.7 1 0.7], 'EdgeColor', 'g', 'LineWidth', 2);
text(table2_x + table2_width/2, table2_y + table2_height/2, 'Table 2', 'HorizontalAlignment', 'center');
rectangle('Position', [bed_x, bed_y, bed_width, bed_height], 'FaceColor', [1 0.7 0.7], 'EdgeColor', 'r', 'LineWidth', 2);
text(bed_x + bed_width/2, bed_y + bed_height/2, 'Bed', 'HorizontalAlignment', 'center');
plot(goal_points(:,1), goal_points(:,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot(robot_path(:,1), robot_path(:,2), 'b-', 'LineWidth', 2);
plot(robot_pos(1), robot_pos(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
theta = linspace(0, 2*pi, 100);
plot(robot_pos(1) + robot_radius * cos(theta), robot_pos(2) + robot_radius * sin(theta), 'b--');
text(10, room_height - 10, sprintf('Coverage: %.1f%%', coverage_percent), 'FontSize', 12, 'Color', 'm');
