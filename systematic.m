clc; clear; close all;

%% Parameters and Setup
room_width = 130;
room_height = 135;
roomba_radius = 5;
speed = 6;
dt = 0.05;
step_size = speed * dt;
max_sim_time = 600;  % seconds

% Obstacles
table1 = [100, 123, 30, 12];
table2 = [115, 0, 15, 22];
bed    = [0, 18, 79, 57];

% Prepare figure and draw room and obstacles
figure; hold on; axis equal;
xlim([-10, room_width+10]); ylim([-10, room_height+10]);
title('Room Layout, Roomba Path & Cleaning Coverage');
xlabel('X (inches)'); ylabel('Y (inches)');
grid on;
rectangle('Position',[0, 0, room_width, room_height], 'EdgeColor','k','LineWidth',2);
rectangle('Position', table1, 'EdgeColor','b','FaceColor',[0.5 0.7 1],'LineWidth',2);
text(table1(1)+table1(3)/2, table1(2)+table1(4)/2, 'Table 1', 'HorizontalAlignment','center');
rectangle('Position', table2, 'EdgeColor','g','FaceColor',[0.7 1 0.7],'LineWidth',2);
text(table2(1)+table2(3)/2, table2(2)+table2(4)/2, 'Table 2', 'HorizontalAlignment','center');
rectangle('Position', bed, 'EdgeColor','r','FaceColor',[1 0.7 0.7],'LineWidth',2);
text(bed(1)+bed(3)/2, bed(2)+bed(4)/2, 'Bed', 'HorizontalAlignment','center');

% Cleaning coverage grid setup (1-inch resolution)
[xGrid, yGrid] = meshgrid(0:room_width, 0:room_height);
freeMask = true(size(xGrid));
freeMask( xGrid >= bed(1)   & xGrid <= (bed(1)+bed(3))   & ...
          yGrid >= bed(2)   & yGrid <= (bed(2)+bed(4)) ) = false;
freeMask( xGrid >= table1(1)& xGrid <= (table1(1)+table1(3)) & ...
          yGrid >= table1(2)& yGrid <= (table1(2)+table1(4)) ) = false;
freeMask( xGrid >= table2(1)& xGrid <= (table2(1)+table2(3)) & ...
          yGrid >= table2(2)& yGrid <= (table2(2)+table2(4)) ) = false;
cleaned = false(size(xGrid));
total_free_cells = sum(freeMask(:));

% Roomba initial conditions and path storage
roomba_pos = [5, 5];
trail_x = roomba_pos(1);
trail_y = roomba_pos(2);
sim_time = 0;

%% Offline Simulation Loop (Sweeping Pattern)
sweep_direction = 1;  % 1: moving right, -1: moving left
row_spacing = 6;      % vertical step (inches)
current_row = roomba_pos(2);

while sim_time < max_sim_time
    mask_circle = ((xGrid - roomba_pos(1)).^2 + (yGrid - roomba_pos(2)).^2) <= roomba_radius^2;
    cleaned = cleaned | (mask_circle & freeMask);
    percent_coverage = (sum(cleaned(:)) / total_free_cells) * 100;
    if percent_coverage >= 100
        break;
    end
    
    new_pos = roomba_pos + [step_size * sweep_direction, 0];
    
    if new_pos(1) <= 0 || new_pos(1) >= room_width
        current_row = current_row + row_spacing;
        if current_row > room_height
            break;
        end
        roomba_pos = [roomba_pos(1), current_row];
        sweep_direction = -sweep_direction;
    else
        if isInsideObstacle(new_pos, bed) || isInsideObstacle(new_pos, table1) || isInsideObstacle(new_pos, table2)
            current_row = current_row + row_spacing;
            if current_row > room_height
                break;
            end
            roomba_pos = [roomba_pos(1), current_row];
            sweep_direction = -sweep_direction;
        else
            roomba_pos = new_pos;
        end
    end
    
    trail_x(end+1) = roomba_pos(1);
    trail_y(end+1) = roomba_pos(2);
    sim_time = sim_time + dt;
end

%% Final Plot
plot(trail_x, trail_y, 'r-', 'LineWidth', 1);
plot(roomba_pos(1), roomba_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
final_coverage_text = sprintf('Coverage: %.2f%%', percent_coverage);
text(5, room_height+5, final_coverage_text, 'FontSize', 12, 'Color', 'k', 'BackgroundColor', 'w');

%% Function: Check if a Position is Inside an Obstacle
function inside = isInsideObstacle(pos, obstacle)
    inside = pos(1) >= obstacle(1) && pos(1) <= (obstacle(1)+obstacle(3)) && ...
             pos(2) >= obstacle(2) && pos(2) <= (obstacle(2)+obstacle(4));
end
