% maze_simulation.m
% First run only: DFS exploration until goal is found

% Carefull, this solution was generated using a large language model (AI). No further modifications were made.
% It is provided as-is and may not be optimal or fully functional.

function maze_simulation()
    %% Parameters
    rows = 12;
    cols = 12;

    % Directions: [dy, dx] for [N, E, S, W]
    dirs = int8([ -1  0;
                   0  1;
                   1  0;
                   0 -1]);

    %% Maze Generation
    rng('shuffle');
    maze = false(rows, cols, 4);
    for r = 1:rows
        for c = 1:cols
            for d = 1:4
                if rand() < 0.4
                    dr = dirs(d, 1); dc = dirs(d, 2);
                    r2 = int16(r) + int16(dr);
                    c2 = int16(c) + int16(dc);
                    if r2 >= 1 && r2 <= rows && c2 >= 1 && c2 <= cols
                        maze(r, c, d) = true;
                        opp_d = mod(d + 1, 4) + 1; % Opposite direction
                        maze(r2, c2, opp_d) = true;
                    end
                end
            end
        end
    end

    %% First Run - DFS to find goal
    explored = false(rows, cols);
    goal = [1, cols];
    start_pos = [rows, 1];
    parent = zeros(rows, cols, 2, 'uint8');
    stack = zeros(rows * cols, 2, 'uint8');
    stack_ptr = 1;
    stack(stack_ptr, :) = start_pos;

    found = false;
    while stack_ptr > 0
        current = stack(stack_ptr, :);
        stack_ptr = stack_ptr - 1;
        y = current(1); x = current(2);

        if explored(y, x), continue; end
        explored(y, x) = true;

        if y == goal(1) && x == goal(2)
            found = true;
            break;
        end

        for d = 1:4
            dy = dirs(d, 1); dx = dirs(d, 2);
            ny = double(y) + double(dy);
            nx = double(x) + double(dx);
            if ny >= 1 && ny <= rows && nx >= 1 && nx <= cols
                if maze(y, x, d) && ~explored(ny, nx)
                    stack_ptr = stack_ptr + 1;
                    stack(stack_ptr, :) = [ny, nx];
                    parent(ny, nx, :) = [y, x];
                end
            end
        end
    end

    if ~found
        disp('No path to goal found!');
        return;
    end

    %% Reconstruct DFS path
    path = zeros(rows * cols, 2, 'uint8');
    path_len = 0;
    current = goal;
    while any(current)
        path_len = path_len + 1;
        path(path_len, :) = current;
        current = squeeze(parent(current(1), current(2), :))';
    end
    path = flipud(path(1:path_len, :));

    %% Visualization
    figure;
    title('DFS Path to Goal');
    hold on;
    plot_maze(maze, rows, cols);

    % Draw blue path: actual robot exploration path (line from parent to child)
    for i = 1:rows
        for j = 1:cols
            if explored(i, j) && any(parent(i, j, :) ~= 0)
                py = parent(i, j, 1);
                px = parent(i, j, 2);
                plot([px, j], [py, i], 'b-', 'LineWidth', 4);
            end
        end
    end

    % Show DFS path in green on top
    plot(path(:, 2), path(:, 1), 'r', 'LineWidth', 2);
    scatter(start_pos(2), start_pos(1), 80, 'b', 'filled');
    scatter(goal(2), goal(1), 80, 'r', 'filled');
    axis equal;
    legend('DFS Path');
    hold off;
end

function plot_maze(maze, rows, cols)
    dirs = [-1 0; 0 1; 1 0; 0 -1];
    for r = 1:rows
        for c = 1:cols
            for d = 1:4
                if maze(r, c, d)
                    dr = dirs(d, 1); dc = dirs(d, 2);
                    r2 = r + dr; c2 = c + dc;
                    if r2 >= 1 && r2 <= rows && c2 >= 1 && c2 <= cols
                        plot([c, c2], [r, r2], 'k');
                    end
                end
            end
        end
    end
end
