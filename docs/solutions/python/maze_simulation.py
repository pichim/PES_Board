# line_maze_min_turn.py
# First run only: DFS exploration until goal is found

# Carefull, this solution was generated using a large language model (AI). No further modifications were made.
# It is provided as-is and may not be optimal or fully functional.

import numpy as np
import matplotlib.pyplot as plt

def line_maze_min_turn():
    # Parameters
    rows = 12
    cols = 12

    # Directions: [dy, dx] for [N, E, S, W]
    dirs = np.array([[-1, 0], [0, 1], [1, 0], [0, -1]], dtype=np.int8)

    # Maze Generation
    np.random.seed()
    maze = np.zeros((rows, cols, 4), dtype=bool)
    for r in range(rows):
        for c in range(cols):
            for d in range(4):
                if np.random.rand() < 0.4:
                    dr, dc = dirs[d]
                    r2, c2 = int(r) + int(dr), int(c) + int(dc)
                    if 0 <= r2 < rows and 0 <= c2 < cols:
                        maze[r, c, d] = True
                        maze[r2, c2, (d + 2) % 4] = True

    # First Run - DFS to find goal
    explored = np.zeros((rows, cols), dtype=bool)
    goal = (0, cols - 1)
    start_pos = (rows - 1, 0)
    parent = np.zeros((rows, cols, 2), dtype=np.uint8)
    stack = np.zeros((rows * cols, 2), dtype=np.uint8)
    stack_ptr = 0
    stack[stack_ptr] = start_pos

    found = False
    while stack_ptr >= 0:
        y, x = stack[stack_ptr]
        stack_ptr -= 1

        if explored[y, x]:
            continue
        explored[y, x] = True

        if (y, x) == goal:
            found = True
            break

        for d in range(4):
            dy, dx = dirs[d]
            ny, nx = int(y) + int(dy), int(x) + int(dx)
            if 0 <= ny < rows and 0 <= nx < cols:
                if maze[y, x, d] and not explored[ny, nx]:
                    stack_ptr += 1
                    stack[stack_ptr] = (ny, nx)
                    parent[ny, nx] = (y, x)

    if not found:
        print("No path to goal found!")
        return

    # Reconstruct DFS path
    path = np.zeros((rows * cols, 2), dtype=np.uint8)
    path_len = 0
    current = goal
    while current[0] or current[1]:
        path[path_len] = current
        path_len += 1
        current = tuple(parent[current])
    path = np.flip(path[:path_len], axis=0)

    # Visualization
    plt.figure()
    plt.title('DFS Path to Goal')
    plot_maze(maze, rows, cols)

    # Draw blue path: actual robot exploration path (line from parent to child)
    for i in range(rows):
        for j in range(cols):
            if explored[i, j] and np.any(parent[i, j] != 0):
                py, px = parent[i, j]
                plt.plot([px, j], [py, i], color='blue', linewidth=4)

    # Show DFS path in green on top
    plt.plot(path[:, 1], path[:, 0], color='red', linewidth=2, label='DFS Path')
    plt.scatter(start_pos[1], start_pos[0], s=80, c='b', label='Start')
    plt.scatter(goal[1], goal[0], s=80, c='r', label='Goal')
    plt.gca().set_aspect('equal')
    plt.legend()
    plt.show()

def plot_maze(maze, rows, cols):
    dirs = np.array([[-1, 0], [0, 1], [1, 0], [0, -1]], dtype=np.int8)
    for r in range(rows):
        for c in range(cols):
            for d in range(4):
                if maze[r, c, d]:
                    dr, dc = dirs[d]
                    r2, c2 = r + dr, c + dc
                    if 0 <= r2 < rows and 0 <= c2 < cols:
                        plt.plot([c, c2], [r, r2], 'k')

if __name__ == '__main__':
    line_maze_min_turn()
