import numpy as np 

import matplotlib.pyplot as plt 

from collections import deque 

 

# Define grid size 

GRID_WIDTH = 50 

GRID_HEIGHT = 25 

 

# Define movement directions (Up, Down, Left, Right) 

DIRECTIONS = [(0, 1), (0, -1), (1, 0), (-1, 0)] 

 

# Define obstacles (expanded by 1 unit for robot clearance) 

OBSTACLES = [(4, 5), (20, 20)]  # Given obstacles (midpoints) 

OBSTACLE_SIZE = 4  # Since robot has a 2-unit diameter, expand obstacles 

obstacle_cells = set() 

for ox, oy in OBSTACLES: 

    for dx in range(-OBSTACLE_SIZE // 2, OBSTACLE_SIZE // 2): 

        for dy in range(-OBSTACLE_SIZE // 2, OBSTACLE_SIZE // 2): 

            nx, ny = ox + dx, oy + dy 

            if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT: 

                obstacle_cells.add((nx, ny))


# Define start point, checkpoints, and goal 

START_POINT = (1, 1) 

CHECKPOINTS = [(8, 2),(2,20)]  # Given checkpoints 

GOAL_POINT = (10, 5)  # Given goal point (clamped within bounds) 

 

 

def expand_obstacles(obstacles, grid_width, grid_height): 

    """ 

    Expands each obstacle by 1 unit in all directions to ensure clearance. 

    Returns a set of blocked cells. 

    """ 

    blocked_cells = set() 

    for ox, oy in obstacles: 

        for dx in range(-OBSTACLE_SIZE // 2 - 1, OBSTACLE_SIZE // 2 + 1): 

            for dy in range(-OBSTACLE_SIZE // 2 - 1, OBSTACLE_SIZE // 2 + 1): 

                nx, ny = ox + dx, oy + dy 

                if 0 <= nx < grid_width and 0 <= ny < grid_height: 

                    blocked_cells.add((nx, ny)) 

    return blocked_cells 

 

 

# Expand obstacles 

BLOCKED_CELLS = expand_obstacles(OBSTACLES, GRID_WIDTH, GRID_HEIGHT) 

 

 

def bfs(start, goals, blocked_cells, grid_width, grid_height): 

    """ 

    Performs BFS to find the shortest path to reach all goal points in any order. 

    Returns a dictionary mapping each goal to its shortest path. 

    """ 

    queue = deque([(start, [])])  # (current_position, path) 

    visited = set([start]) 

 

    paths = {}  # Store shortest paths to each goal 

 

    while queue and len(paths) < len(goals): 

        (x, y), path = queue.popleft() 

 

        # If reached a goal, store the path 

        if (x, y) in goals and (x, y) not in paths: 

            paths[(x, y)] = path + [(x, y)] 

            if len(paths) == len(goals):  # Stop early if all goals reached 

                break 

 

        for dx, dy in DIRECTIONS: 

            nx, ny = x + dx, y + dy 

 

            if (0 <= nx < grid_width and 0 <= ny < grid_height and 

                    (nx, ny) not in blocked_cells and (nx, ny) not in visited): 

                queue.append(((nx, ny), path + [(x, y)])) 

                visited.add((nx, ny)) 

 

    return paths 

# Step 1: Find paths to all checkpoints 

checkpoint_paths = bfs(START_POINT, CHECKPOINTS, BLOCKED_CELLS, GRID_WIDTH, GRID_HEIGHT) 

 

# Step 2: Choose the closest checkpoint as the next start point 

sorted_checkpoints = sorted(CHECKPOINTS, key=lambda p: len(checkpoint_paths[p])) 

path_sequence = [] 

current_position = START_POINT 

 

for checkpoint in sorted_checkpoints: 

    # Find path from current position to the next checkpoint 

    segment_path = bfs(current_position, [checkpoint], BLOCKED_CELLS, GRID_WIDTH, GRID_HEIGHT) 

    if checkpoint in segment_path: 

        if path_sequence: 

            # Exclude the starting point of the segment to avoid duplication 

            path_sequence.extend(segment_path[checkpoint][1:]) 

        else: 

            path_sequence.extend(segment_path[checkpoint]) 

        current_position = checkpoint 

 

# Step 3: Find path from last checkpoint to goal 

goal_path = bfs(current_position, [GOAL_POINT], BLOCKED_CELLS, GRID_WIDTH, GRID_HEIGHT) 

if GOAL_POINT in goal_path: 

    path_sequence.extend(goal_path[GOAL_POINT][1:]) 

 

# Print the path to the console 

print("Path sequence:", path_sequence) 

 

# Visualize the path 

fig, ax = plt.subplots(figsize=(10, 5)) 

ax.set_xlim(0, GRID_WIDTH) 

ax.set_ylim(0, GRID_HEIGHT) 

 

# Draw obstacles 

for ox, oy in obstacle_cells: 

    ax.add_patch(plt.Rectangle((ox, oy), 1, 1, color="red")) 

 

# Draw path 

x_coords, y_coords = zip(*path_sequence) 

ax.plot(x_coords, y_coords, marker="o", color="blue", linestyle="-", label="Path") 

 

# Draw start, checkpoints, and goal 

ax.scatter(*START_POINT, color="green", s=100, label="Start") 

for cp in CHECKPOINTS: 

    ax.scatter(*cp, color="yellow", s=100, label="Checkpoint") 

ax.scatter(*GOAL_POINT, color="purple", s=100, label="Goal") 

 

# Labels 

ax.set_xlabel("X [units]") 

ax.set_ylabel("Y [units]") 

ax.legend() 

ax.set_title("BFS Pathfinding for Robot") 

 

# Show plot 

plt.grid(True) 

plt.show() 

 
