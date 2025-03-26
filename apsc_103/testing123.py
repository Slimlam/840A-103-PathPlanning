import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import math

# Define grid size
GRID_WIDTH = 50
GRID_HEIGHT = 25

# Define movement directions (Up, Down, Left, Right, and Diagonals)
DIRECTIONS = [
    (0, 1),  # Up
    (0, -1),  # Down
    (1, 0),  # Right
    (-1, 0),  # Left
    (1, 1),  # Diagonal (Down-Right)
    (1, -1),  # Diagonal (Up-Right)
    (-1, 1),  # Diagonal (Down-Left)
    (-1, -1)  # Diagonal (Up-Left)
]

# Define obstacles (expanded by 1 unit for robot clearance)
OBSTACLES = [(40, 8), (11, 11)]  # Given obstacles (midpoints)
OBSTACLE_SIZE = 4  # Since robot has a 4-unit diameter, expand obstacles

# Define start point and checkpoints
START_POINT = (1, 1)
CHECKPOINTS = [(5, 20), (18, 6)]  # Given checkpoints


def expand_obstacles(obstacles, grid_width, grid_height):
    """
    Expands each obstacle by 1 unit in all directions to ensure clearance.
    Returns a set of blocked cells.
    """
    blocked_cells = set()
    for ox, oy in obstacles:
        for dx in range(-OBSTACLE_SIZE // 2 - 1, OBSTACLE_SIZE // 2 + 2):
            for dy in range(-OBSTACLE_SIZE // 2 - 1, OBSTACLE_SIZE // 2 + 2):
                nx, ny = ox + dx, oy + dy
                if 0 <= nx < grid_width and 0 <= ny < grid_height:
                    blocked_cells.add((nx, ny))
    return blocked_cells


# Expand obstacles
BLOCKED_CELLS = expand_obstacles(OBSTACLES, GRID_WIDTH, GRID_HEIGHT)


def bfs(start, goal, blocked_cells, grid_width, grid_height):
    """
    Performs BFS to find the shortest path to a single goal point.
    Returns the path to the goal.
    """
    queue = deque([(start, [])])  # (current_position, path)
    visited = set([start])

    while queue:
        (x, y), path = queue.popleft()

        # If reached the goal, return the path
        if (x, y) == goal:
            return path + [(x, y)]

        for dx, dy in DIRECTIONS:
            nx, ny = x + dx, y + dy

            if (0 <= nx < grid_width and 0 <= ny < grid_height and
                    (nx, ny) not in blocked_cells and (nx, ny) not in visited):
                queue.append(((nx, ny), path + [(x, y)]))
                visited.add((nx, ny))

    return None  # Return None if no path is found


def find_closest_checkpoint(start, checkpoints, blocked_cells, grid_width, grid_height):
    """
    Finds the closest checkpoint to the current start point using BFS.
    Returns the path to the closest checkpoint and the checkpoint itself.
    """
    shortest_path = None
    closest_checkpoint = None

    for checkpoint in checkpoints:
        path = bfs(start, checkpoint, blocked_cells, grid_width, grid_height)
        if path and (shortest_path is None or len(path) < len(shortest_path)):
            shortest_path = path
            closest_checkpoint = checkpoint

    return shortest_path, closest_checkpoint


def validate_single_step_path(path_sequence):
    """
    Validates that the path sequence only contains single-step movements.
    """
    for i in range(1, len(path_sequence)):
        x1, y1 = path_sequence[i - 1]
        x2, y2 = path_sequence[i]
        dx, dy = abs(x2 - x1), abs(y2 - y1)
        if dx > 1 or dy > 1:
            print(f"Invalid step found: ({x1}, {y1}) -> ({x2}, {y2})")
            return False
    return True


# Main logic to find a continuous path through all checkpoints
remaining_checkpoints = CHECKPOINTS[:]
current_position = START_POINT
path_sequence = []

while remaining_checkpoints:
    # Find the closest checkpoint and the path to it
    path_to_checkpoint, closest_checkpoint = find_closest_checkpoint(
        current_position, remaining_checkpoints, BLOCKED_CELLS, GRID_WIDTH, GRID_HEIGHT
    )

    if path_to_checkpoint:
        # Add the path to the sequence and update the current position
        path_sequence.extend(path_to_checkpoint[1:])  # Exclude the starting point
        current_position = closest_checkpoint
        remaining_checkpoints.remove(closest_checkpoint)
    else:
        print(f"Could not reach checkpoint {closest_checkpoint}")
        break

# Validate the path sequence
if not validate_single_step_path(path_sequence):
    print("Path sequence contains invalid steps.")
else:
    print("Path sequence is valid.")

# Calculate the total distance traveled
total_distance = len(path_sequence)  # Each step is 1 unit
print("Total distance traveled:", total_distance)

# Visualize the path
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_xlim(0, GRID_WIDTH)
ax.set_ylim(0, GRID_HEIGHT)

# Add additional ticks on the x-axis
ax.set_xticks(np.arange(0, GRID_WIDTH + 1, 1))

# Draw obstacles
for ox, oy in BLOCKED_CELLS:
    ax.add_patch(plt.Rectangle((ox, oy), 1, 1, color="red"))

# Draw path
if path_sequence:
    x_coords, y_coords = zip(*path_sequence)
    ax.plot(x_coords, y_coords, marker="o", color="blue", linestyle="-", label="Path")

# Draw start and checkpoints
ax.scatter(*START_POINT, color="green", s=100, label="Start")
for cp in CHECKPOINTS:
    ax.scatter(*cp, color="yellow", s=100, label="Checkpoint")

# Labels
ax.set_xlabel("X [units]")
ax.set_ylabel("Y [units]")
ax.legend()
ax.set_title("BFS Pathfinding for Robot (Checkpoints Only)")

# Show plot
plt.grid(True)
plt.show()