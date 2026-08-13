import numpy as np
import matplotlib.pyplot as plt

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

# DYNAMIC OBSTACLE REPLANNING

# Load original environment and planned path
base_map = np.load("data/occupancy_grid.npy").copy()

original_path = np.load(
    "data/planned_path.npy",
    allow_pickle=True
).astype(int)

start = tuple(original_path[0])
goal = tuple(original_path[-1])

# Working map
occupancy = base_map.copy()

# Dynamic obstacle settings
# Obstacles will appear when the robot reaches approximately these percentages of its CURRENT route.

OBSTACLE_PROGRESS = [0.35, 0.55, 0.75]

OBSTACLE_RADIUS = 5

# Start replanning this many cells before the detected obstacle
REPLAN_DISTANCE = 5

print("==========================================")
print("   SEQUENTIAL DYNAMIC REPLANNING")
print("==========================================")

print("Map size    :", occupancy.shape)
print("Original path points :", len(original_path))
print("Start       :", start)
print("Goal        :", goal)

# HELPER FUNCTIONS
def calculate_path_length(path):

    if len(path) < 2:
        return 0.0

    dx = np.diff(path[:, 0])
    dy = np.diff(path[:, 1])

    return np.sum(
        np.sqrt(dx ** 2 + dy ** 2)
    )

def create_dynamic_obstacle(center, radius):

    cx, cy = center

    obstacle = set()

    for y in range(
        cy - radius,
        cy + radius + 1
    ):

        for x in range(
            cx - radius,
            cx + radius + 1
        ):

            if (
                0 <= x < occupancy.shape[1]
                and
                0 <= y < occupancy.shape[0]
            ):

                distance = np.sqrt(
                    (x - cx) ** 2 +
                    (y - cy) ** 2
                )

                if distance <= radius:

                    obstacle.add(
                        (x, y)
                    )

    return obstacle

def find_collision(path, obstacle):

    for i, point in enumerate(path):

        x = int(point[0])
        y = int(point[1])

        if (x, y) in obstacle:

            return i

    return None

def run_astar(start_pos, goal_pos, map_grid):

    matrix = 1 - map_grid

    grid = Grid(
        matrix=matrix
    )

    start_node = grid.node(
        int(start_pos[0]),
        int(start_pos[1])
    )

    goal_node = grid.node(
        int(goal_pos[0]),
        int(goal_pos[1])
    )

    finder = AStarFinder()

    nodes, runs = finder.find_path(
        start_node,
        goal_node,
        grid
    )

    if len(nodes) == 0:
        return None

    return np.array(
        [
            [node.x, node.y]
            for node in nodes
        ],
        dtype=int
    )

# INITIAL PATH
current_path = original_path.copy()
all_dynamic_obstacles = set()
replan_count = 0
successful_replans = 0
replanning_points = []

# Store every path segment so that we can visualize how the robot changed its route.
path_history = [current_path.copy()]

# SEQUENTIAL OBSTACLE SIMULATION
for obstacle_number, progress in enumerate(
    OBSTACLE_PROGRESS,
    start=1
):
    print()
    print("------------------------------------------")
    print(f"Dynamic Obstacle {obstacle_number}")
    print("------------------------------------------")

    # Select a point on the CURRENT path
    obstacle_index = int(
        len(current_path) * progress
    )

    obstacle_index = min(
        obstacle_index,
        len(current_path) - 1
    )

    obstacle_center = tuple(
        current_path[obstacle_index]
    )

    print(
        "Obstacle center :",
        obstacle_center
    )

    # Create obstacle
    new_obstacle = create_dynamic_obstacle(
        obstacle_center,
        OBSTACLE_RADIUS
    )

    # Add it to the environment
    for x, y in new_obstacle:

        occupancy[y, x] = 1

    all_dynamic_obstacles.update(
        new_obstacle
    )

    # Detect collision with CURRENT path
    collision_index = find_collision(
        current_path,
        new_obstacle
    )

    if collision_index is None:

        print(
            "Collision detected : NO"
        )

        continue

    print(
        "Collision index    :",
        collision_index
    )

    print(
        "Collision position :",
        tuple(
            current_path[collision_index]
        )
    )

    # Determine replanning point
    replan_index = max(
        0,
        collision_index - REPLAN_DISTANCE
    )

    replan_position = tuple(
    current_path[replan_index]
)

    replanning_points.append(
    replan_position
)

    print(
    "Replanning from    :",
    replan_position
)

    # Make sure the robot's current position is free
    occupancy[
        replan_position[1],
        replan_position[0]
    ] = 0

    # Goal must remain reachable
    occupancy[
        goal[1],
        goal[0]
    ] = 0


    # Run A*
    new_segment = run_astar(
        replan_position,
        goal,
        occupancy
    )

    if new_segment is None:

        print(
            "Replanning failed!"
        )

        break

    # Check new segment for collision
    new_collision = find_collision(
        new_segment,
        all_dynamic_obstacles
    )

    if new_collision is not None:

        print(
            "WARNING: New path still intersects "
            "a dynamic obstacle."
        )

        break

    # Combine old portion + new segment
    current_path = np.vstack(
        (
            current_path[:replan_index],
            new_segment
        )
    )

    path_history.append(
        current_path.copy()
    )

    replan_count += 1
    successful_replans += 1

    print(
        "Replanning successful : YES"
    )


# FINAL VALIDATION

final_collision = find_collision(
    current_path,
    all_dynamic_obstacles
)

goal_reached = (
    tuple(current_path[-1])
    == goal
)


# RESULTS

original_length = calculate_path_length(
    original_path
)

final_length = calculate_path_length(
    current_path
)

print()
print("==========================================")
print("       FINAL REPLANNING RESULTS")
print("==========================================")

print(
    "Dynamic obstacles introduced :",
    len(OBSTACLE_PROGRESS)
)

print(
    "Replans triggered            :",
    replan_count
)

print(
    "Successful replans           :",
    successful_replans
)

print()

print(
    f"Original Path Length         : "
    f"{original_length:.2f} cells"
)

print(
    f"Final Path Length            : "
    f"{final_length:.2f} cells"
)

print(
    f"Additional Distance          : "
    f"{final_length - original_length:.2f} cells"
)

print()

print(
    "Final Path Collision         :",
    "YES" if final_collision is not None
    else "NO"
)

print(
    "Goal Reached                 :",
    "YES" if goal_reached else "NO"
)

print()

if (
    successful_replans == len(OBSTACLE_PROGRESS)
    and
    final_collision is None
    and
    goal_reached
):

    print(
        "STATUS                       : SUCCESS"
    )

else:

    print(
        "STATUS                       : CHECK REQUIRED"
    )

print("==========================================")


# SAVE FINAL PATH
np.save(
    "data/dynamic_replanned_path.npy",
    current_path
)

np.save(
    "data/replanning_history.npy",
    np.array(path_history, dtype=object)
)

np.save(
    "data/replanning_points.npy",
    np.array(replanning_points)
)

print()
print("Saved:")
print("data/dynamic_replanned_path.npy")


# VISUALIZATION
plt.figure(
    figsize=(9, 9)
)

# Original occupancy map
plt.imshow(
    occupancy,
    cmap="gray_r",
    origin="lower"
)

# Original planned path
plt.plot(
    original_path[:, 0],
    original_path[:, 1],
    "b--",
    linewidth=2,
    label="Original Path"
)

# Final replanned path
plt.plot(
    current_path[:, 0],
    current_path[:, 1],
    "g-",
    linewidth=2.5,
    label="Final Replanned Path"
)

# Replanning points
if len(replanning_points) > 0:

    replanning_x = [
        point[0]
        for point in replanning_points
    ]

    replanning_y = [
        point[1]
        for point in replanning_points
    ]

    plt.scatter(
        replanning_x,
        replanning_y,
        marker="X",
        c="orange",
        s=150,
        edgecolors="black",
        linewidths=1.5,
        label="Replanning Point",
        zorder=5
    )

# Dynamic obstacles
if len(all_dynamic_obstacles) > 0:

    obstacle_x = [
        point[0]
        for point in all_dynamic_obstacles
    ]

    obstacle_y = [
        point[1]
        for point in all_dynamic_obstacles
    ]

    plt.scatter(
        obstacle_x,
        obstacle_y,
        c="red",
        s=12,
        label="Dynamic Obstacles"
    )

# Start
plt.scatter(
    start[0],
    start[1],
    c="yellow",
    s=100,
    label="Start"
)

# Goal
plt.scatter(
    goal[0],
    goal[1],
    c="green",
    s=100,
    label="Goal"
)

plt.xlabel("X Position (cells)")
plt.ylabel("Y Position (cells)")
plt.title("Dynamic Obstacle Detection and Sequential A* Replanning")
plt.legend()
plt.axis("equal")
plt.grid(alpha=0.3)
plt.tight_layout()
plt.show()