import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from matplotlib.gridspec import GridSpec
from matplotlib.colors import ListedColormap


# ============================================================
# CONFIGURATION
# ============================================================

OBSTACLE_PROGRESS = [0.35, 0.55, 0.75]

PATH_DRAW_FRAMES = 90
PAUSE_FRAMES = 20
INTERVAL = 40

C = {
    "true": "#046A37",
    "dr": "#DC0686",
    "kalman": "#F36704",
    "astar": "#0408FC",
    "obstacle": "#FE0303",
    "replan": "#060606",
    "start": "#00ACC1",
    "goal": "#15D435",
    "text": "#263238",
    "grid": "#ECEFF1",
    "border": "#CFD8DC",
}


# ============================================================
# LOAD PROJECT DATA
# ============================================================

occupancy = np.load(
    "data/occupancy_grid.npy"
)

planned_path = np.load(
    "data/planned_path.npy",
    allow_pickle=True
).astype(int)

dead_path = np.load(
    "data/dead_reckoning_path.npy",
    allow_pickle=True
).astype(float)

kalman_path = np.load(
    "data/kalman_filtered_path.npy",
    allow_pickle=True
).astype(float)

replanning_history = [
    np.asarray(path).astype(int)
    for path in np.load(
        "data/replanning_history.npy",
        allow_pickle=True
    )
]

replanning_points = np.asarray(
    np.load(
        "data/replanning_points.npy",
        allow_pickle=True
    )
).astype(int)

start = tuple(planned_path[0])
goal = tuple(planned_path[-1])


# ============================================================
# DYNAMIC OBSTACLE LOCATIONS
# ============================================================

def path_point(path, progress):
    index = min(
        int(len(path) * progress),
        len(path) - 1
    )
    return tuple(path[index])


obstacle_centers = [
    path_point(planned_path, p)
    for p in OBSTACLE_PROGRESS
]


# ============================================================
# TERMINAL INFORMATION
# ============================================================

print()
print("=" * 42)
print("       MUM-T NAVIGATION ANIMATION")
print("=" * 42)
print(f"\nOccupancy Grid Shape : {occupancy.shape}")
print(f"A* Path Points       : {len(planned_path)}")
print(f"Dead Reckoning Points: {len(dead_path)}")
print(f"Kalman Points        : {len(kalman_path)}")
print(f"Replanned Path Points: {len(replanning_history[-1])}")
print(f"\nStart : {start}")
print(f"Goal  : {goal}")
print("\nAll project data loaded successfully.")
print("=" * 42)


# ============================================================
# FIGURE LAYOUT
# ============================================================

fig = plt.figure(
    figsize=(15, 8.5),
    facecolor="white"
)

gs = GridSpec(
    2,
    2,
    figure=fig,
    width_ratios=[5.8, 1.35],
    height_ratios=[3.2, 1.25],
    wspace=0.10,
    hspace=0.24
)

ax = fig.add_subplot(gs[:, 0])
panel = fig.add_subplot(gs[0, 1])
error_ax = fig.add_subplot(gs[1, 1])


# ============================================================
# OCCUPANCY GRID
# ============================================================

occupancy_cmap = ListedColormap([
    "#F7F7F7",
    "#666666"
])

ax.imshow(
    occupancy,
    cmap=occupancy_cmap,
    origin="lower",
    interpolation="nearest",
    vmin=0,
    vmax=1
)

ax.set_xlim(
    0,
    occupancy.shape[1]
)

ax.set_ylim(
    0,
    occupancy.shape[0]
)

ax.set_aspect("equal")

ax.set_xlabel(
    "X Position (cells)",
    fontsize=10
)

ax.set_ylabel(
    "Y Position (cells)",
    fontsize=10
)

ax.set_title(
    "MUM-T Autonomous Navigation",
    fontsize=17,
    fontweight="bold",
    pad=12
)

ax.grid(
    color=C["grid"],
    linewidth=0.5,
    alpha=0.7
)


# ============================================================
# START / GOAL
# ============================================================

start_marker, = ax.plot(
    start[0],
    start[1],
    marker="o",
    markersize=8,
    color=C["start"],
    linestyle="None",
    zorder=20
)

goal_marker, = ax.plot(
    goal[0],
    goal[1],
    marker="*",
    markersize=14,
    color=C["goal"],
    linestyle="None",
    zorder=20
)

# ============================================================
# PATHS
# ============================================================

astar_line, = ax.plot(
    [], [],
    "--",
    color=C["astar"],
    linewidth=2.0,
    alpha=0.85,
    zorder=5
)

true_trail, = ax.plot(
    [], [],
    "-",
    color=C["true"],
    linewidth=3.0,
    zorder=10
)

dead_trail, = ax.plot(
    [], [],
    "-",
    color=C["dr"],
    linewidth=1.8,
    alpha=0.9,
    zorder=8
)

kalman_trail, = ax.plot(
    [], [],
    "-.",
    color=C["kalman"],
    linewidth=2.0,
    alpha=0.95,
    zorder=9
)

# Hide all trajectory lines until the robot starts moving
true_trail.set_visible(False)
dead_trail.set_visible(False)
kalman_trail.set_visible(False)

# ============================================================
# ROBOT MARKERS
# ============================================================

true_robot, = ax.plot(
    [], [], marker="o",
    markersize=8,
    color=C["true"],
    linestyle="None",
    zorder=21
)

dead_robot, = ax.plot(
    [], [], marker="o",
    markersize=6,
    color=C["dr"],
    linestyle="None",
    zorder=19
)

kalman_robot, = ax.plot(
    [], [], marker="s",
    markersize=6,
    color=C["kalman"],
    linestyle="None",
    zorder=19
)

# Keep all robot markers hidden until movement begins
true_robot.set_visible(False)
dead_robot.set_visible(False)
kalman_robot.set_visible(False)


# ============================================================
# DYNAMIC OBSTACLES
# ============================================================

obstacle_markers = []

for _ in obstacle_centers:

    marker, = ax.plot(
        [], [],
        marker="o",
        markersize=18,
        color=C["obstacle"],
        linestyle="None",
        zorder=18
    )

    marker.set_visible(False)
    obstacle_markers.append(marker)


# ============================================================
# REPLANNING MARKERS
# ============================================================

replan_markers = []

for _ in replanning_points:

    marker, = ax.plot(
        [], [],
        marker="^",
        markersize=8,
        color=C["replan"],
        linestyle="None",
        zorder=19
    )

    marker.set_visible(False)
    replan_markers.append(marker)


# ============================================================
# SYSTEM STATE PANEL
# ============================================================

panel.set_xlim(0, 1)
panel.set_ylim(0, 1)
panel.axis("off")

panel.text(
    0.04,
    0.94,
    "SYSTEM STATE",
    fontsize=11,
    fontweight="bold",
    color=C["text"],
    va="center"
)

panel.add_patch(
    Rectangle(
        (0.03, 0.57),
        0.94,
        0.31,
        fill=False,
        edgecolor=C["border"],
        linewidth=1
    )
)

status_text = panel.text(
    0.07,
    0.80,
    "INITIALIZING",
    fontsize=9,
    fontweight="bold",
    color=C["astar"],
    va="center"
)

obstacle_text = panel.text(
    0.07,
    0.70,
    "Obstacles detected: 0",
    fontsize=8.5,
    color=C["text"],
    va="center"
)

replan_text = panel.text(
    0.07,
    0.62,
    "Replans completed: 0",
    fontsize=8.5,
    color=C["text"],
    va="center"
)


# ============================================================
# LEGEND TABLE
# ============================================================

panel.text(
    0.04,
    0.51,
    "LEGEND",
    fontsize=10,
    fontweight="bold",
    color=C["text"],
    va="center"
)

legend_box = Rectangle(
    (0.03, 0.05),
    0.94,
    0.43,
    fill=False,
    edgecolor=C["border"],
    linewidth=1
)

panel.add_patch(legend_box)

legend_items = [
    ("True trajectory", C["true"], "-", None),
    ("Dead reckoning", C["dr"], "-", None),
    ("Kalman estimate", C["kalman"], "-.", None),
    ("Original A* path", C["astar"], "--", None),
    ("Dynamic obstacle", C["obstacle"], None, "o"),
    ("Replanning point", C["replan"], None, "^"),
    ("Goal", C["goal"], None, "*")
]

y = 0.43

for label, color, style, marker in legend_items:

    if marker is None:

        panel.plot(
            [0.07, 0.15],
            [y, y],
            style,
            color=color,
            linewidth=2,
            transform=panel.transAxes
        )

    else:

        panel.plot(
            0.11,
            y,
            marker=marker,
            markersize=7,
            color=color,
            linestyle="None",
            transform=panel.transAxes
        )

    panel.text(
        0.20,
        y,
        label,
        fontsize=7.3,
        color=C["text"],
        va="center",
        transform=panel.transAxes
    )

    y -= 0.055


# ============================================================
# LOCALIZATION ERROR GRAPH
# ============================================================

error_ax.set_title(
    "Localization Error",
    fontsize=10,
    fontweight="bold",
    loc="left",
    pad=5
)

error_ax.set_xlabel(
    "Navigation progress",
    fontsize=7.5
)

error_ax.set_ylabel(
    "Error (cells)",
    fontsize=7.5
)

error_ax.tick_params(
    labelsize=7
)

error_ax.grid(
    alpha=0.15
)

error_ax.set_xlim(0, 1)

dr_error_line, = error_ax.plot(
    [],
    [],
    color=C["dr"],
    linewidth=1.8
)

kf_error_line, = error_ax.plot(
    [],
    [],
    color=C["kalman"],
    linewidth=1.8
)

error_ax.legend(
    ["Dead reckoning", "Kalman"],
    fontsize=7,
    frameon=False,
    loc="upper left"
)


# ============================================================
# ACTIVE PATH
# ============================================================

def get_active_path(progress):

    active_path = planned_path
    replans = 0

    for i in range(
        1,
        len(replanning_history)
    ):

        if i - 1 >= len(replanning_points):
            break

        point = replanning_points[i - 1]

        index = np.argmin(
            np.linalg.norm(
                planned_path - point,
                axis=1
            )
        )

        point_progress = (
            index /
            max(1, len(planned_path) - 1)
        )

        if progress >= point_progress:
            active_path = replanning_history[i]
            replans = i

    return active_path, replans


# ============================================================
# LOCALIZATION ERROR
# ============================================================

def calculate_error(
    true_path,
    estimate,
    count
):

    if count <= 0:
        return np.array([])

    count = min(
        count,
        len(estimate)
    )

    indices = np.linspace(
        0,
        len(true_path) - 1,
        count
    ).astype(int)

    return np.linalg.norm(
        estimate[:count]
        - true_path[indices],
        axis=1
    )


# ============================================================
# ANIMATED ARTISTS
# ============================================================

def artists():

    return (
        astar_line,
        true_trail,
        dead_trail,
        kalman_trail,
        true_robot,
        dead_robot,
        kalman_robot,
        *obstacle_markers,
        *replan_markers,
        status_text,
        obstacle_text,
        replan_text,
        dr_error_line,
        kf_error_line
    )


# ============================================================
# ANIMATION UPDATE
# ============================================================
def update(frame):

    # --------------------------------------------------------
    # PHASE 1 — GLOBAL A* PLANNING
    # --------------------------------------------------------

    if frame < PATH_DRAW_FRAMES:

        # Keep all future navigation elements hidden
        true_robot.set_visible(False)
        dead_robot.set_visible(False)
        kalman_robot.set_visible(False)

        true_trail.set_visible(False)
        dead_trail.set_visible(False)
        kalman_trail.set_visible(False)

        for marker in obstacle_markers:
            marker.set_visible(False)

        for marker in replan_markers:
            marker.set_visible(False)

        points = max(
            1,
            int(
                len(planned_path)
                * frame
                / max(
                    1,
                    PATH_DRAW_FRAMES - 1
                )
            )
        )

        astar_line.set_data(
            planned_path[:points, 0],
            planned_path[:points, 1]
        )

        status_text.set_text(
            "A* PATH PLANNING"
        )

        status_text.set_color(
            C["astar"]
        )

        return artists()

    # --------------------------------------------------------
    # PHASE 1 PAUSE
    # --------------------------------------------------------

    if frame < (
        PATH_DRAW_FRAMES
        + PAUSE_FRAMES
    ):

        # Keep navigation elements hidden
        true_robot.set_visible(False)
        dead_robot.set_visible(False)
        kalman_robot.set_visible(False)

        true_trail.set_visible(False)
        dead_trail.set_visible(False)
        kalman_trail.set_visible(False)

        for marker in obstacle_markers:
            marker.set_visible(False)

        for marker in replan_markers:
            marker.set_visible(False)

        astar_line.set_data(
            planned_path[:, 0],
            planned_path[:, 1]
        )

        status_text.set_text(
            "PATH PLANNED"
        )

        status_text.set_color(
            C["astar"]
        )

        return artists()

    # --------------------------------------------------------
    # PHASE 2 — ROBOT MOVEMENT
    # --------------------------------------------------------

    # Reveal navigation elements only after A* planning
    true_robot.set_visible(True)
    dead_robot.set_visible(True)
    kalman_robot.set_visible(True)

    true_trail.set_visible(True)
    dead_trail.set_visible(True)
    kalman_trail.set_visible(True)

    robot_frame = (
        frame
        - PATH_DRAW_FRAMES
        - PAUSE_FRAMES
    )

    progress = np.clip(
        robot_frame /
        max(
            1,
            len(planned_path) - 1
        ),
        0,
        1
    )

    active_path, replans = get_active_path(
        progress
    )

    active_index = min(
        int(
            progress
            * (len(active_path) - 1)
        ),
        len(active_path) - 1
    )

    # --------------------------------------------------------
    # TRUE ROBOT + TRUE TRAJECTORY
    # --------------------------------------------------------

    true_pos = active_path[active_index]

    true_robot.set_data(
        [true_pos[0]],
        [true_pos[1]]
    )

    true_trail.set_data(
        active_path[:active_index + 1, 0],
        active_path[:active_index + 1, 1]
    )

    # --------------------------------------------------------
    # DEAD RECKONING
    # --------------------------------------------------------

    dr_index = min(
        int(
            progress
            * (len(dead_path) - 1)
        ),
        len(dead_path) - 1
    )

    dr_pos = dead_path[dr_index]

    dead_robot.set_data(
        [dr_pos[0]],
        [dr_pos[1]]
    )

    dead_trail.set_data(
        dead_path[:dr_index + 1, 0],
        dead_path[:dr_index + 1, 1]
    )

    # --------------------------------------------------------
    # KALMAN FILTER
    # --------------------------------------------------------

    kf_index = min(
        int(
            progress
            * (len(kalman_path) - 1)
        ),
        len(kalman_path) - 1
    )

    kf_pos = kalman_path[kf_index]

    kalman_robot.set_data(
        [kf_pos[0]],
        [kf_pos[1]]
    )

    kalman_trail.set_data(
        kalman_path[:kf_index + 1, 0],
        kalman_path[:kf_index + 1, 1]
    )

        # --------------------------------------------------------
    # DYNAMIC OBSTACLES
    # --------------------------------------------------------

    obstacle_count = 0

    for i, marker in enumerate(
        obstacle_markers
    ):

        if progress >= OBSTACLE_PROGRESS[i]:

            x, y = obstacle_centers[i]

            marker.set_data(
                [x],
                [y]
            )

            marker.set_visible(True)

            obstacle_count += 1

        else:

            marker.set_data(
                [],
                []
            )

            marker.set_visible(False)


    # --------------------------------------------------------
    # REPLANNING POINTS
    # --------------------------------------------------------

    completed_replans = 0

    for i, marker in enumerate(
        replan_markers
    ):

        if i >= len(replanning_points):

            marker.set_data(
                [],
                []
            )

            marker.set_visible(False)

            continue

        point = replanning_points[i]

        point_index = np.argmin(
            np.linalg.norm(
                planned_path - point,
                axis=1
            )
        )

        point_progress = (
            point_index /
            max(
                1,
                len(planned_path) - 1
            )
        )

        if (
            progress >=
            OBSTACLE_PROGRESS[i]
            and
            progress >= point_progress
        ):

            marker.set_data(
                [point[0]],
                [point[1]]
            )

            marker.set_visible(True)

            completed_replans += 1

        else:

            marker.set_data(
                [],
                []
            )

            marker.set_visible(False)

    # --------------------------------------------------------
    # LOCALIZATION ERROR
    # --------------------------------------------------------

    dr_error = calculate_error(
        active_path,
        dead_path,
        dr_index + 1
    )

    kf_error = calculate_error(
        active_path,
        kalman_path,
        kf_index + 1
    )

    dr_x = np.linspace(
        0,
        progress,
        len(dr_error)
    )

    kf_x = np.linspace(
        0,
        progress,
        len(kf_error)
    )

    dr_error_line.set_data(
        dr_x,
        dr_error
    )

    kf_error_line.set_data(
        kf_x,
        kf_error
    )

    max_error = 1

    if len(dr_error):
        max_error = max(
            max_error,
            np.max(dr_error)
        )

    if len(kf_error):
        max_error = max(
            max_error,
            np.max(kf_error)
        )

    error_ax.set_ylim(
        0,
        max_error * 1.15
    )

    # --------------------------------------------------------
    # SYSTEM STATE
    # --------------------------------------------------------

    obstacle_text.set_text(
        f"Obstacles detected: {obstacle_count}"
    )

    replan_text.set_text(
        f"Replans completed: {replans}"
    )

    if progress >= 1:

        status_text.set_text(
            "GOAL REACHED"
        )

        status_text.set_color(
            C["goal"]
        )

    elif replans > 0:

        status_text.set_text(
            "REPLANNED ROUTE"
        )

        status_text.set_color(
            C["replan"]
        )

    elif obstacle_count > 0:

        status_text.set_text(
            "OBSTACLE DETECTED"
        )

        status_text.set_color(
            C["obstacle"]
        )

    else:

        status_text.set_text(
            "AUTONOMOUS NAVIGATION"
        )

        status_text.set_color(
            C["true"]
        )

    return artists()


# ============================================================
# RUN ANIMATION
# ============================================================

animation = FuncAnimation(
    fig,
    update,
    frames=(
        PATH_DRAW_FRAMES
        + PAUSE_FRAMES
        + len(planned_path)
    ),
    interval=INTERVAL,
    blit=True,
    repeat=False
)

# Save animation for GitHub
animation.save(
    "results/mumt_navigation.gif",
    writer="pillow",
    fps=15
)

plt.show()