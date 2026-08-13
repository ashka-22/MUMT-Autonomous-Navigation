import numpy as np
import matplotlib.pyplot as plt

# 1. REPRODUCIBILITY
np.random.seed(42)

# 2. LOAD A* PATH
path = np.load("data/planned_path.npy")

# planner.py saved each point as:
# [x, y]
true_x = path[:, 0].astype(float)
true_y = path[:, 1].astype(float)

# 3. SENSOR PARAMETERS
# Wheel encoder percentage error
ODOMETRY_STD = 0.01

# IMU heading measurement noise
# degrees
IMU_NOISE_STD = 0.1

# Slowly changing IMU bias
# degrees per step
IMU_BIAS_STD = 0.002

# 4. INITIAL ROBOT STATE
estimated_x = [true_x[0]]
estimated_y = [true_y[0]]

# Initial direction of the robot
dx = true_x[1] - true_x[0]
dy = true_y[1] - true_y[0]

estimated_heading = np.arctan2(dy, dx)
previous_true_heading = estimated_heading

# Initial IMU bias
imu_bias = 0.0

# 5. DEAD RECKONING
for i in range(1, len(path)):

    # TRUE MOTION

    dx_true = true_x[i] - true_x[i - 1]
    dy_true = true_y[i] - true_y[i - 1]

    true_distance = np.hypot(
        dx_true,
        dy_true
    )

    true_heading = np.arctan2(
        dy_true,
        dx_true
    )

    # TRUE HEADING CHANGE
    
    heading_change = (
        true_heading -
        previous_true_heading
    )

    # Wrap angle to [-pi, pi]

    heading_change = np.arctan2(
        np.sin(heading_change),
        np.cos(heading_change)
    )

    previous_true_heading = true_heading

    # WHEEL ENCODER

    odometry_noise = np.random.normal(
        0,
        ODOMETRY_STD
    )

    measured_distance = (
        true_distance *
        (1 + odometry_noise)
    )

    # IMU BIAS

    imu_bias += np.deg2rad(
        np.random.normal(
            0,
            IMU_BIAS_STD
        )
    )

    # IMU HEADING NOISE

    imu_noise = np.deg2rad(
        np.random.normal(
            0,
            IMU_NOISE_STD
        )
    )

    # MEASURED HEADING CHANGE

    measured_heading_change = (
        heading_change
        + imu_noise
        + imu_bias
    )

    # UPDATE ESTIMATED HEADING

    estimated_heading += (
        measured_heading_change
    )

    # Keep heading between -pi and pi

    estimated_heading = np.arctan2(
        np.sin(estimated_heading),
        np.cos(estimated_heading)
    )

    # UPDATE ESTIMATED POSITION

    new_x = (
        estimated_x[-1]
        +
        measured_distance *
        np.cos(estimated_heading)
    )

    new_y = (
        estimated_y[-1]
        +
        measured_distance *
        np.sin(estimated_heading)
    )

    estimated_x.append(new_x)
    estimated_y.append(new_y)

# 6. CONVERT TO ARRAYS
estimated_x = np.array(estimated_x)
estimated_y = np.array(estimated_y)

# 7. POSITION ERROR
position_error = np.sqrt(
    (true_x - estimated_x) ** 2
    +
    (true_y - estimated_y) ** 2
)

average_error = np.mean(position_error)
maximum_error = np.max(position_error)
final_error = position_error[-1]

# 8. PRINT RESULTS
print()
print("==========================================")
print("       DEAD RECKONING LOCALIZATION")
print("==========================================")

print(f"Number of path points : {len(path)}")

print()

print(
    f"True final position      : "
    f"({true_x[-1]:.2f}, {true_y[-1]:.2f})"
)

print(
    f"Estimated final position : "
    f"({estimated_x[-1]:.2f}, {estimated_y[-1]:.2f})"
)

print()

print(
    f"Average Position Error : "
    f"{average_error:.2f} cells"
)

print(
    f"Maximum Position Error : "
    f"{maximum_error:.2f} cells"
)

print(
    f"Final Position Error   : "
    f"{final_error:.2f} cells"
)

print("==========================================")

# 9. SAVE DEAD RECKONING PATH
estimated_path = np.column_stack(
    (
        estimated_x,
        estimated_y
    )
)

np.save(
    "data/dead_reckoning_path.npy",
    estimated_path
)

print()
print("Saved:")
print("data/dead_reckoning_path.npy")

# 10. PLOT

plt.figure(figsize=(8, 8))

# True path
plt.plot(
    true_x,
    true_y,
    "g-",
    linewidth=2,
    label="True Path"
)

# Estimated dead-reckoning path
plt.plot(
    estimated_x,
    estimated_y,
    "r--",
    linewidth=2,
    label="Dead Reckoning"
)

# Start
plt.scatter(
    true_x[0],
    true_y[0],
    c="blue",
    s=90,
    label="Start",
    zorder=5
)

# Goal
plt.scatter(
    true_x[-1],
    true_y[-1],
    c="black",
    s=90,
    label="Goal",
    zorder=5
)

plt.title(
    "GPS-Denied Dead Reckoning Localization"
)

plt.xlabel("X Position (cells)")
plt.ylabel("Y Position (cells)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()