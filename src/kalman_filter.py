import numpy as np
import matplotlib.pyplot as plt

# KALMAN FILTER LOCALIZATION

np.random.seed(42)

# LOAD TRUE PATH
path = np.load(
    "data/planned_path.npy",
    allow_pickle=True
)

true_x = path[:, 0].astype(float)
true_y = path[:, 1].astype(float)

N = len(path)

# LOAD DEAD RECKONING PATH
dead_path = np.load(
    "data/dead_reckoning_path.npy",
    allow_pickle=True
)

dead_x = dead_path[:, 0].astype(float)
dead_y = dead_path[:, 1].astype(float)

print("==========================================")
print("       KALMAN FILTER LOCALIZATION")
print("==========================================")
print()
print("Number of path points :", N)
print()

# SENSOR / MEASUREMENT PARAMETERS

# Visual / map localization noise
VISION_STD = 2.5

# Visual/map measurement available every N steps
VISION_INTERVAL = 20

# KALMAN FILTER STATE
# State = [x, y]

state = np.array([
    dead_x[0],
    dead_y[0]
], dtype=float)

# INITIAL COVARIANCE
P = np.diag([
    1.0,
    1.0
])

# PROCESS NOISE
Q = np.diag([
    0.5,
    0.5
])

# MEASUREMENT MODEL
# Measurement = [x, y]
# H converts the state directly into position measurement.

H = np.eye(2)

# MEASUREMENT NOISE
R = np.diag([
    VISION_STD ** 2,
    VISION_STD ** 2
])

# STORAGE

kalman_x = [state[0]]
kalman_y = [state[1]]

# MAIN KALMAN FILTER LOOP
for i in range(1, N):

   # DEAD RECKONING MOTION
    dx = dead_x[i] - dead_x[i - 1]
    dy = dead_y[i] - dead_y[i - 1]

   # PREDICTION
    predicted_state = state + np.array([
        dx,
        dy
    ])

    # State transition matrix
    F = np.eye(2)

    # Covariance prediction
    predicted_P = (
        F @ P @ F.T +
        Q
    )

    state = predicted_state
    P = predicted_P


    # VISUAL / MAP LOCALIZATION MEASUREMENT

    if i % VISION_INTERVAL == 0:

        # Simulated visual/map measurement
        # The measurement represents a camera/map-matching estimate of the robot's position.
        # Noise is added to make the measurement realistic.

        measurement = np.array([
            true_x[i] +
            np.random.normal(
                0,
                VISION_STD
            ),

            true_y[i] +
            np.random.normal(
                0,
                VISION_STD
            )
        ])

        # INNOVATION

        innovation = (
            measurement -
            H @ state
        )

        # INNOVATION COVARIANCE

        S = (
            H @ P @ H.T +
            R
        )

        # KALMAN GAIN

        K = (
            P @ H.T @
            np.linalg.inv(S)
        )

        # STATE CORRECTION

        state = (
            state +
            K @ innovation
        )

        # COVARIANCE CORRECTION

        I = np.eye(2)

        P = (
            (I - K @ H) @ P
        )

    # SAVE FILTERED POSITION

    kalman_x.append(state[0])
    kalman_y.append(state[1])


# CONVERT TO ARRAYS

kalman_x = np.array(kalman_x)
kalman_y = np.array(kalman_y)


# POSITION ERRORS

dead_error = np.sqrt(
    (true_x - dead_x) ** 2 +
    (true_y - dead_y) ** 2
)

kalman_error = np.sqrt(
    (true_x - kalman_x) ** 2 +
    (true_y - kalman_y) ** 2
)


# ERROR IMPROVEMENT

average_error_reduction = (
    1 -
    np.mean(kalman_error) /
    np.mean(dead_error)
) * 100

final_error_reduction = (
    1 -
    kalman_error[-1] /
    dead_error[-1]
) * 100


# RESULTS

print("==========================================")
print("       KALMAN FILTER RESULTS")
print("==========================================")
print()

print(
    "Dead Reckoning Average Error : "
    f"{np.mean(dead_error):.2f} cells"
)

print(
    "Kalman Filter Average Error  : "
    f"{np.mean(kalman_error):.2f} cells"
)

print()

print(
    "Dead Reckoning Final Error   : "
    f"{dead_error[-1]:.2f} cells"
)

print(
    "Kalman Filter Final Error    : "
    f"{kalman_error[-1]:.2f} cells"
)

print()

print(
    "Dead Reckoning Maximum Error : "
    f"{np.max(dead_error):.2f} cells"
)

print(
    "Kalman Filter Maximum Error  : "
    f"{np.max(kalman_error):.2f} cells"
)

print()

print(
    "Average Error Reduction      : "
    f"{average_error_reduction:.2f}%"
)

print(
    "Final Error Reduction        : "
    f"{final_error_reduction:.2f}%"
)

print()

print(
    "True Final Position          : "
    f"({true_x[-1]:.2f}, {true_y[-1]:.2f})"
)

print(
    "Kalman Final Position        : "
    f"({kalman_x[-1]:.2f}, {kalman_y[-1]:.2f})"
)

print()

# SAVE KALMAN FILTERED PATH

kalman_filtered_path = np.column_stack((
    kalman_x,
    kalman_y
))

np.save(
    "data/kalman_filtered_path.npy",
    kalman_filtered_path
)

print("==========================================")
print()
print("Saved:")
print("data/kalman_filtered_path.npy")
print()

# PLOT

plt.figure(figsize=(9, 9))

# True path
plt.plot(
    true_x,
    true_y,
    "g-",
    linewidth=2,
    label="True Path"
)

# Dead reckoning path
plt.plot(
    dead_x,
    dead_y,
    "r--",
    linewidth=2,
    label="Dead Reckoning"
)

# Kalman filtered path
plt.plot(
    kalman_x,
    kalman_y,
    "b-",
    linewidth=2,
    label="Kalman Filter"
)

# Start
plt.scatter(
    true_x[0],
    true_y[0],
    c="blue",
    s=100,
    label="Start"
)

# Goal
plt.scatter(
    true_x[-1],
    true_y[-1],
    c="black",
    s=100,
    label="Goal"
)

plt.xlabel("X Position (cells)")
plt.ylabel("Y Position (cells)")
plt.title("GPS-Denied Localization: Dead Reckoning vs Kalman Filter")
plt.legend()
plt.grid(alpha=0.3)
plt.axis("equal")
plt.show()