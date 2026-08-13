import numpy as np
import cv2
import matplotlib.pyplot as plt

grid_size = 200

# Create empty map
grid = np.zeros((grid_size, grid_size), dtype=np.uint8)

# Add Boundary Walls
grid[0:3, :] = 1
grid[-3:, :] = 1
grid[:, 0:3] = 1
grid[:, -3:] = 1

# Random Rectangular Obstacles
np.random.seed(42)

for _ in range(30):

    x = np.random.randint(0, 200)
    y = np.random.randint(0, 200)

    w = np.random.randint(10, 25)
    h = np.random.randint(10, 25)

    grid[y:y+h, x:x+w] = 1


# Circular Obstacles (trees)
for _ in range(40):

    center = (
        np.random.randint(10,195),
        np.random.randint(10,195)
    )

    radius = np.random.randint(3,8)

    cv2.circle(grid, center, radius, 1, -1)

#saving the occupancy grid
np.save("data/occupancy_grid.npy", grid)

plt.figure(figsize=(8,8))
plt.imshow(grid,
           cmap='gray_r',
           origin='lower')

plt.title("Synthetic Drone Occupancy Grid")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()