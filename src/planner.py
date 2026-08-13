import numpy as np
import matplotlib.pyplot as plt

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

#loading the map
occupancy = np.load("data/occupancy_grid.npy")

#start and goal coords
start_pos = (10, 10)
goal_pos = (180, 180)
occupancy[start_pos] = 0
occupancy[goal_pos] = 0

#pathfinding expects: 1= walkable 0= obstacle
matrix = 1 - occupancy 
grid = Grid(matrix=matrix)

start = grid.node(start_pos[1], start_pos[0])
end = grid.node(goal_pos[1], goal_pos[0])


finder = AStarFinder()

path, runs = finder.find_path(start, end, grid)
if len(path) == 0:
    print("No path found!")
else:
    print("Path length:", len(path))

plt.imshow(occupancy, cmap = 'gray_r', origin = 'lower')

if path: 
    x = [p.x for p in path]
    y = [p.y for p in path]
    plt.plot(x, y, "b-", linewidth=2, marker=".", markersize=3)

plt.scatter(start.x,start.y,c="green",s=80)
plt.scatter(end.x,end.y,c="red",s=80)
plt.title("A* Path Planning")
plt.show()

path_array = np.array([[node.x, node.y] for node in path])
np.save("data/planned_path.npy", path_array)