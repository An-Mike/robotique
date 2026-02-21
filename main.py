from maze import Maze
from generateMap import generateMaze


rows, cols = 50, 50 # dimensions souhaité du labyrinthe

start = (0, 0)  # position de départ
goal = (49,49)   # position d'arrivée

#génération du grid et de la reward map (voir generateMap.py)
grid, rewardMap = generateMaze(rows, cols, start, goal, mode="simple", seed=42)

#création de l'instance de Maze
laby = Maze(grid, rewardMap, start, goal)

#affichage du grid
print("Grid:")
for row in laby.grid:
    print(row)

#affichage de la reward map
print("Reward Map:")
for row in laby.reward:
    print(row)

#test des méthodes de Maze
print("voisin franchissable à partir de (0, 0)")
print(laby.getPassableNeighbors((0, 0))) 

print("est ce que (5, 5) est à l'intérieur du labyrinthe ?")
print(laby.insideGrid((5, 5)))

print("est ce que (0,0) est franchissable ?")
print(laby.isPassable((0, 0)))

print("chemin optimal du start au goal :")
#path = laby.solveAstar()
#path = laby.solveAstarWithDiag()
#path = laby.dijkstraWithDiag()
path = laby.solveAstarWithPerfectHeuristic()
print(path)