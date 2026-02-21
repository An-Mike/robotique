#Les tests se déroulent dans ce fichier, on y génère un labyrinthe, 
# on affiche sa grid et sa reward map, 
# et on teste les méthodes de base de Maze ainsi que les algorithmes de pathfinding.

from maze import Maze
from generateMap import generateMaze


rows, cols = 100, 100 # dimensions souhaité du labyrinthe (nb de lignes, nb de colonnes)

start = (0, 0)  # position de départ (x, y)
goal = (99, 99)   # position d'arrivée (x, y)



#génération du grid et de la reward map 
#mode : "empty"  -> aucune obstacle
#       "simple" -> obstacles aléatoires mais chemin garanti
#       "blocked" -> aucun chemin possible entre start et goal

#seed : int ou None. Si fourni, rend la génération reproductible.

grid, rewardMap = generateMaze(rows, cols, start, goal, mode="simple", seed=42)

#création de l'instance de Maze
laby = Maze(grid, rewardMap, start, goal)

#affichage du grid dans la console
print("Grid:")
for row in laby.grid:
    print(row)

#affichage de la reward map dans la console
print("Reward Map:")
for row in laby.reward:
    print(row)

#test des méthodes de base de Maze
print("voisin franchissable à partir de (0, 0)") #affiche les voisins qui sont franchissables 
print(laby.getPassableNeighbors((0, 0))) 

print("est ce que (5, 5) est à l'intérieur du labyrinthe ?") #affiche si la position est à l'intérieur du labyrinthe ou pas
print(laby.insideGrid((5, 5)))

print("est ce que (0,0) est franchissable ?") #affiche si la position est franchissable ou pas
print(laby.isPassable((0, 0)))



#test des différentes méthodes de résolution
#décommenter suivant la méthode voulue 

print("chemin optimal du start au goal :")

#résolution par l'algo A* (4-connectés, sans coût de virage)
path = laby.solveAstar()

#résolution par l'algo A* (8-connectés, sans coût de virage)
#path = laby.solveAstarWithDiag()

#résolution par l'algo A* (4-connectés, considère un coût de virage epsilon)
#path = laby.solveAstarWithDirCost(epsilon=0.5)

#résolution par l'algo de Dijkstra (4-connecté, sans coût de virage)
#path = laby.dijkstra()

#résolution par l'algo de Dijkstra (8-connecté, sans coût de virage)
#path = laby.dijkstraWithDiag()

#résolution A* en se basant sur la reward map (pondération négative)
#path = laby.solveAstarNegative()

#résolution Dijkstra en se basant sur la reward map (pondération négative)
#path = laby.dijkstraNegative()

#résolution A* avec heuristique parfaite (8-connectés, sans coût de virage)
#path = laby.solveAstarWithPerfectHeuristic()


print(path)