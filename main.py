# matrice représentant le labyrinthe, 0 = chemin, 1 = mur
grid = [
    [0, 0, 1, 0, 0],
    [1, 0, 1, 0, 1],
    [0, 0, 0, 0, 1],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 1, 0]
]

# matrice de récompenses 
# -1 pour les cases vides,-100 pour les murs, 
# +100 pour l'arrivée, bonus de +5 pour encourager certaines cases)

rewardMap = [
    [-1,  -1,  -100, -1,  -1],
    [-100, -1, -100, -1, -100],
    [-1,  -1,   5,   -1, -100],
    [-1, -100, -100, -1,  -1],
    [-1,  -1,  -1,  -100, 100]
]

start = (0, 0)  # position de départ
goal = (4, 4)   # position d'arrivée

class Maze:
    def __init__(self, grid, reward, start, goal):
        self.grid = grid
        self.reward = reward
        
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        
        self.start = start
        self.goal = goal

    def insideGrid(self, position): # vérifie si une position est à l'intérieur du labytinthe
        x, y = position
        return 0 <= x < self.rows and 0 <= y < self.cols 
    
    def isPassable(self, position): # vérifie si la position est franchissable
        x, y = position
        return self.grid[x][y] == 0

    def getPassableNeighbors(self, position): # retourne les voisins franchissables à partir d'une position
        x, y = position
        neighbors = [] # liste des voisins franchissables
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]: # directions possibles (haut, bas, gauche, droite)
            newPos = (x + dx, y + dy)
            if self.insideGrid(newPos) and self.isPassable(newPos):
                neighbors.append(newPos)
        return neighbors
