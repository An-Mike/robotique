class Maze: #classe Maze 
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
    
    def solveAstar(self): # implémentation de l'algorithme A* pour trouver le chemin optimal du start au goal
        from queue import PriorityQueue
        
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1]) # distance de Manhattan
        
        # Initialisation : crée une file prioritaire pour explorer les nœuds les plus prometteurs en premier
        # On commence par le start avec un coût réel de 0
        open_set = PriorityQueue()
        open_set.put((0, self.start))
        
        came_from = {} #pour chaque position, on garde d'où on vient
        
        g_score = {self.start: 0} #coût réel du chemin depuis le start jusqu'à chaque position explorée
        
        f_score = {self.start: heuristic(self.start, self.goal)} # cout estimé total pour chaque position
        
        explored = set()  # Ensemble pour tracer toutes les cellules explorées
        
        while not open_set.empty():  # Tant qu'il y a des nœuds à explorer
            current = open_set.get()[1] #position avec le coût total le plus faible
            
            explored.add(current)  # Marquer cette cellule comme explorée
            
            # Si on a atteint le goal, on reconstruit et retourne le chemin optimal
            if current == self.goal:
                path = []
                # On remonte le chemin en suivant les "parentés" enregistrées dans came_from
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                final_path = path[::-1] # retourne le chemin du start au goal (en l'inversant)
                
                # Afficher la visualisation avec le chemin trouvé et les cellules explorées
                self.visualizePath(final_path, explored)
                
                return final_path
            
            for neighbor in self.getPassableNeighbors(current): #exploration des voisins franchissables à partir du nœud actuel
                tentative_g_score = g_score[current] + 1 #coût pour atteindre le voisin 
                
                # Si c'est la première fois qu'on visite ce voisin, ou si on a trouvé un chemin plus court :
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # On mémorise d'où vient le voisin (pour reconstruire le chemin plus tard)
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, self.goal) # On calcule le coût estimé total 
                    

                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((f_score[neighbor], neighbor))
        
        # Si aucun chemin n'est trouvé, afficher la grille avec les cellules explorées
        self.visualizePath(None, explored)
        return None # retourne None si aucun chemin n'est trouvé (goal inaccessible)
    
    def visualizePath(self, path, explored=None):
        """
        Visualise la grille avec le chemin trouvé et les cellules explorées
        path : liste des positions du chemin trouvé
        explored : set des positions explorées par l'algorithme (optionnel)
        """
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        import numpy as np
        
        # Créer une matrice pour la visualisation (RGB)
        visual_grid = np.zeros((self.rows, self.cols, 3))
        
        # Remplir la grille de base :
        # Blanc pour les cases franchissables, Noir pour les murs
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] == 1:  # Mur
                    visual_grid[i][j] = [0, 0, 0]  # Noir
                else:  # Case franchissable
                    visual_grid[i][j] = [1, 1, 1]  # Blanc
        
        # Ajouter les cellules explorées en bleu (si fournies)
        if explored:
            for pos in explored:
                if pos != self.start and pos != self.goal:  # Ne pas colorer le start et goal
                    visual_grid[pos[0]][pos[1]] = [0, 0, 1]  # Bleu
        
        # Ajouter le chemin trouvé en vert
        if path:
            for pos in path:
                if pos != self.start and pos != self.goal:  # Ne pas colorer le start et goal
                    visual_grid[pos[0]][pos[1]] = [0, 1, 0]  # Vert
        
        # Marquer le start en rouge et le goal en jaune
        visual_grid[self.start[0]][self.start[1]] = [1, 0, 0]  # Rouge
        visual_grid[self.goal[0]][self.goal[1]] = [1, 1, 0]   # Jaune
        
        # Créer la figure
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(visual_grid, interpolation='nearest')
        
        # Ajouter une grille de délimitation
        ax.set_xticks(np.arange(-0.5, self.cols, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, self.rows, 1), minor=True)
        ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
        
        # Créer la légende
        legend_patches = [
            mpatches.Patch(facecolor=[1, 1, 1], edgecolor='black', label='Chemin franchissable'),
            mpatches.Patch(facecolor=[0, 0, 0], edgecolor='white', label='Mur'),
            mpatches.Patch(facecolor=[0, 0, 1], edgecolor='black', label='Cellules explorées'),
            mpatches.Patch(facecolor=[0, 1, 0], edgecolor='black', label='Chemin trouvé'),
            mpatches.Patch(facecolor=[1, 0, 0], edgecolor='black', label='Start'),
            mpatches.Patch(facecolor=[1, 1, 0], edgecolor='black', label='Goal'),
        ]
        ax.legend(handles=legend_patches, loc='upper left', bbox_to_anchor=(1.05, 1))
        
        ax.set_title('Visualisation de l\'algorithme A*')
        ax.set_xlabel('Colonne')
        ax.set_ylabel('Ligne')
        
        plt.tight_layout()
        plt.show()
