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
    
    def getPassableNeighborsWithDiag(self, position): # retourne les voisins franchissables incluant les diagonales
        x, y = position
        neighbors = [] # liste des voisins franchissables
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # 8 directions (4 cardinales + 4 diagonales)
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
                
                # Calculer le coût total du chemin
                total_cost = g_score[self.goal]
                
                # Afficher la visualisation avec le chemin trouvé et les cellules explorées
                self.visualizePath(final_path, explored, total_cost, len(explored), algo_name='A*')
                
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
        self.visualizePath(None, explored, None, len(explored) if explored else 0, algo_name='A*')
        return None # retourne None si aucun chemin n'est trouvé (goal inaccessible)
    
    def solveAstarWithDiag(self): # A* avec déplacements diagonaux autorisés et coût euclidien
        from queue import PriorityQueue
        import math
        
        def heuristic(a, b):
            # Heuristique euclidienne (distance géométrique)
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        def move_cost(from_pos, to_pos):
            # Coût euclidien du mouvement
            return math.sqrt((from_pos[0] - to_pos[0])**2 + (from_pos[1] - to_pos[1])**2)
        
        # Initialisation : crée une file prioritaire pour explorer les nœuds les plus prometteurs en premier
        open_set = PriorityQueue()
        open_set.put((0, self.start))
        
        came_from = {} # pour chaque position, on garde d'où on vient
        
        g_score = {self.start: 0.0} # coût réel du chemin depuis le start jusqu'à chaque position explorée
        
        f_score = {self.start: heuristic(self.start, self.goal)} # coût estimé total pour chaque position
        
        explored = set() # Ensemble pour tracer toutes les cellules explorées
        
        while not open_set.empty(): # Tant qu'il y a des nœuds à explorer
            current = open_set.get()[1] # position avec le coût total le plus faible
            
            explored.add(current) # Marquer cette cellule comme explorée
            
            # Si on a atteint le goal, on reconstruit et retourne le chemin optimal
            if current == self.goal:
                path = []
                # On remonte le chemin en suivant les "parentés" enregistrées dans came_from
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                final_path = path[::-1] # retourne le chemin du start au goal (en l'inversant)
                
                # Calculer le coût total du chemin
                total_cost = g_score[self.goal]
                
                # Afficher la visualisation avec le chemin trouvé et les cellules explorées
                self.visualizePath(final_path, explored, total_cost, len(explored), algo_name='A* (diag)')
                
                return final_path
            
            for neighbor in self.getPassableNeighborsWithDiag(current): # exploration des voisins (8 directions)
                tentative_g_score = g_score[current] + move_cost(current, neighbor) # coût euclidien
                
                # Si c'est la première fois qu'on visite ce voisin, ou si on a trouvé un chemin plus court :
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # On mémorise d'où vient le voisin (pour reconstruire le chemin plus tard)
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, self.goal) # On calcule le coût estimé total
                    
                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((f_score[neighbor], neighbor))
        
        # Si aucun chemin n'est trouvé, afficher la grille avec les cellules explorées
        self.visualizePath(None, explored, None, len(explored) if explored else 0, algo_name='A* (diag)')
        return None # retourne None si aucun chemin n'est trouvé (goal inaccessible)
    
    def visualizePath(self, path, explored=None, cost=None, num_explored=None, algo_name=None):
        """
        Visualise la grille avec le chemin trouvé et les cellules explorées
        path : liste des positions du chemin trouvé
        explored : set des positions explorées par l'algorithme (optionnel)
        cost : coût total du chemin trouvé (optionnel)
        num_explored : nombre de cellules explorées (optionnel)
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
        
        # Afficher le coût et le nombre de cellules explorées dans le titre
        algo_display = algo_name if algo_name is not None else 'A*'
        title_parts = [f'Visualisation de l\'algorithme: {algo_display}']
        if cost is not None:
            title_parts.append(f'Coût: {cost:.2f}')
        if num_explored is not None:
            title_parts.append(f'Cellules explorées: {num_explored}')
        ax.set_title(' - '.join(title_parts))
        ax.set_xlabel('Colonne')
        ax.set_ylabel('Ligne')
        
        plt.tight_layout()
        plt.show()

    def solveAstarWithDirCost(self, epsilon=0.25):
        """
        Variante de A* qui ajoute un coût fixe `epsilon` quand on change de direction.
        - `epsilon`: coût additionnel appliqué lorsqu'on change de direction par rapport
          au mouvement précédent.
        Retourne le chemin (liste de positions) ou None si aucun chemin trouvé.
        Affiche la grille en utilisant `visualizePath` (avec les cellules explorées).
        """
        from queue import PriorityQueue

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        # Etat : (position, direction) où direction est un tuple (dx,dy) ou None pour le start
        start_state = (self.start, None)

        open_set = PriorityQueue()
        open_set.put((heuristic(self.start, self.goal), start_state))

        came_from = {}  # map state -> previous state
        g_score = {start_state: 0.0}
        f_score = {start_state: heuristic(self.start, self.goal)}

        explored = set()

        while not open_set.empty():
            current_state = open_set.get()[1]
            current_pos, current_dir = current_state

            explored.add(current_pos)

            # Si on a atteint le goal (quelque soit la direction)
            if current_pos == self.goal:
                # reconstruire le chemin en ignorant les directions
                path = []
                s = current_state
                while s in came_from:
                    path.append(s[0])
                    s = came_from[s]
                path.append(self.start)
                final_path = path[::-1]

                # Calculer le coût total du chemin
                total_cost = g_score[current_state]

                # visualiser et retourner
                self.visualizePath(final_path, explored, total_cost, len(explored), algo_name='A* (dir cost)')
                return final_path

            # explorer voisins
            for neighbor in self.getPassableNeighbors(current_pos):
                # direction du mouvement actuel
                move_dir = (neighbor[0] - current_pos[0], neighbor[1] - current_pos[1])

                # coût de base pour se déplacer
                base_cost = 1.0
                # pénalité si on change de direction (si on avait une direction précédente)
                dir_penalty = 0.0 if (current_dir is None or current_dir == move_dir) else float(epsilon)

                tentative_g = g_score[current_state] + base_cost + dir_penalty

                neighbor_state = (neighbor, move_dir)

                if neighbor_state not in g_score or tentative_g < g_score[neighbor_state]:
                    came_from[neighbor_state] = current_state
                    g_score[neighbor_state] = tentative_g
                    f_score[neighbor_state] = tentative_g + heuristic(neighbor, self.goal)

                    # ajouter à la file si pas déjà présent
                    if neighbor_state not in [i[1] for i in open_set.queue]:
                        open_set.put((f_score[neighbor_state], neighbor_state))

        # aucun chemin trouvé -> afficher exploré
        self.visualizePath(None, explored, None, len(explored) if explored else 0, algo_name='A* (dir cost)')
        return None

    def dijkstra(self):
        """
        Résolution par l'algorithme de Dijkstra (4 directions, coût 1 par déplacement).
        Affiche la grille finale avec le coût total et le nombre de cellules explorées.
        Retourne le chemin (liste de positions) ou None si aucun chemin trouvé.
        """
        from queue import PriorityQueue

        open_set = PriorityQueue()
        open_set.put((0, self.start))

        came_from = {}
        g_score = {self.start: 0.0}

        explored = set()

        while not open_set.empty():
            current = open_set.get()[1]

            explored.add(current)

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                final_path = path[::-1]

                total_cost = g_score[self.goal]
                self.visualizePath(final_path, explored, total_cost, len(explored), algo_name='Dijkstra')
                return final_path

            for neighbor in self.getPassableNeighbors(current):
                tentative_g = g_score[current] + 1.0

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((g_score[neighbor], neighbor))

        self.visualizePath(None, explored, None, len(explored) if explored else 0, algo_name='Dijkstra')
        return None

    def dijkstraWithDiag(self):
        """
        Résolution par l'algorithme de Dijkstra en autorisant les déplacements diagonaux.
        Le coût de déplacement est la distance euclidienne entre cases.
        Affiche la grille finale avec le coût total et le nombre de cellules explorées.
        Retourne le chemin (liste de positions) ou None si aucun chemin trouvé.
        """
        from queue import PriorityQueue
        import math

        def move_cost(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_set = PriorityQueue()
        open_set.put((0, self.start))

        came_from = {}
        g_score = {self.start: 0.0}

        explored = set()

        while not open_set.empty():
            current = open_set.get()[1]

            explored.add(current)

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                final_path = path[::-1]

                total_cost = g_score[self.goal]
                self.visualizePath(final_path, explored, total_cost, len(explored), algo_name='Dijkstra (diag)')
                return final_path

            for neighbor in self.getPassableNeighborsWithDiag(current):
                tentative_g = g_score[current] + move_cost(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((g_score[neighbor], neighbor))

        self.visualizePath(None, explored, None, len(explored) if explored else 0, algo_name='Dijkstra (diag)')
        return None

    def solveAstarWithPerfectHeuristic(self):
        """
        Résolution par A* avec une heuristique parfaite (déplacements diagonaux).
        L'heuristique est exactement égale au coût restant pour atteindre le goal.
        Autorise les 8 déplacements (4 cardinaux + 4 diagonaux) avec coût euclidien.
        Cela signifie que seules les cellules du chemin optimal sont explorées.
        Affiche la grille finale avec le coût total et le nombre de cellules explorées.
        Retourne le chemin (liste de positions) ou None si aucun chemin trouvé.
        """
        from queue import PriorityQueue
        import math

        def move_cost(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        # Pré-calculer les distances exactes au goal pour chaque cellule (heuristique parfaite)
        # On utilise Dijkstra rétrograde depuis le goal pour calculer le coût exact
        exact_cost_to_goal = {}
        pq = PriorityQueue()
        pq.put((0, self.goal))
        exact_cost_to_goal[self.goal] = 0

        while not pq.empty():
            cost, pos = pq.get()
            if pos in exact_cost_to_goal and exact_cost_to_goal[pos] < cost:
                continue
            for neighbor in self.getPassableNeighborsWithDiag(pos):
                new_cost = cost + move_cost(pos, neighbor)  # Coût euclidien
                if neighbor not in exact_cost_to_goal or new_cost < exact_cost_to_goal[neighbor]:
                    exact_cost_to_goal[neighbor] = new_cost
                    pq.put((new_cost, neighbor))

        def perfect_heuristic(pos):
            # Retourner le coût exact si connu, sinon 0 (ne devrait pas arriver)
            return exact_cost_to_goal.get(pos, 0)

        # Maintenant résoudre avec A* en utilisant la heuristique parfaite
        open_set = PriorityQueue()
        open_set.put((0, self.start))

        came_from = {}
        g_score = {self.start: 0.0}
        f_score = {self.start: perfect_heuristic(self.start)}

        explored = set()

        while not open_set.empty():
            current = open_set.get()[1]

            explored.add(current)

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                final_path = path[::-1]

                total_cost = g_score[self.goal]
                self.visualizePath(final_path, explored, total_cost, len(explored), algo_name='A* (perfect heuristic)')
                return final_path

            for neighbor in self.getPassableNeighborsWithDiag(current):
                tentative_g = g_score[current] + move_cost(current, neighbor)  # Coût euclidien

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + perfect_heuristic(neighbor)

                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((f_score[neighbor], neighbor))

        self.visualizePath(None, explored, None, len(explored) if explored else 0, algo_name='A* (perfect heuristic)')
        return None

    def solveAstarNegative(self):
        """
        Résolution par A* en utilisant la reward map pour calculer les coûts de déplacement.
        4 directions autorisées (pas de diagonales).
        Coût de déplacement = 1 - (reward / 100)
        - Les rewards positives donnent des coûts bas
        - Les rewards négatives donnent des coûts élevés
        Affiche la grille finale avec le coût total et le nombre de cellules explorées.
        Retourne le chemin (liste de positions) ou None si aucun chemin trouvé.
        """
        from queue import PriorityQueue

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Distance de Manhattan

        def move_cost(pos):
            # Coût basé sur la reward de la cellule destination
            reward = self.reward[pos[0]][pos[1]]
            return 1 - (reward / 100.0)

        open_set = PriorityQueue()
        open_set.put((0, self.start))

        came_from = {}
        g_score = {self.start: 0.0}
        f_score = {self.start: heuristic(self.start, self.goal)}

        explored = set()

        while not open_set.empty():
            current = open_set.get()[1]

            explored.add(current)

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                final_path = path[::-1]

                total_cost = g_score[self.goal]
                self.visualizePath(final_path, explored, total_cost, len(explored), algo_name='A* (reward-based)')
                return final_path

            for neighbor in self.getPassableNeighbors(current):
                tentative_g = g_score[current] + move_cost(neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, self.goal)

                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((f_score[neighbor], neighbor))

        self.visualizePath(None, explored, None, len(explored) if explored else 0, algo_name='A* (reward-based)')
        return None

    def dijkstraNegative(self):
        """
        Résolution par Dijkstra en utilisant la reward map pour calculer les coûts de déplacement.
        4 directions autorisées (pas de diagonales).
        Coût de déplacement = 1 - (reward / 100)
        - Les rewards positives donnent des coûts bas
        - Les rewards négatives donnent des coûts élevés
        Affiche la grille finale avec le coût total et le nombre de cellules explorées.
        Retourne le chemin (liste de positions) ou None si aucun chemin trouvé.
        """
        from queue import PriorityQueue

        def move_cost(pos):
            # Coût basé sur la reward de la cellule destination
            reward = self.reward[pos[0]][pos[1]]
            return 1 - (reward / 100.0)

        open_set = PriorityQueue()
        open_set.put((0, self.start))

        came_from = {}
        g_score = {self.start: 0.0}

        explored = set()

        while not open_set.empty():
            current = open_set.get()[1]

            explored.add(current)

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                final_path = path[::-1]

                total_cost = g_score[self.goal]
                self.visualizePath(final_path, explored, total_cost, len(explored), algo_name='Dijkstra (reward-based)')
                return final_path

            for neighbor in self.getPassableNeighbors(current):
                tentative_g = g_score[current] + move_cost(neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((g_score[neighbor], neighbor))

        self.visualizePath(None, explored, None, len(explored) if explored else 0, algo_name='Dijkstra (reward-based)')
        return None
