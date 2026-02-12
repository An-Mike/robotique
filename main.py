import heapq
import numpy as np
from typing import List, Tuple, Optional, Set
import random


class Maze:
    """
    Classe représentant un labyrinthe pour la résolution de chemins avec Dijkstra et A*.
    
    Attributes:
        height (int): Hauteur de la grille
        width (int): Largeur de la grille
        grid (np.ndarray): Grille du labyrinthe (0=libre, 1=obstacle)
        reward_map (np.ndarray): Matrice des récompenses
        start (Tuple[int, int]): Position de départ (row, col)
        goal (Tuple[int, int]): Position d'arrivée (row, col)
    """
    
    def __init__(self, height: int, width: int, 
                 start: Tuple[int, int], 
                 goal: Tuple[int, int]):
        """
        Initialise un labyrinthe vide.
        
        Args:
            height: Hauteur de la grille
            width: Largeur de la grille
            start: Position de départ (row, col)
            goal: Position d'arrivée (row, col)
        """
        self.height = height
        self.width = width
        self.start = start
        self.goal = goal
        
        # Initialisation de la grille (tout libre par défaut)
        self.grid = np.zeros((height, width), dtype=int)
        
        # Initialisation de la reward map avec pénalité par défaut
        self.reward_map = np.full((height, width), -1.0)
        
        # Récompense importante pour l'arrivée
        self.reward_map[goal] = 100.0
        
    def is_valid(self, row: int, col: int) -> bool:
        """
        Vérifie si une cellule appartient à la grille.
        
        Args:
            row: Ligne de la cellule
            col: Colonne de la cellule
            
        Returns:
            True si la cellule est dans les limites de la grille
        """
        return 0 <= row < self.height and 0 <= col < self.width
    
    def is_free(self, row: int, col: int) -> bool:
        """
        Vérifie si une cellule est franchissable.
        
        Args:
            row: Ligne de la cellule
            col: Colonne de la cellule
            
        Returns:
            True si la cellule est valide et libre (pas d'obstacle)
        """
        return self.is_valid(row, col) and self.grid[row, col] == 0
    
    def get_neighbors(self, row: int, col: int) -> List[Tuple[int, int]]:
        """
        Retourne les cellules voisines accessibles (voisinage 4-connecté).
        
        Args:
            row: Ligne de la cellule
            col: Colonne de la cellule
            
        Returns:
            Liste des positions voisines accessibles [(row, col), ...]
        """
        # Directions : haut, bas, gauche, droite
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if self.is_free(new_row, new_col):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def add_obstacle(self, row: int, col: int) -> bool:
        """
        Ajoute un obstacle à une position donnée.
        
        Args:
            row: Ligne de l'obstacle
            col: Colonne de l'obstacle
            
        Returns:
            True si l'obstacle a été ajouté, False sinon
        """
        # Ne pas ajouter d'obstacle sur le départ ou l'arrivée
        if (row, col) == self.start or (row, col) == self.goal:
            return False
        
        if self.is_valid(row, col):
            self.grid[row, col] = 1
            return True
        return False
    
    def add_random_obstacles(self, obstacle_ratio: float = 0.2, seed: Optional[int] = None):
        """
        Ajoute des obstacles aléatoires dans le labyrinthe.
        
        Args:
            obstacle_ratio: Proportion d'obstacles (entre 0 et 1)
            seed: Graine pour la génération aléatoire (optionnel)
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        num_obstacles = int(self.height * self.width * obstacle_ratio)
        obstacles_added = 0
        
        while obstacles_added < num_obstacles:
            row = random.randint(0, self.height - 1)
            col = random.randint(0, self.width - 1)
            
            if self.add_obstacle(row, col):
                obstacles_added += 1
    
    def add_bonus_rewards(self, positions: List[Tuple[int, int]], reward: float = 5.0):
        """
        Ajoute des bonus à certaines positions.
        
        Args:
            positions: Liste des positions où ajouter des bonus
            reward: Valeur du bonus
        """
        for row, col in positions:
            if self.is_valid(row, col):
                self.reward_map[row, col] = reward
    
    def manhattan_distance(self, row: int, col: int) -> float:
        """
        Calcule la distance de Manhattan jusqu'au goal.
        
        Args:
            row: Ligne de la cellule
            col: Colonne de la cellule
            
        Returns:
            Distance de Manhattan jusqu'au goal
        """
        return abs(row - self.goal[0]) + abs(col - self.goal[1])
    
    def euclidean_distance(self, row: int, col: int) -> float:
        """
        Calcule la distance euclidienne jusqu'au goal.
        
        Args:
            row: Ligne de la cellule
            col: Colonne de la cellule
            
        Returns:
            Distance euclidienne jusqu'au goal
        """
        return np.sqrt((row - self.goal[0])**2 + (col - self.goal[1])**2)
    
    def reconstruct_path(self, prev: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Reconstruit le chemin depuis le dictionnaire des prédécesseurs.
        
        Args:
            prev: Dictionnaire des prédécesseurs
            current: Position actuelle (généralement le goal)
            
        Returns:
            Liste ordonnée des positions constituant le chemin
        """
        path = []
        while current is not None:
            path.append(current)
            current = prev.get(current)
        path.reverse()
        return path
    
    def dijkstra(self) -> Optional[List[Tuple[int, int]]]:
        """
        Résout le labyrinthe avec l'algorithme de Dijkstra.
        
        Returns:
            Liste des positions du chemin optimal, ou None si aucun chemin n'existe
        """
        # Initialisation
        dist = {self.start: 0}
        prev = {self.start: None}
        
        # File de priorité : (distance, position)
        pq = [(0, self.start)]
        visited = set()
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
            # Si déjà visité, passer
            if current in visited:
                continue
            
            visited.add(current)
            
            # Si on atteint le goal, reconstruire le chemin
            if current == self.goal:
                return self.reconstruct_path(prev, current)
            
            # Explorer les voisins
            for neighbor in self.get_neighbors(current[0], current[1]):
                if neighbor in visited:
                    continue
                
                # Coût du mouvement (basé sur la reward map)
                # On utilise -reward_map car on cherche à minimiser le coût
                move_cost = -self.reward_map[neighbor]
                alt_dist = current_dist + move_cost
                
                # Mise à jour si meilleur chemin trouvé
                if neighbor not in dist or alt_dist < dist[neighbor]:
                    dist[neighbor] = alt_dist
                    prev[neighbor] = current
                    heapq.heappush(pq, (alt_dist, neighbor))
        
        # Aucun chemin trouvé
        return None
    
    def solve(self, algorithm: str = 'astar') -> Optional[List[Tuple[int, int]]]:
        """
        Résout le labyrinthe avec l'algorithme A*.
        
        Args:
            algorithm: 'astar' ou 'dijkstra'
            
        Returns:
            Liste des positions du chemin optimal, ou None si aucun chemin n'existe
        """
        if algorithm == 'dijkstra':
            return self.dijkstra()
        
        # A* implementation
        # g[n] : coût réel depuis le départ
        g = {self.start: 0}
        prev = {self.start: None}
        
        # File de priorité : (f(n), position) où f(n) = g(n) + h(n)
        h_start = self.manhattan_distance(self.start[0], self.start[1])
        pq = [(h_start, self.start)]
        
        # Open set pour vérifier rapidement l'appartenance
        open_set = {self.start}
        closed_set = set()
        
        while pq:
            _, current = heapq.heappop(pq)
            
            # Retirer de l'open set
            if current in open_set:
                open_set.remove(current)
            
            # Si déjà exploré, passer
            if current in closed_set:
                continue
            
            # Si on atteint le goal, reconstruire le chemin
            if current == self.goal:
                return self.reconstruct_path(prev, current)
            
            # Marquer comme exploré
            closed_set.add(current)
            
            # Explorer les voisins
            for neighbor in self.get_neighbors(current[0], current[1]):
                if neighbor in closed_set:
                    continue
                
                # Coût du mouvement
                move_cost = -self.reward_map[neighbor]
                tentative_g = g[current] + move_cost
                
                # Si meilleur chemin trouvé
                if neighbor not in g or tentative_g < g[neighbor]:
                    g[neighbor] = tentative_g
                    prev[neighbor] = current
                    
                    # Calcul de f(n) = g(n) + h(n)
                    h = self.manhattan_distance(neighbor[0], neighbor[1])
                    f = tentative_g + h
                    
                    if neighbor not in open_set:
                        heapq.heappush(pq, (f, neighbor))
                        open_set.add(neighbor)
        
        # Aucun chemin trouvé
        return None
    
    def get_path_cost(self, path: List[Tuple[int, int]]) -> float:
        """
        Calcule le coût total d'un chemin.
        
        Args:
            path: Liste des positions du chemin
            
        Returns:
            Coût total du chemin
        """
        if not path:
            return float('inf')
        
        cost = 0
        for pos in path:
            cost -= self.reward_map[pos]  # On inverse car reward_map contient des récompenses
        
        return cost
    
    def print_maze(self, path: Optional[List[Tuple[int, int]]] = None):
        """
        Affiche le labyrinthe dans la console.
        
        Args:
            path: Chemin à afficher (optionnel)
        """
        path_set = set(path) if path else set()
        
        print("\n" + "=" * (self.width * 2 + 1))
        for row in range(self.height):
            line = "|"
            for col in range(self.width):
                if (row, col) == self.start:
                    line += "S "
                elif (row, col) == self.goal:
                    line += "G "
                elif (row, col) in path_set:
                    line += "· "
                elif self.grid[row, col] == 1:
                    line += "█ "
                else:
                    line += "  "
            line += "|"
            print(line)
        print("=" * (self.width * 2 + 1))
        
        if path:
            print(f"\nLongueur du chemin : {len(path)}")
            print(f"Coût du chemin : {self.get_path_cost(path):.2f}")


# Fonctions utilitaires pour générer des labyrinthes

def generate_empty_maze(height: int = 10, width: int = 10) -> Maze:
    """Génère un labyrinthe vide sans obstacles."""
    start = (0, 0)
    goal = (height - 1, width - 1)
    return Maze(height, width, start, goal)


def generate_random_maze(height: int = 10, width: int = 10, 
                         obstacle_ratio: float = 0.2, 
                         seed: Optional[int] = None) -> Maze:
    """Génère un labyrinthe avec des obstacles aléatoires."""
    start = (0, 0)
    goal = (height - 1, width - 1)
    maze = Maze(height, width, start, goal)
    maze.add_random_obstacles(obstacle_ratio, seed)
    return maze


def generate_impossible_maze(height: int = 10, width: int = 10) -> Maze:
    """Génère un labyrinthe impossible (goal inaccessible)."""
    start = (0, 0)
    goal = (height - 1, width - 1)
    maze = Maze(height, width, start, goal)
    
    # Créer un mur qui bloque complètement l'accès au goal
    for col in range(width):
        maze.add_obstacle(height - 2, col)
    
    return maze


if __name__ == "__main__":
    print("=== Test 1: Labyrinthe vide ===")
    maze1 = generate_empty_maze(8, 8)
    path1_astar = maze1.solve('astar')
    path1_dijkstra = maze1.solve('dijkstra')
    
    print("\nA*:")
    maze1.print_maze(path1_astar)
    
    print("\nDijkstra:")
    maze1.print_maze(path1_dijkstra)
    
    print("\n=== Test 2: Labyrinthe avec obstacles ===")
    maze2 = generate_random_maze(10, 10, obstacle_ratio=0.25, seed=42)
    path2_astar = maze2.solve('astar')
    path2_dijkstra = maze2.solve('dijkstra')
    
    print("\nA*:")
    maze2.print_maze(path2_astar)
    
    print("\nDijkstra:")
    maze2.print_maze(path2_dijkstra)
    
    print("\n=== Test 3: Labyrinthe impossible ===")
    maze3 = generate_impossible_maze(8, 8)
    path3 = maze3.solve('astar')
    
    if path3 is None:
        print("\nAucun chemin trouvé (comme attendu)")
        maze3.print_maze()
    else:
        maze3.print_maze(path3)
