import random
from collections import deque


def generateMaze(rows, cols, start, goal, mode="empty", seed=None):
    """
    Génère une grille et une reward map.

    Arguments:
    - rows, cols: dimensions
    - start, goal: tuples (x, y)
    - mode:
        - "empty"  -> aucune obstacle
        - "simple" -> obstacles aléatoires mais chemin garanti
        - "blocked" -> aucun chemin possible entre start et goal
    - seed: int ou None. Si fourni, rend la génération reproductible.

    Retourne: (grid, reward_map)
    """

    # Utiliser un générateur local pour pouvoir être reproductible sans reseeder
    rng = random.Random(seed) if seed is not None else random

    # Vérification start/goal valides
    sx, sy = start
    gx, gy = goal

    if not (0 <= sx < rows and 0 <= sy < cols):
        raise ValueError("Start hors grille")
    if not (0 <= gx < rows and 0 <= gy < cols):
        raise ValueError("Goal hors grille")

    # Création d'une grille vide
    def make_empty_grid():
        return [[0 for _ in range(cols)] for _ in range(rows)]

    grid = make_empty_grid()

    if mode == "empty":
        pass

    elif mode == "simple":
        obstacle_density = 0.25

        # Générer jusqu'à obtenir un labyrinthe avec chemin (boucle contrôlée)
        max_attempts = 1000
        for attempt in range(max_attempts):
            grid = make_empty_grid()
            for i in range(rows):
                for j in range(cols):
                    if (i, j) != start and (i, j) != goal:
                        if rng.random() < obstacle_density:
                            grid[i][j] = 1

            # Toujours garantir start et goal libres
            grid[sx][sy] = 0
            grid[gx][gy] = 0

            if pathExists(grid, start, goal):
                break
        else:
            raise RuntimeError("Impossible de générer un labyrinthe simple avec chemin après plusieurs tentatives")

    elif mode == "blocked":
        grid = make_empty_grid()
        mid = rows // 2
        for j in range(cols):
            grid[mid][j] = 1

        # S'assurer start et goal sont libres
        grid[sx][sy] = 0
        grid[gx][gy] = 0

    else:
        raise ValueError("Mode inconnu")

    # Création reward map cohérente avec la grid
    reward_map = []
    for i in range(rows):
        row = []
        for j in range(cols):
            if grid[i][j] == 1:
                row.append(-100)
            else:
                row.append(-1)
        reward_map.append(row)

    # reward finale
    reward_map[gx][gy] = 100

    return grid, reward_map


# vérifier l'existence d'un chemin
def pathExists(grid, start, goal):
    rows = len(grid)
    cols = len(grid[0])
    visited = set()
    queue = deque([start])

    while queue:
        x, y = queue.popleft()

        if (x, y) == goal:
            return True

        if (x, y) in visited:
            continue

        visited.add((x, y))

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy

            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] == 0:
                    queue.append((nx, ny))

    return False