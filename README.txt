____________________________________________________________________________________
TP N°01 de robotique : résolution d'un labyrinthe (comparaison entre Dijkstra et A*)
____________________________________________________________________________________

Les fichiers contenus dans ce projet :
- main.py 
- maze.py 
- generateMap.py 

main.py
    C'est dans ce fichier que s'effectue les tests.

maze.py
    il contient la classe Maze.
    les méthodes à connaitre : 
    -   insideGrid(position) : vérifie si une position (x,y) est à l'intérieur du labyrinthe
    -   isPassable(position) : vérifie si une position (x,y) est franchissable
    -   getPassableNeighbors(position) : retourne les voisins franchissables à partir d'une position (x,y) -- on considère 4 déplacements possible
    -   getPassableNeighborsWithDiag(position) : retourne les voisins franchissables à partir d'une position (x,y) -- on autorise les déplacements diagonaux
    -   solveAstar() : résolution par l'algo A* (4-connectés, sans coût de virage)
    -   solveAstarWithDiag() : résolution par l'algo A* (8-connectés, sans coût de virage)
    -   solveAstarWithDirCost(epsilon=number) : résolution par l'algo A* (4-connectés, considère un coût de virage epsilon)
    -   dijkstra() : résolution par l'algo de Dijkstra (4-connecté, sans coût de virage)
    -   dijkstraWithDiag() : résolution par l'algo de Dijkstra (8-connecté, sans coût de virage)
    -   solveAstarNegative() : résolution A* en se basant sur la reward map (pondération négative)
    -   dijkstraNegative() : résolution Dijkstra en se basant sur la reward map (pondération négative)
    -   solveAstarWithPerfectHeuristic(): résolution A* avec heuristique parfaite (8-connectés, sans coût de virage)


generateMap.py 
    il contient la fonction generateMaze() qui génère une grille et un reward map de manière pseudo-aléatoire en fonction d'un seed
    Arguments:
    - rows, cols: dimensions
    - start, goal: tuples (x, y)
    - mode:
        - "empty"  -> aucune obstacle
        - "simple" -> obstacles aléatoires mais chemin garanti
        - "blocked" -> aucun chemin possible entre start et goal
    - seed: int ou None. Si fourni, rend la génération reproductible.

    Retourne: (grid, reward_map)
    exemple : grid, rewardMap = generateMaze(rows, cols, start, goal, mode="simple", seed=42)