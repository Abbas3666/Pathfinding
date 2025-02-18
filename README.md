# Projet Path Finding

Ce projet implémente et compare quatre algorithmes de recherche de chemin dans un environnement 2D :
- **BFS** (Breadth-First Search)
- **Dijkstra**
- **A***
- **Algorithme Glouton**

Les algorithmes sont implémentés en **Julia** et testés sur des cartes 2D provenant de jeux vidéo et de villes réelles.

---

## Structure du Projet

Le projet est organisé comme suit :
```
projet-pathfinding/
├── src/        # Contient les fichiers de code
│   ├── pathfinding.jl  # Fichier principal avec les algorithmes
├── dat/        # Contient les fichiers de carte (.map)
├── doc/        # Contient le rapport du projet
├── results/    # Contient les résultats (tableaux, graphiques)
└── README.md   # Ce fichier
```

---

## Consignes et Objectifs

### **Objectifs**
1. Implémenter les quatre algorithmes de recherche de chemin (BFS, Dijkstra, A*, Glouton).
2. Tester les algorithmes sur des cartes 2D provenant de jeux vidéo et de villes réelles.
3. Comparer les performances des algorithmes en termes de :
   - Temps d'exécution.
   - Nombre d'états explorés.
   - Distance du chemin trouvé.
4. Produire un rapport détaillé et une documentation claire.

### **Consignes**
- Les algorithmes doivent être implémentés **from scratch** (sans utiliser de bibliothèques de graphes existantes).
- Les résultats doivent inclure :
  - La distance du chemin trouvé.
  - Le nombre d'états explorés.
  - Le chemin parcouru.
- Le projet doit être documenté et structuré sur GitHub.

---

## Installation et Utilisation

### **1. Prérequis**
- **Julia** : Téléchargez et installez Julia depuis [https://julialang.org/downloads/](https://julialang.org/downloads/).
- **Dépendances** : Installez les packages nécessaires en exécutant :
  ```bash
  julia -e 'using Pkg; Pkg.add(["DataStructures", "Plots"])'
  ```

### **2. Téléchargement du Projet**
Clonez le dépôt GitHub :
```bash
git clone https://github.com/ton-utilisateur/projet-pathfinding.git
cd projet-pathfinding
```

### **3. Exécution des Algorithmes**
Les algorithmes peuvent être invoqués avec les fonctions suivantes :
```julia
run_bfs(fname::String, start::Tuple{Int, Int}, goal::Tuple{Int, Int})
run_dijkstra(fname::String, start::Tuple{Int, Int}, goal::Tuple{Int, Int})
run_greedy(fname::String, start::Tuple{Int, Int}, goal::Tuple{Int, Int})
run_astar(fname::String, start::Tuple{Int, Int}, goal::Tuple{Int, Int})
```

**Paramètres**
- `fname` : Nom du fichier de carte (exemple : "bbb.map").
- `start` : Coordonnées du point de départ `(x, y)`.
- `goal` : Coordonnées du point d'arrivée `(x, y)`.

**Exemple d'Appel**
```julia
fname = "dat/bbb.map"
start = (12, 5)  # Point de départ
goal = (2, 12)   # Point d'arrivée

# Appeler les algorithmes
run_bfs(fname, start, goal)
run_dijkstra(fname, start, goal)
run_greedy(fname, start, goal)
run_astar(fname, start, goal)
```

### **Résultats Attendus**
Chaque algorithme affichera les résultats suivants :
- **Distance Start → Goal** : La distance totale du chemin trouvé.
- **Nombre d'états explorés** : Le nombre de cases visitées par l'algorithme.
- **Chemin trouvé** : Séquence des coordonnées du chemin parcouru.

**Exemple de Sortie**
```
Solution (BFS) :
Distance Start → Goal : 21
Nombre d'états explorés : 321
Chemin : (12, 5) → (11, 5) → ... → (2, 12)
```

---

## Comparaison des Algorithmes

### **Performances**
- **BFS** : Simple mais inefficace sur des graphes pondérés.
- **Dijkstra** : Trouve le chemin optimal mais plus lent que A*.
- **A*** : Rapide et efficace grâce à l'utilisation d'une heuristique.
- **Algorithme Glouton** : Rapide mais ne garantit pas l'optimalité.

### **Résultats**
Exemple de comparaison des performances :

| Algorithme | Distance | États Explorés | Temps d'Exécution (s) |
|------------|---------|---------------|----------------------|
| BFS        | 21      | 321           | 0.00069              |
| Dijkstra   | 14.0    | 162           | 0.040                |
| Glouton    | 23      | 71            | 0.00028              |
| A*         | 21      | 95            | 0.00050              |
# Pathfinding
