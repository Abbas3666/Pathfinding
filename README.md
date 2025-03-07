# Projet Path Finding

Ce projet implémente et compare quatre algorithmes de recherche de chemin dans un environnement 2D :
- **BFS** (Recherche en largeur d'abord)
- **Dijkstra**
- **A*** (A étoile)
- **Algorithme Glouton**

Les algorithmes sont implémentés en **Julia** et testés sur des cartes 2D provenant de jeux vidéo et de villes réelles.

---

## Objectifs du Projet

1. **Implémenter plusieurs algorithmes** de recherche de chemin.
2. **Comparer leurs performances** en termes de temps d'exécution, de nombre d'états explorés, et de distance du chemin trouvé.
3. **Appliquer ces algorithmes** à des cartes 2D issues de jeux vidéo ou de villes réelles.

---

## Structure des Fichiers

Le projet contient les fichiers suivants :
```
projet-pathfinding/
├── projet.jl    # Contient les implémentations des algorithmes
├── bbb.map      # Fichier de carte textuel représentant l'environnement
└── README.md    # Documentation du projet
```

---

## Consignes du Projet

- **Implémenter les algorithmes from scratch** (sans bibliothèques de graphes existantes).
- **Tester les algorithmes sur au moins trois cartes différentes.**
- **Comparer les performances** en mesurant la distance, le nombre d'états explorés et le temps d'exécution.

---

## Installation et Utilisation

### **1. Prérequis**
- **Julia** : [Télécharger Julia](https://julialang.org/downloads/)
- **Dépendances** : Installer les packages nécessaires :
  ```bash
  julia -e 'using Pkg; Pkg.add(["DataStructures"])'
  ```

### **2. Exécution des Algorithmes**

Les algorithmes peuvent être exécutés avec les fonctions suivantes :
```julia
algoBFS(nom_fichier, depart, arrivee)
algoDijkstra(nom_fichier, depart, arrivee)
algoGlouton(nom_fichier, depart, arrivee)
algoAstar(nom_fichier, depart, arrivee)
```

**Paramètres**
- `nom_fichier` : Nom du fichier de carte (exemple : "bbb.map").
- `depart` : Coordonnées du point de départ `(x, y)`.
- `arrivee` : Coordonnées du point d'arrivée `(x, y)`.
##Example
algoBFS("bbb.map", (12, 5), (2, 18))
algoDijkstra("bbb.map", (12, 5), (2, 18))
algoGlouton("bbb.map", (12, 5), (2, 18))
algoAstar("bbb.map", (12, 5), (2, 18))
```

**Paramètres** :
- `"bbb.map"` : Fichier de carte en texte.
- `(12, 5)` : Coordonnées du point de départ (ligne, colonne).
- `(2, 18)` : Coordonnées du point d'arrivée (ligne, colonne).

### **Résultats Attendus**
Chaque algorithme affiche :
- **Distance** : Distance totale du chemin trouvé.
- **États explorés** : Nombre de cases visitées.
- **Chemin trouvé** : Séquence des coordonnées du chemin parcouru.

**Exemple de sortie**
```
Solution (BFS) :
Distance Départ → Arrivée : 21
Nombre d'états explorés : 321
Chemin : (12, 5) → (11, 5) → ... → (2, 18)
```

---

## Comparaison des Algorithmes

### **Performances**
- **BFS** : Simple mais inefficace sur des graphes pondérés.
- **Dijkstra** : Trouve le chemin optimal mais plus lent que A*.
- **A*** : Rapide et efficace grâce à l'utilisation d'une heuristique.
- **Algorithme Glouton** : Rapide mais ne garantit pas l'optimalité.


| Algorithme | Distance | États Explorés | Temps d'Exécution (s) |
|------------|---------|---------------|----------------------|
| BFS        | 23      | 370           | 0.00069              |
| Dijkstra   | 23      | 383           | 0.040                |
| Glouton    | 29      | 74            | 0.00028              |
| A*         | 23      | 70            | 0.00050              |



