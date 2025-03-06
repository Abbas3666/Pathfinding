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
  julia -e 'using Pkg; Pkg.add(["DataStructures", "Printf"])'
  ```

### **2. Exécution des Algorithmes**

Les algorithmes peuvent être exécutés avec les fonctions suivantes :
```julia
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

| Algorithme | Distance | États Explorés | Temps d'Exécution (s) |
|------------|---------|---------------|----------------------|
| BFS        | 21      | 321           | 0.00069              |
| Dijkstra   | 14.0    | 162           | 0.040                |
| Glouton    | 23      | 71            | 0.00028              |
| A*         | 21      | 95            | 0.00050              |



