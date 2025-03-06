# **Projet Pathfinding**

Ce projet implémente plusieurs algorithmes de recherche de chemin en **Julia** pour naviguer dans une carte 2D.  

### **Algorithmes Implémentés :**  
- **BFS** (Recherche en largeur d'abord)  
- **Dijkstra**  
- **A*** (A étoile)  
- **Glouton**  

---

## **Fichiers du Projet**  
```
projet-pathfinding/
├── projet.jl   # Code source en Julia
├── bbb.map     # Fichier contenant la carte
└── README.md   # Documentation du projet
```

---

## **Installation et Exécution**  

### **1. Prérequis**  
- Installer **Julia** : [https://julialang.org/downloads/](https://julialang.org/downloads/)  
- Installer les dépendances :  
  ```julia
  using Pkg
  Pkg.add(["DataStructures", "Printf"])
  ```

### **2. Exécuter un Algorithme**  
Charger et exécuter le fichier `projet.jl` dans Julia :  
```julia
include("projet.jl")
```

Lancer un algorithme :  
```julia
algoBFS("bbb.map", (12, 5), (2, 18))
```

**Paramètres :**  
- `"bbb.map"` → Nom du fichier de carte  
- `(12, 5)` → Coordonnées du point de départ  
- `(2, 18)` → Coordonnées du point d’arrivée  

---

## **Exemple de Sortie**  
```
Solution BFS :
Distance : 23
États explorés : 370
Chemin : (12, 5) → (11, 5) → ... → (2, 18)
```

### **Comparaison des Algorithmes**
| Algorithme | Distance | États Explorés | Temps (s) |
|------------|---------|---------------|----------|
| BFS        | 23      | 370           | 0.018    |
| Dijkstra   | 23.0    | 383           | 0.054    |
| Glouton    | 29      | 74            | 0.000047 |
| A*         | 23.0    | 70            | 0.000053 |

---

## **Résumé des Algorithmes**  
- **BFS** : Explore tout, inefficace pour les grands graphes.  
- **Dijkstra** : Trouve le chemin optimal mais peut être lent.  
- **A\*** : Rapide et optimal grâce à une heuristique.  
- **Glouton** : Très rapide mais ne garantit pas le chemin optimal.  

---

