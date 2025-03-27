
#  Projet Path Finding (Recherche de Chemin)

Ce projet implémente et compare plusieurs algorithmes de recherche de chemin sur des cartes 2D issues de jeux ou de villes réelles.

## Algorithmes Implémentés

- **BFS** (Breadth-First Search) – Recherche en largeur
-  **Dijkstra** – Algorithme optimal basé sur les coûts
-  **A\*** (A étoile) – Recherche optimale avec heuristique
-  **Greedy Best-First** – Glouton, basé uniquement sur l’heuristique
- Weighted A* (WA*) – Une variante de l'algorithme A* utilisant un facteur de pondération, avec trois versions possibles :

-Standard : lorsque le poids (omega) est supérieur ou égal à 1.
-Pondéré entre 0 et 1 : lorsque le poids est compris entre 0 et 1 (inclus).
-Dynamique : le poids commence avec une valeur initiale (par exemple 2.5) et diminue progressivement au fil de l'exploration, en fonction du nombre d’états évalués.


##  Fichiers Principaux

- `aaa.jl` : Code principal regroupant tous les algorithmes
- `didactic.map`, `easy.map`, `difficult.map bbb.map` : Fichiers de cartes tests
- `README.md` : Documentation du projet

##  Utilisation

### Installation

```bash
julia -e 'using Pkg; Pkg.add(["DataStructures", "BenchmarkTools"])'
```

### Lancer un algorithme

Dans le terminal Julia, exécute :

```julia
include("aaa.jl")
algoBFS("bbb.map", (12, 5), (2, 18))
algoDijkstra("bbb.map", (12, 5), (2, 18))
algoGlouton("bbb.map", (12, 5), (2, 18))
algoAstar("bbb.map", (12, 5), (2, 18))
**Paramètres** :
- "bbb.map" : Fichier de carte en texte.
- (12, 5) : Coordonnées du point de départ (ligne, colonne).
- (2, 18) : Coordonnées du point d'arrivée (ligne, colonne)

algoAstar("easy.map", (1,1), (5,10))
algoWAstarStandard("difficult.map", (189,193), (226,437))
algoWAstarDynamique("easy.map", (1,1), (5,10))
algoWAstarWeighted01("didactic.map", (1,1), (5,10))
```

##  Résultats Affichés

Chaque appel affiche :
-  Le chemin trouvé
-  La distance
-  Le nombre d’états explorés
-  Le temps CPU

##  À savoir sur WA*

- `ω = 1.5` : parfois moins efficace que A* selon le cas.
- `ω = 1.6` : excellent compromis dans les grandes cartes.
- `ω = 2.0` : très rapide et efficace, sans perte d’optimalité dans notre test.
- `mode = :dynamic` : le poids diminue en avançant dans la recherche.
