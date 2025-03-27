
# 🚀 Projet Path Finding (Recherche de Chemin)

Ce projet implémente et compare plusieurs algorithmes de recherche de chemin sur des cartes 2D issues de jeux ou de villes réelles.

## 🧠 Algorithmes Implémentés

- 🔁 **BFS** (Breadth-First Search) – Recherche en largeur
- 🟦 **Dijkstra** – Algorithme optimal basé sur les coûts
- 🌟 **A\*** (A étoile) – Recherche optimale avec heuristique
- ⚡ **Greedy Best-First** – Glouton, basé uniquement sur l’heuristique
- 🧮 **Weighted A\*** (WA*) – Variante de A\* avec pondération :
  - Standard (\(\omega \geq 1\))
  - Pondéré 0–1 (\(0 \leq \omega \leq 1\))
  - Dynamique (\(\omega\) diminue pendant l'exploration)

## 📂 Fichiers Principaux

- `aaa.jl` : Code principal regroupant tous les algorithmes
- `didactic.map`, `easy.map`, `difficult.map` : Fichiers de cartes tests
- `README.md` : Documentation du projet

## ⚙️ Utilisation

### Installation

```bash
julia -e 'using Pkg; Pkg.add(["DataStructures", "BenchmarkTools"])'
```

### Lancer un algorithme

Dans le terminal Julia, exécute :

```julia
include("aaa.jl")
algoAstar("easy.map", (1,1), (5,10))
algoWAstarStandard("difficult.map", (189,193), (226,437))
algoWAstarDynamique("easy.map", (1,1), (5,10))
algoWAstarWeighted01("didactic.map", (1,1), (5,10))
```

## 🔍 Résultats Affichés

Chaque appel affiche :
- 🟢 Le chemin trouvé
- 📏 La distance
- 🔄 Le nombre d’états explorés
- ⏱ Le temps CPU

## 💡 À savoir sur WA*

- `ω = 1.5` : parfois moins efficace que A* selon le cas.
- `ω = 1.6` : excellent compromis dans les grandes cartes.
- `ω = 2.0` : très rapide et efficace, sans perte d’optimalité dans notre test.
- `mode = :dynamic` : le poids diminue en avançant dans la recherche.
