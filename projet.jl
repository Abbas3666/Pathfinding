using DataStructures
using Printf  # Pour formater l'affichage des nombres
using BenchmarkTools

# Fonction pour charger la carte à partir d'un fichier
function loadMap(fname) 
    # Lire toutes les lignes du fichier
    lines = readlines(fname)
    
    # Filtrer les lignes de commentaires (celles qui commencent par '#')
    lines = filter(line -> !startswith(line, "#"), lines)

    # Trouver les indices des lignes contenant la hauteur, la largeur et le début de la carte
    height_line_index = findfirst(line -> occursin("height", line), lines)
    width_line_index = findfirst(line -> occursin("width", line), lines)
    map_line_index = findfirst(line -> occursin("map", line), lines)

    # Vérifier si les lignes nécessaires sont présentes
    if height_line_index === nothing || width_line_index === nothing || map_line_index === nothing
        error("Ligne manquante pour la hauteur, la largeur ou le début de la carte dans le fichier.")
    end

    # Extraire la hauteur et la largeur de la carte
    height = parse(Int, split(lines[height_line_index])[2])
    width = parse(Int, split(lines[width_line_index])[2])

    # Initialiser la carte avec des valeurs par défaut pour les cellules infranchissables
    map = fill(Inf, height, width)  # Toutes les cellules sont infranchissables par défaut
    start_line = map_line_index + 1  # Ligne où commence la carte

    # Remplir la carte avec les valeurs correspondantes aux caractères lus
    for i in 1:height
        line = lines[start_line + i - 1]  # Lire la ligne correspondante
        for j in 1:width
            char = line[j]  # Lire le caractère à la position (i, j)
            if char == '.'
                map[i, j] = 1  # Terrain normal, coût de déplacement de 1
            elseif char == 'T'
                map[i, j] = 1   # Terrain passable, coût de déplacement de 1
            elseif char == 'S'
                map[i, j] = 5   # Terrain spécial, coût de déplacement de 5
            elseif char == 'W'
                map[i, j] = 8   # Terrain difficile, coût de déplacement de 8
            elseif char == '@'
                map[i, j] = Inf  # Terrain infranchissable, coût infini
            end
        end
    end

    return map  # Retourner la carte générée
end

# Fonction pour vérifier si une position est valide sur la carte
function isValid(map, pos)
    x, y = pos  # Extraire les coordonnées x et y
    # Vérifier si la position est dans les limites de la carte et si elle est franchissable
    return x >= 1 && x <= size(map, 1) && y >= 1 && y <= size(map, 2) && map[x, y] != Inf
end

# Fonction pour effectuer une recherche en largeur (BFS)
function bfs(map, start, goal)
    # Vérifier si le point de départ ou d'arrivée est infranchissable
    if !isValid(map, start) || !isValid(map, goal)
        println("Le point de départ ou d'arrivée est infranchissable.")
        return [], -1, 0  # Retourner un chemin vide, un coût de -1 et 0 état évalué
    end

    # Initialiser la file d'attente pour stocker les positions et le coût du chemin courant
    queue = Queue{Tuple{Tuple{Int, Int}, Int}}()
    enqueue!(queue, (start, 0))  # Ajouter le point de départ avec un coût de 0

    # Initialiser un ensemble pour garder une trace des positions visitées
    visited = Set{Tuple{Int, Int}}([start])

    # Directions possibles : haut, bas, gauche, droite
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Dictionnaire pour stocker le chemin (prédécesseur de chaque position)
    path = Dict{Tuple{Int, Int}, Tuple{Int, Int}}(start => start)

    # Compteur pour le nombre d'états évalués
    states_evaluated = 0

    # Boucle principale de l'algorithme BFS
    while !isempty(queue)
        # Extraire la position actuelle et le coût accumulé
        current, cost = dequeue!(queue)
        states_evaluated += 1  # Incrémenter le compteur d'états évalués

        # Si l'objectif est atteint
        if current == goal
            # Reconstruire le chemin en remontant les prédécesseurs
            path_list = []
            current_pos = goal
            while current_pos != start
                push!(path_list, current_pos)  # Ajouter la position actuelle au chemin
                current_pos = path[current_pos]  # Remonter au prédécesseur
            end
            push!(path_list, start)  # Ajouter le point de départ
            reverse!(path_list)  # Inverser pour obtenir le chemin du début à la fin
            return path_list, cost, states_evaluated  # Retourner le chemin, le coût et le nombre d'états évalués
        end

        # Explorer les voisins (haut, bas, gauche, droite)
        for (dx, dy) in directions
            next_pos = (current[1] + dx, current[2] + dy)  # Calculer la nouvelle position

            # Vérifier si la nouvelle position est valide et non visitée
            if isValid(map, next_pos) && !(next_pos in visited)
                push!(visited, next_pos)  # Marquer comme visité
                path[next_pos] = current  # Enregistrer le prédécesseur pour la reconstruction du chemin
                next_cost = cost + 1  # BFS utilise un coût uniforme de 1 par pas
                enqueue!(queue, (next_pos, next_cost))  # Ajouter à la file d'attente
            end
        end
    end

    # Si la file d'attente est vide et l'objectif n'est pas atteint
    println("Aucun chemin trouvé.")
    return [], -1, states_evaluated  # Retourner un chemin vide, un coût de -1 et le nombre d'états évalués
end

# Fonction pour effectuer l'algorithme de Dijkstra
function dijkstra(map, start, goal)
    # Vérifier si le point de départ ou d'arrivée est infranchissable
    if !isValid(map, start) || !isValid(map, goal)
        return [], -1, 0  # Retourner un chemin vide, un coût de -1 et 0 état évalué
    end

    # Initialiser une file de priorité pour stocker les positions et leurs coûts
    pq = PriorityQueue{Tuple{Int, Int}, Float64}()
    enqueue!(pq, start, 0.0)  # Ajouter le point de départ avec un coût de 0

    # Initialiser un dictionnaire pour stocker les distances minimales
    distances = Dict{Tuple{Int, Int}, Float64}((i, j) => Inf for i in 1:size(map, 1), j in 1:size(map, 2))
    distances[start] = 0.0  # La distance du point de départ à lui-même est 0

    # Dictionnaire pour stocker les prédécesseurs (reconstruction du chemin)
    predecessors = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()

    # Directions possibles : haut, bas, gauche, droite
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Compteur pour le nombre d'états évalués
    states_evaluated = 0

    # Boucle principale de l'algorithme de Dijkstra
    while !isempty(pq)
        # Extraire la position actuelle et sa distance minimale
        current, current_dist = dequeue_pair!(pq)
        states_evaluated += 1  # Incrémenter le compteur d'états évalués

        # Si l'objectif est atteint
        if current == goal
            # Reconstruire le chemin en remontant les prédécesseurs
            path = []
            while current != start
                push!(path, current)  # Ajouter la position actuelle au chemin
                current = predecessors[current]  # Remonter au prédécesseur
            end
            push!(path, start)  # Ajouter le point de départ
            reverse!(path)  # Inverser pour obtenir le chemin du début à la fin
            return path, distances[goal], states_evaluated  # Retourner le chemin, la distance et le nombre d'états évalués
        end

        # Explorer les voisins (haut, bas, gauche, droite)
        for (dx, dy) in directions
            next_pos = (current[1] + dx, current[2] + dy)  # Calculer la nouvelle position

            # Vérifier si la nouvelle position est valide
            if isValid(map, next_pos)
                # Calculer la nouvelle distance
                new_dist = distances[current] + map[next_pos...]

                # Si la nouvelle distance est meilleure que celle enregistrée
                if new_dist < distances[next_pos]
                    distances[next_pos] = new_dist  # Mettre à jour la distance
                    predecessors[next_pos] = current  # Enregistrer le prédécesseur
                    enqueue!(pq, next_pos, new_dist)  # Ajouter à la file de priorité
                end
            end
        end
    end

    # Si la file de priorité est vide et l'objectif n'est pas atteint
    return [], -1, states_evaluated  # Retourner un chemin vide, un coût de -1 et le nombre d'états évalués
end

# Fonction pour effectuer l'algorithme Greedy Best-First Search (version optimisée)
function greedy_best_first(map, start, goal)
    # Vérification des positions de départ et d'arrivée
    if !isValid(map, start) || !isValid(map, goal)
        println("Le point de départ ou d'arrivée est infranchissable.")
        return [], -1, 0  # Retourne un chemin vide avec un coût de -1 et 0 état évalué
    end

    # Définition de l'heuristique (distance de Manhattan)
    heuristic(a, b) = abs(a[1] - b[1]) + abs(a[2] - b[2])

    # Initialisation de la file de priorité avec le point de départ
    pq = PriorityQueue{Tuple{Int, Int}, Float64}()
    enqueue!(pq, start, heuristic(start, goal))

    # Dictionnaire pour stocker les prédécesseurs (reconstruction du chemin)
    predecessors = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()

    # Directions possibles : haut, bas, gauche, droite
    directions = [(-1,0), (1,0), (0,-1), (0,1)]

    # Compteur d'états évalués et ensemble des nœuds visités
    states_evaluated = 0
    visited = Set{Tuple{Int, Int}}()  # Pour éviter les ré-explorations

    # Boucle principale de l'algorithme
    while !isempty(pq)
        # Extraction du nœud courant (celui avec la meilleure heuristique)
        current = dequeue!(pq)

        # Si le nœud a déjà été visité, on passe au suivant
        if current in visited
            continue
        end

        # Marquer le nœud comme visité et incrémenter le compteur
        push!(visited, current)
        states_evaluated += 1

        # Si on a atteint le but
        if current == goal
            # Reconstruction du chemin en remontant les prédécesseurs
            path = []
            while current != start
                push!(path, current)
                current = predecessors[current]
            end
            push!(path, start)
            reverse!(path)  # Inversion pour avoir le chemin dans l'ordre
            return path, length(path)-1, states_evaluated
        end

        # Exploration des voisins
        for (dx, dy) in directions
            next_pos = (current[1]+dx, current[2]+dy)

            # Vérification de la validité et non-visite du voisin
            if isValid(map, next_pos) && !(next_pos in visited) && !haskey(predecessors, next_pos)
                predecessors[next_pos] = current  # Enregistrement du prédécesseur
                enqueue!(pq, next_pos, heuristic(next_pos, goal))  # Ajout à la file
            end
        end
    end

    # Si aucun chemin n'a été trouvé
    println("Aucun chemin trouvé.")
    return [], -1, states_evaluated
end


# Fonction pour effectuer l'algorithme A*
function astar(map, start, goal)
    # Vérifier si le point de départ ou d'arrivée est infranchissable
    if !isValid(map, start) || !isValid(map, goal)
        println("Le point de départ ou d'arrivée est infranchissable.")
        return [], -1, 0  # Retourner un chemin vide, un coût de -1 et 0 état évalué
    end

    # Heuristique : distance de Manhattan (estimation du coût restant)
    function heuristic(a, b)
        return abs(a[1] - b[1]) + abs(a[2] - b[2])  # |x1 - x2| + |y1 - y2|
    end

    # File de priorité pour stocker les nœuds à explorer, triés par f_cost (g_cost + h_cost)
    pq = PriorityQueue{Tuple{Int, Int}, Float64}()
    enqueue!(pq, start, 0.0)  # Ajouter le point de départ avec un f_cost initial de 0

    # Dictionnaire pour stocker le coût réel (g_cost) pour atteindre chaque case
    g_cost = Dict{Tuple{Int, Int}, Float64}(start => 0.0)

    # Dictionnaire pour stocker le coût total (f_cost = g_cost + h_cost) pour chaque case
    f_cost = Dict{Tuple{Int, Int}, Float64}(start => heuristic(start, goal))

    # Dictionnaire pour stocker les prédécesseurs (reconstruction du chemin)
    predecessors = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()

    # Directions possibles : haut, bas, gauche, droite
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Compteur pour le nombre d'états évalués
    states_evaluated = 0

    # Boucle principale de l'algorithme A*
    while !isempty(pq)
        # Extraire la case avec le f_cost le plus bas
        current = dequeue!(pq)
        states_evaluated += 1  # Incrémenter le compteur d'états évalués

        # Si l'objectif est atteint
        if current == goal
            # Reconstruire le chemin en remontant les prédécesseurs
            path = []
            while current != start
                push!(path, current)  # Ajouter la position actuelle au chemin
                current = predecessors[current]  # Remonter au prédécesseur
            end
            push!(path, start)  # Ajouter le point de départ
            reverse!(path)  # Inverser pour obtenir le chemin du début à la fin
            return path, g_cost[goal], states_evaluated  # Retourner le chemin, le coût et le nombre d'états évalués
        end

        # Explorer les voisins (haut, bas, gauche, droite)
        for (dx, dy) in directions
            next_pos = (current[1] + dx, current[2] + dy)  # Calculer la nouvelle position

            # Vérifier si la nouvelle position est valide
            if isValid(map, next_pos)
                # Calculer le nouveau coût réel (g_cost) pour atteindre next_pos
                tentative_g_cost = g_cost[current] + map[next_pos...]

                # Si le nouveau g_cost est meilleur que celui enregistré
                if !haskey(g_cost, next_pos) || tentative_g_cost < g_cost[next_pos]
                    # Mettre à jour le g_cost
                    g_cost[next_pos] = tentative_g_cost

                    # Calculer le f_cost (g_cost + heuristique)
                    f_cost[next_pos] = tentative_g_cost + heuristic(next_pos, goal)

                    # Enregistrer le prédécesseur pour la reconstruction du chemin
                    predecessors[next_pos] = current

                    # Mettre à jour la file de priorité
                    if next_pos in keys(pq)
                        # Si le nœud est déjà dans la file, mettre à jour sa priorité
                        pq[next_pos] = f_cost[next_pos]
                    else
                        # Sinon, l'ajouter à la file
                        enqueue!(pq, next_pos, f_cost[next_pos])
                    end
                end
            end
        end
    end

    # Si la file de priorité est vide et l'objectif n'est pas atteint
    println("Aucun chemin trouvé.")
    return [], -1, states_evaluated  # Retourner un chemin vide, un coût de -1 et le nombre d'états évalués
end

# ------------------ Algorithmes WA* (Weighted A*) ------------------

# Variante STANDARD de WA* (ω >=1)
function wa_standard(map, start, goal, ω)
    # Initialisation des structures de données
    ouverts = PriorityQueue{Tuple{Int, Int}, Float64}()  # File de priorité
    enqueue!(ouverts, start, 0.0)  # Ajout du point de départ
    g = Dict(start => 0.0)  # Coûts réels depuis le départ
    pred = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()  # Prédécesseurs
    n = 0  # Compteur d'états évalués

    # Boucle principale
    while !isempty(ouverts)
        courant = dequeue!(ouverts)  # Extraction du nœud courant
        n += 1  # Incrémentation du compteur

        # Si objectif atteint
        if courant == goal
            # Reconstruction du chemin
            chemin = [courant]
            while courant in keys(pred)
                courant = pred[courant]
                push!(chemin, courant)
            end
            return reverse(chemin), g[goal], n
        end

        # Exploration des voisins
        for (dx, dy) in [(-1,0), (1,0), (0,-1), (0,1)]
            voisin = (courant[1]+dx, courant[2]+dy)
            
            # Vérification de la validité
            if isValid(map, voisin)
                tentative_g = g[courant] + map[voisin...]  # Coût réel
                
                # Si meilleur chemin trouvé
                if tentative_g < get(g, voisin, Inf)
                    g[voisin] = tentative_g  # Mise à jour du coût
                    pred[voisin] = courant  # Mise à jour du prédécesseur
                    f = tentative_g + ω * heuristique(voisin, goal)  # Calcul de f = g + ω*h
                    ouverts[voisin] = f  # Ajout/mise à jour dans la file
                end
            end
        end
    end
    
    # Si aucun chemin trouvé
    println("Aucun chemin trouvé.")
    return [], -1, n
end

# Variante WA* avec ω ∈ [0, 1]
function wa_weighted01(map, start, goal, ω)
    # Initialisation similaire à wa_standard
    ouverts = PriorityQueue{Tuple{Int, Int}, Float64}()
    enqueue!(ouverts, start, 0.0)
    g = Dict(start => 0.0)
    pred = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()
    n = 0

    while !isempty(ouverts)
        courant = dequeue!(ouverts)
        n += 1

        if courant == goal
            # Reconstruction du chemin
            chemin = [courant]
            while courant in keys(pred)
                courant = pred[courant]
                push!(chemin, courant)
            end
            return reverse(chemin), g[goal], n
        end

        for (dx, dy) in [(-1,0), (1,0), (0,-1), (0,1)]
            voisin = (courant[1]+dx, courant[2]+dy)
            
            if isValid(map, voisin)
                tentative_g = g[courant] + map[voisin...]
                
                if tentative_g < get(g, voisin, Inf)
                    g[voisin] = tentative_g
                    pred[voisin] = courant
                    h = heuristique(voisin, goal)
                    # Formule alternative: f = ω*g + (1-ω)*h
                    f = ω * tentative_g + (1 - ω) * h
                    ouverts[voisin] = f
                end
            end
        end
    end
    
    println("Aucun chemin trouvé.")
    return [], -1, n
end

# Variante WA* avec ω dynamique (décroissant)
function wa_dynamique(map, start, goal, ω_initial)
    # Initialisation similaire
    ouverts = PriorityQueue{Tuple{Int, Int}, Float64}()
    enqueue!(ouverts, start, 0.0)
    g = Dict(start => 0.0)
    pred = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()
    n = 0

    while !isempty(ouverts)
        courant = dequeue!(ouverts)
        n += 1

        if courant == goal
            # Reconstruction du chemin
            chemin = [courant]
            while courant in keys(pred)
                courant = pred[courant]
                push!(chemin, courant)
            end
            return reverse(chemin), g[goal], n
        end

        for (dx, dy) in [(-1,0), (1,0), (0,-1), (0,1)]
            voisin = (courant[1]+dx, courant[2]+dy)
            
            if isValid(map, voisin)
                tentative_g = g[courant] + map[voisin...]
                
                if tentative_g < get(g, voisin, Inf)
                    g[voisin] = tentative_g
                    pred[voisin] = courant
                    # ω dynamique qui diminue avec le nombre d'états évalués
                    ω_dyn = max(1.0, ω_initial - 0.001 * n)
                    f = tentative_g + ω_dyn * heuristique(voisin, goal)
                    ouverts[voisin] = f
                end
            end
        end
    end
    
    println("Aucun chemin trouvé.")
    return [], -1, n
end


# Fonction pour exécuter l'algorithme BFS et afficher les résultats
function algoBFS(fname, D, A)
    map = loadMap(fname)

    # Exécuter une seule fois pour récupérer les vrais résultats
    path, cost, states_evaluated = bfs(map, D, A)

    # Mesurer le temps moyen avec @btime (sans modifier les résultats)
    cpu_time = @belapsed bfs($map, $D, $A)

    if cost != -1
        println("\nSolution BFS :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)
        println("Distance D → A : ", cost)
        println("Nombre d'états évalués : ", states_evaluated)
        println("Chemin D → A : ", join(["($x, $y)" for (x, y) in path], " → "))
    else
        println("Aucun chemin trouvé.")
    end
end

# Fonction pour exécuter l'algorithme de Dijkstra et afficher les résultats
function algoDijkstra(fname, D, A)
    map = loadMap(fname)

    path, distance, states_evaluated = dijkstra(map, D, A)
    cpu_time = @belapsed dijkstra($map, $D, $A)

    if distance != -1
        println("\nSolution Dijkstr :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)
        println("Distance D → A : ", distance)
        println("Nombre d'états évalués : ", states_evaluated)
        println("Chemin D → A : ", join(["($x, $y)" for (x, y) in path], " → "))
    else
        println("Aucun chemin trouvé.")
    end
end

# Fonction pour exécuter l'algorithme Greedy Best-First et afficher les résultats
function algoGlouton(fname, D, A)
    map = loadMap(fname)

    path, cost, states_evaluated = greedy_best_first(map, D, A)
    cpu_time = @belapsed greedy_best_first($map, $D, $A)

    if cost != -1
        println("\nSolution Glouton :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)
        println("Distance D → A : ", cost)
        println("Nombre d'états évalués : ", states_evaluated)
        println("Chemin D → A : ", join(["($x, $y)" for (x, y) in path], " → "))
    else
        println("Aucun chemin trouvé.")
    end
end

# Fonction pour exécuter l'algorithme A* et afficher les résultats
function algoAstar(fname, D, A)
    map = loadMap(fname)

    path, cost, states_evaluated = astar(map, D, A)
    cpu_time = @belapsed astar($map, $D, $A)

    if cost != -1
        println("\nSolution A_étoile :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)
        println("Distance D → A : ", cost)
        println("Nombre d'états évalués : ", states_evaluated)
       # println("Chemin D → A : ", join(["($x, $y)" for (x, y) in path], " → "))
    else
        println("Aucun chemin trouvé.")
    end
end



# --- WA* Variante STANDARD (ω ≥ 1)
function algoWAstarStandard(fname, D, A; ω=1.5)
    map = loadMap(fname)
    path, cost, nb_etats = WA_etoile(map, D, A; ω=ω, mode=:standard)
    cpu_time = @belapsed WA_etoile($map, $D, $A; ω=$ω, mode=:standard)

    if cost != -1
        println("\n Résultat WA* STANDARD (ω = $ω) :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)
        println("Distance D → A : ", cost)
        println("États évalués : ", nb_etats)
        println("Chemin : ", join(["($x,$y)" for (x,y) in path], " → "))
    else
        println("Aucun chemin trouvé.")
    end
end

# --- WA* Variante ω ∈ [0, 1]
function algoWAstarWeighted01(fname, D, A; ω=0.8)
    map = loadMap(fname)
    path, cost, nb_etats = WA_etoile(map, D, A; ω=ω, mode=:weighted01)
    cpu_time = @belapsed WA_etoile($map, $D, $A; ω=$ω, mode=:weighted01)

    if cost != -1
        println("\n Résultat WA* (0 ≤ ω ≤ 1) (ω = $ω) :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)
        println("Distance D → A : ", cost)
        println("États évalués : ", nb_etats)
        println("Chemin : ", join(["($x,$y)" for (x,y) in path], " → "))
    else
        println("Aucun chemin trouvé.")
    end
end

# --- WA* Variante DYNAMIQUE
function algoWAstarDynamique(fname, D, A; ω=2.5)
    map = loadMap(fname)
    path, cost, nb_etats = WA_etoile(map, D, A; ω=ω, mode=:dynamic)
    cpu_time = @belapsed WA_etoile($map, $D, $A; ω=$ω, mode=:dynamic)

    if cost != -1
        println("\n Résultat WA* DYNAMIQUE (ω initial = $ω) :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)
        println("Distance D → A : ", cost)
        println("États évalués : ", nb_etats)
        println("Chemin : ", join(["($x,$y)" for (x,y) in path], " → "))
    else
        println("Aucun chemin trouvé.")
    end
end


