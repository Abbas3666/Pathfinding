using DataStructures
using Printf  # Pour formater l'affichage des nombres

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
                map[i, j] = 1  
            elseif char == 'T'
                map[i, j] = 1   # Terrain passable
            elseif char == 'S'
                map[i, j] = 5
            elseif char == 'W'
                map[i, j] = 8    # Terrain difficile, coût plus élevé
            elseif char == '@'
                map[i, j] = Inf  # Terrain infranchissable
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
                next_cost = cost + 1  #  BFS utilise un coût uniforme de 1 par pas
                enqueue!(queue, (next_pos, next_cost))  # Ajouter à la file d'attente
            end
        end
    end

    # Si la file d'attente est vide et l'objectif n'est pas atteint
    println("Aucun chemin trouvé.")
    return [], -1, states_evaluated  # Retourner un chemin vide, un coût de -1 et le nombre d'états évalués
end
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

function Glouton(map, start, goal)
    # Vérifier si le point de départ ou d'arrivée est infranchissable
    if !isValid(map, start) || !isValid(map, goal)
        println("Le point de départ ou d'arrivée est infranchissable.")
        return [], -1, 0  # Retourner un chemin vide, un coût de -1 et 0 état évalué
    end

    # Heuristique : distance de Manhattan (estimation du coût restant)
    function heuristic(a, b)
        return abs(a[1] - b[1]) + abs(a[2] - b[2])  # |x1 - x2| + |y1 - y2|
    end

    # File de priorité pour stocker les nœuds à explorer, triés par heuristique
    pq = PriorityQueue{Tuple{Int, Int}, Float64}()
    enqueue!(pq, start, heuristic(start, goal))  # Ajouter le point de départ avec son heuristique

    # Dictionnaire pour stocker les prédécesseurs (reconstruction du chemin)
    predecessors = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()

    # Directions possibles : haut, bas, gauche, droite
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Compteur pour le nombre d'états évalués
    states_evaluated = 0

    # Boucle principale de l'algorithme Greedy Best-First
    while !isempty(pq)
        # Extraire la case avec l'heuristique la plus basse
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
            return path, length(path) - 1, states_evaluated  # Retourner le chemin, le nombre de pas et le nombre d'états évalués
        end

        # Explorer les voisins (haut, bas, gauche, droite)
        for (dx, dy) in directions
            next_pos = (current[1] + dx, current[2] + dy)  # Calculer la nouvelle position

            # Vérifier si la nouvelle position est valide et non visitée
            if isValid(map, next_pos) && !haskey(predecessors, next_pos)
                # Enregistrer le prédécesseur pour la reconstruction du chemin
                predecessors[next_pos] = current

                # Ajouter la nouvelle position à la file de priorité avec son heuristique
                enqueue!(pq, next_pos, heuristic(next_pos, goal))
            end
        end
    end

    # Si la file de priorité est vide et l'objectif n'est pas atteint
    println("Aucun chemin trouvé.")
    return [], -1, states_evaluated  # Retourner un chemin vide, un coût de -1 et le nombre d'états évalués
end


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






function algoBFS(fname, D, A)
    # Charger la carte à partir du fichier
    map = loadMap(fname)

    # Mesurer le temps d'exécution avec @elapsed
    cpu_time = @elapsed begin
        # Exécuter l'algorithme BFS
        path, cost, states_evaluated = bfs(map, D, A)
    end

    # Si un chemin a été trouvé (coût != -1)
    if cost != -1
        println("\nSolution BFS :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)  # Afficher le temps d'exécution
        println("Distance D → A : ", cost)  # Afficher la distance (coût total)
        println("Nombre d'états évalués : ", states_evaluated)  # Afficher le nombre d'états évalués
        println("Chemin D → A : ", join(["($x, $y)" for (x, y) in path], " → "))  # Afficher le chemin
    else
        println("Aucun chemin trouvé.")  # Afficher un message si aucun chemin n'est trouvé
    end
end

function algoDijkstra(fname, D, A)
    # Charger la carte à partir du fichier
    map = loadMap(fname)

    # Mesurer le temps d'exécution avec @elapsed
    cpu_time = @elapsed begin
        # Exécuter l'algorithme de Dijkstra
        path, distance, states_evaluated = dijkstra(map, D, A)
    end

    # Si un chemin a été trouvé (distance != -1)
    if distance != -1
        println("\nSolution Dijkstr :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)  # Afficher le temps d'exécution
        println("Distance D → A : ", distance)  # Afficher la distance (coût total)
        println("Nombre d'états évalués : ", states_evaluated)  # Afficher le nombre d'états évalués
        println("Chemin D → A : ", join(["($x, $y)" for (x, y) in path], " → "))  # Afficher le chemin
    else
        println("Aucun chemin trouvé.")  # Afficher un message si aucun chemin n'est trouvé
    end
end
function algoGlouton(fname, D, A)
    # Charger la carte à partir du fichier
    map = loadMap(fname)

    # Mesurer le temps d'exécution avec @elapsed
    cpu_time = @elapsed begin
        # Exécuter l'algorithme Greedy Best-First
        path, cost, states_evaluated = greedy_best_first(map, D, A)
    end

    # Si un chemin a été trouvé (coût != -1)
    if cost != -1
        println("\nSolution Glouton :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)  # Afficher le temps d'exécution
        println("Distance D → A : ", cost)  # Afficher la distance (nombre de pas)
        println("Nombre d'états évalués : ", states_evaluated)  # Afficher le nombre d'états évalués
        println("Chemin D → A : ", join(["($x, $y)" for (x, y) in path], " → "))  # Afficher le chemin
    else
        println("Aucun chemin trouvé.")  # Afficher un message si aucun chemin n'est trouvé
    end
end
function algoAstar(fname, D, A)
    # Charger la carte à partir du fichier
    map = loadMap(fname)

    # Mesurer le temps d'exécution avec @elapsed
    cpu_time = @elapsed begin
        # Exécuter l'algorithme A*
        path, cost, states_evaluated = astar(map, D, A)
    end

    # Si un chemin a été trouvé (coût != -1)
    if cost != -1
        println("\nSolution A_étoile :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)  # Afficher le temps d'exécution
        println("Distance D → A : ", cost)  # Afficher la distance (coût total)
        println("Nombre d'états évalués : ", states_evaluated)  # Afficher le nombre d'états évalués
        println("Chemin D → A : ", join(["($x, $y)" for (x, y) in path], " → "))  # Afficher le chemin
    else
        println("Aucun chemin trouvé.")  # Afficher un message si aucun chemin n'est trouvé
    end
end
