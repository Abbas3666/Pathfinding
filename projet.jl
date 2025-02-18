using DataStructures
using Printf  # Pour formater l'affichage des nombres

function loadMap(fname)
    lines = readlines(fname)
    lines = filter(line -> !startswith(line, "#"), lines)  # Filtrer les commentaires

    # Trouver les indices des lignes contenant la hauteur, la largeur et le début de la carte
    height_line_index = findfirst(line -> occursin("height", line), lines)
    width_line_index = findfirst(line -> occursin("width", line), lines)
    map_line_index = findfirst(line -> occursin("map", line), lines)

    if height_line_index === nothing || width_line_index === nothing || map_line_index === nothing
        error("Ligne manquante pour la hauteur, la largeur ou le début de la carte dans le fichier.")
    end

    height = parse(Int, split(lines[height_line_index])[2])
    width = parse(Int, split(lines[width_line_index])[2])

    # Initialiser la carte avec des valeurs par défaut pour les cellules infranchissables
    map = fill(Inf, height, width)  # Toutes les cellules sont infranchissables par défaut
    start_line = map_line_index + 1

    for i in 1:height
        line = lines[start_line + i - 1]
        for j in 1:width
            char = line[j]
            if char == '.'
                map[i, j] = 1  # Terrain passable
            elseif char == 'T'
                map[i, j] = 2  # Terrain difficile, coût plus élevé
            elseif char == '@'
                map[i, j] = Inf  # Terrain infranchissable
            end
        end
    end

    return map
end



function isValid(map, pos)
    x, y = pos
    return x >= 1 && x <= size(map, 1) && y >= 1 && y <= size(map, 2) && map[x, y] != Inf
end
function bfs(map, start, goal)
    if !isValid(map, start) || !isValid(map, goal)
        println("Le point de départ ou d'arrivée est infranchissable.")
        return [], -1, 0
    end

    queue = Queue{Tuple{Tuple{Int, Int}, Int}}()  # Stocke les positions et le coût du chemin courant
    enqueue!(queue, (start, 0))  # Commence avec la position de départ et un coût de 0
    visited = Set{Tuple{Int, Int}}([start])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Directions de mouvement
    path = Dict{Tuple{Int, Int}, Tuple{Int, Int}}(start => start)  # Pour reconstruire le chemin plus tard
    states_evaluated = 0

    while !isempty(queue)
        current, cost = dequeue!(queue)
        states_evaluated += 1

        # Quand l'objectif est atteint
        if current == goal
            # Reconstruire le chemin
            return reconstruire_chemin(path, start, goal), cost, states_evaluated
        end

        for (dx, dy) in directions
            next_pos = (current[1] + dx, current[2] + dy)
            if isValid(map, next_pos) && !(next_pos in visited)
                push!(visited, next_pos)  # Marquer comme visité
                path[next_pos] = current  # Définir le prédécesseur pour la reconstruction du chemin
                next_cost = cost + map[next_pos...]
                enqueue!(queue, (next_pos, next_cost))
            end
        end
    end

    println("Aucun chemin trouvé.")
    return [], -1, states_evaluated
end


function dijkstra(map, start, goal)
    if !isValid(map, start) || !isValid(map, goal)
        return [], -1, 0
    end

    pq = PriorityQueue{Tuple{Int, Int}, Float64}()
    enqueue!(pq, start, 0.0)
    distances = Dict{Tuple{Int, Int}, Float64}((i, j) => Inf for i in 1:size(map, 1), j in 1:size(map, 2))
    distances[start] = 0.0
    predecessors = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    states_evaluated = 0

    while !isempty(pq)
        current, current_dist = dequeue_pair!(pq)
        states_evaluated += 1

        if current == goal
            path = []
            while current != start
                push!(path, current)
                current = predecessors[current]
            end
            push!(path, start)
            reverse!(path)
            return path, distances[goal], states_evaluated
        end

        for (dx, dy) in directions
            next_pos = (current[1] + dx, current[2] + dy)
            if isValid(map, next_pos)
                new_dist = distances[current] + map[next_pos...]
                if new_dist < distances[next_pos]
                    distances[next_pos] = new_dist
                    predecessors[next_pos] = current
                    enqueue!(pq, next_pos, new_dist)
                end
            end
        end
    end

    return [], -1, states_evaluated
end




function greedy_best_first(map, start, goal)
    if !isValid(map, start) || !isValid(map, goal)
        println("Le point de départ ou d'arrivée est infranchissable.")
        return [], -1, 0
    end

    # Heuristique : distance de Manhattan
    function heuristic(a, b)
        return abs(a[1] - b[1]) + abs(a[2] - b[2])
    end

    # File de priorité pour stocker les nœuds à explorer
    pq = PriorityQueue{Tuple{Int, Int}, Float64}()
    enqueue!(pq, start, heuristic(start, goal))

    # Dictionnaires pour stocker les prédécesseurs
    predecessors = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    states_evaluated = 0

    while !isempty(pq)
        current = dequeue!(pq)
        states_evaluated += 1

        if current == goal
            # Reconstruire le chemin
            path = []
            while current != start
                push!(path, current)
                current = predecessors[current]
            end
            push!(path, start)
            reverse!(path)
            return path, length(path) - 1, states_evaluated  # Coût = nombre de pas
        end

        for (dx, dy) in directions
            next_pos = (current[1] + dx, current[2] + dy)
            if isValid(map, next_pos) && !haskey(predecessors, next_pos)
                predecessors[next_pos] = current
                enqueue!(pq, next_pos, heuristic(next_pos, goal))
            end
        end
    end

    println("Aucun chemin trouvé.")
    return [], -1, states_evaluated
end




function astar(map, start, goal)
    if !isValid(map, start) || !isValid(map, goal)
        println("Le point de départ ou d'arrivée est infranchissable.")
        return [], -1, 0
    end

    # Heuristique : distance de Manhattan
    function heuristic(a, b)
        return abs(a[1] - b[1]) + abs(a[2] - b[2])
    end

    # File de priorité pour stocker les nœuds à explorer
    pq = PriorityQueue{Tuple{Int, Int}, Float64}()
    enqueue!(pq, start, 0.0)

    # Dictionnaires pour stocker les coûts et les prédécesseurs
    g_cost = Dict{Tuple{Int, Int}, Float64}(start => 0.0)
    f_cost = Dict{Tuple{Int, Int}, Float64}(start => heuristic(start, goal))
    predecessors = Dict{Tuple{Int, Int}, Tuple{Int, Int}}()
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    states_evaluated = 0

    while !isempty(pq)
        current = dequeue!(pq)
        states_evaluated += 1

        if current == goal
            # Reconstruire le chemin
            path = []
            while current != start
                push!(path, current)
                current = predecessors[current]
            end
            push!(path, start)
            reverse!(path)
            return path, g_cost[goal], states_evaluated
        end

        for (dx, dy) in directions
            next_pos = (current[1] + dx, current[2] + dy)
            if isValid(map, next_pos)
                tentative_g_cost = g_cost[current] + map[next_pos...]
                if !haskey(g_cost, next_pos) || tentative_g_cost < g_cost[next_pos]
                    g_cost[next_pos] = tentative_g_cost
                    f_cost[next_pos] = tentative_g_cost + heuristic(next_pos, goal)
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

    println("Aucun chemin trouvé.")
    return [], -1, states_evaluated
end






function algoBFS(fname, D, A)
    map = loadMap(fname)

    # Mesurer le temps d'exécution avec @elapsed
    cpu_time = @elapsed begin
        path, cost, states_evaluated = bfs(map, D, A)
    end

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

function algoDijkstra(fname, D, A)
    map = loadMap(fname)


    cpu_time = @elapsed begin
        path, distance, states_evaluated = dijkstra(map, D, A)
    end

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

function algoGlouton(fname, D, A)
    map = loadMap(fname)

   
    cpu_time = @elapsed begin
        path, cost, states_evaluated = greedy_best_first(map, D, A)
    end

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

function algoAstar(fname, D, A)
    map = loadMap(fname)

    cpu_time = @elapsed begin
        path, cost, states_evaluated = astar(map, D, A)
    end

    if cost != -1
        println("\nSolution A_étoile :")
        @printf("CPUtime (s) : %.1e\n", cpu_time)
        println("Distance D → A : ", cost)
        println("Nombre d'états évalués : ", states_evaluated)
        println("Chemin D → A : ", join(["($x, $y)" for (x, y) in path], " → "))
    else
        println("Aucun chemin trouvé.")
    end
end
