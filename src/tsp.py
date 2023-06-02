import numpy as np

def tsp_dp(distances, start_node):
    n = len(distances)
    all_points_set = set(range(n))

    # Initialize memoization table with base cases
    # https://shorturl.at/afCX1 <- memoization explained (stores computation and make algo more efficient)
    # For each city, the cost to visit it from itself is 0
    memo = {}
    # for i in range(n):
    visited_cities = tuple([start_node])
    last_city_visited = start_node
    total_cost_to_reach = 0
    previous_city = None
    memo[(visited_cities, last_city_visited)] = (total_cost_to_reach, previous_city)


    # Initialize queue for BFS with each city
    queue = []
    # for i in range(n):
    # visited_cities = tuple([i])
    # last_city_visited = i
    queue.append((visited_cities, last_city_visited))

    while queue:
        # Get current state from queue
        prev_visited, prev_last_point = queue.pop(0)
        prev_dist, _ = memo[(prev_visited, prev_last_point)]

        # Generate all possible states from current state
        to_visit = all_points_set.difference(set(prev_visited))
        for new_last_point in to_visit:
            new_visited = tuple(sorted(list(prev_visited) + [new_last_point]))
            new_dist = prev_dist + distances[prev_last_point][new_last_point]

            # If this state hasn't been visited before, add it to the queue and memo
            if (new_visited, new_last_point) not in memo:
                memo[(new_visited, new_last_point)] = (new_dist, prev_last_point)
                queue.append((new_visited, new_last_point))
            else:
                # If this state has been visited and the new cost is lower, update the memo
                if new_dist < memo[(new_visited, new_last_point)][0]:
                    memo[(new_visited, new_last_point)] = (new_dist, prev_last_point)
                    
    # Retrieve the path and cost of the optimal solution
    optimal_path, optimal_cost = retrace_optimal_path(memo, n)
    # Add the start to the end of the path, and the distance to the start to the cost
    optimal_path.append(optimal_path[0])
    optimal_cost += distances[optimal_path[-2]][optimal_path[0]]

    return optimal_path, optimal_cost


# Retrieve the path and cost of the optimal solution
def retrace_optimal_path(memo, n):
    points_to_retrace = tuple(range(n))

    # Create a new dictionary to store only those entries in the memoization table
    # that have the same visited cities as points_to_retrace
    full_path_memo = {}
    for key, value in memo.items():
        visited_cities, _ = key
        if visited_cities == points_to_retrace:
            full_path_memo[key] = value

    # Find the key corresponding to the minimum cost in full_path_memo.
    # The key is a tuple where the first element is the visited cities and the second element is the last visited city.
    min_cost = float('inf')
    path_key = None
    for key in full_path_memo.keys():
        cost, _ = full_path_memo[key]
        if cost < min_cost:
            min_cost = cost
            path_key = key
            
    # Start backtracking from the last point
    last_point = path_key[1]
    optimal_cost, next_to_last_point = memo[path_key]

    # Initialize optimal path with the last point
    optimal_path = [last_point]
    
    # Remove the last point from the points to retrace
    points_to_retrace = remove_visited_point(points_to_retrace, last_point)
    
    # Continue backtracking until we reach the start of the path
    while next_to_last_point is not None:
        last_point = next_to_last_point
        path_key = (points_to_retrace, last_point)
        _, next_to_last_point = memo[path_key]

        optimal_path.insert(0, last_point)
        # Remove the last point from the points to retrace
        points_to_retrace = remove_visited_point(points_to_retrace, last_point)

    return optimal_path, optimal_cost

def remove_visited_point(points_to_retrace, last_point):
    # Create a set from the points to retrace
    points_to_retrace_set = set(points_to_retrace)

    # Remove the last point from the set of points to retrace
    points_to_retrace_set = points_to_retrace_set.difference({last_point})

    # Convert the set back to a sorted tuple
    updated_points_to_retrace = tuple(sorted(points_to_retrace_set))

    return updated_points_to_retrace


def create_distance_matrix(deliveryList, find_path):
    n = len(deliveryList)  # Number of locations
    matrix = [[0]*n for _ in range(n)]  # Initialize n x n matrix with zeros

    for i in range(n):
        for j in range(i+1, n):  # Only calculate distances for the upper triangle of the matrix
            if i != j:  # Skip calculating distance from a location to itself
                # print(deliveryList[i], deliveryList[j])
                path = find_path(deliveryList[i], deliveryList[j])  # Get path from location i to location j
                distance = len(path)  # Calculate distance (assuming distance is the length of the path)
                matrix[i][j] = distance  # Store distance in matrix
                matrix[j][i] = distance  # Use symmetry to fill in the lower triangle

    return matrix