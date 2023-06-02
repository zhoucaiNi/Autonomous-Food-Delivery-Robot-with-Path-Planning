# Define the distance matrix
# For simplicity, the distances are randomly generated.
# In a real-world problem, you would replace these values with actual distances.
import numpy as np
from tsp import tsp_dp

distances = np.array([[0, 20, 42, 35, 25],
                      [20, 0, 30, 34, 50],
                      [42, 30, 0, 12, 40],
                      [35, 34, 12, 0, 15],
                      [25, 50, 40, 15, 0]])

# Solve the TSP
optimal_path, optimal_cost = tsp_dp(distances, 4)

# Print the results
print("The optimal path is {} with total cost {}".format(optimal_path, optimal_cost))

