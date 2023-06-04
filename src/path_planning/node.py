class Node:
    """Represents each point in the OccupancyGrid--used in the A* algorithm implementation"""

    def __init__(self, state, heuristic, cost, parent=None):
        self.state = state
        self.heuristic = heuristic
        self.cost = cost
        self.parent = parent

    def priority(self):
        return self.heuristic + self.cost  # definition of priority in this case (from class slides)

    def __lt__(self, other):
        return self.priority() < other.priority()