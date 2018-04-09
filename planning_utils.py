from enum import Enum
from queue import PriorityQueue
from scipy.spatial import KDTree
from shapely.geometry import Point, Polygon
import networkx as nx
import numpy as np
import time


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


def heuristic(position, goal_position):
    """
    Calculates heuristics of the distance from current position to the goal position.
    :param position: current position
    :param goal_position: goal position
    :return: heuristics distance, here we use euclidean distance.
    """
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def find_path_using_grid(grid, grid_start, grid_goal):
    """
    Find path using A* in the grid directly. This algorithm is simpler to implement,
    however slower to run and the number of way-points can be large.
    """
    class Action(Enum):
        """
        An action is represented by a 3 element tuple.

        The first 2 values are the delta of the action relative
        to the current grid position. The third and final value
        is the cost of performing the action.
        """

        WEST = (0, -1, 1)
        EAST = (0, 1, 1)
        NORTH = (-1, 0, 1)
        SOUTH = (1, 0, 1)
        WESTNORTH = (-1, -1, np.sqrt(2))
        WESTSOUTH = (1, -1, np.sqrt(2))
        EASTNORTH = (-1, 1, np.sqrt(2))
        EASTSOUTH = (1, 1, np.sqrt(2))

        @property
        def cost(self):
            return self.value[2]

        @property
        def delta(self):
            return (self.value[0], self.value[1])

    def valid_actions(grid, current_node):
        """
        Returns a list of valid actions given a grid and current node.
        Diagonal action has the cost of sqrt(2).
        """
        valid_actions = list(Action)
        n, m = grid.shape[0] - 1, grid.shape[1] - 1
        x, y = current_node

        # check if the node is off the grid or
        # it's an obstacle

        if x - 1 < 0 or grid[x - 1, y] == 1:
            valid_actions.remove(Action.NORTH)
        if x + 1 > n or grid[x + 1, y] == 1:
            valid_actions.remove(Action.SOUTH)
        if y - 1 < 0 or grid[x, y - 1] == 1:
            valid_actions.remove(Action.WEST)
        if y + 1 > m or grid[x, y + 1] == 1:
            valid_actions.remove(Action.EAST)
        if x - 1 < 0 or y - 1 < 0 or grid[x - 1][y - 1] == 1:
            valid_actions.remove(Action.WESTNORTH)
        if x + 1 > n or y - 1 < 0 or grid[x + 1][y - 1] == 1:
            valid_actions.remove(Action.WESTSOUTH)
        if x - 1 < 0 or y + 1 > m or grid[x - 1][y + 1] == 1:
            valid_actions.remove(Action.EASTNORTH)
        if x + 1 > n or y + 1 > m or grid[x + 1][y + 1] == 1:
            valid_actions.remove(Action.EASTSOUTH)

        return valid_actions

    def a_star(grid, h, start, goal):
        """
        A* algorithm on the grid.
        """
        path = []
        path_cost = 0
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)

        branch = {}
        found = False

        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            if current_node == start:
                current_cost = 0.0
            else:
                current_cost = branch[current_node][0]

            if current_node == goal:
                print('Found a path.')
                found = True
                break
            else:
                for action in valid_actions(grid, current_node):
                    # get the tuple representation
                    da = action.delta
                    next_node = (current_node[0] + da[0], current_node[1] + da[1])
                    branch_cost = current_cost + action.cost
                    queue_cost = branch_cost + h(next_node, goal)

                    if next_node not in visited:
                        visited.add(next_node)
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))

        if found:
            # retrace steps
            n = goal
            path_cost = branch[n][0]
            path.append(goal)
            while branch[n][1] != start:
                path.append(branch[n][1])
                n = branch[n][1]
            path.append(branch[n][1])
        else:
            print('**********************')
            print('Failed to find a path!')
            print('**********************')
        return path[::-1], path_cost

    def check_collinear(a, b, c):
        """
        Check whether three points a, b, c collinear. Returns True if they do.
        """
        EPSILON = 1e-6
        m = [a + (1,), b + (1,), c + (1,)]
        d = np.linalg.det(m)
        return np.abs(d) < EPSILON

    def prune_path(path):
        """
        Prune path by merging consecutive collinear way-points.
        """
        first = None
        second = None
        new_path = []
        for p in path:
            if first is None:
                first = p
            elif second is None:
                second = p
            elif check_collinear(first, second, p):
                second = p
            else:
                new_path.append(first)
                first = second
                second = p
        if first is not None:
            new_path.append(first)
        if second is not None:
            new_path.append(second)
        return new_path

    start_time = time.time()

    # Run A* to find a path from start to goal
    path, path_cost = a_star(grid, heuristic, grid_start, grid_goal)

    # prune path to minimize number of waypoints
    path = prune_path(path)

    print("Number of waypoints: {0}. Path cost: {1}".format(len(path), path_cost))

    print("Time used: ", time.time() - start_time)

    return path


def find_path_using_graph(grid, grid_start, grid_goal):
    SAMPLE_COUNT = 1000
    GRAPH_DEGREE = 15

    def sample_states(grid, sample_count):
        """
        Return a random list of points in the grid that are not occupied.
        """
        # Get indices of all zero elements.
        nz = np.transpose(np.nonzero(1 - grid))
        # Randomly pick sample_count of them.
        nz_samples = np.random.choice(len(nz), sample_count)
        # Make sure each element in states is a tuple (networkx.Graph requires it,
        # as numpy.ndarray cannot be hashed).
        # Also make sure each number is int, instead of numpy.int64 (msgpack requires it,
        # as numpy.int64 cannot be serialized).
        return [(int(s[0]), int(s[1])) for s in nz[nz_samples]]

    def check_collide(grid, a, b):
        """
        Check whether the line connecting two points a and b touches any occupied cell in the grid.
        This algorithm uses only integer computations, hence efficient.
        Returns True if it does.
        """
        if a[0] > b[0]:
            # Makes sure a is always on the left of b.
            b, a = a, b

        dx = b[0] - a[0]
        dy = b[1] - a[1]
        x = a[0]
        y = a[1]
        if a[1] <= b[1]:
            # The line goes up to the right.
            d = dy - dx
            while x <= b[0] and y <= b[1]:
                if grid[x][y] == 1:
                    return True
                if d >= 0:
                    y += 1
                    d -= 2 * dx
                if d < 0:
                    d += 2 * dy
                    x += 1
        else:
            # The line goes down to the right.
            d = dy + dx
            while x <= b[0] and y >= b[1]:
                if grid[x][y] == 1:
                    return True
                if d <= 0:
                    y -= 1
                    d += 2 * dx
                if d > 0:
                    d += 2 * dy
                    x += 1
        return False

    def a_star(graph, h, start, goal):
        """
        Runs A* on the graph.
        """
        path = []
        path_cost = 0
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)

        branch = {}
        found = False

        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            if current_node == start:
                current_cost = 0.0
            else:
                current_cost = branch[current_node][0]

            if current_node == goal:
                print('Found a path.')
                found = True
                break
            else:
                for next_node in graph.neighbors(current_node):
                    branch_cost = current_cost + np.linalg.norm(np.array(current_node) - np.array(next_node))
                    queue_cost = branch_cost + h(next_node, goal)

                    if next_node not in visited:
                        visited.add(next_node)
                        branch[next_node] = (branch_cost, current_node)
                        queue.put((queue_cost, next_node))

        if found:
            # retrace steps
            n = goal
            path_cost = branch[n][0]
            path.append(goal)
            while branch[n][1] != start:
                path.append(branch[n][1])
                n = branch[n][1]
            path.append(branch[n][1])
        else:
            print('**********************')
            print('Failed to find a path!')
            print('**********************')
        return path[::-1], path_cost

    start_time = time.time()

    states = sample_states(grid, SAMPLE_COUNT)

    # Include start and goal in the list of stages.
    if grid_start not in states:
        states += [grid_start]
    if grid_goal not in states:
        states += [grid_goal]

    # Randomly connect each node with its closest neighbors to construct a graph.
    # Uses KDTree for fast neighbor query.
    tree = KDTree(states)
    graph = nx.Graph()
    for s in states:
        graph.add_node(s)
    for s in states:
        for n in tree.query(s, GRAPH_DEGREE)[1]:
            if not check_collide(grid, s, states[n]):
                graph.add_edge(s, states[n])

    path, path_cost = a_star(graph, heuristic, grid_start, grid_goal)
    print("Number of waypoints: {0}. Path cost: {1}".format(len(path), path_cost))

    print("Time used: ", time.time() - start_time)

    return path