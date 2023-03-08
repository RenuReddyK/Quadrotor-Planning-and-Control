from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

from collections import defaultdict


n_grid = np.stack(np.meshgrid([-1, 0, 1],[-1, 0, 1],[-1, 0, 1]), axis= -1).reshape(-1, 3)
n_grid = np.delete(n_grid, 13, axis=0)

def currentnode_neighbors(current_node_index,occ_map):
    neighbors = n_grid + current_node_index
 
    neighbors = neighbors[np.all(neighbors >= 0, axis=1)] 
    neighbors = neighbors[np.all(neighbors < occ_map.map.shape, axis=1)]
    X = neighbors[:,0]
    Y = neighbors[:,1]
    Z = neighbors[:,2]
    neighbors_ind = occ_map.map[X, Y, Z]
    neighbors = neighbors[np.where(neighbors_ind == False)]

    # for i in range(len(neighbors)):
    #     if occ_map.map[neighbors[i,0],neighbors[i,1],neighbors[i,2]] == True:
    #         np.delete(neighbors, i)
    return neighbors

def cost(current_node_index, other):
    sol = np.sqrt((other[0]-current_node_index[0])**2 + (other[1]-current_node_index[1])**2 + (other[2]-current_node_index[2])**2)
    return sol

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    occ_map = OccupancyMap(world, resolution, margin)

    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    Q, explored_nodes, parents  = [(0, start_index)], set(), { }

    distance = defaultdict(lambda: float("inf"))
    distance[start_index] = cost(goal_index ,start_index )


    def is_goal():
        path = []
        path.append(goal)

        node_index = goal_index
        
        
        while node_index:
            parent = parents[node_index]
            if parent == start_index:
                path.insert(0, start)
                break

            parent_metric = occ_map.index_to_metric_center(parent)
            path.insert(0, parent_metric)
            node_index = parent
        return np.array(path), len(explored_nodes)

    while Q:
        current_node_index = heappop(Q)[1]

        if current_node_index == goal_index:
            return is_goal()
        explored_nodes.add(current_node_index)
        neighbors = currentnode_neighbors(np.array(current_node_index),occ_map)

        for neighbor in neighbors:
            cost_to_come = cost(current_node_index,neighbor)
            cost_to_come = cost_to_come + distance[current_node_index]
            heuristic_cost_to_come = cost(neighbor, goal_index)

            if astar == False:
                total_cost_to_come = cost_to_come
            elif astar == True:
                total_cost_to_come = cost_to_come + heuristic_cost_to_come
            # print(neighbor)
            # print(tuple(neighbor))
            def to_tuple(list):
                    return ( *list, )
            if total_cost_to_come < distance[to_tuple(neighbor)]: 
                distance[to_tuple(neighbor)] = cost_to_come
                parents[to_tuple(neighbor)] = current_node_index
                heappush(Q, (total_cost_to_come, to_tuple(neighbor)))

    # Return a tuple (path, nodes_expanded)
    return None, 0
