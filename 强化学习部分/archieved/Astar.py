

from heapq import heappush, heappop
from sys import maxsize
import copy
import sys
import time

sys.path.append("..")
from util.Grid import Cell, Grid, parse_grid, map2grid, view_path, grid2world, world2grid



# Represent each node as a list, ordering the elements so that a heap of nodes
# is ordered by f = g + h, with h as a first, greedy tie-breaker and num as a
# second, definite tie-breaker. Store the redundant g for fast and accurate
# calculations.

F, H, NUM, G, POS, OPEN, VALID, PARENT = range(8)


def astar(DUNGEON, width, height, start_pos, start_g, destination, limit=maxsize):
    """Find the shortest path from start to goal.

    Arguments:

      start_pos      - The starting position.
      neighbors(pos) - A function returning all neighbor positions of the given
                       position.
      goal(pos)      - A function returning true given a goal position, false
                       otherwise.
      start_g        - The starting cost.
      cost(a, b)     - A function returning the cost for moving from one
                       position to another.
      heuristic(pos) - A function returning an estimate of the total cost
                       remaining for reaching goal from the given position.
                       Overestimates can yield suboptimal paths.
      limit          - The maximum number of positions to search.
      debug(nodes)   - This function will be called with a dictionary of all
                       nodes.

    The function returns the best path found. The returned path excludes the
    starting position.
    """

    grid = parse_grid(DUNGEON, width, height)
    def goal(pos):
        return pos == destination

    def cost(from_pos, to_pos):
        from_y, from_x = from_pos
        to_y, to_x = to_pos
        return 14 if to_y - from_y and to_x - from_x else 10

    def heuristic(pos):
        y, x = pos
        goal_y, goal_x = destination
        dy, dx = abs(goal_y - y), abs(goal_x - x)
        return min(dy, dx) * 14 + abs(dy - dx) * 10

    def debug(nodes):
        nodes = nodes

    # Create the start node.
    nums = iter(range(maxsize))
    start_h = heuristic(start_pos)
    start = [start_g + start_h, start_h, nums.__next__(), start_g, start_pos, True,
             True, None]

    def neighbors(pos):
        cell = grid[pos]
        if cell.neighbors is None:
            y, x = pos
            cell.neighbors = []
            for neighbor_y, neighbor_x in grid.neighbors(y, x):
                if grid[neighbor_y, neighbor_x].char != '#':
                    cell.neighbors.append((neighbor_y, neighbor_x))
        return cell.neighbors

    # Track all nodes seen so far.
    nodes = {start_pos: start}

    # Maintain a heap of nodes.
    heap = [start]

    # Track the best path found so far.
    best = start

    while heap:

        # Pop the next node from the heap.
        current = heappop(heap)
        current[OPEN] = False

        # Have we reached the goal?
        if goal(current[POS]):
            best = current
            break

        # Visit the neighbors of the current node.
        for neighbor_pos in neighbors(current[POS]):
            neighbor_g = current[G] + cost(current[POS], neighbor_pos)
            neighbor = nodes.get(neighbor_pos)
            if neighbor is None:

                # Limit the search.
                if len(nodes) >= limit:
                    continue

                # We have found a new node.
                neighbor_h = heuristic(neighbor_pos)
                neighbor = [neighbor_g + neighbor_h, neighbor_h, nums.__next__(),
                            neighbor_g, neighbor_pos, True, True, current[POS]]
                nodes[neighbor_pos] = neighbor
                heappush(heap, neighbor)
                if neighbor_h < best[H]:
                    # We are approaching the goal.
                    best = neighbor

            elif neighbor_g < neighbor[G]:

                # We have found a better path to the neighbor.
                if neighbor[OPEN]:

                    # The neighbor is already open. Finding and updating it
                    # in the heap would be a linear complexity operation.
                    # Instead we mark the neighbor as invalid and make an
                    # updated copy of it.

                    neighbor[VALID] = False
                    nodes[neighbor_pos] = neighbor = neighbor[:]
                    neighbor[F] = neighbor_g + neighbor[H]
                    neighbor[NUM] = nums.__next__()
                    neighbor[G] = neighbor_g
                    neighbor[VALID] = True
                    neighbor[PARENT] = current[POS]
                    heappush(heap, neighbor)

                else:

                    # Reopen the neighbor.
                    neighbor[F] = neighbor_g + neighbor[H]
                    neighbor[G] = neighbor_g
                    neighbor[PARENT] = current[POS]
                    neighbor[OPEN] = True
                    heappush(heap, neighbor)

        # Discard leading invalid nodes from the heap.
        while heap and not heap[0][VALID]:
            heappop(heap)
        

    if debug is not None:
        # Pass the dictionary of nodes to the caller.
        debug(nodes)

    # Return the best path as a list.
    path = []
    current = best
    while current[PARENT] is not None:
        path.append(current[POS])
        current = nodes[current[PARENT]]
    path.reverse()
    return path


def pathprocess(path):
    len = path.__len__()
    tmp = copy.deepcopy(path)
    for i, val in enumerate(path):
        if i + 1 < len and i - 1 >= 0:
            biasx1 = (path[i][0] - path[i - 1][0])
            biasx2 = (path[i + 1][0] - path[i][0])
            biasy1 = (path[i][1] - path[i - 1][1])
            biasy2 = (path[i + 1][1] - path[i][1])
            if biasx1 == biasx2 and biasy1 == biasy2:
                tmp.remove(val)
    return tmp


if __name__ == '__main__':
    import random
    import string
    import Box2D
    DUNGEON2 = """
        #################
                        #
                        #           ###########
                        #                     #
    #############       #                     #
    #                   #                     #
    #                   #                     #
    #          ###################            #
    #                            #            #
    #                            #            #
    #                            #       #    #
    #                #############       #    #
    #                                    #
    ###############                      #          #
                                         #          #
                                         #          #
                                         #          #
                               ######################
    """


    class Engine(object):

        def __init__(self, DUNGEON, width, height):
            self.width = width
            self.height = height
            self.DUNGEON=DUNGEON


        def update_path(self):
            target = world2grid(Box2D.b2Vec2(4, 1.5))
            start = world2grid(Box2D.b2Vec2(0.5, 4.5))
            world = grid2world(target)
            print(world)
            print(target)
            self.path = astar(self.DUNGEON, self.width, self.height, start, 0, target)  # (y,x)




    width = 200
    height = 120
    DUNGEON = map2grid(width, height)
    engine = Engine(DUNGEON, width, height)

    m = DUNGEON.__len__()
    engine.update_path()
    tmp = engine.path
    path = pathprocess(tmp)
    str2 = view_path(DUNGEON, engine.path, width)

    print(str2)
    mylen = 0 * (width) + 1
    str2 = str2[:mylen] + '@' + str2[mylen + 1:]
    print(engine.path)


