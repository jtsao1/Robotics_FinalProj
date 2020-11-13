from PIL import Image, ImageDraw
from queue import Queue
import numpy as np
import math
import random

def distance(a, b):
    return math.sqrt(math.pow(a[0] - b[0], 2) + math.pow(a[1] - b[1], 2))


class Vertex:
    def __init__(self, state):
        self.state = state
        self.neighbors = []

    def add_edge(self, u):
        self.neighbors.append(u)

    def __str__(self):
        return f"(v: {self.state[0]}, {self.state[1]}))"

class RRT:
    def __init__(self, img):
        self.img = img
        self.tree = []

    def build(self, x_init, K, delta):
        self.tree.append(Vertex(x_init))
        self.source = x_init
        for i in range(K):
            #random sampling
            x_rand = self._random_state()
            v_near = self.nearest_neighbor(x_rand)
            x_near = v_near.state
            x_new = self._extend(x_near, x_rand, delta)
            #collision detection
            if not self.img.has_obstacle(x_new[0], x_new[1]):
                for vertex in self.tree:
                    if vertex.state == x_new:
                        v = vertex
                        break
                else:
                    v = Vertex(x_new)
                v.add_edge(v_near)
                self.tree.append(v)
                #print(v)

    def nearest_neighbor(self, x):
        #find nearest vertex
        min_d = np.inf
        v_near = None

        for v in self.tree:
            d = distance(v.state, x)
            if d < min_d:
                v_near = v
                min_d = d
        return v_near

    def _random_state(self):
        x = random.randint(0, self.img.width)
        y = random.randint(0, self.img.height)
        return (x, y)

    def _extend(self, x_near, x_rand, delta):
        delta_x = x_rand[0] - x_near[0]
        delta_y = x_rand[1] - x_near[1]
        delta_d = distance(x_near, x_rand)
        if delta_d == 0:
            return x_new
        #unit vector
        x_new = (delta * delta_x/delta_d + x_near[0], delta * delta_y/delta_d + x_near[1])
        return x_new

    def shortest_path(self, goal):
        #add goal vertex
        goal_vertex = Vertex(goal)
        #connect to the neaert neighbor
        goal_vertex.add_edge(self.nearest_neighbor(goal))
        self.tree.append(goal_vertex)

        #breath-first search is able to find the shortest path
        q = Queue()
        visited = []
        prev = []
        #reverse search
        q.put(self.tree[-1])
        visited.append(self.tree[-1])

        while not q.empty():
            v = q.get()
            #store the path
            for n in v.neighbors:
                if n not in visited:
                    visited.append(n)
                    q.put(n)
                    prev.append((n, v))

        #construct path
        #forward tracing
        source = self.source
        path = []
        while True:
            if source == goal:
                break
            for u, v in prev:
                if u.state == source:
                    path.append(source)
                    source = v.state
                    break
        path.append(goal)
        return path

    def get_vertex_pair(self):
        for v in self.tree:
            if len(v.neighbors) == 0:
                yield (v.state, v.state)
            elif len(v.neighbors) == 1:
                yield (v.state, v.neighbors[0].state)
            else:
                for n in v.neighbors:
                    yield (v.state, n.state)
    """


    ##searching using dijkstra
    source = 0  #the index of source node

    def get(dist, visited):
        min = np.inf
        min_index = -1

        for i in range(len(self.tree)):
            if dist[i] < min and visited[i] == False:
                min = dist[i]
                min_index = i
        return min_index

    #initilize distance list and previous_node list and frontier list
    dist = [np.inf] * len(self.tree)
    prev = [None] * len(self.tree)
    visited = [False] * len(self.tree)
    dist[0] = 0

    for i in range(len(self.tree)):
        v = get(dist, visited)
        if v == -1:
            print("error")
        visited[v] = True

        for n in self.tree[v].neighbors:
            u = self.tree.index(n)
            if not visited[u] and (dist[v] > dist[u] + 1):

    """
