
import heapq
import math
from suaviza.mientras_llueve import approximate_b_spline_path

class AstarAckermann:
    def __init__(self, start, goal, grid):
        self.start = start
        self.goal = goal
        self.grid = grid
        self.rows, self.cols = self.grid.shape
    def is_valid(self, coord):
        x, y = coord
        return 0 <= x < self.rows and 0 <= y < self.cols and self.grid[x, y] == 0
    def heuristic(self, coord):
        x1, y1 = coord
        x2, y2 = self.goal
        #return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        return abs(x1 - x2) + abs(y1 - y2)
    def costObstacle(self, neighbor, directions):
        distancia = []
        costo = 0
        for direction2 in directions:  # Vamos a revisar si el nodo actual en revision tiene vecinos ocuapos demasiado ccerca de el, de ser asi vamos a aumentar su costo para que el aloritmo prefiera caminos mas libres
            neighbor2 = (neighbor[0] + direction2[0], neighbor[1] + direction2[1])
            if not self.is_valid(neighbor2) and neighbor2[0]>=0 and neighbor2[1]>=0:  # Si algun vecino del vecino disponible es un obstaculo vamos a aumentar el costo de ese nodo
                distancia.append(math.sqrt(direction2[0]**2 + direction2[1]**2))
        if len(distancia) > 0:
            if min(distancia) == 1: #Vecinos del vecino no validos mas cercanos implican un costo mayor
                costo = 1.4
            else: #Vecinos del vecino no validos mas lejanos implican un costo menor
                costo = 1.1
        return costo
    def astar(self):
        heap = [(0 + self.heuristic(self.start), 0, self.start)]
        visited = set()
        distance = {self.start: 0}
        previous = {}
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, 1), (-1, -1), (1, -1)]
        while heap:
            _, dist, current = heapq.heappop(heap)
            if current == self.goal or math.sqrt((current[0] - self.goal[0]) ** 2 + (current[1] - self.goal[1]) ** 2) <= 0.2:
                path = []
                while current in previous:
                    path.append(current)
                    current = previous[current]
                path.append(self.start)
                path.reverse()
                return path, self.smoother(path)
            visited.add(current)
            for direction in directions:
                next_x, next_y = current[0] + direction[0], current[1] + direction[1]
                neighbor = (next_x, next_y)
                if self.is_valid(neighbor) and neighbor not in visited:
                    new_distance = distance[current] + math.sqrt(direction[0]**2 + direction[1]**2)
                    if neighbor not in distance or new_distance < distance[neighbor]:
                        distance[neighbor] = new_distance
                        previous[neighbor] = current
                        heapq.heappush(heap, (new_distance + self.heuristic(neighbor) + self.costObstacle(neighbor, directions), new_distance, neighbor))
        return None
    def smoother(self, path):
        if path:
            n_course_point = 400
            x, y = zip(*path)
            rax, ray, heading, curvature = approximate_b_spline_path(
                y, x, n_course_point, s=0.5)
            return list(zip(ray, rax, heading))
        else:
            return False