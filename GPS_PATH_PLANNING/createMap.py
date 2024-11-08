import geojson
import math
import json
import heapq
from scipy.spatial import cKDTree

class Graph:
    def __init__(self):
        self.graph = {}

    def addNode(self, node):
        if node not in self.graph:
            self.graph[node] = {}
    
    def getNode(self, node):
        return self.graph.get(node, None)

    def addEdge(self, node1, node2, weight):
        self.addNode(node1)
        self.addNode(node2)
        self.graph[node1][node2] = weight
        self.graph[node2][node1] = weight

    def getNeighbors(self, node):
        return self.graph.get(node, {})

    def getEdgeWeight(self, node1, node2):
        return self.graph[node1].get(node2)
    
    def generateMap(self):
        latitudes = [node[0] for node in self.graph]
        longitudes = [node[1] for node in self.graph]

        map_center = [sum(latitudes) / len(latitudes), sum(longitudes) / len(longitudes)]
        
        nodes = [{"lat": node[0], "lng": node[1]} for node in self.graph]
        edges = []
        for node, neighbors in self.graph.items():
            for neighbor, weight in neighbors.items():
                edges.append({
                    "start": {"lat": node[0], "lng": node[1]},
                    "end": {"lat": neighbor[0], "lng": neighbor[1]},
                    "weight": weight
                })
        map_data = {
            "center": map_center,
            "nodes": nodes,
            "edges": edges
        }
        with open("./static/map_data.js", "w") as f:
            f.write(f"var mapData = {json.dumps(map_data)};")
    
    def heuristic(self, node, end):
        return distance(node, end)
    
    def shortestPath(self, start, end): #A* Algorithm
        frontier = [(0, start)] 
        came_from = {}
        cost_so_far = {start: 0}

        while frontier:
            current_cost, current = heapq.heappop(frontier)
            if current == end:
                break
            for next_node in self.getNeighbors(current):
                new_cost = cost_so_far[current] + self.getEdgeWeight(current, next_node)
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, end)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current
        path = []
        current = end
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                return None
        path.append(start)
        path.reverse()

        return path
        
def degreesToRadians(degrees):
  return degrees * math.pi / 180;

def distance(node1, node2):
    earthRadiusKm = 6371

    dLat = degreesToRadians(node2[0] - node1[0])
    dLon = degreesToRadians(node2[1] - node1[1])

    lat1 = degreesToRadians(node1[0])
    lat2 = degreesToRadians(node2[0])

    a = math.sin(dLat/2) * math.sin(dLat/2) + math.sin(dLon/2) * math.sin(dLon/2) * math.cos(lat1) * math.cos(lat2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return earthRadiusKm * c * 1000  # convert to meters

def parseMap(filepath):
    with open(filepath) as f:
        jsonData = geojson.load(f)
    features = jsonData['features']
    graph = Graph()
    all_nodes = set()

    for feature in features:
        geometryType = feature['geometry']['type']
        coordinates = feature['geometry']['coordinates']
        if geometryType == 'LineString':
            for i in range(len(coordinates) - 1):
                node1 = (coordinates[i][1], coordinates[i][0])
                node2 = (coordinates[i+1][1], coordinates[i+1][0])
                all_nodes.add(node1)
                all_nodes.add(node2)
                weight = distance(node1, node2)
                graph.addEdge(node1, node2, weight)
        elif geometryType == 'Polygon':
            for ring in coordinates:
                for i in range(len(ring) - 1):
                    node1 = (ring[i][1], ring[i][0])
                    node2 = (ring[i+1][1], ring[i+1][0])
                    all_nodes.add(node1)
                    all_nodes.add(node2)
                    weight = distance(node1, node2)
                    graph.addEdge(node1, node2, weight)
        else:
            print("Geometry type not supported")
            exit()

    return graph
