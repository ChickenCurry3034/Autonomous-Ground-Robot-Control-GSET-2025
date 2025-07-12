import math

nodes = {
    "A" : [0,0,0],
    "B" : [1,2,math.pi/4],
    "C" : [2,3,math.pi/3],
    "D" : [3,2,math.pi/2],
    "E" : [4,1,-math.pi/6],
    "F" : [4,2,-math.pi/3],
    "G" : [4,3,math.pi/4]
}

connections = {
    "A" : ["B", "C"],
    "B" : ["A", "C", "D"],
    "C" : ["A", "B", "D", "G"],
    "D" : ["B", "C", "E", "F", "G"],
    "E" : ["D", "F"],
    "F" : ["D", "G"],
    "G" : ["C", "F"]
}

def dijkstra(nodes, connections, start, end):
    distances = {node: float('inf') for node in nodes}
    distances[start] = 0
    
    unvisited = list(nodes.keys())
    prev = {}

    while unvisited:
        min_distance_node = None
        for node in unvisited:
            if min_distance_node is None:
                min_distance_node = node
            elif distances[node] < distances[min_distance_node]:
                min_distance_node = node
        
        if distances[min_distance_node] == float('inf'):
            break

        current_node = min_distance_node
        unvisited.remove(current_node)

        if current_node == end:
            break

        for neighbor in connections[current_node]:
            if neighbor in unvisited:
                distance = math.hypot(nodes[neighbor][0] - nodes[current_node][0], nodes[neighbor][1] - nodes[current_node][1])
                
                alt = distances[current_node] + distance
                
                if alt < distances[neighbor]:
                    distances[neighbor] = alt
                    prev[neighbor] = current_node
        
        path = []
        current = end
        while current is not None:
            path.insert(0, current)
            current = prev.get(current)
            if current == start:
                path.insert(0, start)
                break
        if path and path[0] != start:
            path.insert(0, start)
        elif not path and start == end:
            path = [start]
        elif not path and prev.get(end) is None and end != start:
            return []
    
    return path

path = dijkstra(nodes, connections, "A", "G")
print(path)