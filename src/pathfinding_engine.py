import heapq
class Node:
    def __init__(self, x, y, g=float('inf'), h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.parent = parent
        
    @property
    def f(self):
        return self.g + self.h
    def __lt__(self, other):
        if self.f != other.f:
            return self.f < other.f
        return self.g > other.g

min_heap = []
heapq.heapify(min_heap)

def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def a_star(cost_map, start, goal):
    open_list = []
    closed_list = {}
    start_h = manhattan_distance(start, goal)
    start_node = Node(start[0], start[1], g=0, h=start_h)
    heapq.heappush(open_list, start_node)
    closed_list[(start[0], start[1])] = 0
    
    while open_list:
        current = heapq.heappop(open_list)
        current_pos = (current.x, current.y)
        
        if current.g > closed_list.get(current_pos, float('inf')):
            continue
        
        if current_pos == goal:
            path=[]
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]
        
        for dx, dy in [(1,0), (0,1), (-1,0), (0,-1)]:
            nx, ny = current.x + dx, current.y + dy
            nbr_pos = (nx, ny)
            if 0 <= ny < len(cost_map) and 0 <= nx < len(cost_map[0]):
                step_cost = cost_map[ny][nx]
                
                # Ignorar obstáculos
                if step_cost == float('inf'):
                    continue
                    
                tentative_g = current.g + step_cost
                
                if tentative_g < closed_list.get(nbr_pos, float('inf')):
                    closed_list[nbr_pos] = tentative_g
                    neighbor_h = manhattan_distance(nbr_pos, goal)
                    neighbor_node = Node(nx, ny, g=tentative_g, h=neighbor_h, parent=current)
                    heapq.heappush(open_list, neighbor_node)   
    return []

def visualize_path(cost_map, path, start, goal):
    if not path:
        print("No path found.")
    
    height = len(cost_map)
    width = len(cost_map[0])
    for y in range(height):
        row = ""
        for x in range(width):
            pos = (x,y)
            if pos == start:
                row += "S "
            elif pos == goal:
                row += "G "
            elif pos in path:
                row += "* "
            elif cost_map[y][x] == float('inf'):
                row += "# "
            else:
                row += ". "
        print(row)