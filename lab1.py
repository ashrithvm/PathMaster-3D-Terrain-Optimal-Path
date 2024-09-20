import sys
from PIL import Image, ImageDraw
import heapq
import math

def map_rgb_to_terrain(rgb):
    terrain_mapping = {
        (248, 148, 18): 1.0,    # Open Land  
        (255, 192, 0): 2.0,     # Rough Meadow
        (255, 255, 255): 1.5,   # Easy Movement Forest
        (2, 208, 60): 2.5,      # Slow Run Forest
        (2, 136, 40): 3.0,      # Walk Forest
        (5, 73, 24): float('inf'),  # Impassable Vegetation
        (0, 0, 255): 20,        # Lake/Swamp/Marsh
        (71, 51, 3): 1.2,       # Paved Road
        (0, 0, 0): 1.0,         # Footpath (Black)
        (205, 0, 101): float('inf')  # Out of Bounds
    }

    def color_difference(c1, c2):
        return sum(abs(c1[i] - c2[i]) for i in range(3))

    closest_value = None
    min_difference = float('inf')
    
    for terrain_color, terrain_value in terrain_mapping.items():
        difference = color_difference(rgb, terrain_color)
        if difference < min_difference:
            min_difference = difference
            closest_value = terrain_value

    return closest_value

def load_terrain_image(image_path):
    try:
        img = Image.open(image_path).convert('RGB')
        img = img.resize((395, 500))  # Resize to match elevation matrix height
        pixels = img.load()

        terrain_matrix = [[0.0 for _ in range(img.width)] for _ in range(img.height)]

        for y in range(img.height):  
            for x in range(img.width):
                rgb_value = pixels[x, y] 
                terrain_matrix[y][x] = map_rgb_to_terrain(rgb_value) 

        return terrain_matrix
    except Exception as e:
        print(f"Error loading terrain image: {e}")
        return None

def load_elevation(filename):
    elevation_matrix = []
    try:
        with open(filename, 'r') as f:
            for line in f:
                values = line.strip().split()[:395]
                elevation_matrix.append([float(val) for val in values if val])
        return elevation_matrix
    except Exception as e:
        print(f"Error loading elevation file: {e}")
        return None

class Node:
    def __init__(self, position, parent, g, h, distance):
        self.position = position  
        self.parent = parent  
        self.g = g  
        self.h = h 
        self.f = g + h
        self.distance = distance  

    def __lt__(self, other):
        return self.f < other.f  

def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def astar(grid, elevation_matrix, start, goal, x_step=1, y_step=1):
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1), 
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]
    
    open_list = []
    closed_list = set()
    g_scores = {start: 0}  
    
    start_node = Node(start, None, 0, euclidean_distance(start, goal), 0)
    heapq.heappush(open_list, start_node)
    
    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == goal:
            path = []
            total_distance = current_node.distance  
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1], total_distance  
        
        if current_node.position in closed_list:
            continue
        
        closed_list.add(current_node.position)
        
        for direction in directions:
            neighbor_pos = (current_node.position[0] + direction[0], current_node.position[1] + direction[1])
            
            if not (0 <= neighbor_pos[0] < len(grid) and 0 <= neighbor_pos[1] < len(grid[0])):
                continue  

            terrain_multiplier = grid[neighbor_pos[0]][neighbor_pos[1]]
            if terrain_multiplier == float('inf'):
                continue 
            
            current_elevation = elevation_matrix[current_node.position[0]][current_node.position[1]]
            neighbor_elevation = elevation_matrix[neighbor_pos[0]][neighbor_pos[1]]
            elevation_diff = neighbor_elevation - current_elevation
            
            if direction[0] != 0 and direction[1] != 0:
                base_distance = math.sqrt(x_step ** 2 + y_step ** 2)
            else:
                base_distance = x_step if direction[0] != 0 else y_step

            distance = math.sqrt(base_distance ** 2 + elevation_diff ** 2)  

            tentative_g = current_node.g + (distance * terrain_multiplier)  
            actual_distance = current_node.distance + distance 
            
            if neighbor_pos not in g_scores or tentative_g < g_scores[neighbor_pos]:
                g_scores[neighbor_pos] = tentative_g
                h_cost = euclidean_distance(neighbor_pos, goal) 
                
                neighbor_node = Node(neighbor_pos, current_node, tentative_g, h_cost, actual_distance)
                
                heapq.heappush(open_list, neighbor_node)
    
    return None 

def color_path_on_image(terrain_image_path, path, output_image_path, save=False):
    img = Image.open(terrain_image_path)
    
    img_copy = img.copy()
    draw = ImageDraw.Draw(img_copy)

    path_color = (118, 63, 231)

    for (x, y) in path:
        draw.point((y, x), fill=path_color)

    if save:
        img_copy.save(output_image_path)
    else:
        img_copy.show()


def main():
    terrain_image_path = sys.argv[1]
    elevation_file_path = sys.argv[2]
    checkpoint_file_path = sys.argv[3]
    output_image_path = sys.argv[4]

    y_step = 10.29
    x_step = 7.55

    grid = load_terrain_image(terrain_image_path)
    elevation_matrix = load_elevation(elevation_file_path)

    if not grid or not elevation_matrix:
        print("Error loading terrain or elevation data.")
        return

    if len(grid) != len(elevation_matrix) or len(grid[0]) != len(elevation_matrix[0]):
        print("Terrain and elevation matrices do not have matching dimensions.")
        return

    with open(checkpoint_file_path, 'r') as f:
        steps = [tuple(map(int, line.strip().split())) for line in f]

    path = []
    total_distance = 0

    for i in range(1, len(steps)):
        start = tuple(reversed(steps[i - 1]))
        end = tuple(reversed(steps[i]))
        segment_path, distance = astar(grid, elevation_matrix, start, end, x_step, y_step)
        
        total_distance += distance
        
        if segment_path:
            if path and segment_path:
                path.extend(segment_path[1:])
            else:
                path.extend(segment_path)

    print(total_distance)

    if path:
        color_path_on_image(terrain_image_path, path, output_image_path, save=True)
    else:
        print("No complete path found.")


if __name__ == '__main__':
    main()

