import random
import math

def distance(q1, q2):
    return math.sqrt((q2[0] - q1[0]) ** 2 + (q2[1] - q1[1]) ** 2)

def normalize(vector):
    norm = math.sqrt(sum(x ** 2 for x in vector))
    return [x / norm for x in vector]

def intersects(q1, q2, obstacle):
    # Implement collision checking logic (e.g., using line segment intersection)
    pass

def random_sample(config_space_min, config_space_max):
    return (random.uniform(config_space_min[0], config_space_max[0]), 
            random.uniform(config_space_min[1], config_space_max[1]))

def nearest_node(T, q_rand):
    return min(T, key=lambda q: distance(q, q_rand))

def steer(q_nearest, q_rand, delta_q):
    direction = normalize([q_rand[i] - q_nearest[i] for i in range(len(q_nearest))])
    q_new = tuple([q_nearest[i] + delta_q * direction[i] for i in range(len(q_nearest))])
    return q_new

def collision_free(q_nearest, q_new, obstacles):
    for obstacle in obstacles:
        if intersects(q_nearest, q_new, obstacle):
            return False
    return True

def goal_reached(q_new, q_goal, epsilon):
    return distance(q_new, q_goal) < epsilon

def extract_path(parent, q_start, q_goal):
    path = [q_goal]
    while path[-1] != q_start:
        path.append(parent[path[-1]])
    path.reverse()
    return path

def RRT(q_start, q_goal, config_space_min, config_space_max, delta_q, epsilon, max_iterations, obstacles):
    q_start = tuple(q_start)
    q_goal = tuple(q_goal)
    T = [q_start]
    parent = {q_start: None}
    
    for i in range(max_iterations):
        q_rand = random_sample(config_space_min, config_space_max)
        q_nearest = nearest_node(T, q_rand)
        q_new = steer(q_nearest, q_rand, delta_q)
        
        if collision_free(q_nearest, q_new, obstacles):
            T.append(q_new)
            parent[q_new] = q_nearest
            
            if goal_reached(q_new, q_goal, epsilon):
                return extract_path(parent, q_start, q_new)
    
    return None  # No path found

# Example usage
config_space_min = (0, 0)
config_space_max = (10, 10)
delta_q = 0.5
epsilon = 0.1
max_iterations = 10000
obstacles = [((2, 2), (2, 8)), ((8, 2), (8, 8))]
q_start = (1, 1)
q_goal = (9, 9)

path = RRT(q_start, q_goal, config_space_min, config_space_max, delta_q, epsilon, max_iterations, obstacles)
print(path)