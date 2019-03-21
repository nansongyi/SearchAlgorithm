from NLinkArm import *
# from matplotlib import pyplot as plt
import collections
import heapq

class PriorityQueue:
	def __init__(self):
		self.elements = []
	def empty(self):
		return len(self.elements) == 0
	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))
	def get(self):
		# get the cost, cuz it's a tuple (item, cost)
		return heapq.heappop(self.elements)[1]

class Graph(object):

	def __init__(self, grid, start, goal):
		self.grid = grid
		self.start = start
		self.goal = goal

	def check_range(self,p):
		def out_of_range(x):
			return x <= 99 and x >= 0
		return not all(map(out_of_range, p))

	def neighbors(self, currunt):
		neighbors = []
		i,j = currunt
		p1 = (i-1,j)
		p2 = (i,j-1)
		p3 = (i+1,j)
		p4 = (i,j+1)
		if (not self.check_range(p1)) and self.grid[p1] != 1:
			neighbors.append(p1)
		if (not self.check_range(p2)) and self.grid[p2] != 1:
			neighbors.append(p2)
		if (not self.check_range(p3)) and self.grid[p3] != 1:
			neighbors.append(p3)
		if (not self.check_range(p4)) and self.grid[p4] != 1:
			neighbors.append(p4)
		return neighbors

	def compute_heuristic(self, config):
		return np.abs(self.goal[1] - config[1]) + np.abs(self.goal[0] - config[0])

	def reconstruct_path(self, came_from, start, goal):
		current = goal
		path = []
		while current != start:
			# print("appending:",current)
			path.append(current)
			current = came_from[current]
		path.append(start) # optional
		path.reverse() # optional
		return path

		
def main():
	# step 0: simulation parameters
	M = 100	 
	link_length = [0.5, 1.5]
	initial_link_angle = [0, 0]
	obstacles = [[1.75, 0.75, 0.6], [0.55, 1.5, 0.5], [0, -1, 0.7]]
	goal_found = False

	# step 1: robot and environement setup
	arm = NLinkArm(link_length, initial_link_angle)
	grid = get_occupancy_grid(arm, obstacles, M)

	print("\nstep 1: robot and environment ready.")

	while not goal_found:
		 # step 2: set start and random goal
		start = angle_to_grid_index(initial_link_angle,M)
		goal = (randint(0,M-1),randint(0,M-1)) 
		start_js = grid_index_to_angle(start, M)
		goal_js = grid_index_to_angle(goal, M)
		goal_pos = forward_kinematics(link_length, goal_js)
		start_pos = forward_kinematics(link_length, start_js)

		if not (grid[start[0]][start[1]] == 0):
			print("Start pose is in collision with obstacles. Close system.")
			break

		elif (grid[goal[0]][goal[1]] == 0):
			print("\nstep 2: \n\tstart_js  = {}, goal_js  = {}".format(start_js, goal_js))
			print("\tstart_pos = {}, goal_pos = {}".format(start_pos, goal_pos))
			goal_found = True

	if(goal_found):
		# step 3: motion planning
		# graph  = Graph(grid, start, goal)
		# came_from, cost_so_far = a_star_search(graph, start, goal)
		# route = graph.reconstruct_path(came_from, start, goal)
		print(start)
		print(goal)
		route = astar_search(grid, start, goal, M)
		print("\nstep 3: motion planning completed.")

		# step 4: visualize result	
		print("\nstep 4: start visualization.")
		if len(route) >= 0:
			animate(grid, arm, route, obstacles, M, start_pos, goal_pos, start, goal)

		print("\nGoal reached.")

def forward_kinematics(link_length, joint_angles):
	posx = link_length[0]*np.cos(joint_angles[0])+link_length[1]*np.cos(np.sum(joint_angles))
	posy = link_length[0]*np.sin(joint_angles[0])+link_length[1]*np.sin(np.sum(joint_angles))
	return (posx,posy)
	 
def inverse_kinematics(posx, posy, link_length):
	goal_th = atan2(posy,posx)
	A = link_length[0]
	B = sqrt(posx*posx+posy*posy)
	C = link_length[1]

	C_th = np.arccos(float((A*A + B*B - C*C)/(2.0*A*B))) 
	B_th = np.arccos(float((A*A + C*C - B*B)/(2.0*A*C))) 
	theta1_sol1 = simplify_angle(goal_th + C_th)
	theta2_sol1 = simplify_angle(-pi + B_th)
	theta1_sol2 = simplify_angle(goal_th - C_th)
	theta2_sol2 = simplify_angle(pi - B_th)

	return [[theta1_sol1, theta2_sol1],[theta1_sol2, theta2_sol2]] 

def detect_collision(line_seg, circle):
	# TODO: Return True if the line segment is intersecting with the circle
	#       Otherwise return false.
	# 	 (1) line_seg[0][0], line_seg[0][1] is one point (x,y) of the line segment
	#	 (2) line_seg[1][0], line_seg[1][1] is another point (x,y) of the line segment
	#	 (3) circle[0],circle[1],circle[2]: the x,y of the circle center and the circle radius
	# 	 Hint: the closest point on the line segment should be greater than radius to be collision free.
	#       Useful functions: np.linalg.norm(), np.dot()
	xa, ya = line_seg[0][0], line_seg[0][1]
	xb, yb = line_seg[1][0], line_seg[1][1]
	seg_v = np.array([xb-xa, yb-ya])
	x, y, r = circle
	center = np.array([x,y])
	pt_v = np.array([x-xa, y-ya])
	proj_v = np.dot(pt_v, seg_v)/np.linalg.norm(seg_v)
	if proj_v < 0:
		closest = np.array([xa, ya])
	elif proj_v > np.linalg.norm(seg_v):
		closest = np.array([xb, yb])
	else:
		closest = np.array([xa, ya]) + proj_v * seg_v/np.linalg.norm(seg_v)
	dist_v = np.linalg.norm(center - closest)
	if dist_v < r:
		return True
	return False

def get_occupancy_grid(arm, obstacles, M):
	grid = [[0 for _ in range(M)] for _ in range(M)]
	theta_list = [2 * i * pi / M for i in range(-M // 2, M // 2 + 1)]
	for i in range(M):
		for j in range(M):
			# TODO: traverse through all the grid vertices, i.e. (theta1, theta2) combinations
			# 	 Find the occupacy status of each robot pose.
			# 	 Useful functions/variables: arm.update_joints(), arm.points, theta_list[index]
			goal_joint_angles = np.array([theta_list[i], theta_list[j]])
			arm.update_joints(goal_joint_angles)
			points = arm.points

			collision_detected = False
			for k in range(len(points) - 1):
				for obstacle in obstacles:
					# TODO: define line_seg and detect collisions
					# 	 Useful functions/variables: detect_collision(), points[index]
					line_seg = [points[k],points[k+1]]
					collision_detected = detect_collision(line_seg, obstacle)
					if collision_detected:
						break
				if collision_detected:
					break
			grid[i][j] = int(collision_detected)
	return np.array(grid)

def a_star_search(graph, start, goal):

    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()            
        if current == goal:
            print("break")
            break            
        for next_term in graph.neighbors(current):
            new_cost = cost_so_far[current] + 1
            if next_term not in cost_so_far or new_cost < cost_so_far[next_term]:
                cost_so_far[next_term] = new_cost
                priority = new_cost + graph.compute_heuristic(next_term)
                frontier.put(next_term, priority)
                came_from[next_term] = current

    return came_from, cost_so_far

def astar_search(grid, start_node, goal_node, M):
	colors = ['white', 'black', 'red', 'pink', 'yellow', 'green', 'orange']
	levels = [0, 1, 2, 3, 4, 5, 6, 7]
	cmap, norm = from_levels_and_colors(levels, colors)

	grid[start_node] = 4
	grid[goal_node] = 5

	print("show grid")
	plt.imshow(grid, interpolation='nearest')
	plt.show()
	print(grid)

	parent_map = [[() for _ in range(M)] for _ in range(M)]
	heuristic_map = calc_heuristic_map(M, goal_node)
	plt.imshow(heuristic_map, interpolation='nearest')
	plt.show()

	evaluation_map = np.full((M, M), np.inf)
	distance_map = np.full((M, M), np.inf)
	evaluation_map[start_node] = heuristic_map[start_node]
	distance_map[start_node] = 0
	while True:
		grid[start_node] = 4
		grid[goal_node] = 5

		current_node = np.unravel_index(np.argmin(evaluation_map, axis=None), evaluation_map.shape)
		min_distance = np.min(evaluation_map)

		if (current_node == goal_node) or np.isinf(min_distance):
			break

		grid[current_node] = 2
		evaluation_map[current_node] = np.inf

		i, j = current_node[0], current_node[1]

		neighbors = find_neighbors(i, j, M)

		for neighbor in neighbors:
			if grid[neighbor] == 0 or grid[neighbor] == 5 or grid[neighbor] == 3:

				evaluation_map[neighbor] = 1
				# TODO: Update the score in the following maps
				#	 (1) evaluation_map[neighbor]
				#	 (2) distance_map[neighbor]:  update distance using distance_map[current_node]
				#	 (3) parent_map[neighbor]:    set to current node
				# 	 (4) grid[neighbor]: 	      set value to 3
				# 	 Update criteria: new evaluation_map[enighbor] value is decreased		 
	if np.isinf(evaluation_map[goal_node]):
		route = []
		print("No route found.")
	else:
		route = [goal_node]
		while parent_map[route[0][0]][route[0][1]] != ():
			break
			# TODO: find the optimal route based on your exploration result
			#       Useful functions: 
			#	(1) route.insert(index, element): to add new node to route
			#	(2) parent_map[index1][index2]: find the parent of the node at grid coordinates (index1,index2)
			# 	(3) route[0][0], route[0][1] to access the grid coordinates of a node
			# route.insert(something...)
		print("The route found covers %d grid cells." % len(route))
	return route


def find_neighbors(i, j, M):
	neighbors = []

	# TODO: add the four neighbor nodes to the neighbor list
	# 	 grid index: i goes from 0 to M-1 (M possible values)
	# 	 grid index: j goes from 0 to M-1 (M possible values)

	# at i=0, theta1 = -pi
	# at j=0, theta2 = -pi
	# at i=M-1, theta1 = pi
	# at j=M-1, theta2 = pi
	p1 = (i-1,j)
	p2 = (i,j-1)
	p3 = (i+1,j)
	p4 = (i,j+1)
	if not check_range(p1):
		neighbors.append(p1)
	if not check_range(p2):
		neighbors.append(p2)
	if not check_range(p3):
		neighbors.append(p3)
	if not check_range(p4):
		neighbors.append(p4)
	# So be aware of the rounding effect in finding the neighbors at border nodes.
	# Useful function: neighbors.append((index1, index2)), where index1, index2 are the grid indices of it's neighbors
	return neighbors

def check_range(p):
	def out_of_range(x):
		return x <= 99 and x >= 0
	return not all(map(out_of_range, p))

def calc_heuristic_map(M, goal_node):
	X, Y = np.meshgrid([i for i in range(M)], [i for i in range(M)])
	heuristic_map = np.abs(X - goal_node[1]) + np.abs(Y - goal_node[0])
	for i in range(heuristic_map.shape[0]):
		for j in range(heuristic_map.shape[1]):
			heuristic_map[i, j] = min(heuristic_map[i, j],
			i + 1 + heuristic_map[M - 1, j],
			M - i + heuristic_map[0, j],
			j + 1 + heuristic_map[i, M - 1],
			M - j + heuristic_map[i, 0]
			)
	return heuristic_map

def simplify_angle(angle):
	if angle > pi:
		angle -= 2*pi
	elif angle < -pi:
		angle += 2*pi
	return angle

if __name__ == '__main__':
	main()
	# plt.imshow(new)



