from NLinkArm import *
from utils import *
from Astar import *
from matplotlib import pyplot as plt
import time

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
		graph  = Graph(grid, start, goal)
		start_time = time.time()
		came_from, cost_so_far = a_star_search(graph, start, goal)
		route = graph.reconstruct_path(came_from, start, goal)
		time_elapsed = time.time() - start_time
		print('time_elapsed:',time_elapsed)
		# route = astar_search(grid, start, goal, M)
		print("\nstep 3: motion planning completed.")

		# step 4: visualize result	
		print("\nstep 4: start visualization.")
		if len(route) >= 0:
			animate(grid, arm, route, obstacles, M, start_pos, goal_pos, start, goal)

		print("\nGoal reached.")

if __name__ == '__main__':
	main()



