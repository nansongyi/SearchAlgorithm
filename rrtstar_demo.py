from NLinkArm import *
from utils import *
from Astar import *
from matplotlib import pyplot as plt
from MapEnvironment import MapEnvironment
from RRTPlanner import RRTPlanner
from RRTStarPlanner import RRTStarPlanner
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
    small_grid = get_occupancy_grid(arm, obstacles, M)
    grid = map_gen(small_grid)
    # grid = small_grid
    # grid[0:3,0:3] = np.array([[2,2,2],[2,2,2],[2,2,2]])

    # simple_show(grid)
    print("\nstep 1: robot and environment ready.")
    # return

    while not goal_found:
         # step 2: set start and random goal
        start = angle_to_grid_index(initial_link_angle,M)
        goal = (randint(0,M-1),randint(0,M-1)) 
        start, goal = (40+100, 95+100), (40+100, 50+100)
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
        planning_env = MapEnvironment(grid, start, goal)
        planner = RRTStarPlanner(planning_env)

        start_time = time.time()
        pre_route = planner.Plan(start, goal)
        time_elapsed = time.time() - start_time
        print('time_elapsed:',time_elapsed)

        planner.visualize_plan(planner.name, pre_route, planner.tree)
        route = []
        # round our route to int type for animating 
        for item in pre_route:
            x,y = item
            x_int, y_int = int(x), int(y)
            route.append((x_int, y_int))
        print(route)

        print("\nstep 3: motion planning completed.")

        # step 4: visualize result  
        print("\nstep 4: start visualization.")
        if len(route) >= 0:
            animate(grid, arm, route, obstacles, M, start_pos, goal_pos, start, goal)

        print("\nGoal reached.")

if __name__ == '__main__':
    main()



