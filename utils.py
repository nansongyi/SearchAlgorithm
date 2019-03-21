import numpy as np
from math import *
from matplotlib.colors import from_levels_and_colors
import collections
import heapq
from matplotlib import pyplot as plt


def simple_show(grid):
    colors = ['white', 'black', 'red', 'pink', 'yellow', 'green', 'orange']
    levels = [0, 1, 2, 3, 4, 5, 6, 7]
    cmap, norm = from_levels_and_colors(levels, colors)
    plt.imshow(grid, cmap=cmap, norm=norm, interpolation=None)
    plt.show()

def map_gen(small_grid):
    M = 100
    grid = np.zeros([3*M, 3*M])
    for i in range(9):
        row, col = i/3, i%3
        print(row,col)
        grid[row*M:(row+1)*M, col*M:(col+1)*M] = small_grid
    # grid[10:13,50:53] = np.array([[2,2,2],[2,2,2],[2,2,2]])
    return grid

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
        M = 100
        i, j = currunt
        ileft = i-1
        iright = i+1
        jlow = j-1
        jhigh = j+1
        if (i == 0):
            ileft = M-1
        if (i == M-1):
            iright=0
        if (j==0):
            jlow = M-1
        if (j==M-1):
            jhigh=0
        p1, p2, p3, p4 = (ileft,j), (iright,j), (i,jhigh), (i,jlow)
        if self.grid[p1] == 0:
            neighbors.append(p1)
        if self.grid[p2] == 0:
            neighbors.append(p2)
        if self.grid[p3] == 0:
            neighbors.append(p3)
        if self.grid[p4] == 0:
            neighbors.append(p4)
        return neighbors

    def compute_heuristic(self, config):
        return np.abs(self.goal[0] - config[1]) + np.abs(self.goal[1] - config[0])

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

def detect_collision(line_seg, circle):
    # TODO: Return True if the line segment is intersecting with the circle
    #       Otherwise return false.
    #    (1) line_seg[0][0], line_seg[0][1] is one point (x,y) of the line segment
    #    (2) line_seg[1][0], line_seg[1][1] is another point (x,y) of the line segment
    #    (3) circle[0],circle[1],circle[2]: the x,y of the circle center and the circle radius
    #    Hint: the closest point on the line segment should be greater than radius to be collision free.
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
            #    Find the occupacy status of each robot pose.
            #    Useful functions/variables: arm.update_joints(), arm.points, theta_list[index]
            goal_joint_angles = np.array([theta_list[i], theta_list[j]])
            arm.update_joints(goal_joint_angles)
            points = arm.points

            collision_detected = False
            for k in range(len(points) - 1):
                for obstacle in obstacles:
                    # TODO: define line_seg and detect collisions
                    #    Useful functions/variables: detect_collision(), points[index]
                    line_seg = [points[k],points[k+1]]
                    collision_detected = detect_collision(line_seg, obstacle)
                    if collision_detected:
                        break
                if collision_detected:
                    break
            grid[i][j] = int(collision_detected)
    return np.array(grid)

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


def simplify_angle(angle):
    if angle > pi:
        angle -= 2*pi
    elif angle < -pi:
        angle += 2*pi
    return angle



    
