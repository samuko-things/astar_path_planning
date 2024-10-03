"""

Bidirectional A* grid planning

author: Erwin Lejeune (@spida_rwin)

See Wikipedia article (https://en.wikipedia.org/wiki/Bidirectional_search)

"""

import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import time
import ogmap

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)




    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)



    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # # show graph
            # if show_animation:  # pragma: no cover
            #     plt.plot(self.calc_grid_position(current.x, self.min_x),
            #              self.calc_grid_position(current.y, self.min_y), "xc")
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect('key_release_event',
            #                                  lambda event: [exit(
            #                                      0) if event.key == 'escape' else None])
            #     if len(closed_set.keys()) % 10 == 0:
            #         plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry



    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry



    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d



    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos



    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)



    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)



    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True



    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break



    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion












# this lists holds the location of the boundaries and the static obstacles of the grid map
bx = []
by = []



def tri_angle(p0, p1, p2):
    a = math.hypot(p0[0]-p1[0], p0[1]-p1[1])
    b = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
    c = math.hypot(p0[0]-p2[0], p0[1]-p2[1])

    angle = (a**2 + b**2 - c**2)/(2*a*b)
    if angle>1:
        angle = 1
    elif angle < -1:
        angle = -1
    angle = math.degrees(math.acos(angle))

    return round(angle)


def a_star_path(path_x, path_y):
    px = path_x
    py = path_y

    i = 0

    if len(px)<=2:
        return px,py

    while (i+2)<len(px):
        angle = tri_angle([px[i], py[i]], [px[i+1], py[i+1]], [px[i+2], py[i+2]])

        if angle==180:
            px.pop(i+1)
            py.pop(i+1)
        else:
            i+=1
    
    return px,py



def reverse_list(mylist):
    order = [-1*i for i in range(1, len(mylist)+1) ]
    reordered_list = [mylist[i] for i in order]
    return reordered_list



def get_pos_val(val):
    return round(val)


def get_line (p0, p1):
    px = []
    py = []
    m = 0
    c = 0

    x0 = get_pos_val(p0[0])
    y0 = get_pos_val(p0[1])
    x1 = get_pos_val(p1[0])
    y1 = get_pos_val(p1[1])

    dx = x1-x0
    dy = y1-y0

    if x0 < x1:
        sx = 1
    else:
        sx = -1

    if y0 < y1:
        sy = 1
    else:
        sy = -1


    if abs(dy) < abs(dx):
        try:
            m = dy/dx
        except ZeroDivisionError:
            m=0
        c = y0-(m*x0)

        px.append(x0)
        py.append(y0)
        while x0 != x1:   
            x0+=sx
            y0=get_pos_val(((m*x0) + c))
            px.append(x0)
            py.append(y0)


    else:
        try:
            m = dx/dy
        except ZeroDivisionError:
            m=0
        c = x0-(m*y0)

        px.append(x0)
        py.append(y0)
        while y0 != y1:   
            y0+=sy
            x0=get_pos_val(((m*y0) + c))
            px.append(x0)
            py.append(y0)

    return px,py



def add_shape_to_obstacle(px, py):
    global bx
    global by
    
    for i in range(0,len(px)):
        bx.append(px[i])
        by.append(py[i])
    
def draw_polygon(px, py):
    p_x, p_y = [], []

    loop_length = len(px)
    for i in range(0, loop_length):
        j = (i+1)%loop_length
        points_x, points_y = get_line([px[i], py[i]], [px[j], py[j]])
        for i in range(0,len(points_x)):
            p_x.append(points_x[i])
            p_y.append(points_y[i])
        add_shape_to_obstacle(points_x, points_y)

    return p_x, p_y








# functions for path analysis ###################################################################
def get_path_coord(path_x, path_y, res):
    actual_path_coord = []
    grid_path_coord = []
    path_index = []
    for i in range(0, len(path_x)):
        grid_path_coord.append( (path_x[i], path_y[i]) )
        actual_path_coord.append( (path_x[i]*res, path_y[i]*res) )
        path_index.append(i+1)
    
    return actual_path_coord, grid_path_coord, path_index


def get_total_path_dist(path_x, path_y, res):
    actual_total_dist = 0
    grid_total_dist = 0

    actual_path_dist = []
    grid_path_dist = []

    actual_path_dist.append(0)
    grid_path_dist.append(0)

    for i in range(1, len(path_x)):
        dist = math.hypot(path_x[i]-path_x[i-1], path_y[i]-path_y[i-1])
        grid_total_dist += dist
        grid_path_dist.append(round(dist,2))

        dist = dist*res
        actual_total_dist += dist
        actual_path_dist.append(round(dist,2))

    return actual_path_dist, round(actual_total_dist, 2), grid_path_dist, round(grid_total_dist, 2)


def get_path_angles(path_x, path_y):
    path_angle = []
    path_angle.append(0)
    i = 0

    if len(path_x)<=2:
        path_angle.append(180)
        return path_angle

    while (i+2)<len(path_x):
        angle = tri_angle([path_x[i], path_y[i]], [path_x[i+1], path_y[i+1]], [path_x[i+2], path_y[i+2]])
        path_angle.append(angle)
        i+=1
    
    path_angle.append(0)

    return path_angle


def print_data(point_index, coord, distance, angle):
    df = pd.DataFrame({
        "POINTS":point_index,
        "COORDINATES":coord,
        "DISTANCE":distance,
        "ANGLE":angle })
    
    filename = "my_data_1.csv"
    df.to_csv(filename, index=False)
    print(df)
####################################################################################################









########################################################################################################################

map_filename = "maps/main/mech_building_reduced.png"

ogm_data = ogmap.png_to_ogm(map_filename, normalized=True)
ogm_data_arr = np.array(ogm_data)

# this converts the data to only zeros and ones
threshold = 0.9

for id, data in np.ndenumerate(ogm_data_arr):
    if data < threshold:
        ogm_data_arr[id[0]][id[1]] = 0
    else:
        ogm_data_arr[id[0]][id[1]] = 1


for id, data in np.ndenumerate(ogm_data_arr):
    if data == 0:
        by.append(id[0])
        bx.append(id[1])
    else:
        pass

##################################################################################












def main():
    global bx
    global by

    start_time = time.time()
    # start and goal position
    sx, sy = 27, 284
    gx, gy = 164, 45

    grid_size = 1  
    robot_radius = 50 # cm 
    map_resolution = 10
    robot_radius_grid = robot_radius/map_resolution

    start = [sx, sy]
    goal = [gx, gy]


    ###############################################################
    # px = [15, 15, 22, 22] # x_cordinates of the polygon
    # py = [48, 55, 55, 48] # y_cordinates of the polygon

    # draw_polygon(px, py, grid_size)
    ###############################################################


    # set obstacle positions
    ox, oy = bx, by

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        # plt.plot(sx, sy, "og")
        # plt.plot(gx, gy, "ob")

        plt.plot(start[0], start[1], "ok")
        plt.plot(goal[0], goal[1], "ob")
        
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius_grid)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    
    ax,ay = a_star_path(rx,ry)
    ax, ay = reverse_list(ax), reverse_list(ay)

    stop_time = time.time()

    runtime = stop_time-start_time
    print(runtime, "sec")

    path_coord, _, point_index = get_path_coord(ax, ay, map_resolution)
    path_dist, path_total_dist, _, _ = get_total_path_dist(ax, ay, map_resolution) 
    path_angles = get_path_angles(ax, ay)

    # print(len(path_coord))
    # print(len(path_dist))
    # print(len(path_coord))

    # print("optimized distance :",opt_path_dist*map_resolution, "cm")
    print("total_dist :", path_total_dist*map_resolution, "cm")
    # print("optimized path_angles :", opt_path_angles)

    
    print_data(point_index, path_coord, path_dist, path_angles)

    # # print(ax)
    # # print(ay)

    if show_animation:  # pragma: no cover
        plt.plot(ax, ay, "-b")
        # plt.scatter(ax, ay, c="b", marker="o")
        plt.pause(.0001)
        plt.show()


if __name__ == '__main__':
    main()
