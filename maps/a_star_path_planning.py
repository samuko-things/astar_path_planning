"""

Bidirectional A* grid planning

author: Erwin Lejeune (@spida_rwin)

See Wikipedia article (https://en.wikipedia.org/wiki/Bidirectional_search)

"""

import math
import matplotlib.pyplot as plt
import numpy as np
import png


# ########################################################################
# import rospy
# from std_msgs.msg import String


# node_name = "a_star_planning"
# pub_topic = "delivery_robot/path"

# rospy.init_node(node_name, anonymous=True)
# pub = rospy.Publisher(pub_topic, String, queue_size=1000)


# def convert_2d_array_to_string(array_2d):
#     string = ""
#     l = len(array_2d[0])
#     for i in range(0,l):
#         string += str(array_2d[0][i])
#         if i != l-1:
#             string += ","
    
#     string += ";"

#     for i in range(0,l):
#         string += str(array_2d[1][i])
#         if i != l-1:
#             string += ","

#     return string

# def sendPath(path):
#     mystr = String()
#     mystr.data = convert_2d_array_to_string(path)
#     pub.publish(mystr)
#     rospy.loginfo(mystr.data)
# ##########################################################################################






######################################################################################################
def png_to_ogm(filename, normalized=False, origin='lower'):
    """
    Convert a png image to occupancy data.
    :param filename: the image filename
    :param normalized: whether the data should be normalised, i.e. to be in value range [0, 1]
    :param origin:
    :return:
    """
    r = png.Reader(filename)
    img = r.read()
    img_data = list(img[2])

    out_img = []
    bitdepth = img[3]['bitdepth']

    for i in range(len(img_data)):

        out_img_row = []

        for j in range(len(img_data[0])):
            if j % img[3]['planes'] == 0:
                if normalized:
                    out_img_row.append(img_data[i][j]*1.0/(2**bitdepth))
                else:
                    out_img_row.append(img_data[i][j])

        out_img.append(out_img_row)

    if origin == 'lower':
        out_img.reverse()

    return out_img
######################################################################################################












show_animation = True


class BidirectionalAStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x, self.min_y = None, None
        self.max_x, self.max_y = None, None
        self.x_width, self.y_width, self.obstacle_map = None, None, None
        self.resolution = resolution
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

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
        Bidirectional A star path search

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

        open_set_A, closed_set_A = dict(), dict()
        open_set_B, closed_set_B = dict(), dict()
        open_set_A[self.calc_grid_index(start_node)] = start_node
        open_set_B[self.calc_grid_index(goal_node)] = goal_node

        current_A = start_node
        current_B = goal_node
        meet_point_A, meet_point_B = None, None

        while 1:
            if len(open_set_A) == 0:
                print("Open set A is empty..")
                break

            if len(open_set_B) == 0:
                print("Open set B is empty..")
                break

            c_id_A = min(
                open_set_A,
                key=lambda o: self.find_total_cost(open_set_A, o, current_B))

            current_A = open_set_A[c_id_A]

            c_id_B = min(
                open_set_B,
                key=lambda o: self.find_total_cost(open_set_B, o, current_A))

            current_B = open_set_B[c_id_B]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current_A.x, self.min_x),
                         self.calc_grid_position(current_A.y, self.min_y),
                         "xc")
                plt.plot(self.calc_grid_position(current_B.x, self.min_x),
                         self.calc_grid_position(current_B.y, self.min_y),
                         "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set_A.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current_A.x == current_B.x and current_A.y == current_B.y:
                print("Found goal")
                meet_point_A = current_A
                meet_point_B = current_B
                break

            # Remove the item from the open set
            del open_set_A[c_id_A]
            del open_set_B[c_id_B]

            # Add it to the closed set
            closed_set_A[c_id_A] = current_A
            closed_set_B[c_id_B] = current_B

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):

                c_nodes = [self.Node(current_A.x + self.motion[i][0],
                                     current_A.y + self.motion[i][1],
                                     current_A.cost + self.motion[i][2],
                                     c_id_A),
                           self.Node(current_B.x + self.motion[i][0],
                                     current_B.y + self.motion[i][1],
                                     current_B.cost + self.motion[i][2],
                                     c_id_B)]

                n_ids = [self.calc_grid_index(c_nodes[0]),
                         self.calc_grid_index(c_nodes[1])]

                # If the node is not safe, do nothing
                continue_ = self.check_nodes_and_sets(c_nodes, closed_set_A,
                                                      closed_set_B, n_ids)

                if not continue_[0]:
                    if n_ids[0] not in open_set_A:
                        # discovered a new node
                        open_set_A[n_ids[0]] = c_nodes[0]
                    else:
                        if open_set_A[n_ids[0]].cost > c_nodes[0].cost:
                            # This path is the best until now. record it
                            open_set_A[n_ids[0]] = c_nodes[0]

                if not continue_[1]:
                    if n_ids[1] not in open_set_B:
                        # discovered a new node
                        open_set_B[n_ids[1]] = c_nodes[1]
                    else:
                        if open_set_B[n_ids[1]].cost > c_nodes[1].cost:
                            # This path is the best until now. record it
                            open_set_B[n_ids[1]] = c_nodes[1]

        rx, ry = self.calc_final_bidirectional_path(
            meet_point_A, meet_point_B, closed_set_A, closed_set_B)

        return rx, ry

    # takes two sets and two meeting nodes and return the optimal path
    def calc_final_bidirectional_path(self, n1, n2, setA, setB):
        rx_A, ry_A = self.calc_final_path(n1, setA)
        rx_B, ry_B = self.calc_final_path(n2, setB)

        rx_A.reverse()
        ry_A.reverse()

        rx = rx_A + rx_B
        ry = ry_A + ry_B

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def check_nodes_and_sets(self, c_nodes, closedSet_A, closedSet_B, n_ids):
        continue_ = [False, False]
        if not self.verify_node(c_nodes[0]) or n_ids[0] in closedSet_A:
            continue_[0] = True

        if not self.verify_node(c_nodes[1]) or n_ids[1] in closedSet_B:
            continue_[1] = True

        return continue_

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def find_total_cost(self, open_set, lambda_, n1):
        g_cost = open_set[lambda_].cost
        h_cost = self.calc_heuristic(n1, open_set[lambda_])
        f_cost = g_cost + h_cost
        return f_cost

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

############################################################################################



#### bresenhams line algorithm ##########################################
def get_pos_val(val, res):
    return round(val/res)*res

def get_line (p0, p1, grid_res):
    px = []
    py = []
    m = 0
    c = 0

    x0 = get_pos_val(p0[0], grid_res)
    y0 = get_pos_val(p0[1], grid_res)
    x1 = get_pos_val(p1[0], grid_res)
    y1 = get_pos_val(p1[1], grid_res)

    dx = x1-x0
    dy = y1-y0

    if x0 < x1:
        sx = grid_res
    else:
        sx = -1*grid_res

    if y0 < y1:
        sy = grid_res
    else:
        sy = -1*grid_res


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
            y0=get_pos_val(((m*x0) + c),grid_res)
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
            x0=get_pos_val(((m*y0) + c),grid_res)
            px.append(x0)
            py.append(y0)

    return px,py
#######################################################################


###### this checks if a line crosses any obstacle or wall or boundary #######
def cross_obstacle(lx, ly, obstacle_map, x_min, y_min, grid_res):
    '''
    '''
    length = len(lx)
    obstacle = False

    for i in range(0,length):
        if obstacle_map[int((lx[i]-x_min)/grid_res)][int((ly[i]-y_min)/grid_res)]:
            obstacle = True
            break
    
    return obstacle
#############################################################################



#################################################################################
def smoothen_path(path_x, path_y, grid_size, obstacle_map, x_min, y_min):
    new_path_x = []
    new_path_y = []

    length = len(path_x)
    touch_obstacle = False
    index = 1

    # get initial point
    x_init = path_x[0]
    y_init = path_y[0]

    x_goal = path_x[length-1]
    y_goal = path_y[length-1]

    new_path_x.append(x_init)
    new_path_y.append(y_init)

    x_next = x_init
    y_next = y_init
    x_prev = x_init
    y_prev = y_init

    if length == 1:
        return new_path_x, new_path_y
    
    else:
        while index < length-1:
            for i in range(index, length):
                x_next = path_x[i]
                y_next = path_y[i]

                lx,ly = get_line([x_init, y_init], [x_next, y_next], grid_size)
                
                touch_obstacle = cross_obstacle(lx,ly,obstacle_map,x_min,y_min,grid_size)
                # print(touch_obstacle)
                
                if touch_obstacle:
                    new_path_x.append(x_prev)
                    new_path_y.append(y_prev)
                    x_init = path_x[i-1]
                    y_init = path_y[i-1]
                    index -= 1
                    break   
                else:
                    x_prev = x_next
                    y_prev = y_next
                    index += 1
            
        new_path_x.append(x_goal)
        new_path_y.append(y_goal)  
        return new_path_x, new_path_y
#################################################################################


def optimize_path(path_x, path_y, grid_size, obstacle_map, x_min, y_min):
    optimized_path_x = []
    optimized_path_y = []

    ref_node_x = path_x[0]
    ref_node_y = path_y[0]
    next_node_x = 0
    next_node_y = 0

    index_pos = 1
    touch_obstacle = False

    list_len = len(path_x)



    if list_len == 2:
        optimized_path_x = path_x
        optimized_path_y = path_y

        return optimized_path_x, optimized_path_y



    optimized_path_x.append(ref_node_x)
    optimized_path_y.append(ref_node_y)

    while index_pos < (list_len-1):
        next_node_x = path_x[index_pos]
        next_node_y = path_y[index_pos]

        lx,ly = get_line([ref_node_x, ref_node_y], [next_node_x, next_node_y], grid_size)       
        touch_obstacle = cross_obstacle(lx,ly,obstacle_map,x_min,y_min,grid_size)

        if touch_obstacle == True:
            if index_pos == 1:
                pass
            else:
                ref_node_x = path_x[index_pos-1]
                ref_node_y = path_y[index_pos-1]

                optimized_path_x.append(ref_node_x)
                optimized_path_y.append(ref_node_y)

        else:
            index_pos+=1 
    
    optimized_path_x.append(path_x[list_len-1])
    optimized_path_y.append(path_y[list_len-1])

    return optimized_path_x, optimized_path_y



#####################################################################################
def add_shape_to_obstacle(px, py):
    global bx
    global by
    for i in range(0,len(px)):
        bx.append(px[i])
        by.append(py[i])
    
def draw_polygon(px,py, grid_res):
    loop_length = len(px)
    for i in range(0, loop_length):
        j = (i+1)%loop_length
        points_x, points_y = get_line([px[i], py[i]], [px[j], py[j]], grid_res)
        add_shape_to_obstacle(points_x, points_y)

##################################################################################

filename = "/home/samuko/ros_ws/src/robot_automatic_control/scripts/maps/ayedun_map_reduced.png"

ogm_data = png_to_ogm(filename, normalized=True)
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















def convert_path(path, resolution):
    px = []
    py = []

    for i in path[0]:
        px.append(resolution*i)
    
    for i in path[1]:
        py.append(resolution*i)

    return px, py





################### the main function #############################################

def main():
    global bx
    global by

    resolution = 10

    # start and goal grid position
    sx = 6
    sy = 6
    gx = 15
    gy = 30

    grid_size = 1   # grid size should not be adjust but left as equal to 1 
    robot_radius = 3    # robot radius can be ajusted  (1 grid_size = 10cm) 

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

        plt.plot(start[0], start[1], "ok")
        plt.plot(goal[0], goal[1], "ob")
        
        plt.grid(True)
        plt.axis("equal")

    bidir_a_star = BidirectionalAStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = bidir_a_star.planning(sx, sy, gx, gy)
    # print(rx)
    # print(ry)
    
    nx,ny = smoothen_path(rx,ry,grid_size,bidir_a_star.obstacle_map,bidir_a_star.min_x,bidir_a_star.min_y)
    # nx,ny = smoothen_path(nx,ny,grid_size,bidir_a_star.obstacle_map,bidir_a_star.min_x,bidir_a_star.min_y)
    print(nx)
    print(ny)

    # path = convert_path([nx,ny], resolution)
    # sendPath(path)
    # print(path)
    

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-g")
        plt.plot(nx, ny, "-r")
        plt.scatter(nx, ny, c="r", marker="o")
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
