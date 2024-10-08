import math
import pandas as pd

'''
this contains functions for path optimization/smoothening algorithm, 
loading maps and adding obstacles in polygon form to map to complement the astar algorithm

author: Obiagba Samuel

references: 
    bresenham line algorithm:http://floppsie.comp.glam.ac.uk/Glamorgan/gaius/gametools/6.html
'''



#### bresenhams line algorithm ##########################################
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
#######################################################################


###### this checks if a line crosses any obstacle or wall or boundary #######
def cross_obstacle(lx, ly, obstacle_map, x_min, y_min):
    '''
    '''
    length = len(lx)
    obstacle = False

    for i in range(0,length):
        if obstacle_map[int(lx[i]-x_min)][int(ly[i]-y_min)]:
            obstacle = True
            break
    
    return obstacle
#############################################################################


################################################################################
def smoothen_path(path_x, path_y, obstacle_map, x_min, y_min, grid_size=1):
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

    x_prev = x_init
    y_prev = x_init
    x_next = x_init
    y_next = x_init

    if length == 1:
        return new_path_x, new_path_y
    
    else:
        while index < length-1:
            x_next = path_x[index]
            y_next = path_y[index]

            lx,ly = get_line([x_init, y_init], [x_next, y_next])
            
            touch_obstacle = cross_obstacle(lx,ly,obstacle_map,x_min,y_min)
            
            if touch_obstacle:
                new_path_x.append(x_prev)
                new_path_y.append(y_prev)
                x_init = x_prev
                y_init = y_prev
                # x_init = path_x[i-1]
                # y_init = path_y[i-1]
                # index -= 1
                # print(x_init, y_init)
                # break   
            else:
                x_prev = x_next
                y_prev = y_next
                index += 1

            # print(index)   
            
        new_path_x.append(x_goal)
        new_path_y.append(y_goal)  
        return new_path_x, new_path_y
#################################################################################


#################################################################################

def draw_polygon(px, py):
    p_x, p_y = [], []

    loop_length = len(px)
    for i in range(0, loop_length):
        j = (i+1)%loop_length
        points_x, points_y = get_line([px[i], py[i]], [px[j], py[j]])
        for i in range(0,len(points_x)):
            p_x.append(points_x[i])
            p_y.append(points_y[i])
        # add_shape_to_obstacle(points_x, points_y)

    return p_x, p_y

##################################################################################


##################################################################################

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




def remove_redundant_path(path_x, path_y):
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

########################################################################################

'''
end of path optimization/smoothening algorithm, loading maps and adding obstacles in polygon form to the map
'''


























########################################################################################
def calc_grid_position(index, min_position):
    pos = index + min_position
    return pos


def calc_obstacle_map(ox, oy, robot_radius_in_grid, obstacle_map):
    obs_map = obstacle_map

    min_x = round(min(ox)) - round(robot_radius_in_grid) - 1
    min_y = round(min(oy)) - round(robot_radius_in_grid) - 1
    max_x = round(max(ox)) + round(robot_radius_in_grid) + 1
    max_y = round(max(oy)) + round(robot_radius_in_grid) + 1

    # print(min_x, min_y, max_x, max_y)

    x_width = round(max_x - min_x)
    y_width = round(max_y - min_y)

    # obstacle map generation
    
    for ix in range(x_width):
        x = calc_grid_position(ix, min_x)
        for iy in range(y_width):
            y = calc_grid_position(iy, min_y)
            for iox, ioy in zip(ox, oy):
                d = math.hypot(iox - x, ioy - y)
                if d <= robot_radius_in_grid:
                    obs_map[x][y] = True
                    break

    return obs_map
########################################################################################




























###################################################################################
# functions for path analysis
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


def print_data(path_index, coord, distance, angle):
    df = pd.DataFrame({
        "POINTS":path_index,
        "COORDINATE":coord,
        "DISTANCE":distance,
        "ANGLE":angle })
    
    filename = "my_data.csv"
    df.to_csv(filename, index=False)
    print(df)

###################################################################################