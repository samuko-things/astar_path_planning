import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import modules.ogmap as ogmap
import modules.astar as astar
import modules.func as func
import modules.cubic_spline_planner as csp
import time





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
        angle = func.tri_angle([path_x[i], path_y[i]], [path_x[i+1], path_y[i+1]], [path_x[i+2], path_y[i+2]])
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
####################################################################################################












def actual_to_grid_pos(actual_pos, map_resolution):
    return round(actual_pos/map_resolution)






def create_spline(x, y):  # pragma: no cover

    # x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    # y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    ds = 0.1  # [m] distance of each intepolated points

    sp = csp.Spline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    if astar.show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-g")
        plt.pause(0.01)






def plan_and_smoothen_path(map_image_file, robot_radius_cm, map_resolution, current_loc, target_loc, obs_pos=()):
    x, y = 0, 1

    robot_radius_in_grid = robot_radius_cm/map_resolution
    new_map = ogmap.get_obs_map(map_image_file, robot_radius_cm, map_resolution)

    min_x = new_map['min_x']
    min_y = new_map['min_y']
    max_x = new_map['max_x']
    max_y = new_map['max_y']
    x_width = new_map['x_width']
    y_width = new_map['y_width']
    ox = new_map['ox']
    oy = new_map['oy']
    obstacle_map = new_map['map']


    ###############################################################
    ##### adding obstacle to the created map ######################
    ###############################################################
    if len(obs_pos) != 0:
    
        for loc in range(0, len(obs_pos)):
            polygon_vertices_x = obs_pos[loc][x] # x_cordinates of the polygon
            polygon_vertices_y = obs_pos[loc][y] # y_cordinates of the polygon
            new_ox, new_oy = func.draw_polygon(polygon_vertices_x, polygon_vertices_y)
            obstacle_map = func.calc_obstacle_map(new_ox, new_oy, robot_radius_in_grid, obstacle_map)
            for i in range(0,len(new_ox)):
                ox.append(new_ox[i])
                oy.append(new_oy[i])
            pass
        

    # print map info
    print("MAP INFORMATION")
    print("x_width:", x_width, "grid cells")
    print("y_width:", y_width, "grid cells")
    print("map resolution:", map_resolution, "cm/grid")
    print("actual_x_width:", x_width*map_resolution, "cm")
    print("actual_y_width:", y_width*map_resolution, "cm")
    print()

    a_star = astar.AStarPlanner(min_x, min_y, max_x, max_y, x_width, y_width, obstacle_map)
    rx, ry = a_star.planning(current_loc[x], current_loc[y], target_loc[x], target_loc[y])
    rx = func.reverse_list(rx)
    ry = func.reverse_list(ry)

    astar_path_x, astar_path_y = func.a_star_path(rx, ry) 

    optimized_path_x, optimized_path_y = func.smoothen_path(rx, ry, a_star.obstacle_map, a_star.min_x, a_star.min_y)
    optimized_path_x, optimized_path_y = func.smoothen_path(optimized_path_x, optimized_path_y, a_star.obstacle_map, a_star.min_x, a_star.min_y)
    optimized_path_x, optimized_path_y = func.smoothen_path(optimized_path_x, optimized_path_y, a_star.obstacle_map, a_star.min_x, a_star.min_y)
    

    # Rx,Ry,ryaw,rk = create_spline(astar_path_x, astar_path_y)
    

    if astar.show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")

        plt.plot(current_loc[x], current_loc[y], "ok")
        plt.plot(target_loc[x], target_loc[y], "ob")
        
        plt.grid(True)
        plt.axis("equal")
    
        # plt.plot(astar_path_x, astar_path_y, "-b")
        plt.plot(optimized_path_x, optimized_path_y, "-g")
        # plt.plot(Rx, Ry, "-g")
        plt.scatter(optimized_path_x, optimized_path_y, c="r", marker="o")
        plt.pause(0.01)

    
    return optimized_path_x, optimized_path_y, astar_path_x, astar_path_y

















if __name__ == '__main__':

    map_image_file = "maps/test/floor_map_reduced.png"
    robot_radius_cm = 25 # in cm
    map_resolution = 10 # cm per grid based on the map

    # start and goal grid position and not the actual pos in cm
    sx, sy = 40, 75
    gx, gy = 90, 70

    # # start and goal actual position in cm
    # sx, sy = 120, 3080
    # gx, gy = 1650, 160
    # sx, sy, gx, gy = actual_to_grid_pos(sx, sy, gx, gy, map_resolution)


    start = [sx, sy]
    goal = [gx, gy]

    

    # this part enables addition of obstacles to the map giving the obsatcle coordinate vertices
    # one can add one or more obstacles to the map

    # px1 = [20, 40, 5] # x_cordinates of the polygon
    # py1 = [300, 300, 277] # y_cordinates of the polygon
    
    # px2 = [140, 160, 160, 140]# x_cordinates of the polygon
    # py2 = [50, 50, 30, 30] # y_cordinates of the polygon

    # obs_positions = ( (px1, py1) ) # single obstacles
    # obs_positions = ( (px1, py1), (px2, py2) ) # multiple obstacles
    # ... = plan_and_smoothen_path(map_image_file, robot_radius_cm, map_resolution, start, goal, obs_positions)



    # plans and gets the generated path
    
    optimized_path_x, optimized_path_y, astar_path_x, astar_path_y = plan_and_smoothen_path(map_image_file, robot_radius_cm, map_resolution, start, goal)
   

    opt_path_coord, _, point_index = get_path_coord(optimized_path_x, optimized_path_y, map_resolution)
    opt_path_dist, opt_path_total_dist, _, _ = get_total_path_dist(optimized_path_x, optimized_path_y, map_resolution) 
    opt_path_angles = get_path_angles(optimized_path_x, optimized_path_y)

    # create_spline(astar_path_x, astar_path_y)
    # create_spline(optimized_path_x, optimized_path_y)

    # print(len(opt_path_coord))
    # print(len(opt_path_dist))
    # print(len(opt_path_coord))
    
    # show planned map
    plt.show()