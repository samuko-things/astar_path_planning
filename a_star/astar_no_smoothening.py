import matplotlib.pyplot as plt
import modules.ogmap as ogmap
from modules import astar
import modules.func as func
import time


show_animation = True



###################################################################################
def actual_to_grid_pos(actual_pos, map_resolution):
    return round(actual_pos/map_resolution)


def plan_path(map_image_file, robot_radius_cm, map_resolution, current_loc, target_loc, obs_pos=()):
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


    plt.plot(ox, oy, ".k")

    plt.plot(current_loc[x], current_loc[y], marker = '.', ms = 20, mfc = 'k')
    plt.plot(target_loc[x], target_loc[y], marker = '.', ms = 20, mfc = 'b')
    
    plt.grid(True)
    plt.axis("equal")


    a_star = astar.AStarPlanner(min_x, min_y, max_x, max_y, x_width, y_width, obstacle_map)
    rx, ry = a_star.planning(current_loc[x], current_loc[y], target_loc[x], target_loc[y])
    
    ax, ay = func.remove_redundant_path(rx, ry)
    astar_path_x, astar_path_y = func.reverse_list(ax), func.reverse_list(ay)


    plt.plot(ox, oy, ".k")

    plt.plot(current_loc[x], current_loc[y], "ok")
    plt.plot(target_loc[x], target_loc[y], "ob")
    
    plt.grid(True)
    plt.axis("equal")

    plt.plot(astar_path_x, astar_path_y, "-b")
    plt.pause(0.01)

    
    return astar_path_x, astar_path_y

###################################################################################














if __name__ == '__main__':

    start_time = time.time()

    map_image_file = "maps/test/floor_map_reduced.png"
    robot_radius_cm = 15 # in cm
    map_resolution = 10 # cm per grid based on the map

    # start and goal grid position and not the actual pos in cm
    # sx, sy = 42, 75
    # gx, gy = 94, 13

    sx, sy = 15, 15
    gx, gy = 90, 15

    # # start and goal actual position in cm
    # sx, sy = 120, 3080
    # gx, gy = 1650, 160
    # sx, sy, gx, gy = actual_to_grid_pos(sx, sy, gx, gy, map_resolution)


    start = [sx, sy]
    goal = [gx, gy]



    # plans and gets the generated path
    
    px, py = plan_path(map_image_file, robot_radius_cm, map_resolution, start, goal)
    
    stop_time = time.time()

    run_time = stop_time-start_time
    print("run_time :",run_time)



    path_coord, _, point_index = func.get_path_coord(px, py, map_resolution)
    path_dist, path_total_dist, _, _ = func.get_total_path_dist(px, py, map_resolution) 
    path_angles = func.get_path_angles(px, py)

    # print(len(path_coord))
    # print(len(path_dist))
    # print(len(path_coord))

    print("total_dist :", path_total_dist*map_resolution, "cm")

    
    func.print_data(point_index, path_coord, path_dist, path_angles)
    
    # show planned map
    plt.show()