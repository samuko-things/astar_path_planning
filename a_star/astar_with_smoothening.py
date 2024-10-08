import matplotlib.pyplot as plt
import modules.ogmap as ogmap
from modules import astar
import modules.func as func
import time


show_animation = True




###################################################################################
def actual_to_grid_pos(actual_pos, map_resolution):
    return round(actual_pos/map_resolution)


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


    plt.plot(ox, oy, ".k")

    plt.plot(current_loc[x], current_loc[y], marker = '.', ms = 20, mfc = 'k')
    plt.plot(target_loc[x], target_loc[y], marker = '.', ms = 20, mfc = 'b')
    
    plt.grid(True)
    plt.axis("equal")


    a_star = astar.AStarPlanner(min_x, min_y, max_x, max_y, x_width, y_width, obstacle_map)
    rx, ry = a_star.planning(current_loc[x], current_loc[y], target_loc[x], target_loc[y])
    
    ax, ay = func.remove_redundant_path(rx, ry)
    astar_path_x, astar_path_y = func.reverse_list(ax), func.reverse_list(ay)

    print("waiting for path smoothening ...")
    optimized_path_x, optimized_path_y = func.smoothen_path(astar_path_x, astar_path_y, a_star.obstacle_map, a_star.min_x, a_star.min_y)

    plt.plot(ox, oy, ".k")

    plt.plot(current_loc[x], current_loc[y], "ok")
    plt.plot(target_loc[x], target_loc[y], "ob")
    
    plt.grid(True)
    plt.axis("equal")

    plt.plot(astar_path_x, astar_path_y, "-b")
    plt.plot(optimized_path_x, optimized_path_y, "-r")
    
    return optimized_path_x, optimized_path_y, astar_path_x, astar_path_y

###################################################################################














if __name__ == '__main__':

    start_time = time.time()

    map_image_file = "maps/test/floor_map_reduced.png"
    robot_radius_cm = 25 # in cm
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
    
    stop_time = time.time()

    run_time = stop_time-start_time
    print("run_time :",run_time)



    opt_path_coord, _, point_index = func.get_path_coord(optimized_path_x, optimized_path_y, map_resolution)
    opt_path_dist, opt_path_total_dist, _, _ = func.get_total_path_dist(optimized_path_x, optimized_path_y, map_resolution) 
    opt_path_angles = func.get_path_angles(optimized_path_x, optimized_path_y)

    # print(len(opt_path_coord))
    # print(len(opt_path_dist))
    # print(len(opt_path_coord))

    # print("optimized distance :",opt_path_dist*map_resolution, "cm")
    print("total_dist :", opt_path_total_dist*map_resolution, "cm")
    # print("optimized path_angles :", opt_path_angles)



    # astar_path_coord = func.get_path_coord(astar_path_x, astar_path_y)
    # astar_path_dist, astar_path_total_dist = func.get_total_path_dist(astar_path_x, astar_path_y)
    # astar_path_angles = func.get_path_angles(astar_path_x, astar_path_y)

    # print(len(astar_path_coord))
    # print(len(astar_path_dist))
    # print(len(astar_path_coord))

    # print("un_optimized distance :", astar_path_dist*map_resolution, "cm")
    # print("total_dist :", astar_path_total_dist*map_resolution, "cm")
    # print("un_optimized path_angles :", astar_path_angles)



    # dist_diff = astar_path_dist-opt_path_dist
    # print("difference :",diff*map_resolution, "cm")
    
    func.print_data(point_index, opt_path_coord, opt_path_dist, opt_path_angles)
    
    # show planned map
    plt.show()