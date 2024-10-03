
import png
import math
import pickle
import numpy as np
import matplotlib.pyplot as plt
import os





'''
units are in centimeter [cm], grid
map_resolution is in cm/grid
'''





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






"""
this part is gotten from the actual A* grid planning code by
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
"""

def calc_grid_position(index, min_position):
    pos = index + min_position
    return pos


def calc_obstacle_map(ox, oy, robot_radius_in_grid):

    min_x = round(min(ox))
    min_y = round(min(oy))
    max_x = round(max(ox))
    max_y = round(max(oy))

    x_width = round(max_x - min_x)
    y_width = round(max_y - min_y)

    # obstacle map generation
    obstacle_map = [[False for _ in range(y_width)]
                            for _ in range(x_width)]
    for ix in range(x_width):
        x = calc_grid_position(ix, min_x)
        for iy in range(y_width):
            y = calc_grid_position(iy, min_y)
            for iox, ioy in zip(ox, oy):
                d = math.hypot(iox - x, ioy - y)
                if d <= robot_radius_in_grid:
                    obstacle_map[ix][iy] = True
                    break
    

    return min_x, min_y, max_x, max_y, x_width, y_width, obstacle_map

"""
################################################################################
"""








'''
the following functions generates an obstacle map for the astar algorithm
the obstacle map generated takes into account the occupancy_grid_map and robot radius so as to ensure path safety

authors: Olaniyi Taofeeq and Obiagba Samuel
'''




def delObsMap(pickle_filename):
    if os.path.exists(pickle_filename):
        os.remove(pickle_filename)
        print("file succefully removed")
    else:
        print("file does not exist")

def saveObsMap(ogm_map, pickle_filename):   
    # Its important to use binary mode 
    pickle_map_file = open(pickle_filename, 'ab') 
      
    # source, destination 
    pickle.dump(ogm_map, pickle_map_file)                      
    pickle_map_file.close() 


def loadObsMap(pickle_filename): 
    # for reading also binary mode is important 
    pickle_map_file = open(pickle_filename, 'rb')      
    obsMap = pickle.load(pickle_map_file) 
    pickle_map_file.close() 

    return obsMap


def change_name(map_filename, radius_cm):
    name = map_filename.split('.')
    name[0].strip()
    pickle_filename=name[0]+"_pickle_rr_"+str(radius_cm)

    return pickle_filename




def display_map(bx, by):
    plt.plot(bx, by, ".k")
    plt.show()


def create_new_pickle_map(pickle_filename, map_image_file, robot_radius_in_grid):
    bx = []
    by = []
    # this creates an occupancy grid map from the .png map image
    ogm_data = png_to_ogm(map_image_file, normalized=True)
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

    min_x, min_y, max_x, max_y, x_width, y_width, obstacle_map = calc_obstacle_map(bx, by, robot_radius_in_grid)

    # database 
    Map = {} 
    Map['min_x'] = min_x
    Map['min_y'] = min_y
    Map['max_x'] = max_x
    Map['max_y'] = max_y
    Map['x_width'] = x_width
    Map['y_width'] = y_width
    Map['ox'] = bx
    Map['oy'] = by
    Map['map'] = obstacle_map 

    saveObsMap(Map, pickle_filename)

    new_map = loadObsMap(pickle_filename)

    return new_map





def get_obs_map(map_image_file, robot_radius_cm, map_resolution):
    robot_radius_in_grid = robot_radius_cm/map_resolution
    
    pickle_filename = change_name(map_image_file, robot_radius_cm)

    if os.path.exists(pickle_filename):
        new_map = loadObsMap(pickle_filename)
        print(pickle_filename, " is opened")
        print()
        return new_map

    else:
        print("no pickle map file found")
        print("please wait, creating new pickle map file . . .")
        new_map = create_new_pickle_map(pickle_filename, map_image_file, robot_radius_in_grid)
        print(pickle_filename, "successfully created")
        print()
        return new_map
 


# if __name__ == '__main__':
#     map_image_file = "other_test_codes/path_planning/maps/test/test_map1_reduced.png"
#     robot_radius_cm = 20 # in cm
#     map_resolution = 10 # cm per grid based on the map

#     new_map = get_obs_map(map_image_file, robot_radius_cm, map_resolution)
#     display_map(new_map['ox'], new_map['oy'])