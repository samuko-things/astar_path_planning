
import math
import matplotlib.pyplot as plt
import numpy as np
import modules.ogmap as ogmap   






def display_map(bx, by):
    plt.plot(bx, by, ".k")
    plt.show()






# this lists holds the location of the boundaries and the static obstacles of the grid map
bx = []
by = []

filename = "/home/samuko/python_dev/path_planning/maps/test/simple_house_map.pgm"

ogm_data = ogmap.png_to_ogm(filename, normalized=True)
ogm_data_arr = np.array(ogm_data)

# this converts the data to only zeros and ones
threshold = 0.4

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




display_map(bx, by)