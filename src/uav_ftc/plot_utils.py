import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot3_points(point_array, axes_names=['x', 'y', 'z']):
    x = point_array[:,0]
    y = point_array[:,1]
    z = point_array[:,2]

    fig = plt.figure()
    ax = fig.add_subplot('111', projection='3d')
    ax.set_proj_type('ortho')
    ax.scatter(x, y, z, c='b', marker='o')

    ax.set_xlabel(axes_names[0])
    ax.set_ylabel(axes_names[1])
    ax.set_zlabel(axes_names[2])

    plt.show()

