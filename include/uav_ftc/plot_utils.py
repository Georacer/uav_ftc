import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot3_points(point_array):
    x = point_array[:,0]
    y = point_array[:,1]
    z = point_array[:,2]

    fig = plt.figure()
    ax = fig.add_subplot('111', projection='3d')
    ax.scatter(x, y, z, c='b', marker='o')

    plt.show()

