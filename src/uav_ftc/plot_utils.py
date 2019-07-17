from math import sqrt, atan2, asin, cos, pi
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D


def plot3_points(point_array, axes_names=['x', 'y', 'z']):
    x = point_array[:, 0]
    y = point_array[:, 1]
    z = point_array[:, 2]

    fig = plt.figure()
    ax = fig.add_subplot('111', projection='3d')
    ax.set_proj_type('ortho')
    ax.scatter(x, y, z, c='b', marker='o')

    ax.set_xlabel(axes_names[0])
    ax.set_ylabel(axes_names[1])
    ax.set_zlabel(axes_names[2])

    plt.show()


def get_airdata(u, v, w):
    airspeed = np.sqrt(u**2+v**2+w**2)
    alpha = np.arctan2(w, u)
    if u == 0:
        if v == 0:
            beta = 0
        else:
            beta = np.arcsin(v/np.abs(v))
    else:
        beta = np.arctan2(v, u)

    return (airspeed, alpha, beta)


def get_turn_radius(va, psi_dot, gamma):
    return va/psi_dot*np.cos(gamma)


def quat2euler2(x, y, z, w):
    q01 = w*x
    q02 = w*y
    q03 = w*z
    q11 = x*x
    q12 = x*y
    q13 = x*z
    q22 = y*y
    q23 = y*z
    q33 = z*z
    psi = atan2(2.0 * (q03 + q12), 1.0 - 2.0 * (q22 - q33))
    if psi < 0.0:
        psi += 2.0*pi

    theta = asin(2.0 * (q02 - q13))
    phi = atan2(2.0 * (q01 + q23), 1.0 - 2.0 * (q11 + q22))

    return (phi, theta, psi)


def rmse(x, y):
    return np.sqrt(np.mean(np.power(x-y, 2)))


def plot_points(ah, point_arr, style, color):
    x = point_arr[0, :]
    y = point_arr[1, :]
    ah.scatter(x, y, marker=style, c=color)


def plot_points_3(ah, point_arr, style, color, alpha=1):
    x = point_arr[0, :]
    y = point_arr[1, :]
    z = point_arr[2, :]
    ah.scatter(x, y, z, marker=style, c=color, alpha=alpha)


def plot_line(ah, point_1, point_2, color):
    ah.plot([point_1[0], point_2[0]], [point_1[1], point_2[1]], c=color)


def plot_polygons_3(ah, face_points, colorcode=None, alpha=None):
    # Select the plane color
    if colorcode == 'r':
        plane_color = [0.3, 0, 0]
    else:
        plane_color = [0, 1, 0.2]

    poly_collection = mplot3d.art3d.Poly3DCollection(face_points)
    poly_collection.set_alpha = alpha
    poly_collection.set_facecolor(plane_color)
    poly_collection.set_edgecolor('k')
    ah.add_collection3d(poly_collection)
