#!python3

import itertools as it
import numpy as np
import scipy as sp
import pulp
import matplotlib.pyplot as plt
import cdd

import plot_utils as pu

def calc_optimal_offset(w, set_yes):
    b_array = np.dot(w.transpose(), set_yes.transpose())
    return np.min(b_array)


def calc_optimal_offset_2(set_yes, set_no, w):
    pass


# Create a convex separating polytope
def convex_separator(set_yes, set_no):
    # Set halfplane set (E-set) to empty
    e_set = []
    # Set no-points dinstances to inf
    s = float('inf')*np.ones((1, len(set_no)))
    # compute centroid
    p = set_yes.mean(axis=0)
    print(f'Centroid found at {p}')
    while np.max(s)>0:
    # for i in range(5):
        # choose the farthest no-point
        farthest_point_idx = np.argmax(s)
        farthest_point = set_no[farthest_point_idx, :]
        print(f'Choosing no-point {farthest_point_idx} with coordinates {farthest_point}')
        # compute the unit vector from the centroid to the farthest point
        w = (p-farthest_point)/np.linalg.norm(p-farthest_point)
        w = w.reshape((len(w), 1))
        # find the optimal line separator from set_yes
        b = calc_optimal_offset(w, set_yes)
        # add it to the E-set
        e_set.append(list(np.append(w.transpose(), b)))
        # Update distances of no-set
        current_distances = np.dot(w.transpose(), set_no.transpose()) - b
        for idx in range(s.shape[1]):
            s[0,idx] = min(s[0, idx], current_distances[0, idx])
        s[0, farthest_point_idx] = 0
        # print(f'Updated distance vector: {s}')
    
    return e_set

# Answer if p is inside the circle c,r
def circle_ind(p, c, r):
    distance = np.linalg.norm(p-c)
    test = distance <= r
    return test

def plot_sets(set_yes, set_no, e_set):
    fig = plt.figure()
    ah = fig.add_subplot()
    pu.plot_points(ah, set_yes, 'o', 'g')
    pu.plot_points(ah, set_no, 'X', 'r')
    ah.set_xlim(left=-10, right=10)
    ah.set_ylim(top=-10, bottom=10)

    # Plot half-planes
    x_min = -10
    x_max = 10
    y_min = -10
    y_max = 10
    for equation in e_set:
        try:
            point_1 = [x_min, get_y(equation, x_min)]
        except RuntimeWarning:
            point_1 = [x_min, y_min]
        try:
            point_2 = [x_max, get_y(equation, x_max)]
        except RuntimeWarning:
            point_2 = [x_max, y_max]
        pu.plot_line(ah, point_1, point_2, 'k')
    plt.show()

def get_y(equation, x):
    return (-equation[0]*x+equation[2])/equation[1]


if __name__ == '__main__':
    # define the indicator function
    ind = circle_ind

    # define a domain
    x = np.linspace(-10, 10, 11)
    y = np.linspace(-10, 10, 11)
    c = np.array([[4],[4]])
    r = 5

    # Sample a set of points
    set_yes = []
    set_no = []
    for x_coord, y_coord in it.product(x, y):
        point = np.array([x_coord, y_coord])
        sample = ind(point, c, r)
        if sample:
            set_yes.append((x_coord, y_coord))
        else:
            set_no.append((x_coord, y_coord))

    set_yes = np.array(set_yes)
    set_no = np.array(set_no)
    
    # Create a convex polytope
    e_set = convex_separator(set_yes, set_no)
    # print(e_set)
    converted_set = []
    for equation in e_set:
        converted_set.append([-equation[-1], equation[0], equation[1]])
    print(converted_set)
    matrix = cdd.Matrix(converted_set)
    matrix.rep_type = cdd.RepType.INEQUALITY
    cp = cdd.Polyhedron(matrix)
    print(cp)
    matrix = cp.get_generators()
    cpoints = matrix
    print(cpoints)

    # Simplify the polytope

    plot_sets(set_yes, set_no, e_set)