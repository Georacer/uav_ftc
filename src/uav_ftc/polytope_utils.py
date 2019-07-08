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

# Calculate optimal offset but have offset less than a predefined value


def calc_optimal_offset_constrained(w, set_yes, bound):
    b_array = np.dot(w.transpose(), set_yes.transpose())
    b_array = b_array[b_array > bound]
    return np.min(b_array)


def calc_optimal_offset_2(set_yes, set_no, w):
    pass


# Find unit vector joining two points
def get_unit_direction(point_1, point_2):
    w = (point_1-point_2)/np.linalg.norm(point_1-point_2)
    w = w.reshape((len(w), 1))
    return w

# Create a convex separating polytope


def convex_separator(set_yes, set_no):
    # Set halfplane set (E-set) to empty
    e_set = []
    # Set no-points dinstances to inf
    s = float('inf')*np.ones((1, len(set_no)))
    # compute centroid
    p = set_yes.mean(axis=0)
    print(f'Centroid found at {p}')
    while np.max(s) > 0:
        # for i in range(5):
        # choose the farthest no-point
        farthest_point_idx = np.argmax(s)
        farthest_point = set_no[farthest_point_idx, :]
        print(
            f'Choosing no-point {farthest_point_idx} with coordinates {farthest_point}')
        # compute the unit vector from the centroid to the farthest point
        w = get_unit_direction(p, farthest_point)
        # find the optimal line separator from set_yes
        b = calc_optimal_offset(w, set_yes)
        # add it to the E-set
        e_set.append(list(np.append(w.transpose(), b)))
        # Update distances of no-set
        current_distances = np.dot(w.transpose(), set_no.transpose()) - b
        for idx in range(s.shape[1]):
            s[0, idx] = min(s[0, idx], current_distances[0, idx])
        s[0, farthest_point_idx] = 0
        # print(f'Updated distance vector: {s}')

    return e_set


def get_bounded_separator(set_yes, no_point):
    p = set_yes.mean(axis=0)
    w = get_unit_direction(p, no_point)
    bound = np.dot(w.transpose(), no_point)
    b = calc_optimal_offset_constrained(w, set_yes, bound)
    return list(np.append(w.transpose(), b))

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


def is_inside(point, equations):
    answer = True
    for equation in equations:
        if point[0]*equation[0] + point[1]*equation[1] - equation[2] < 0:
            answer = False
            break
    return answer


def convert_equations_for_cdd(e_set):
    converted_set = []
    for equation in e_set:
        converted_set.append([-equation[-1], equation[0], equation[1]])
    return converted_set

def convert_equations_for_user(e_set):
    converted_set = []
    for equation in e_set:
        converted_set.append([equation[-2], equation[-1], -equation[0]])
    return converted_set

def get_equations_from_cdd_matrix(matrix):
    cdd_e_set = matrix[:]
    e_set = convert_equations_for_user(cdd_e_set)
    return e_set


if __name__ == '__main__':
    # define the indicator function
    ind = circle_ind

    # define a domain
    x = np.linspace(-10, 10, 11)
    y = np.linspace(-10, 10, 11)
    c = np.array([[4], [4]])
    r = 5

    # Sample a set of points
    set_all = set()
    set_yes = set()
    for x_coord, y_coord in it.product(x, y):
        point = np.array([x_coord, y_coord])
        set_all.add((x_coord, y_coord))
        sample = ind(point, c, r)
        if sample:
            set_yes.add((x_coord, y_coord))
    # Add manual points
    set_yes.add((0, 0))
    set_yes.add((0, -2))

    set_no = set_all - set_yes

    # Convert to numpy arrays
    set_yes = np.array(list(set_yes))
    set_no = np.array(list(set_no))

    # Create a convex polytope
    e_set = convex_separator(set_yes, set_no)
    # print(e_set)
    cdd_e_set = convert_equations_for_cdd(e_set)
    print(cdd_e_set)
    matrix = cdd.Matrix(cdd_e_set)
    matrix.rep_type = cdd.RepType.INEQUALITY
    matrix.canonicalize() # Reduce redundant equations
    cp = cdd.Polyhedron(matrix)
    print(f'Resulting Polyhedron')
    print(cp)
    points_matrix = cp.get_generators()
    print(f'Points matrix:')
    print(points_matrix)

    plot_sets(set_yes, set_no, e_set)

    # Reject invalid no-points inside the convex polytope
    invalid_points = []
    new_equations = []
    for no_point in set_no:
        if is_inside(no_point, e_set):
            invalid_points.append(no_point)
            print(f'Adding intruding point {no_point}')

    for point in invalid_points:
        e = get_bounded_separator(set_yes, point)
        new_equations.append(e)
        e_set.append(e)

    new_cdd_e_set = convert_equations_for_cdd(e_set)
    new_matrix = cdd.Matrix(new_cdd_e_set) 
    new_matrix.rep_type = cdd.RepType.INEQUALITY
    new_matrix.canonicalize()
    new_cp = cdd.Polyhedron(new_matrix)
    new_points_matrix = new_cp.get_generators()
    print(f'New points matrix')
    print(new_points_matrix)

    new_equations = get_equations_from_cdd_matrix(new_matrix)

    plot_sets(set_yes, set_no, new_equations)

    # Simplify the polytope
