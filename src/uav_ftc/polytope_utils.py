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
        # print(f'Choosing no-point {farthest_point_idx} with coordinates {farthest_point}')
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

    return e_set


# Return a hyperplane separating no_point from the centroid of set_yes
def get_bounded_separator(set_yes, no_point):
    p = set_yes.mean(axis=0)
    w = get_unit_direction(p, no_point)
    bound = np.dot(w.transpose(), no_point)
    b = calc_optimal_offset_constrained(w, set_yes, bound)
    return list(np.append(w.transpose(), b))


# Answer if p is inside the circle c,r
def circle_ind(p, c, r):
    distance = np.linalg.norm(p-c)
    return distance <= r


def plot_sets(set_yes, set_no, polytope):
    equations_matrix = polytope.get_inequalities()
    equations = get_equations_from_cdd_matrix(equations_matrix)

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
    for equation in equations:
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


def align_to_grid(value, resolution):
    return np.sign(value)*(np.abs(value)//resolution)*resolution


# Return the coordinates which are up to eps away from the polygon boundary
# domain is a 2x2 numpy array with the dimension interval at each row
def get_boundary_points(polygon, eps, domain, include_internal=False):
    points = []
    points_matrix = polygon.get_generators()
    for point_idx in range(points_matrix.row_size):
        row = points_matrix[point_idx]
        if row[0] == 1:  # This is a point
            points.append(row[1:])
    points = np.array(points)  # Convert to a numpy array

    # Align the number resolution to eps
    x_min = max(np.min(points[:, 0], axis=0), domain[0, 0])
    x_min = align_to_grid(x_min-eps, eps)
    x_max = min(np.max(points[:, 0], axis=0), domain[0, 1])
    x_max = align_to_grid(x_max+eps, eps)
    y_min = max(np.min(points[:, 1], axis=0), domain[1, 0])
    y_min = align_to_grid(y_min-eps, eps)
    y_max = min(np.max(points[:, 1], axis=0), domain[1, 1])
    y_max = align_to_grid(y_max+eps, eps)

    x_lim = [x_min, x_max]
    y_lim = [y_min, y_max]

    n_x = (x_lim[1] - x_lim[0])//eps
    n_y = (y_lim[1] - y_lim[0])//eps
    x_samples = np.linspace(x_lim[0], x_lim[1], n_x+1)
    y_samples = np.linspace(y_lim[0], y_lim[1], n_y+1)

    equations_matrix = polygon.get_inequalities()
    e_set = get_equations_from_cdd_matrix(equations_matrix)

    result = []
    for x, y in it.product(x_samples, y_samples):
        # verify that the point is inside the domain of interest
        if not domain[0, 0] <= x <= domain[0, 1]:
            continue
        if not domain[1, 0] <= y <= domain[1, 1]:
            continue

        point = np.array([[x], [y]])
        for equation in e_set:
            w = np.array(equation[0:2])
            b = np.array(equation[-1])
            distance = (np.dot(w.transpose(), point) - b)/np.linalg.norm(w)
            # Include all internal points
            if include_internal and (distance <= eps):
                result.append((x, y))
                break
            # Include only points close to the boundary
            elif np.abs(distance) <= eps:
                result.append((x, y))
                break

    return result


# Create a convex polytope around elements of set_yes that contains zero elements of set_no
def build_safe_polytope(set_yes, set_no):

    # Create a convex polytope around set_yes
    e_set = convex_separator(set_yes, set_no)

    # Find all the invalid points inside the polytope
    invalid_points = []
    new_equations = []
    for no_point in set_no:
        if is_inside(no_point, e_set):
            invalid_points.append(no_point)
            print(f'Adding intruding point {no_point}')

    # Create one new equation for each one, separating them from the majority of set-yes
    for point in invalid_points:
        e = get_bounded_separator(set_yes, point)
        new_equations.append(e)
        e_set.append(e)

    # Construct a new polytope from the new equation set
    new_cdd_e_set = convert_equations_for_cdd(e_set)
    new_matrix = cdd.Matrix(new_cdd_e_set)
    new_matrix.rep_type = cdd.RepType.INEQUALITY
    new_matrix.canonicalize()

    return cdd.Polyhedron(new_matrix)


# Sample points around a polytope boundary
def sample_points(initial_polytope, ind, eps, domain, set_yes, set_no, init=False):
    set_all = get_boundary_points(
        initial_polytope, eps, domain, include_internal=init)
    for coords in set_all:
        point = np.array(coords).reshape(2, 1)
        sample = ind(point, c, r)
        if sample:
            set_yes.add(coords)
        else:
            set_no.add(coords)

    return set_yes, set_no


if __name__ == '__main__':
    # define the indicator function
    ind = circle_ind
    c = np.array([[4], [4]])
    r = 6

    # define a domain
    x_lim = [-10, 10]
    y_lim = [-10, 10]
    domain = np.array([x_lim, y_lim])

    eps = 5
    set_yes = set()
    set_no = set()

    # define an initial polygon
    domain_rows = [[1, -10, -10], [1, -10, 10], [1, 10, 10],
                   [1, 10, -10]]  # Need to add 1 in front of every point
    initial_matrix = cdd.Matrix(domain_rows)
    initial_matrix.rep_type = cdd.RepType.GENERATOR
    initial_polytope = cdd.Polyhedron(initial_matrix)

    # Run a polytope generation iteration
    set_yes, set_no = sample_points(
        initial_polytope, ind, eps, domain, set_yes, set_no, init=True)

    # Add manual points
    set_yes.add((0, 0))
    set_yes.add((0, -2))

    # Convert to numpy arrays
    arr_yes = np.array(list(set_yes))
    arr_no = np.array(list(set_no))

    # Create the safe convex polytope
    safe_cp = build_safe_polytope(arr_yes, arr_no)

    plot_sets(arr_yes, arr_no, safe_cp)

    # Increase resolution and start anew
    eps = eps/2
    set_yes, set_no = sample_points(safe_cp, ind, eps, domain, set_yes, set_no)

    # Convert to numpy arrays
    arr_yes = np.array(list(set_yes))
    arr_no = np.array(list(set_no))

    # Create the safe convex polytope
    safe_cp = build_safe_polytope(arr_yes, arr_no)

    plot_sets(arr_yes, arr_no, safe_cp)

    # Increase resolution and start anew
    eps = eps/2
    set_yes, set_no = sample_points(safe_cp, ind, eps, domain, set_yes, set_no)

    # Convert to numpy arrays
    arr_yes = np.array(list(set_yes))
    arr_no = np.array(list(set_no))

    # Create the safe convex polytope
    safe_cp = build_safe_polytope(arr_yes, arr_no)

    plot_sets(arr_yes, arr_no, safe_cp)
