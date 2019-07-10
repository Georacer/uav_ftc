#!python3

# Convections:
# List: numpy 1-D array
# Point: numpy Mx1 array
# Point List: numpy MxN array
# Vector: Mx1 array
# Set of points: set of M-tuples
# Equations: list of lists

import itertools as it
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import cdd

import plot_utils as pu


class Polytope:
    _polytope_engine = None

    def __init__(self):
        pass

    def set_points(self, points):
        array = points.transpose()
        n_points = points.shape[1]
        array = np.concatenate((np.ones((n_points, 1)), array), axis=1)
        point_rows = list(array)
        point_matrix = cdd.Matrix(point_rows)
        point_matrix.rep_type = cdd.RepType.GENERATOR
        self._polytope_engine = cdd.Polyhedron(point_matrix)

    # Construct a new polytope from an equation set
    def set_equations(self, equations):
        cdd_e_set = self._convert_equations_for_cdd(equations)
        e_matrix = cdd.Matrix(cdd_e_set)
        e_matrix.rep_type = cdd.RepType.INEQUALITY
        e_matrix.canonicalize()
        self._polytope_engine = cdd.Polyhedron(e_matrix)

    def get_points(self):
        points = []
        points_matrix = self._polytope_engine.get_generators()
        for point_idx in range(points_matrix.row_size):
            row = points_matrix[point_idx]
            if row[0] == 1:  # This is a point
                points.append(row[1:])
        points = np.array(points).transpose()  # Convert to a numpy array
        return points

    def get_equations(self):
        e_matrix = self._polytope_engine.get_inequalities()
        return self._get_equations_from_cdd_matrix(e_matrix)

    def __contains__(self, point):
        equations = self.get_equations()
        answer = True
        for equation in equations:
            if point[0, 0]*equation[0] + point[1, 0]*equation[1] - equation[2] < 0:
                answer = False
                break
        return answer

    def _convert_equations_for_cdd(self, e_set):
        converted_set = []
        for equation in e_set:
            converted_set.append([-equation[-1], equation[0], equation[1]])
        return converted_set

    def _convert_equations_for_user(self, cdd_e_set):
        converted_set = []
        for equation in cdd_e_set:
            converted_set.append([equation[-2], equation[-1], -equation[0]])
        return converted_set

    def _get_equations_from_cdd_matrix(self, matrix):
        cdd_e_set = matrix[:]
        e_set = self._convert_equations_for_user(cdd_e_set)
        return e_set


class SafeConvexPolytope:

    domain = None
    indicator = None
    points = None
    equations = None
    eps = None
    points_yes = None
    points_no = None

    _polytope = None
    _domain_polytope = None
    _set_points = None
    _set_yes = None
    _set_no = None
    _init_division = 4

    def __init__(self, ind_func, domain):
        self.indicator = ind_func
        self.domain = domain
        self._set_points = set()
        self._set_yes = set()
        self._set_no = set()
        self._polytope = Polytope()
        self._domain_polytope = Polytope()
        self.eps = (domain[0, 1]-domain[0, 0])/self._init_division

        # define an initial polygon
        x_min = domain[0, 0]
        x_max = domain[0, 1]
        y_min = domain[1, 0]
        y_max = domain[1, 1]
        initialization_points = np.array(
            [[x_min, y_min], [x_min, y_max], [x_max, y_max], [x_max, y_min]]).transpose()
        self._polytope.set_points(initialization_points)
        self._domain_polytope.set_points(initialization_points)

        # Run a polytope generation iteration
        self.sample(init=True)

    @staticmethod
    def _array_to_set(points):
        return set(map(tuple, list(points.transpose())))

    @staticmethod
    def _set_to_array(points_set):
        return np.array(list(points_set)).transpose()

    @ staticmethod
    def _align_to_grid(value, resolution):
        return np.sign(value)*(np.abs(value)//resolution)*resolution

    # Find unit vector joining two points
    @staticmethod
    def _get_unit_direction(point_1, point_2):
        return (point_1-point_2)/np.linalg.norm(point_1-point_2)

    # Find the bounding axis-aligned hyperrectangle of the points
    @staticmethod
    def _get_bounding_box(points):
        shape = points.shape
        dim = len(shape)
        domain = np.zeros((dim, 2))
        for dim_idx in range(dim):
            domain[dim_idx, 0] = np.min(points[dim_idx, :])
            domain[dim_idx, 1] = np.max(points[dim_idx, :])
        return domain

    @staticmethod
    def _get_y(equation, x):
        return (-equation[0]*x+equation[2])/equation[1]

    # Set the sampling resolution
    def set_eps(self, eps):
        self.eps = eps

    def _update_points(self):
        self.points_yes = self._set_to_array(self._set_yes)
        self.points_no = self._set_to_array(self._set_no)

    def add_points(self, points):
        new_points = self._filter_points(points)
        new_points_set = self._array_to_set(new_points)
        self._set_points |= new_points_set

        for new_row in new_points.transpose():
            new_point = new_row.transpose()
            group = self.indicator(new_point)
            if group:
                self._set_yes |= self._array_to_set(new_point)
            else:
                self._set_no |= self._array_to_set(new_point)
        self._update_points()

    def add_yes_points(self, points):
        points_set = self._array_to_set(points)
        self._set_points |= points_set
        self._set_yes |= points_set
        self._set_no -= points_set
        self._update_points()

    def add_no_points(self, points):
        points_set = self._array_to_set(points)
        self._set_points |= points_set
        self._set_no |= points_set
        self._set_yes -= points_set
        self._update_points()

    # Find which of the input points are not already part of the known points
    def _filter_points(self, points):
        set_new_points = points-self._set_points
        return np.array(set_new_points).transpose()

    # Return the coordinates which are up to eps away from the polytope boundary
    def _get_boundary_points(self, include_internal=False):
        vertices = self._polytope.get_points()
        vertex_x_min = np.min(vertices[0, :])
        vertex_x_max = np.max(vertices[0, :])
        vertex_y_min = np.min(vertices[1, :])
        vertex_y_max = np.max(vertices[1, :])

        domain_x_min = self.domain[0, 0]
        domain_x_max = self.domain[0, 1]
        domain_y_min = self.domain[1, 0]
        domain_y_max = self.domain[1, 1]

        # Align the number resolution to eps
        x_min = max(vertex_x_min, domain_x_min)
        x_min = self._align_to_grid(x_min - self.eps, self.eps)
        x_max = min(vertex_x_max, domain_x_max)
        x_max = self._align_to_grid(x_max + self.eps, self.eps)
        y_min = max(vertex_y_min, domain_y_min)
        y_min = self._align_to_grid(y_min - self.eps, self.eps)
        y_max = min(vertex_y_max, domain_y_max)
        y_max = self._align_to_grid(y_max + self.eps, self.eps)

        n_x = int((x_max - x_min)//self.eps)
        n_y = int((y_max - y_min)//self.eps)
        x_samples = np.linspace(x_min, x_max, n_x+1)
        y_samples = np.linspace(y_min, y_max, n_y+1)

        e_set = self._polytope.get_equations()

        result = []
        for x, y in it.product(x_samples, y_samples):
            point = np.array([[x], [y]])
            # verify that the point is inside the domain of interest
            if point not in self._domain_polytope:
                continue

            for equation in e_set:
                w = np.array(equation[0:2])
                b = np.array(equation[-1])
                distance = (np.dot(w.transpose(), point) - b)/np.linalg.norm(w)
                # Include all internal points
                if include_internal and (distance <= self.eps):
                    result.append((x, y))
                    break
                # Include only points close to the boundary
                elif np.abs(distance) <= self.eps:
                    result.append((x, y))
                    break

        return np.array(result).transpose()

    # Sample points around a polytope boundary
    def sample(self, init=False):
        boundary_points = self._get_boundary_points(init)
        for coords in boundary_points.transpose():
            point = coords.reshape(2, 1)
            sample = self.indicator(point)
            if sample:
                self.add_yes_points(point)
            else:
                self.add_no_points(point)

    def _calc_optimal_offset(self, w):
        b_array = np.dot(w.transpose(), self.points_yes)
        return np.min(b_array, axis=1)

    # Calculate optimal offset but have offset less than a predefined value
    def _calc_optimal_offset_constrained(self, w, bound):
        b_array = np.dot(w.transpose(), self.points_yes)
        b_array = b_array[b_array > bound]
        return np.min(b_array)

    # Create a convex separating polytope
    def _convex_separator(self):
        # Set halfplane set (E-set) to empty
        e_set = []
        # Set no-points dinstances to inf
        s = float('inf')*np.ones(len(self._set_no))
        # compute centroid
        p = self.points_yes.mean(axis=1).reshape(2,1)
        print('Centroid found at\n{}'.format(p))

        # While not all no-points have been separated
        while np.max(s) > 0:
            # choose the farthest no-point
            farthest_point_idx = np.argmax(s)
            farthest_point = self.points_no[:, [farthest_point_idx]]
            # compute the unit vector from the centroid to the farthest point
            w = self._get_unit_direction(p, farthest_point)
            # find the optimal line separator from set_yes
            b = self._calc_optimal_offset(w)
            # add it to the E-set
            e_set.append(list(np.append(w.transpose(), b)))
            # Update distances of no-set
            current_distances = np.dot(w.transpose(), self.points_no) - b
            for idx in range(s.shape[0]):
                s[idx] = min(s[idx], current_distances[0, idx])
            s[farthest_point_idx] = 0

        convex_polytope = Polytope()
        convex_polytope.set_equations(e_set)
        return convex_polytope

    # Return a hyperplane separating no_point from the centroid of set_yes
    def _get_bounded_separator(self, no_point):
        p = self.points_yes.mean(axis=1)
        w = self._get_unit_direction(p, no_point)
        bound = np.dot(w.transpose(), no_point)
        b = self._calc_optimal_offset_constrained(w, bound)
        return list(np.append(w.transpose(), b))

    # Create a convex polytope around elements of set_yes that contains zero elements of set_no
    def build_safe_polytope(self):
        # Create a convex polytope around set_yes
        convex_polytope = self._convex_separator()
        e_set = convex_polytope.get_equations()

        # Find all the invalid points inside the polytope
        invalid_points = []
        for coords in self.points_no.transpose():
            no_point = coords.reshape(2,1)
            if no_point in convex_polytope:
                invalid_points.append(no_point)
                print('Adding intruding point\n {}'.format(no_point))

        # Create one new equation for each one, separating them from the majority of set-yes
        for point in invalid_points:
            e = self._get_bounded_separator(point)
            e_set.append(e)

        # Construct a new polytope from the new equation set
        self._polytope.set_equations(e_set)

    # Halve resolution, resample boundary and get new safe polytope
    def enhance(self):
        self.eps = self.eps/2
        self.sample()
        self.build_safe_polytope()

    def plot(self):
        equations = self._polytope.get_equations()

        fig = plt.figure()
        ah = fig.add_subplot(111)
        pu.plot_points(ah, self.points_yes, 'o', 'g')
        pu.plot_points(ah, self.points_no, 'X', 'r')
        ah.set_xlim(left=-10, right=10)
        ah.set_ylim(top=-10, bottom=10)

        # Plot half-planes
        all_points = self._set_to_array(self._set_points)
        domain = self._get_bounding_box(all_points)
        x_min = domain[0, 0]
        x_max = domain[0, 1]
        y_min = domain[1, 0]
        y_max = domain[1, 1]

        for equation in equations:
            try:
                point_1 = [x_min, self._get_y(equation, x_min)]
            except RuntimeWarning:
                point_1 = [x_min, y_min]
            try:
                point_2 = [x_max, self._get_y(equation, x_max)]
            except RuntimeWarning:
                point_2 = [x_max, y_max]
            pu.plot_line(ah, point_1, point_2, 'k')
        plt.show()


# Answer if p is inside the circle c,r
def circle_ind(p):
    c = np.array([[4], [4]])
    r = 6
    distance = np.linalg.norm(p-c)
    return distance <= r


if __name__ == '__main__':

    # define a domain
    x_lim = [-10, 10]
    y_lim = [-10, 10]
    domain = np.array([x_lim, y_lim])

    safe_poly = SafeConvexPolytope(circle_ind, domain)
    safe_poly.set_eps(5)

    # Add manual points
    additional_yes_points = np.array([[0, 0], [0, -2]]).transpose()
    safe_poly.add_yes_points(additional_yes_points)

    # Create the safe convex polytope
    safe_poly.build_safe_polytope()
    # safe_poly.plot()

    # Increase resolution and start anew
    safe_poly.enhance()
    # safe_poly.plot()

    # Increase resolution and start anew
    safe_poly.enhance()
    safe_poly.plot()