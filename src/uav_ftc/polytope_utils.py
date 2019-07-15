#!python3

# Convections:
# List: numpy 1-D array
# Point: numpy Mx1 array
# Point List: numpy MxN array
# Vector: Mx1 array
# Set of points: set of M-tuples
# Equations: numpy NxM array with each row the equation coefficients [A b]

import sys
import itertools as it
import numpy as np
import scipy as sp
from scipy.cluster.vq import kmeans2
import matplotlib.pyplot as plt
import cdd

import plot_utils as pu

# Get the distance of a point from a hypersurface
# Essentially this is an evaluation of the point for the hypersurface expression
# Applicable to multiple points and equations at once
def evaluate_hypersurface(points, equations):
    # Catch case of single-dimensional equation array
    if len(equations.shape) == 1:
        equations = equations.reshape(1, -1)
    if len(points.shape) == 1:
        points = points.reshape(-1, 1)

    A = equations[:, 0:-1]
    b = np.repeat(equations[:, [-1]], points.shape[1], axis=1)
    distances = (np.dot(A, points) - b)
    return distances


class Polytope:
    _polytope_engine = None

    _equations = None
    _points = None

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
        self._build_equations()
        self._build_points()

    # Construct a new polytope from an equation set
    def set_equations(self, equations):
        cdd_e_set = self._convert_equations_for_cdd(equations)
        e_matrix = cdd.Matrix(cdd_e_set)
        e_matrix.rep_type = cdd.RepType.INEQUALITY
        e_matrix.canonicalize()
        self._polytope_engine = cdd.Polyhedron(e_matrix)
        self._build_equations()
        self._build_points()

    def _build_points(self):
        points = []
        points_matrix = self._polytope_engine.get_generators()
        for point_idx in range(points_matrix.row_size):
            row = points_matrix[point_idx]
            if row[0] == 1:  # This is a point
                points.append(row[1:])
        self._points = np.array(points).transpose()  # Convert to a numpy array

    def _build_equations(self):
        e_matrix = self._polytope_engine.get_inequalities()
        self._equations = self._get_equations_from_cdd_matrix(e_matrix)

    def get_equations(self):
        return self._equations

    def get_points(self):
        return self._points

    def __contains__(self, point):
        if point.shape[1]>1:
            raise IndexError('Use explicit "contains" method to test multiple points')
        return self.contains(point)

    # Explicit containment method for multidimensional output
    def contains(self, points):
        equations = self.get_equations()
        distances = evaluate_hypersurface(points, equations)
        answer = np.all(distances>=0, axis=0)

        return answer

    def _convert_equations_for_cdd(self, e_array):
        converted_set = []
        for equation in e_array:
            dim = len(equation)
            A = equation[0:-1].reshape(dim-1)
            b = equation[-1].reshape(1)
            converted_set.append(np.concatenate((-b, A)))
        return converted_set

    def _convert_equations_for_user(self, cdd_e_list):
        num_equations = len(cdd_e_list)
        n_coeffs = len(cdd_e_list[0])
        converted_array = np.zeros((num_equations, n_coeffs))
        idx = 0

        for equation in cdd_e_list:
            A = np.array(equation[1:]).reshape(1, n_coeffs-1)
            b = np.array(equation[0]).reshape(1, 1)
            converted_array[idx, :] = np.concatenate((A, -b), axis=1)
            idx = idx+1
        return converted_array

    def _get_equations_from_cdd_matrix(self, matrix):
        cdd_e_set = matrix[:]
        e_set = self._convert_equations_for_user(cdd_e_set)
        return e_set


class SafeConvexPolytope:

    domain = None
    indicator = None
    equations = None
    eps = None  # a (n_dim, ) array with the sampling resolution
    points_yes = None
    points_no = None

    _polytope = None
    _domain_polytope = None
    _set_points = None
    _set_yes = None
    _set_no = None
    _n_dim = None
    _init_division = 4

    def __init__(self, ind_func, domain, eps=None):
        self.indicator = ind_func
        self.domain = domain
        self._n_dim = domain.shape[0]
        self._set_points = set()
        self._set_yes = set()
        self._set_no = set()
        self._polytope = Polytope()
        self._domain_polytope = Polytope()
        if any(eps == None):
            self.eps = (domain[:, 1] - domain[:, 0])/self._init_division
        else:
            self.set_eps(eps)

        # define an initial polygon
        initialization_points = np.zeros((self._n_dim, 2**self._n_dim))
        idx = 0
        for point_tuple in it.product(*tuple(domain)):
            initialization_points[:, idx] = np.array(point_tuple)
            idx += 1
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
        dim = shape[0]
        domain = np.zeros((dim, 2))
        for dim_idx in range(dim):
            domain[dim_idx, 0] = np.min(points[dim_idx, :])
            domain[dim_idx, 1] = np.max(points[dim_idx, :])
        return domain

    @staticmethod
    def _get_y(equation, x):
        return (-equation[0]*x+equation[2])/equation[1]

    @staticmethod
    def _get_z(equation, x, y):
        known_vec = np.array([x, y]).reshape(2, 1)
        equation_vec = equation.reshape(len(equation), 1)
        return (equation_vec[-1] - np.dot(equation_vec[0:-2, :].transpose(), known_vec))/equation_vec[-2]

    # Set the sampling resolution
    def set_eps(self, eps):
        dimensions = self.domain.shape[0]
        if tuple([dimensions]) != eps.shape:
            raise IndexError('Eps dimension mismatch')
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

    def get_centroid(self):
        # compute centroid
        return self.points_yes.mean(axis=1).reshape(self._n_dim, 1)

    # Return the coordinates which are up to eps away from the polytope boundary
    def _get_boundary_points(self, include_internal=False):
        vertices = self._polytope.get_points()
        bbox = self._get_bounding_box(vertices)

        num_intervals = np.zeros(self._n_dim)
        sample_points = []

        new_domain = np.zeros(bbox.shape)
        for dim_idx in range(self._n_dim):
            # Align the number resolution to eps
            unaligned_min = max(bbox[dim_idx, 0], self.domain[dim_idx, 0])
            eps = self.eps[dim_idx]
            aligned_min = self._align_to_grid(unaligned_min - eps, eps)
            new_domain[dim_idx, 0] = aligned_min

            unaligned_max = min(bbox[dim_idx, 1], self.domain[dim_idx, 1])
            eps = self.eps[dim_idx]
            aligned_max = self._align_to_grid(unaligned_max + eps, eps)
            new_domain[dim_idx, 1] = aligned_max

            # Construct the sampling values for each dimension
            n = int((aligned_max-aligned_min)//eps) + 1
            num_intervals[dim_idx] = n
            sample_points.append(np.linspace(aligned_min, aligned_max, n))

        result = []
        for coords_tuple in it.product(*sample_points):
            point = np.array(coords_tuple).reshape((self._n_dim, 1))

            if self._is_point_eligible(point, include_internal):
                result.append(point[:, 0].transpose())

        return np.array(result).transpose()

    # Check if a point is eligible for inclusion in the sampling grid
    def _is_point_eligible(self, point, include_internal=False):
        # verify that the point is inside the general domain of interest
        if point not in self._domain_polytope:
            return False

        # Do not resample already sampled points
        if tuple(point.transpose()[0, :]) in self._set_points:
            return False

        # Include all internal points if requested
        if include_internal:
            return True

        # Include if the points is close to any boundary line
        e_array = self._polytope.get_equations()
        for equation in e_array:
            n = equation[0:-1].reshape(self._n_dim, 1)
            n_0 = n/np.linalg.norm(n)
            normal_vector = evaluate_hypersurface( point, equation) * n_0
            distances = np.array(map(np.abs, normal_vector))
            if  np.all(distances < self.eps):
                return True

    # Sample points around a polytope boundary
    def sample(self, init=False):
        boundary_points = self._array_to_set(self._get_boundary_points(init))
        boundary_points -= self._set_points
        yes_points = []
        no_points = []
        for coords in boundary_points:
            point = np.array(coords).reshape(-1, 1)
            sample = self.indicator(point)
            coords_tuple = tuple(point.transpose()[0, :])
            if sample:
                yes_points.append(coords_tuple)
            else:
                no_points.append(coords_tuple)

        self._set_yes.update(yes_points)
        self._set_no.update(no_points)
        self._set_points = self._set_yes | self._set_no
        self._update_points()

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
        # get centroid
        p = self.get_centroid()

        # While not all no-points have been separated
        while np.max(s) > 0:
            # choose the farthest no-point
            farthest_point_idx = np.argmax(s)
            farthest_point = self.points_no[:, [farthest_point_idx]]
            # compute the unit vector from the centroid to the farthest point
            w = self._get_unit_direction(p, farthest_point)
            # find the optimal line separator from set_yes
            b = self._calc_optimal_offset(w)
            equation = np.vstack((w, b)).transpose().reshape((1, -1))
            # add it to the E-set
            e_set.append(list(equation.reshape(-1)))
            # Update distances of no-set
            current_distances = evaluate_hypersurface( self.points_no, equation)
            for idx in range(s.shape[0]):
                s[idx] = min(s[idx], current_distances[0, idx])
            s[farthest_point_idx] = 0

        convex_polytope = Polytope()
        e_array = np.array(e_set)
        convex_polytope.set_equations(e_array)
        return convex_polytope

    # Return a hyperplane separating no_point from the centroid of set_yes
    def _get_bounded_separator(self, no_point):
        p = self.get_centroid()
        w = self._get_unit_direction(p, no_point)
        bound = np.dot(w.transpose(), no_point)
        b = self._calc_optimal_offset_constrained(w, bound)
        return list(np.append(w.transpose(), b))

    # Create a convex polytope around elements of set_yes that contains zero elements of set_no
    def build_safe_polytope(self):
        # Create a convex polytope around set_yes
        convex_polytope = self._convex_separator()
        e_array = convex_polytope.get_equations()

        # Find all the invalid points inside the polytope
        invalid_points = []
        for coords in self.points_no.transpose():
            no_point = coords.reshape(self._n_dim, 1)
            if no_point in convex_polytope:
                invalid_points.append(no_point)

        # Create one new equation for each one, separating them from the majority of set-yes
        new_e_set = []
        for point in invalid_points:
            e = self._get_bounded_separator(point)
            new_e_set.append(e)
        new_e_array = np.array(new_e_set)
        if len(new_e_array)>0:
            e_array = np.vstack((e_array, new_e_array))

        # Construct a new polytope from the new equation set
        self._polytope.set_equations(e_array)

    # Delete points which are more than eps away from boundaries
    def remove_distant_points(self):
        all_equations = self._polytope.get_equations()
        # all_points = np.hstack((self.points_yes, self.points_no))

        # Evaluate the distances of k hypersurface from m points (k x m array)
        yes_distances = np.abs(evaluate_hypersurface(self.points_yes, all_equations))
        eps_array = np.max(self.eps) * np.ones(yes_distances.shape)
        points_to_keep_mask = np.any(yes_distances<=eps_array, axis=0)
        self.points_yes = self.points_yes[:, points_to_keep_mask]

        no_distances = np.abs(evaluate_hypersurface(self.points_no, all_equations))
        eps_array = np.max(self.eps) * np.ones(no_distances.shape)
        points_to_keep_mask = np.any(no_distances<=eps_array, axis=0)
        self.points_no = self.points_no[:, points_to_keep_mask]

        self._set_yes = self._array_to_set(self.points_yes)
        self._set_no = self._array_to_set(self.points_no)
        self._set_points = self._set_yes | self._set_no

    # Delete points which are more than eps away from the boundary of the polytope
    # judging radially from the centroid
    def remove_distant_points_2(self):
        # get centroid
        p = self.get_centroid()
        # Set contraction distance
        eps_out = 1.5*np.max(self.eps)
        eps_in = 1.5*np.max(self.eps)

        # Contract points and delete outside points
        recentered_yes_points = (self.points_yes - p)
        norms_yes = np.linalg.norm(recentered_yes_points, axis=0).reshape(1, -1)
        factors_yes = np.repeat((norms_yes-eps_out)/norms_yes, p.shape[0], axis=0)
        contracted_yes_points = recentered_yes_points*factors_yes + p
        points_to_keep_mask = self._polytope.contains(contracted_yes_points)
        self.points_yes = self.points_yes[:, points_to_keep_mask]

        # Contract polytope and delete inside points
        polytope_points = self._polytope.get_points()-p
        norms_polytope_points = np.linalg.norm(polytope_points, axis=0).reshape(1,-1)
        factors_yes = np.repeat((norms_polytope_points-eps_in)/norms_polytope_points, p.shape[0], axis=0)
        contracted_polytope_points = polytope_points*factors_yes + p
        contracted_polytope = Polytope()
        contracted_polytope.set_points(contracted_polytope_points)
        points_to_keep_mask = contracted_polytope.contains(self.points_yes)==False
        self.points_yes = self.points_yes[:, points_to_keep_mask]

        # Contract points and delete outside points
        recentered_no_points = (self.points_no - p)
        norms_no = np.linalg.norm(recentered_no_points, axis=0).reshape(1, -1)
        factors_no = np.repeat((norms_no-eps_out)/norms_no, p.shape[0], axis=0)
        contracted_no_points = recentered_no_points*factors_no + p
        points_to_keep_mask = self._polytope.contains(contracted_no_points)
        self.points_no = self.points_no[:, points_to_keep_mask]

        # Contract polytope and delete inside points
        polytope_points = self._polytope.get_points()-p
        norms_polytope_points = np.linalg.norm(polytope_points, axis=0).reshape(1,-1)
        factors_no = np.repeat((norms_polytope_points-eps_in)/norms_polytope_points, p.shape[0], axis=0)
        contracted_polytope_points = polytope_points*factors_no + p
        contracted_polytope = Polytope()
        contracted_polytope.set_points(contracted_polytope_points)
        points_to_keep_mask = contracted_polytope.contains(self.points_no)==False
        self.points_no = self.points_no[:, points_to_keep_mask]

        self._set_yes = self._array_to_set(self.points_yes)
        self._set_no = self._array_to_set(self.points_no)
        self._set_points = self._set_yes | self._set_no

    def cluster(self, num_vertices):
        vertices = self._polytope.get_points()
        centroids, _ = kmeans2(vertices.transpose(),
                               num_vertices, minit='points')
        self._polytope.set_points(centroids.transpose())

    # Halve resolution, resample boundary and get new safe polytope
    def enhance(self):
        self.eps = self.eps/2
        self.sample()
        self.build_safe_polytope()
        self.remove_distant_points_2()

    # Return a list containing one MxN array for each face of the polytope
    # where M is the domain dimension and N is the number of points on each face, which may vary
    def _get_face_points(self):
        face_list = []
        all_points = self._polytope.get_points()
        all_equations = self._polytope.get_equations()

        for equation in all_equations:
            face_points = []
            for point in all_points.transpose():
                if np.abs(evaluate_hypersurface(point, equation)) < 1e-5:
                    face_points.append(point)
            face_list.append(face_points)
        return face_list

    def plot(self):

        # 2D plotting
        if self._n_dim == 2:
            domain = self.domain
            x_min = domain[0, 0]
            x_max = domain[0, 1]
            y_min = domain[1, 0]
            y_max = domain[1, 1]

            fig = plt.figure()
            ah = fig.add_subplot(111)
            pu.plot_points(ah, self.points_yes, 'o', 'g')
            pu.plot_points(ah, self.points_no, 'X', 'r')
            ah.set_xlim(x_min, x_max)
            ah.set_ylim(y_min, y_max)

            # Plot half-planes
            all_points = self._set_to_array(self._set_points)
            domain = self._get_bounding_box(all_points)

            equations = self._polytope.get_equations()
            for equation in equations:
                try:
                    point_1 = [x_min, self._get_y(equation, x_min)]
                except ZeroDivisionError:
                    point_1 = [x_min, y_min]
                try:
                    point_2 = [x_max, self._get_y(equation, x_max)]
                except ZeroDivisionError:
                    point_2 = [x_max, y_max]
                pu.plot_line(ah, point_1, point_2, 'k')
            plt.draw()

        # 3D plotting
        if self._n_dim == 3:
            # Get the plotting domain
            all_points = self._set_to_array(self._set_points)
            domain = self._get_bounding_box(all_points)
            x_min = domain[0, 0]
            x_max = domain[0, 1]
            y_min = domain[1, 0]
            y_max = domain[1, 1]
            z_min = domain[2, 0]
            z_max = domain[2, 1]

            # Build the polytope faces
            face_points = self._get_face_points()

            fig = plt.figure()
            ah = fig.add_subplot(111, projection='3d', proj_type='ortho')
            pu.plot_points_3(ah, self.points_yes, 'o', 'g')
            pu.plot_points_3(ah, self.points_no, 'X', 'r', alpha=0.2)
            pu.plot_polygons_3(ah, face_points)

            ah.set_xlim(x_min, x_max)
            ah.set_ylim(y_min, y_max)
            ah.set_zlim(z_min, z_max)

            plt.draw()

        if self._n_dim > 3:
            raise UserWarning(
                'Plotting not implemented for more than 3 dimensions')


# Answer if p is inside the hypersphere c, r
def hypersphere(p):
    c = 4*np.ones(p.shape)
    r = 6
    distance = np.linalg.norm(p-c)
    return distance <= r


if __name__ == '__main__':
    # Select the number of dimensions for this example
    if len(sys.argv) > 1:
        n_dim = int(sys.argv[1])
    else:
        n_dim = 2
    print('Testing polytope code for {} dimensions'.format(n_dim))

    ind_func = hypersphere

    # flag_plot = True
    flag_plot = False

    # define a domain as a n_dim x 2 array
    print('Creating problem domain and sampling initial points')
    domain = np.zeros((n_dim, 2))
    domain[:, 0] = -10
    domain[:, 1] = 10

    # Set the starting precision
    eps = 5*np.ones((n_dim))
    safe_poly = SafeConvexPolytope(ind_func, domain, eps)

    safe_poly.set_eps(5*np.ones((safe_poly._n_dim)))

    # Manually add random
    point_1 = np.random.rand(n_dim).reshape(n_dim, 1)
    point_1 = 10*(point_1 - 0.5*np.ones((n_dim, 1)))
    point_2 = np.random.rand(n_dim).reshape(n_dim, 1)
    point_2 = 10*(point_2 - 0.5*np.ones((n_dim, 1)))
    additional_yes_points = np.concatenate((point_1, point_2), axis=1)
    safe_poly.add_yes_points(additional_yes_points)

    # Create the safe convex polytope
    print('Building first-pass safe polytope')
    safe_poly.build_safe_polytope()
    if flag_plot:
        safe_poly.plot()

    # Increase resolution and start anew
    print('Building second-pass safe polytope')
    safe_poly.enhance()
    if flag_plot:
        safe_poly.plot()

    # Increase resolution and start anew
    print('Building third-pass safe polytope')
    safe_poly.enhance()
    if flag_plot:
        safe_poly.plot()

    # Approximate the safe convex polytope with k vertices
    print('Performing clustering')
    safe_poly.cluster(2**n_dim)

    if flag_plot:
        print('Plotting')
        safe_poly.plot()
        plt.show()
