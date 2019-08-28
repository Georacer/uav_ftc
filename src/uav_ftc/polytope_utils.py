#!python3

# Convections:
# List: numpy 1-D array
# Point: numpy Mx1 array
# Point List: numpy MxN array
# Vector: Mx1 array
# Set of points: set of M-tuples
# Equations: numpy NxM array with each row the equation coefficients [A b]
# Hyperplane evaluation: performing Ax+b should yield < 0 inside the convex polytope

import sys
from warnings import warn
import itertools as it
import time
import fractions as fr

import numpy as np
import scipy as sp
from scipy.cluster.vq import kmeans2
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError
import matplotlib as mpl
import matplotlib.pyplot as plt
import cdd
import ppl
import click

import plot_utils as pu

# Get the distance of a point from a hypersurface
# Essentially this is an evaluation of the point for the hypersurface expression
# Applicable to multiple points and equations at once


def evaluate_hyperplane(points, equations):
    # Catch case of single-dimensional equation array
    # print("Evaluating point\n{} on equation\n{}".format(points, equations))
    if len(equations.shape) == 1:
        equations = equations.reshape(1, -1)
    if len(points.shape) == 1:
        points = points.reshape(-1, 1)

    A = equations[:, 0:-1]
    b = np.dot(
        equations[:, [-1]], np.ones((1, points.shape[1]))
    )  # Faster than np.repeat, esp. for large sizes
    distances = np.dot(A, points) + b
    return distances


class Polytope:
    _polytope_engine = "cdd"
    # _polytope_engine = None
    _decimal_places = 3

    _equations = None
    _points = None

    def __init__(self, polytope_engine="ppl", decimal_places=3):
        self._polytope_engine = polytope_engine
        self._decimal_places = decimal_places

    # Construct a new polytope from a point (generator) set
    def set_points(self, points):
        if self._polytope_engine == "cdd":
            self._set_points_cdd(points)
        elif self._polytope_engine == "ppl":
            self._set_points_ppl(points)
        
        self._points = points
        # print("Set polytope points to:\n{}".format(points))

    # Construct a new polytope from an equation set
    def set_equations(self, equations):
        if self._polytope_engine == "cdd":
            self._set_equations_cdd(equations)
        elif self._polytope_engine == "ppl":
            self._set_equations_ppl(equations)
        
        self._equations = equations
        # print("Set polytope equations to:\n{}".format(equations))

    def _set_points_cdd(self, points):
        array = points.transpose()
        n_points = points.shape[1]
        array = np.concatenate((np.ones((n_points, 1)), array), axis=1)
        point_rows = list(array)
        point_matrix = cdd.Matrix(point_rows)
        point_matrix.rep_type = cdd.RepType.GENERATOR
        cdd_poly = cdd.Polyhedron(point_matrix)
        self._build_equations_from_cdd(cdd_poly)

    def _set_equations_cdd(self, equations):
        cdd_e_set = self._convert_equations_to_cdd(equations)
        e_matrix = cdd.Matrix(cdd_e_set)
        e_matrix.rep_type = cdd.RepType.INEQUALITY
        e_matrix.canonicalize()
        cdd_poly = cdd.Polyhedron(e_matrix)
        self._build_points_from_cdd(cdd_poly)

    def _set_points_ppl(self, points):
        # print("Setting polytope points")
        # print(points)
        # print("Truncated points:")
        gs = ppl.Generator_System()
        for pointT in points.T:
            # print(pointT)
            # Flatten the point and cutoff its precision for rational arithmetic
            point = pointT.reshape((-1)).round(self._decimal_places)
            multiplier = 10**self._decimal_places

            point_ppl_expression = ppl.Linear_Expression(point*multiplier, 0)
            ppl_point = ppl.point(point_ppl_expression, multiplier)
            # print(point_ppl_expression)
            # print(ppl_point)
            gs.insert(ppl_point)
        ppl_poly = ppl.C_Polyhedron(gs)
        self._build_equations_from_ppl(ppl_poly)
        # print(gs)
        # print(ppl_poly)
        # print(ppl_poly.constraints())
        # print(ppl_poly.generators())

    def _set_equations_ppl(self, equations):
        # print("Setting ppl equations")
        cs = ppl.Constraint_System()
        multiplier = 10**self._decimal_places
        scaled_equations = equations.round(self._decimal_places)*multiplier
        # print(scaled_equations)
        for equation in scaled_equations:
            equation_ppl_expression = ppl.Linear_Expression(equation[:-1], equation[-1])
            cs.insert(equation_ppl_expression >= 0)
        ppl_poly = ppl.C_Polyhedron(cs)
        # print(equations)
        # print(cs)
        # print(ppl_poly)
        self._build_points_from_ppl(ppl_poly)

    def _build_points_from_cdd(self, cdd_poly):
        points = []
        points_matrix = cdd_poly.get_generators()
        points = np.array(points_matrix[:])
        points_mask = points[:, 0] == 1  # Only rows starting with 1 are points
        points = points[points_mask, 1:]
        self._points = points.T

    def _build_points_from_ppl(self, ppl_poly):
        # print("Building points from polytope")
        ppl_points = ppl_poly.minimized_generators()
        dim = ppl_poly.space_dimension()
        points = np.zeros((dim, len(ppl_points)))
        # print(ppl_poly)
        # print(ppl_points)
        for idx, point_ppl in enumerate(ppl_points):
            coefficients = np.array(point_ppl.coefficients()).astype(float)
            divisor = np.array(point_ppl.divisor()).astype(float)
            points[:,idx] = coefficients/divisor
        # print(points)
        self._points = points

    def _build_equations_from_cdd(self, cdd_poly):
        e_matrix = cdd_poly.get_inequalities()
        try:
            self._equations = self._get_equations_from_cdd_matrix(e_matrix)
        except IndexError:
            raise IndexError(
                "Could not build cdd equations. e_matrix = {}".format(e_matrix)
            )

    def _build_equations_from_ppl(self, ppl_poly):
        # print("Building equations from polytope")
        ppl_equations = ppl_poly.minimized_constraints()
        dim = ppl_equations.space_dimension()
        equations = np.zeros((len(ppl_equations), dim+1))
        for idx, constraint in enumerate(ppl_equations):
            A = np.array(constraint.coefficients()).astype(float)
            b = np.array(constraint.inhomogeneous_term()).astype(float)
            # print(A, b)
            equations[idx,:-1] = A
            equations[idx, -1] = b
            # print(equations[idx,:])
        self._equations = equations

    def get_equations(self):
        return np.copy(self._equations)

    def get_points(self):
        return np.copy(self._points)

    def __contains__(self, point):
        if point.shape[1] > 1:
            raise IndexError('Use explicit "contains" method to test multiple points')
        return self.contains(point)

    # Explicit containment method for multidimensional output
    def contains(self, points):
        equations = self.get_equations()
        distances = evaluate_hyperplane(points, equations)
        answer = np.all(distances >= 0, axis=0)

        return answer

    # Return a new polytope which is scaled on its axes by the axis_multipliers
    # axis_mutlipliers is an nparray of dimension (D,), where D is the polytope dimension
    def scale(self, axis_multipliers):
        scale = axis_multipliers.reshape(-1, 1)
        scaled_points = self.get_points() * scale
        scaled_polytope = Polytope()
        scaled_polytope.set_points(scaled_points)
        return scaled_polytope

    def _convert_equations_to_cdd(self, e_array):
        converted_set = []
        for equation in e_array:
            dim = len(equation)
            A = equation[0:-1].reshape(dim - 1)
            b = equation[-1].reshape(1)
            converted_set.append(np.concatenate((b, A)))
        return converted_set

    def _convert_equations_from_cdd(self, cdd_e_list):
        num_equations = len(cdd_e_list)
        if len(cdd_e_list) == 0:
            raise IndexError("Equation list is empty")
        n_coeffs = len(cdd_e_list[0])
        converted_array = np.zeros((num_equations, n_coeffs))
        idx = 0

        for equation in cdd_e_list:
            A = np.array(equation[1:]).reshape(1, n_coeffs - 1)
            b = np.array(equation[0]).reshape(1, 1)
            converted_array[idx, :] = np.concatenate((A, b), axis=1)
            idx = idx + 1
        return converted_array

    def _get_equations_from_cdd_matrix(self, matrix):
        cdd_e_set = matrix[:]
        e_set = self._convert_equations_from_cdd(cdd_e_set)
        return e_set


class SafeConvexPolytope:

    domain = None
    indicator = None
    equations = None
    eps = None  # a (n_dim, ) array with the desired final sampling resolution
    angular_sample_no = 2 ** 5
    points_yes = None
    points_no = None
    enable_plotting = False
    wait_for_user = False

    _polytope = None
    _reduced_polytope = None
    _domain_polytope = None
    _normalized_domain = None
    _set_points = None
    _set_yes = None
    _set_no = None
    _n_dim = None
    _initialized = False
    _default_init_division = 2 ** 2  # Default initial division of the search space
    _default_final_division = 2 ** 4  # Default final division of the search space
    _normalized_eps = None
    _final_normalized_eps = 1
    _volume_approx_factor = 0.1
    _sampling_method = "radial"
    _patch_polytope_holes = True
    _samples_taken = 0
    _sample_decimal_places = 3

    _axis_handle = None
    axis_label_list = None
    plotting_mask = None

    def __init__(self, ind_func, domain, eps=None):
        self.indicator = ind_func
        self.domain = domain
        self._n_dim = domain.shape[0]
        self._set_points = set()
        self._set_yes = set()
        self._set_no = set()
        self._polytope = Polytope()
        self._domain_polytope = Polytope()
        # Set sampling precision values
        self.eps = eps
        self._normalize_domain(domain, eps)

        # Initialize the polytope and the domain polytope
        self._reset_polytope()

        self.axis_label_list = ["Variable {}".format(i) for i in range(self._n_dim)]
        self.plotting_mask = [i < 3 for i in range(self._n_dim)]

    @staticmethod
    def _array_to_set(points):
        if type(points) == set:
            return points
        else:
            return set(map(tuple, list(points.transpose())))

    @staticmethod
    def _set_to_array(points_set):
        return np.array(list(points_set)).transpose()

    @staticmethod
    def _align_to_grid(value, resolution):
        return np.sign(value) * (np.abs(value) // resolution) * resolution

    # Find unit vector joining two points
    @staticmethod
    def _get_unit_direction(point_1, point_2):
        return (point_1 - point_2) / np.linalg.norm(point_1 - point_2)

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

    # Check if a point is inside a rectilinear domain
    @staticmethod
    def _is_in_domain(point, domain):
        if np.any(point < domain[:, [0]]):
            return False
        if np.any(point > domain[:, [1]]):
            return False
        return True

    @staticmethod
    def _get_y(equation, x):
        return (-equation[0] * x - equation[2]) / equation[1]

    @staticmethod
    def _get_z(equation, x, y):
        known_vec = np.array([x, y]).reshape(2, 1)
        equation_vec = equation.reshape(len(equation), 1)
        return (
            - equation_vec[-1] - np.dot(equation_vec[0:-2, :].transpose(), known_vec)
        ) / equation_vec[-2]

    # Normalize the domain over eps to achieve a hypersquare grid
    def _normalize_domain(self, domain, eps):
        ranges = np.diff(domain, axis=1)
        # If eps is not provided
        if eps is None:
            eps = ranges / self._default_final_division
        # If eps is partially provided
        else:
            for idx in range(len(eps)):
                if eps[idx] is None:
                    eps[idx] = ranges[idx] / self._default_final_division

        # Scale the domain by the eps setting
        final_eps_ext = np.repeat(np.array(eps).reshape(self._n_dim, 1), 2, axis=1)
        self._normalized_domain = (domain / final_eps_ext).astype(int)
        normalized_ranges = np.diff(self._normalized_domain, axis=1)
        # Select the initial normalized eps
        self._normalized_eps = np.maximum(
            normalized_ranges / self._default_init_division,
            np.ones(normalized_ranges.shape),
        )
        print("Original domain given:\n{}".format(domain))
        print("Converted normalized domain:\n{}".format(self._normalized_domain))
        print("Original eps given:\n{}".format(eps))
        print("Initial eps:\n{}".format(self._normalized_eps))

    # Set the sampling resolution
    # TODO: Needs conversion to normalized logic
    def set_eps(self, eps):
        dimensions = self.domain.shape[0]
        if tuple([dimensions]) != eps.shape:
            raise IndexError("Eps dimension mismatch")
        self.eps = eps

    def set_sampling_method(self, method):
        self._sampling_method = method

    def _reset_polytope(self):
        print("Resetting Polytope")
        self._initialized = False
        # define an initial polygon
        initialization_points = np.zeros((self._n_dim, 2 ** self._n_dim))
        idx = 0
        for point_tuple in it.product(*tuple(self._normalized_domain)):
            initialization_points[:, idx] = np.array(point_tuple)
            idx += 1
        self._polytope.set_points(initialization_points)
        self._domain_polytope.set_points(initialization_points)
        self._reduced_polytope = None

    def _update_points(self):
        self.points_yes = self._set_to_array(self._set_yes)
        self.points_no = self._set_to_array(self._set_no)

    def clear_points(self):
        self._set_yes = set()
        self._set_no = set()
        self._set_points = set()
        self.points_yes = np.array([])
        self.points_no = np.array([])

    # Add points to the class
    # points is a point list
    # NOT USED
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

    def remove_yes_points(self, points):
        points_set = self._array_to_set(points)
        self._set_points -= points_set
        self._set_yes -= points_set
        self._update_points()

    def remove_no_points(self, points):
        points_set = self._array_to_set(points)
        self._set_points -= points_set
        self._set_no -= points_set
        self._update_points()

    # Find which of the input points are not already part of the known points
    def _filter_points(self, points):
        set_new_points = self._array_to_set(points) - self._set_points
        return np.array(set_new_points).transpose()

    # Find the index of yes-point in the self.point_yes array
    def _find_yes_point(self, point):
        return self._find_point_in_list(point, self.points_yes)

    # Find the index of no-point in the self.point_no array
    def _find_no_point(self, point):
        return self._find_point_in_list(point, self.points_no)

    # Find the index of the point in the point list
    # Returns a 1D numpy array with the indices where point is found
    @staticmethod
    def _find_point_in_list(point, point_list):
        ans = np.where((point_list == point).all(axis=0))[0]
        return ans

    @staticmethod
    def get_centroid(points):
        # compute centroid
        return points.mean(axis=1).reshape(-1, 1)

    # Sample a rectilinear grid, aligning with the coordinate center
    def _sample_rectilinear(self):
        sampling_domain = self._normalized_domain
        intervals = []
        for dim in range(self._n_dim):
            intervals.append(
                slice(
                    sampling_domain[dim, 0],
                    sampling_domain[dim, 1] + 1,
                    self._normalized_eps[dim],
                )
            )
        samples = np.mgrid[intervals].reshape(self._n_dim, -1)
        return samples

    # Return the coordinates which are up to eps away from the polytope boundary
    # Sampled from a rectilinear grid
    def _get_boundary_points(self, include_internal=False):
        vertices = self._polytope.get_points()
        bbox = self._get_bounding_box(vertices)

        num_intervals = np.zeros(self._n_dim)
        sample_points = []

        new_domain = np.zeros(bbox.shape)
        for dim_idx in range(self._n_dim):
            # Align the number resolution to eps
            unaligned_min = max(bbox[dim_idx, 0], self._normalized_domain[dim_idx, 0])
            eps = self._normalized_eps[dim_idx]
            aligned_min = self._align_to_grid(unaligned_min - eps, eps)
            new_domain[dim_idx, 0] = aligned_min

            unaligned_max = min(bbox[dim_idx, 1], self._normalized_domain[dim_idx, 1])
            eps = self._normalized_eps[dim_idx]
            aligned_max = self._align_to_grid(unaligned_max + eps, eps)
            new_domain[dim_idx, 1] = aligned_max

            # Construct the sampling values for each dimension
            n = int((aligned_max - aligned_min) // eps) + 1
            num_intervals[dim_idx] = n
            sample_points.append(np.linspace(aligned_min, aligned_max, n))

        result = []
        for coords_tuple in it.product(*sample_points):
            point = np.array(coords_tuple).reshape((self._n_dim, 1))

            if self._is_point_eligible(point, include_internal):
                result.append(point[:, 0].transpose())

        return np.array(result).transpose()

    # Sample boundary points in a radial grid
    def _get_boundary_points_radial(self):
        # Build unit vectors
        eps = self._normalized_eps
        samples = np.random.randn(self._n_dim, self.angular_sample_no)
        samples *= self._normalized_eps
        # samples = np.random.randn(self._n_dim, 1)
        samples /= np.linalg.norm(samples, axis=0)

        p = self.get_centroid(self.points_yes)
        if np.any(np.isnan(p)):
            raise RuntimeError("points_yes set is empty")

        # Transfer the polytope at the centroid
        all_equations = self._polytope.get_equations()
        all_equations[:, [-1]] += np.matmul(all_equations[:, :-1], p)

        A = all_equations[:, :-1]
        b = np.repeat(all_equations[:, [-1]], samples.shape[1], axis=1)

        # For each unit vector find its projection onto the hyperplane sand keep the closest one
        all_k = -b / np.dot(A, samples)
        all_k[all_k < 0] = np.inf
        k_indices = np.argmin((all_k), axis=0)
        k = all_k[k_indices, range(len(k_indices))]  # .reshape(1, -1)

        # Add 3 samples for each
        samples_boundary = k * samples  # Scale samples onto the bounding polytope
        samples_inside = (k - np.matmul(eps.T, samples)) * samples
        samples_outside = (k + np.matmul(eps.T, samples)) * samples
        samples = np.hstack((samples_boundary, samples_inside, samples_outside)) + p

        # Force out-of bounds samples onto boundary
        # Not needed because polytope includes boundary constraints
        # lower_bound = np.repeat(
        #     self._normalized_domain[:, [0]], samples.shape[1], axis=1
        # )
        # upper_bound = np.repeat(
        #     self._normalized_domain[:, [1]], samples.shape[1], axis=1
        # )
        # lower_bound_mask = np.all(samples > lower_bound, axis=0)
        # upper_bound_mask = np.all(samples < upper_bound, axis=0)
        # mask = lower_bound_mask & upper_bound_mask
        # samples = samples[:, mask]

        return samples

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
            n_0 = n / np.linalg.norm(n)
            normal_vector = evaluate_hyperplane(point, equation) * n_0
            distances = np.array(map(np.abs, normal_vector))
            if np.all(distances < self._normalized_eps):
                return True

    # Sample points around a polytope boundary
    def sample(self, init=False, method="rectilinear"):
        if init:  # First-pass sample of the grid
            print("Initial rectilinear sampling")
            sampled_points = self._array_to_set(self._sample_rectilinear())
        else:
            if method == "rectilinear" or init:
                sampled_points = self._array_to_set(self._get_boundary_points(init))
                # Do not add points which are already sampled
                sampled_points -= self._set_points
            elif method == "radial":
                sampled_points = self._array_to_set(self._get_boundary_points_radial())
                # No need to check if samples already exist, because they are sampled randomly

        self._samples_taken += len(sampled_points)
        yes_points = []
        no_points = []
        for coords in sampled_points:
            point = np.array(coords).reshape(-1, 1)
            coords_tuple = tuple(point.transpose()[0, :])
            # Sample the point to find in which set it belongs
            # Scale the point to real coordinates
            if self._is_in_domain(point, self._normalized_domain):
                sample = self.indicator(point * self.eps.reshape(point.shape))
                if sample:
                    yes_points.append(coords_tuple)
                else:
                    no_points.append(coords_tuple)
            else:
                no_points.append(coords_tuple)

        self._set_yes.update(yes_points)
        self._set_no.update(no_points)
        self._set_points = self._set_yes | self._set_no
        self._update_points()

    # Restrict points on the domain boundary
    def _restrict_domain_boundary(self):
        sampling_domain = self._normalized_domain
        samples_list = []
        # Build the slice list
        intervals = []
        for dim in range(self._n_dim):
            intervals.append(
                slice(
                    sampling_domain[dim, 0],
                    sampling_domain[dim, 1] + 1,
                    self._normalized_eps[dim],
                )
            )
        # Iterate over all dimensions and capture the domain end-points
        for dim in range(self._n_dim):
            # Pick one dimension to enforce boundary condition
            # Sample min
            temp_intervals = list(intervals)
            temp_intervals[dim] = slice(
                sampling_domain[dim, 0], (sampling_domain[dim, 0] + 1)
            )
            samples_list.append(np.mgrid[temp_intervals].reshape(self._n_dim, -1))
            # Sample max
            temp_intervals = list(intervals)
            temp_intervals[dim] = slice(
                sampling_domain[dim, 1], (sampling_domain[dim, 1] + 1)
            )
            samples_list.append(np.mgrid[temp_intervals].reshape(self._n_dim, -1))

        boundary_samples = np.hstack(samples_list)

        self._set_no.update(self._array_to_set(boundary_samples))
        self._set_points = self._set_yes | self._set_no
        self._update_points()

    def _calc_optimal_offset(self, w):
        b_array = np.dot(w.transpose(), self.points_yes)
        return -np.min(b_array, axis=1)

    # Calculate optimal offset but have offset less than a predefined value
    def _calc_optimal_offset_constrained(self, w, bound):
        b_array = np.dot(w.transpose(), self.points_yes)
        b_array = b_array[b_array > bound]
        return -np.min(b_array)

    # Create a convex separating polytope
    def _convex_separator(self):
        # Set halfplane set (E-set) to empty
        e_list = []
        # Set no-points dinstances to inf
        s = float("inf") * np.ones(len(self._set_no))
        # get centroid
        p = self.get_centroid(self.points_yes)
        if np.any(np.isnan(p)):
            raise IndexError(
                "Could not obtain centroid for convex separator. Size of points_yes: {}".format(
                    self.points_yes.shape
                )
            )

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
            e_list.append(list(equation.reshape(-1)))
            # Update distances of no-set
            current_distances = evaluate_hyperplane(self.points_no, equation)
            d_len = current_distances.shape[1]
            range_mask = s > current_distances[0, np.arange(d_len)]
            s[range_mask] = current_distances[0, range_mask]
            # for idx in range(s.shape[0]):
            #     s[idx] = min(s[idx], current_distances[0, idx])
            s[farthest_point_idx] = 0

        convex_polytope = Polytope()
        e_array = np.array(e_list)
        domain_equations = self._domain_polytope.get_equations()
        # print("Resulting equations")
        # print(e_array)
        # print("Additional domain equation")
        # print(domain_equations)

        # Add the domain constraints to make sure polytope remains bounded
        e_array = np.vstack((e_array, domain_equations))
        # e_list += list(self._domain_polytopeget_equations())
        convex_polytope.set_equations(e_array)
        return convex_polytope

    # Return a hyperplane separating no_point from the centroid of set_yes
    def _get_bounded_separator(self, no_point):
        p = self.get_centroid(self.points_yes)
        w = self._get_unit_direction(p, no_point)
        bound = np.dot(w.transpose(), no_point)
        b = self._calc_optimal_offset_constrained(w, bound)
        return list(np.append(w.transpose(), b))

    def _get_polytope_volume(self):
        points = self._polytope.get_points()
        hull = ConvexHull(points.T, qhull_options="QJ")
        # hull = ConvexHull(points.T)
        return hull.volume

    def _get_polytope_volume_approximate(self):
        # Calculate the volume of the circumscribed axis-aligned rectangle
        points = self._polytope.get_points()
        mins = np.min(points, axis=1)
        maxs = np.max(points, axis=1)
        return reduce(np.multiply, maxs-mins)

    # Return a list containing one MxN array for each face of the polytope
    # where M is the domain dimension and N is the number of points on each face, which may vary
    def _get_face_points(self, polytope):
        face_list = []
        all_points = polytope.get_points()
        all_equations = polytope.get_equations()
        # print("Getting face points")
        # print(all_points)
        # print(all_equations)
        multiplier = 10**(-3) # Same as significant decimal places of polyotpe

        for equation in all_equations:
            equation_normalized = equation/np.linalg.norm(equation)
            face_points = []
            # print("New face")
            for point in all_points.transpose():
                point_distance = np.abs(evaluate_hyperplane(point, equation_normalized))
                # print(point_distance)
                if point_distance < multiplier:
                    face_points.append(point)
            face_list.append(face_points)
        return face_list

    # Create a convex polytope around elements of set_yes that contains zero elements of set_no
    def build_safe_polytope(self):
        # Calculate the volume of the old polytope, as an index of approximation convergence
        print('Calculating existing polytope volume')
        old_volume = self._get_polytope_volume_approximate()

        # Create a convex polytope around set_yes
        print("Creating first-pass convex polytope")
        convex_polytope = self._convex_separator()
        e_array = convex_polytope.get_equations()

        # Find all the invalid points inside the polytope
        invalid_points = []
        invalid_points_mask = np.zeros((self.points_no.shape[1]), bool)
        idx = 0
        for coords in self.points_no.transpose():
            no_point = coords.reshape(self._n_dim, 1)
            if no_point in convex_polytope:
                invalid_points.append(no_point)
                invalid_points_mask[idx] = True
            idx += 1

        if self._patch_polytope_holes:
            # Remove invalid points which are completely inside the convex shape and violate the no-holes assumption
            print("Patching holes in the polytope")
            invalid_points = self._filter_false_invalid_points(
                invalid_points, invalid_points_mask
            )
            if self.enable_plotting:
                self.plot()

        # Create one new equation for each one, separating them from the majority of set-yes
        new_e_set = []
        for point in invalid_points:
            e = self._get_bounded_separator(point)
            new_e_set.append(e)
        new_e_array = np.array(new_e_set)
        if len(new_e_array) > 0:
            e_array = np.vstack((e_array, new_e_array))

        # Add the domain constraints
        e_array = np.vstack((e_array, self._domain_polytope.get_equations()))

        # Construct a new polytope from the new equation set
        self._polytope.set_equations(e_array)

        # print('Calculating new polytope volume')
        new_volume = self._get_polytope_volume_approximate()
        if abs((new_volume - old_volume) / old_volume) < self._volume_approx_factor:
            return True
        else:
            return False

    # Takes a list of invalid points and removes them if they are holes in the polytope
    # invalid_points is a list of points
    # invalid_points_mask is a mask for self.points_no where invalid points reside
    def _filter_false_invalid_points(self, invalid_points, invalid_points_mask):
        valid_no_points = self.points_no[:, np.invert(invalid_points_mask)]
        # For each invalid point
        internal_no_points = []
        no_points_to_remove = set()
        for no_point in invalid_points:
            # Find its distances from the valid_no_points
            distances = np.abs(valid_no_points - no_point)
            # If it's close to any valid no-point, then leave it
            if np.any(np.all(distances <= self._normalized_eps, axis=0)):
                internal_no_points.append(no_point)
                # print("Point {} is valid".format(no_point))
            else:
                # If it's close to some valid no-point, then leave it
                no_points_to_remove.add(tuple(no_point.reshape(-1)))
                # print("Removing point {} ".format(no_point))

        self.remove_no_points(no_points_to_remove)

        return internal_no_points

    # Delete points which are more than eps away from boundaries
    def remove_distant_points(self):
        all_equations = self._polytope.get_equations()
        # all_points = np.hstack((self.points_yes, self.points_no))

        # Evaluate the distances of k hypersurface from m points (k x m array)
        yes_distances = np.abs(evaluate_hyperplane(self.points_yes, all_equations))
        eps_array = np.max(self._normalized_eps) * np.ones(yes_distances.shape)
        points_to_keep_mask = np.any(yes_distances <= eps_array, axis=0)
        self.points_yes = self.points_yes[:, points_to_keep_mask]

        no_distances = np.abs(evaluate_hyperplane(self.points_no, all_equations))
        eps_array = np.max(self._normalized_eps) * np.ones(no_distances.shape)
        points_to_keep_mask = np.any(no_distances <= eps_array, axis=0)
        self.points_no = self.points_no[:, points_to_keep_mask]

        self._set_yes = self._array_to_set(self.points_yes)
        self._set_no = self._array_to_set(self.points_no)
        self._set_points = self._set_yes | self._set_no

    # Delete points which are more than eps away from the boundary of the polytope
    # judging radially from the centroid
    def remove_distant_points_2(self):
        # get centroid
        p = self.get_centroid(self.points_yes)
        # Set contraction distance
        eps_out = 1 * np.max(self._normalized_eps)
        eps_in = 1 * np.max(self._normalized_eps)

        # Contract yes points and delete outside points
        recentered_yes_points = self.points_yes - p
        norms_yes = np.linalg.norm(recentered_yes_points, axis=0).reshape(1, -1)
        factors_yes = np.repeat((norms_yes - eps_out) / norms_yes, p.shape[0], axis=0)
        # Dont' allow the contracted point to fly to the other side of p
        factors_yes = np.maximum(factors_yes, np.zeros(factors_yes.shape))
        contracted_yes_points = recentered_yes_points * factors_yes + p
        points_to_keep_mask = self._polytope.contains(contracted_yes_points)
        self.points_yes = self.points_yes[:, points_to_keep_mask]

        # Contract no points and delete outside points
        if len(self.points_no) > 0:
            recentered_no_points = self.points_no - p
            norms_no = np.linalg.norm(recentered_no_points, axis=0).reshape(1, -1)
            factors_no = np.repeat((norms_no - eps_out) / norms_no, p.shape[0], axis=0)
            # Dont' allow the contracted point to fly to the other side of p
            factors_no = np.maximum(factors_no, np.zeros(factors_no.shape))
            contracted_no_points = recentered_no_points * factors_no + p
            points_to_keep_mask = self._polytope.contains(contracted_no_points)
            self.points_no = self.points_no[:, points_to_keep_mask]

        # Contract yes points and delete outside points
        recentered_yes_points = self.points_yes - p
        norms_yes = np.linalg.norm(recentered_yes_points, axis=0).reshape(1, -1)
        factors_yes = np.repeat((norms_yes + eps_in) / norms_yes, p.shape[0], axis=0)
        # Dont' allow the contracted point to fly to the other side of p
        factors_yes = np.maximum(factors_yes, np.zeros(factors_yes.shape))
        contracted_yes_points = recentered_yes_points * factors_yes + p
        points_to_keep_mask = np.invert(self._polytope.contains(contracted_yes_points))
        self.points_yes = self.points_yes[:, points_to_keep_mask]

        # Contract no points and delete outside points
        if len(self.points_no) > 0:
            recentered_no_points = self.points_no - p
            norms_no = np.linalg.norm(recentered_no_points, axis=0).reshape(1, -1)
            factors_no = np.repeat((norms_no + eps_in) / norms_no, p.shape[0], axis=0)
            # Dont' allow the contracted point to fly to the other side of p
            factors_no = np.maximum(factors_no, np.zeros(factors_no.shape))
            contracted_no_points = recentered_no_points * factors_no + p
            points_to_keep_mask = np.invert(
                self._polytope.contains(contracted_no_points)
            )
            self.points_no = self.points_no[:, points_to_keep_mask]

        self._set_yes = self._array_to_set(self.points_yes)
        self._set_no = self._array_to_set(self.points_no)
        self._set_points = self._set_yes | self._set_no

    def cluster(self, num_vertices):
        vertices = self._polytope.get_points()
        centroids, _ = kmeans2(vertices.transpose(), num_vertices, minit="points")
        self._reduced_polytope = Polytope()
        self._reduced_polytope.set_points(centroids.transpose())

    def _decrease_eps(self):
        self._normalized_eps = np.maximum(
            self._normalized_eps / 2, np.ones(self._normalized_eps.shape)
        )

    # Make a step of the polytope fitting problem
    def step(self):
        # if not self._initialized:
        #     # Restrict the domain boundaries at the current resolution
        #     self._restrict_domain_boundary()

        try:
            convergence = self.improve_polytope(not self._initialized)
            if self.enable_plotting:
                self.plot()
        except RuntimeError as e:
            print(e)
            # If this is the first iteration, increase initial sampling and try again
            if not self._initialized:
                warn(
                    """Could not build safe polytope on the first try.
                Increasing resolution and trying again.
                Consider providing a tighter fit of the domain to the indicator function."""
                )
                self._decrease_eps()
                return False
            else:
                warn(
                    """
                Could not build safe polytope.
                Resetting and resampling.
                """
                )
                self._reset_polytope()
                return False
        except QhullError as e:  # QHull could not build a hull out of the previous or current safe polytope
            # It should not be because of the previous polytope, because it would have crashed in the previous iteration (as current polytope)
            warn(
                "QHull could not build a convex polytope out of the polytopes",
                UserWarning,
            )
            self._reset_polytope()
            return False

        print("Removing distant points")
        self.remove_distant_points_2()
        if self.points_yes.shape[1] == 0:
            warn(
                "Could not build a convex polytope from the remaining points_yes. Increasing resolution and resetting..."
            )
            self._decrease_eps()
            self._reset_polytope()

        if not self._initialized:
            self._initialized = True

        if self.enable_plotting:
            self.plot()

        algorithm_end = False
        if convergence:
            print("Polytope converged in the current eps value")
            if np.all(self._normalized_eps <= self._final_normalized_eps):
                print("Desired polytope accuracy achieved")
                algorithm_end = True
            else:
                print("Increasing sampling accuracy")
                self._decrease_eps()
                print("New normalized eps:\n{}".format(self._normalized_eps))

        return algorithm_end

    # Take more samples and re-fit the safe convex polygon
    def improve_polytope(self, init=False):
        print("Getting polytope samples")
        self.sample(init=init, method=self._sampling_method)

        # Add boundary no-points if none were found
        if len(self.points_no) == 0:
            print("No points_no exist in samples, adding boundaries...")
            self._restrict_domain_boundary()

        if self.enable_plotting:
            print("Plotting new sampled points")
            self.plot()

        try:
            print("Finding new safe convex polytope")
            convergence = self.build_safe_polytope()
            return convergence
        except IndexError:
            raise RuntimeError("Could not build a safe polytope")
        except QhullError as e:
            print("QHull could not create a hull")
            raise e

    def new_plot(self):
        self._axis_handle = None
        self.plot()

    @staticmethod
    def _slice_face_points(face_points, dim_slice):
        # print("Slicing points")
        # print(face_points)
        new_face_points = []
        for idx_outer in range(len(face_points)):
            point_list = face_points[idx_outer]
            new_point_list = []
            for idx_inner in range(len(point_list)):
                point = point_list[idx_inner]
                print(point)
                new_point_list.append(point[dim_slice])
            new_face_points.append(new_point_list)
        return new_face_points

    def _plot_polytope_2d(self, ah, domain, polytope, color="k"):
        # It would be nicer if I could plot the polytope given and not intervene
        # Maybe introduce a scaling method for Polytope class?
        x_min = self.domain[0, 0]
        x_max = self.domain[0, 1]
        y_min = self.domain[1, 0]
        y_max = self.domain[1, 1]

        equations = polytope.get_equations()
        for equation in equations:
            try:
                point_1 = np.array([x_min, self._get_y(equation, x_min)])
            except (FloatingPointError, ZeroDivisionError):
                point_1 = [x_min, y_min]
            try:
                point_2 = [x_max, self._get_y(equation, x_max)]
            except (FloatingPointError, ZeroDivisionError):
                point_2 = [x_max, y_max]
            pu.plot_line(ah, point_1, point_2, color)

    def plot(self, color="k"):
        print("Plotting")
        if self._n_dim > 3:
            warn("Plotting a slice of polytope in 3D space", UserWarning)

        # Create axis handle
        if self._axis_handle is None:
            self._figure_handle = plt.figure()
            fig = self._figure_handle
            # figcanvas.mpl_connect('key_press_event', self._figure_event_callback)
            if self._n_dim == 2:
                self._axis_handle = fig.add_subplot(111)
            if self._n_dim >= 3:
                self._axis_handle = fig.add_subplot(
                    111, projection="3d", proj_type="ortho"
                )
        ah = self._axis_handle
        ah.clear()

        # Use non-normalized coordinates
        domain = self.domain[self.plotting_mask, :]
        eps = self.eps.reshape(self._n_dim, 1)
        polytope = self._polytope.scale(eps)
        if self._reduced_polytope is None:
            reduced_polytope = None
        else:
            reduced_polytope = self._reduced_polytope.scale(eps)
        points_yes = self.points_yes * eps
        points_no = self.points_no * eps
        if len(self.points_yes) > 0: 
            p = self.get_centroid(self.points_yes) * eps

        # Build the axis label list
        label_list = list(it.compress(self.axis_label_list, self.plotting_mask))

        # 2D plotting
        if self._n_dim == 2:
            x_min = domain[0, 0]
            x_max = domain[0, 1]
            y_min = domain[1, 0]
            y_max = domain[1, 1]

            pu.plot_points(ah, points_yes, "o", "g")
            pu.plot_points(ah, points_no, "X", "r")
            if len(self.points_yes) > 0: 
                pu.plot_points(ah, p, "D", "b")
            ah.set_xlim(x_min, x_max)
            ah.set_ylim(y_min, y_max)
            ah.xaxis.set_minor_locator(
                mpl.ticker.MultipleLocator(self._normalized_eps[0] * eps[0])
            )
            ah.yaxis.set_minor_locator(
                mpl.ticker.MultipleLocator(self._normalized_eps[1] * eps[1])
            )
            ah.set_xlabel(label_list[0])
            ah.set_ylabel(label_list[1])
            ah.grid(True, which="minor")

            # Plot polytope half-planes
            self._plot_polytope_2d(ah, domain, polytope)

            # Plot reduced polytope half-planes
            if self._reduced_polytope is not None:
                self._plot_polytope_2d(ah, domain, reduced_polytope, color="r")

        # 3D plotting
        if self._n_dim >= 3:
            # Get the plotting domain
            x_min = domain[0, 0]
            x_max = domain[0, 1]
            y_min = domain[1, 0]
            y_max = domain[1, 1]
            z_min = domain[2, 0]
            z_max = domain[2, 1]

            # Plot the points
            if self._n_dim == 3:
                pu.plot_points_3(ah, points_yes, "o", "g")
                pu.plot_points_3(ah, points_no, "X", "r", alpha=0.2)
            else:
                pu.plot_points_3(ah, points_yes[self.plotting_mask, :], "o", "g")
                pu.plot_points_3(
                    ah, points_no[self.plotting_mask, :], "X", "r", alpha=0.2
                )

            # Get the object to plot
            if self._reduced_polytope is not None:
                temp_polytope = reduced_polytope
            else:
                temp_polytope = polytope
            face_points = self._get_face_points(temp_polytope)
            # print("Face points size: {}".format(len(face_points)))
            # print(face_points)

            # Set the colors and transparency
            if self._reduced_polytope is None:
                colorcode = color
                alpha = 0.3
                # Trasparency not working, probably due to matplotlib bug: https://github.com/matplotlib/matplotlib/issues/10237
            else:
                colorcode = "r"
                alpha = None

            # Slice extra dimensions if needed
            if self._n_dim > 3:
                print("Slicing coordinates to plot in 3D space")
                face_points = self._slice_face_points(face_points, self.plotting_mask)

            # Plot the convex polytope
            pu.plot_polygons_3(ah, face_points, colorcode=colorcode, alpha=alpha)

            ah.set_xlim(x_min, x_max)
            ah.set_ylim(y_min, y_max)
            ah.set_zlim(z_min, z_max)
            # Minor locators for 3D axes not implemented yet: https://stackoverflow.com/questions/3910517/mplot3d-how-do-i-display-minor-ticks
            ah.xaxis.set_minor_locator(
                mpl.ticker.MultipleLocator(self._normalized_eps[0] * eps[0])
            )
            ah.yaxis.set_minor_locator(
                mpl.ticker.MultipleLocator(self._normalized_eps[1] * eps[1])
            )
            ah.zaxis.set_minor_locator(
                mpl.ticker.MultipleLocator(self._normalized_eps[2] * eps[2])
            )
            ah.set_xlabel(label_list[0])
            ah.set_ylabel(label_list[1])
            ah.set_zlabel(label_list[2])
            ah.grid(True, which="minor")

        plt.draw()
        plt.pause(0.01)
        if self.wait_for_user:
            plt.waitforbuttonpress(timeout=-1)

        # Return the axis handle for later use
        return ah


# Answer if p is inside the hypersphere c, r
def hypersphere(p):
    c = 4 * np.ones(p.shape)
    r = 4
    distance = np.linalg.norm(p - c)
    # print('Evaluated point {} with distance {} as {}'.format(p, distance, distance<=r))
    return distance <= r


def hypercube(p):
    c = 4 * np.ones(p.shape)
    r = 4
    distance = np.max(np.abs(p - c))
    return distance <= r


def hypertriangle(p):
    c = 0 * np.ones(p.shape)
    r = 8
    if np.any(p - c < 0):
        return False
    if np.sum(p, axis=0) > r:
        return False
    return True


@click.command()
@click.option(
    "-d", "--dimensions", default=2, type=int, help="Dimensions of the problem"
)
@click.option(
    "-p", "--plot", is_flag=True, help="Enable plotting. Only for 2D and 3D problems."
)
@click.option(
    "-i",
    "--interactive",
    is_flag=True,
    help="Wait for user input after each plot refresh",
)
@click.option(
    "-s",
    "--shape",
    type=click.Choice(["sphere", "cube", "triangle"]),
    default="sphere",
    help="Select the indicator function (shape) to approximate.",
)
@click.option(
    "-e",
    "--polytope-engine",
    type=click.Choice(["cdd", "ppl"]),
    default="cdd",
    help="Select the polytope engine module",
)
def test_code(dimensions, plot, interactive, shape, polytope_engine):

    n_dim = dimensions
    print("Testing polytope code for {} dimensions".format(n_dim))
    flag_plot = plot

    # define the indicator function to approximate
    if shape == "sphere":
        ind_func = hypersphere
    elif shape == "cube":
        ind_func = hypercube
    elif shape == "triangle":
        ind_func = hypertriangle

    # define a domain as a n_dim x 2 array
    print("Creating problem domain and sampling initial points")
    domain = np.zeros((n_dim, 2))
    # domain[:, 0] = -(np.arange(n_dim) + 5)
    # domain[:, 1] = np.arange(n_dim) + 5

    domain[:, 0] = -3
    domain[:, 1] = 10

    # domain[0, 0] = 0
    # domain[0, 1] = 10
    # domain[1, 0] = 0
    # domain[1, 1] = 50

    # Set the desired precision
    eps = 1 * np.ones(n_dim)

    # Initialize the polytope
    safe_poly = SafeConvexPolytope(ind_func, domain, eps)
    # safe_poly.set_eps(eps)
    safe_poly.set_sampling_method("radial")

    # Manually add random
    point_1 = np.random.rand(n_dim).reshape(n_dim, 1)
    point_1 = 10 * (point_1 - 0.5 * np.ones((n_dim, 1)))
    point_2 = np.random.rand(n_dim).reshape(n_dim, 1)
    point_2 = 10 * (point_2 - 0.5 * np.ones((n_dim, 1)))
    additional_yes_points = np.concatenate((point_1, point_2), axis=1)
    safe_poly.add_yes_points(additional_yes_points)

    if flag_plot:
        safe_poly.enable_plotting = True
    if interactive:
        safe_poly.wait_for_user = True

    # Iteratively sample the polytope
    print("Progressive sampling of the polytope")
    algorithm_end = False
    while not algorithm_end:
        print("New safe convex polytope iteration")
        algorithm_end = safe_poly.step()

    print("Final number of sampled points: {}".format(len(safe_poly._set_points)))
    print("Total number of samples taken: {}".format(safe_poly._samples_taken))

    # Approximate the safe convex polytope with k vertices
    print("Performing clustering")
    safe_poly.cluster(2 ** n_dim)

    if flag_plot:
        print("Plotting")
        safe_poly.plot()
        plt.show()


if __name__ == "__main__":
    test_code()
