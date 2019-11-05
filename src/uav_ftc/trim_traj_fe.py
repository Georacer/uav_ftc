#!/usr/bin/python
#
# # Trim the aircraft model to find the domain where a trim input set exists

import numpy as np
from numpy import cos, sin
from scipy.optimize import minimize
import xarray as xr
import itertools as it

# import xyzpy as xyz
import os
import time
import matplotlib as mpl
import matplotlib.pyplot as plt
import click
import imp

import uav_model as mdl  # Import UAV model library
import plot_utils as plu  # Import plotting utilities
import polytope_utils as poly  # Used for SafeConvexPoly class

tcpp = imp.load_source(
    "trimmer",
    "/home/george/ros_workspaces/uav_ftc/src/last_letter/last_letter_lib/src/trimmer.py",
)


# Raise an error when invalid floating-point operations occur
np.seterr(invalid="raise")


class OptimResult:
    def __init__(self, x_star, cost, success):
        self.x = x_star
        self.fun = cost
        self.success = success


class Trimmer:

    des_trajectory = None  # A len-3 list with desired Va, Gamma, R

    # bound_phi = (np.deg2rad(-5), np.deg2rad(5))  # Allow only upright flight
    # bound_theta = (np.deg2rad(-5), np.deg2rad(5))
    # bound_Va = (13, 15)
    # bound_alpha = (np.deg2rad(0), np.deg2rad(2))
    # bound_beta = (np.deg2rad(0), np.deg2rad(0))
    # bound_p = (0, 0)
    # bound_q = (0, 0)
    # bound_r = (0, 0)
    bound_deltaa = (-1, 1)
    bound_deltae = (-1, 1)
    bound_deltat = (0, 1)
    bound_deltar = (-1, 1)

    optim_algorithm = None
    optim_cost_threshold = None
    optim_time_accumulator = 0  # Store the total time spend finding trim points

    def __init__(self):

        # Set default values

        # Setup optimization method
        print("Building nlopt_cpp for trim calculations")
        self.nlopt_cpp_trimmer = tcpp.TrimmerState("skywalker_2013_mod")
        self.optim_algorithm = self.nlopt_cpp_optimize
        self.optim_cost_threshold = 1

    def nlopt_cpp_optimize(self, trajectory):
        trim_values = self.nlopt_cpp_trimmer.find_trim_values(trajectory)
        success = trim_values[8] > 0.1
        return OptimResult(trim_values[0:7], trim_values[7], success)

    # Indicator function for use with polytope discovery module
    def indicator(self, point):
        # point is a 3x1 numpy array with trim Va, gamma, psi_dot values
        R = point[0,0]*np.cos(point[1,0])/point[2,0]  # R = Va/R*cos(gamma)
        trajectory = np.array((point[0,0], point[1,0], R))
        trim_values, cost, success = self.find_trim_input(trajectory)
        
        # Extract the control input values
        delta_a = trim_values[3]
        delta_e = trim_values[4]
        delta_t = trim_values[5]
        delta_r = trim_values[6]

        optim_mask = success
        cost_mask = cost < self.optim_cost_threshold

        da_mask = (delta_a >= self.bound_deltaa[0]) * (
            delta_a <= self.bound_deltaa[1]
        )
        de_mask = (delta_e >= self.bound_deltae[0]) * (
            delta_e <= self.bound_deltae[1]
        )
        dt_mask = (delta_t >= self.bound_deltat[0]) * (
            delta_t <= self.bound_deltat[1]
        )
        dr_mask = (delta_r >= self.bound_deltar[0]) * (
            delta_r <= self.bound_deltar[1]
        )
        input_mask = da_mask * de_mask * dt_mask * dr_mask
        overall_mask = optim_mask * cost_mask * input_mask

        # if optim_mask and not cost_mask:
        #     print('Rejected solution due to cost')

        # if optim_mask and not input_mask:
        #     print('Rejected solution due to input constraint violation')

        return overall_mask

    # Return a wrapped indicator function for use in SafeConvexPolytope operations
    def get_indicator(self, list_defaults, list_fixed):
        def indicator_wrapper(point):
            # point: a Mx1 numpy array
            if len(list_fixed) - sum(list_fixed) != point.shape[0]:
                raise AssertionError(
                    "Provided non-fixed parameters are not as many as free dimension"
                )

            # Populate the trimmer function arguments
            trajectory = np.zeros((len(list_defaults), 1))
            idx2 = 0
            for idx in range(len(list_defaults)):
                if list_fixed[idx]:
                    trajectory[idx, 0] = list_defaults[idx]
                else:
                    trajectory[idx, 0] = point[idx2]
                    idx2 += 1
            return self.indicator(trajectory)

        return indicator_wrapper

    def find_trim_input(self, trajectory, verbose=False):
        # Find a trim input wich satisfies the trim state
        # Returns: The optimal trim input
        # The final optimization cost
        # The success flag

        # Perform the optimization step and time execution
        t_start = time.time()
        res = self.optim_algorithm(trajectory)
        t_end = time.time()
        self.optim_time_accumulator = self.optim_time_accumulator + (t_end - t_start)

        if res.success:
            optim_result_s = "SUCCESS"
        else:
            optim_result_s = "FAILURE"

        if verbose:
            print("Optimization result: {}\n".format(optim_result_s))
            print(
                """
                Optimization error: {}
                """.format(
                    res.fun
                )
            )
            print("Optimization result:\n{}".format(res.x))

        return (res.x, res.fun, res.success)


# def find_nav_trim(points_vector):
#     return np.apply_along_axis(state_to_nav_vec, 1, points_vector)


# def state_to_nav(Va, alpha, beta, phi, theta, r):
#     # Convert from aircraft trim state to navigation triplet Va, R, gamma
#     R_max = 1000

#     gamma = theta - alpha  # Disregards wind
#     R = Va / r * cos(gamma) * cos(phi) * cos(theta)  # Disregards wind
#     if np.isinf(R):
#         R = R_max

#     return Va, gamma, R


# def state_to_nav_vec(states):
#     return state_to_nav(*tuple(states))

def print_c_arrays(eqn_array):
    string = ''
    for i, eqn in enumerate(eqn_array):
        string += 'const double {}a{}[{}] = {{'.format(' '*(2-len(str(i))), i, eqn_array.shape[1]-1)
        string += ', '.join('{:>10.1f}'.format(c) for c in eqn)
        string += '};\n'
    return string


@click.command()
@click.option(
    "-p", "--plot", is_flag=True, help="Enable plotting. Only for 2D and 3D problems."
)
@click.option(
    "-i",
    "--interactive",
    is_flag=True,
    help="Wait for user input after each plot refresh",
)
@click.option("-e", "--export", is_flag=True, help="Export relevant data in .csv files")
def test_code(plot, interactive, export):
    trimmer = Trimmer()

    # Trajectory Trimming
    # Va_des = 15
    # gamma_des = np.deg2rad(0)
    # R_des = 100
    # trimmer.setup_trim_trajectory(Va_des, gamma_des, R_des)
    # (trim_state, trim_inputs) = trimmer.find_trim_state()

    # Provide default values
    Va_des = 10
    gamma_des = np.deg2rad(0)
    psi_dot_des = np.infty
    list_defaults = [Va_des, gamma_des, psi_dot_des]

    # Set search degrees of freedom
    fix_Va = False
    fix_gamma = False
    fix_R = False

    list_fixed = [fix_Va, fix_gamma, fix_R]

    # Provide domain bounds
    Va_min = 5.0
    Va_max = 25.0
    gamma_min = -30  # degrees
    gamma_max =  30  # degrees
    R_min = 100 # meters

    psi_dot_max = Va_max/R_min

    Va_domain = (Va_min, Va_max)
    gamma_domain = (np.deg2rad(gamma_min), np.deg2rad(gamma_max))
    psi_dot_domain = (-psi_dot_max, psi_dot_max)

    domain = np.array(
        [Va_domain, gamma_domain, psi_dot_domain]
    )
    current_domain = domain[np.invert(list_fixed)]
    n_dim = len(current_domain)

    # Create indicator function
    indicator = trimmer.get_indicator(list_defaults, list_fixed)

    # Select eps values
    eps_Va = 2
    eps_gamma = np.deg2rad(5)
    eps_psi_dot = 0.05
    eps = np.array([eps_Va, eps_gamma, eps_psi_dot])
    current_eps = eps[np.invert(list_fixed)]

    # Initialize starting time
    t_start = time.time()

    # Initialize the polytope
    safe_poly = poly.SafeConvexPolytope(indicator, current_domain, current_eps)

    # Pass the variable strings
    string_Va = "Va"
    string_gamma = "Gamma"
    string_R = "R"
    string_list = [
        string_Va,
        string_gamma,
        string_R
    ]
    safe_poly.axis_label_list = list(it.compress(string_list, np.invert(list_fixed)))
    # safe_poly.plotting_mask = [False, True, True, True]

    # User interface options
    if plot:
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
    print("Total optimization time required: {}".format(trimmer.optim_time_accumulator))

    # Approximate the safe convex polytope with k vertices
    print("Performing clustering")
    # cluster_num = 2 ** n_dim
    # cluster_num = 2 * (n_dim-3)
    if n_dim == 2:
        cluster_num = 4
    elif n_dim == 3:
        cluster_num = 8

    safe_poly.cluster(cluster_num)

    t_end = time.time()
    print("Total script time: {}".format(t_end - t_start))

    # Print human-readable polytope equations
    unscaled_polytope = safe_poly._reduced_polytope.scale(
        safe_poly.eps.reshape(safe_poly._n_dim, 1)
    )
    eqns = unscaled_polytope.get_equations()
    divisor = max(np.abs(eqns).min(), 1)
    eqns = eqns / divisor
    # Switch coefficient signs where appropriate to make the hyperplane equation<0 sastisfy the polytope
    centroid = safe_poly.get_centroid(unscaled_polytope.get_points())
    for i, eqn in enumerate(eqns):
        sign = -np.sign(poly.evaluate_hyperplane(centroid, eqn))[0, 0]
        eqns[i, :] = sign * eqn

    print("Polytope equations ({}):".format(eqns.shape[0]))
    header = list(safe_poly.axis_label_list)
    header.append("Offset")
    print("".join("{:>15s}".format(v) for v in header))
    for eqn in eqns:
        line_string = ""
        for elem in eqn:
            line_string += "{:>15.1f}".format(elem)
        line_string += "{:>5s}".format("<0")
        print(line_string)

    print("C-type definition:")
    print(print_c_arrays(eqns))

    if plot:
        print("Plotting")
        safe_poly.plot()
        plt.show()

    if export:

        unscaled_yes_points = safe_poly.points_yes * safe_poly.eps.reshape(
            safe_poly._n_dim, 1
        )
        unscaled_no_points = safe_poly.points_no * safe_poly.eps.reshape(
            safe_poly._n_dim, 1
        )
        unscaled_polytope = safe_poly._polytope.scale(
            safe_poly.eps.reshape(safe_poly._n_dim, 1)
        )
        eqns = unscaled_polytope.get_equations()
        divisor = max(np.abs(eqns).min(), 1)
        eqns = eqns / divisor
        # Switch coefficient signs where appropriate to make the hyperplane equation<0 sastisfy the polytope
        centroid = safe_poly.get_centroid(unscaled_polytope.get_points())
        for i, eqn in enumerate(eqns):
            sign = -np.sign(poly.evaluate_hyperplane(centroid, eqn))[0, 0]
            eqns[i, :] = sign * eqn
        vertices = unscaled_polytope.get_points()
        print(vertices)

        output_path = "/home/george/ros_workspaces/uav_ftc/src/uav_ftc/logs/flight_envelope_va_gamma"
        header_txt = ",".join(safe_poly.axis_label_list)
        np.savetxt(
            "{}/points_yes.csv".format(output_path),
            unscaled_yes_points.transpose(),
            fmt="%.3g",
            delimiter=",",
            header=header_txt,
            comments="",
        )
        np.savetxt(
            "{}/points_no.csv".format(output_path),
            unscaled_no_points.transpose(),
            fmt="%.3g",
            delimiter=",",
            header=header_txt,
            comments="",
        )
        np.savetxt(
            "{}/separators.csv".format(output_path),
            eqns,
            fmt="%15.1f",
            delimiter=",",
            header=(header_txt + ",Offset"),
            comments="",
        )
        np.savetxt(
            "{}/vertices.csv".format(output_path),
            vertices.transpose(),
            fmt="%15.1f",
            delimiter=",",
            header=header_txt,
            comments="",
        )

if __name__ == "__main__":
    test_code()
