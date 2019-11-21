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

    bound_deltaa = (-1, 1)
    bound_deltae = (-1, 1)
    bound_deltat = (0, 1)
    bound_deltar = (-1, 1)

    optim_algorithm = None
    optim_cost_threshold = None
    optim_time_accumulator = 0  # Store the total time spend finding trim points

    def __init__(self, uav_name):

        # Set default values

        # Setup optimization method
        self.nlopt_cpp_trimmer = tcpp.TrimmerState(uav_name)
        self.optim_algorithm = self.nlopt_cpp_optimize
        self.optim_cost_threshold = (
            50
        )  # WARNING: The level of this constant is dependent on the weighting costs of the trimmer.cpp cost function

    def set_model_parameter(self, param_type, name, value):
        return self.nlopt_cpp_trimmer.set_model_parameter(param_type, name, value)

    def update_model(self):
        self.nlopt_cpp_trimmer.update_model()

    def nlopt_cpp_optimize(self, trajectory):
        trim_values = self.nlopt_cpp_trimmer.find_trim_values(trajectory)
        success = trim_values[8] > 0.1
        return OptimResult(trim_values[0:7], trim_values[7], success)

    # Indicator function for use with polytope discovery module
    def indicator(self, point):
        # point is a 3x1 numpy array with trim Va, gamma, psi_dot values
        R = point[0, 0] * np.cos(point[1, 0]) / point[2, 0]  # R = Va/R*cos(gamma)
        trajectory = np.array((point[0, 0], point[1, 0], R))
        trim_values, cost, success = self.find_trim_input(trajectory)

        # Extract the control input values
        delta_a = trim_values[3]
        delta_e = trim_values[4]
        delta_t = trim_values[5]
        delta_r = trim_values[6]

        optim_mask = success
        cost_mask = cost < self.optim_cost_threshold

        da_mask = (delta_a >= self.bound_deltaa[0]) * (delta_a <= self.bound_deltaa[1])
        de_mask = (delta_e >= self.bound_deltae[0]) * (delta_e <= self.bound_deltae[1])
        dt_mask = (delta_t >= self.bound_deltat[0]) * (delta_t <= self.bound_deltat[1])
        dr_mask = (delta_r >= self.bound_deltar[0]) * (delta_r <= self.bound_deltar[1])
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
        self.optim_time_accumulator = 0
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


class FlightEnvelope:
    flag_plot = False
    flag_interactive = False
    flag_verbose = False
    uav_name = None

    _trimmer = None
    _flag_setup = False
    # Default parameter values, in case they get fixed
    _default_Va = 10.0
    _default_gamma = 0.0
    _default_R = np.infty
    _list_defaults = None
    # default parameter domain
    _Va_min = 5.0
    _Va_max = 25.0
    _gamma_min_deg = -30.0  # degrees
    _gamma_max_deg = 30.0  # degrees
    _R_min = 100.0  # meters
    # flags for fixing parameters
    _fix_Va = False
    _fix_gamma = False
    _fix_R = False
    _list_fixed = None
    # Variable search resolution
    _eps_Va = 2
    _eps_gamma = np.deg2rad(5)
    _eps_psi_dot = 0.05
    _current_domain = None

    _domain_Va = None
    _domain_gamma = None
    _domain_psi_dot = None
    _current_domain = None
    _indicator = None

    def __init__(self, uav_name):
        self.uav_name = uav_name
        self.initialize(uav_name)

    def set_Va(self, Va):
        self._default_Va = Va
        self._flag_setup = False

    def set_gamma(self, gamma):
        self._default_gamma = gamma
        self._flag_setup = False

    def set_R(self, R):
        self._default_R = R
        self._flag_setup = False

    def set_domain_Va(self, tuple_Va):
        self._domain_Va = tuple_Va
        self._flag_setup = False

    def set_domain_gamma(self, tuple_gamma):
        self._domain_gamma = tuple_gamma
        self._flag_setup = False

    def _set_domain_psi_dot(self, tuple_psi_dot):
        self._domain_psi_dot = tuple_psi_dot
        self._flag_setup = False

    def set_R_min(self, R_min):
        self._R_min = R_min
        self._flag_setup = False

    def fix_Va(self, value=True):
        self._fix_Va = value
        self._flag_setup = False

    def fix_gamma(self, value=True):
        self._fix_gamma = value
        self._flag_setup = False

    def fix_R(self, value=True):
        self._fix_R = value
        self._flag_setup = False

    def set_model_parameter(self, param_type, param_name, param_value):
        return self._trimmer.set_model_parameter(param_type, param_name, param_value)

    def update_model(self):
        self._trimmer.update_model()

    def initialize(self, uav_name=None):
        if uav_name is None:
            uav_name = self.uav_name

        # Build parameter default values
        default_psi_dot = self._default_Va / self._default_R
        self._list_defaults = [self._default_Va, self._default_gamma, default_psi_dot]

        # Build variable domains
        self.set_domain_Va((self._Va_min, self._Va_max))
        self.set_domain_gamma(
            (np.deg2rad(self._gamma_min_deg), np.deg2rad(self._gamma_max_deg))
        )
        psi_dot_max = self._Va_max / self._R_min
        self._set_domain_psi_dot((-psi_dot_max, psi_dot_max))

        # Set search degrees of freedom
        self._list_fixed = [self._fix_Va, self._fix_gamma, self._fix_R]

        domain = np.array([self._domain_Va, self._domain_gamma, self._domain_psi_dot])
        self._current_domain = domain[np.invert(self._list_fixed)]

        # Select eps values
        eps = np.array([self._eps_Va, self._eps_gamma, self._eps_psi_dot])
        self._current_eps = eps[np.invert(self._list_fixed)]

        # Create indicator function
        self._trimmer = Trimmer(uav_name)
        self._trimmer.optim_time_accumulator = 0
        self._indicator = self._trimmer.get_indicator(
            self._list_defaults, self._list_fixed
        )

        self._flag_setup = True

    def find_flight_envelope(self):

        if not self._flag_setup:
            raise RuntimeError(
                "You must initalize the FlightEnvelope before extracting it"
            )

        # Initialize starting time
        t_start = time.time()

        # Initialize the polytope
        safe_poly = poly.SafeConvexPolytope(
            self._indicator, self._current_domain, self._current_eps
        )

        # Pass the variable strings
        string_Va = "Va"
        string_gamma = "Gamma"
        string_psi_dot = "psi_dot"
        string_list = [string_Va, string_gamma, string_psi_dot]
        safe_poly.axis_label_list = list(
            it.compress(string_list, np.invert(self._list_fixed))
        )
        # safe_poly.plotting_mask = [False, True, True, True]

        # User interface options
        if self.flag_plot:
            safe_poly.enable_plotting = True
        if self.flag_interactive:
            safe_poly.wait_for_user = True

        if self.flag_verbose:
            print("Starting Flight Envelope detection")
        # Iteratively sample the polytope
        algorithm_end = False
        while not algorithm_end:
            algorithm_end = safe_poly.step()

        if self.flag_verbose:
            print("Final number of sampled points: {}".format(len(safe_poly._set_points)))
            print("Total number of samples taken: {}".format(safe_poly._samples_taken))
            print(
                "Total optimization time required: {}".format(
                    self._trimmer.optim_time_accumulator
                )
            )

        t_end = time.time()
        if self.flag_verbose:
            print("Total script time: {}".format(t_end - t_start))

        if self.flag_plot:
            safe_poly.plot()
            plt.waitforbuttonpress(timeout=-1)

        return safe_poly


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
@click.option(
    "-e",
    "--export-path",
    default=None,
    help="Path to export relevant data in .csv files",
)
@click.option(
    "-v",
    "--verbose",
    is_flag=True,
    default=True,
    help="Enable verbose output",
)
def test_code(plot, interactive, export_path, verbose):

    uav_name = "skywalker_2013_mod"
    flight_envelope = FlightEnvelope(uav_name)
    flight_envelope.flag_verbose = verbose

    flight_envelope.flag_plot = plot
    flight_envelope.flag_interactive = interactive
    flight_envelope.set_domain_Va((5, 25))
    flight_envelope.set_domain_gamma((-30, 30))
    flight_envelope.set_R_min(100)
    flight_envelope.initialize(uav_name)

    safe_poly = flight_envelope.find_flight_envelope()
    safe_poly._flag_verbose = verbose

    # Approximate by ellipsoid
    safe_poly.ellipsoid_fit()
    print("Fitted Ellipsoid coefficients:\n")
    print(safe_poly._el_v)
    if plot:
        safe_poly.plot_ellipsoid()
        # plt.ion()
        plt.show()

    print("Changing model parameter and re-extracting")
    result = flight_envelope.set_model_parameter(
        5, "motor1/omega_max", 10
    )  # Zero-out propeller efficiency
    if result:
        print("Succeeded")
    else:
        print("Failed")
    flight_envelope.update_model()  # Register model changes
    # Set the model parameters. They will not take effect (be written)
    # param_type defined as:
    # typedef enum {
    #     PARAM_TYPE_WORLD = 0,
    #     PARAM_TYPE_ENV,
    #     PARAM_TYPE_INIT,
    #     PARAM_TYPE_INERTIAL,
    #     PARAM_TYPE_AERO,
    #     PARAM_TYPE_PROP,
    #     PARAM_TYPE_GROUND
    # } ParamType_t;
    safe_poly = flight_envelope.find_flight_envelope()
    # Approximate by ellipsoid
    safe_poly.ellipsoid_fit()
    print("Fitted Ellipsoid coefficients:\n")
    print(safe_poly._el_v)
    if plot:
        safe_poly.plot_ellipsoid()
        plt.show()

    # Approximate the safe convex polytope with k vertices
    print("Performing clustering")
    # cluster_num = 2 ** n_dim
    # cluster_num = 2 * (n_dim-3)
    if safe_poly._n_dim == 2:
        cluster_num = 4
    elif safe_poly._n_dim == 3:
        cluster_num = 12

    safe_poly.cluster(cluster_num)
    # if plot:
    #     safe_poly.plot()
    #     plt.show()

    safe_poly.display_constraints()

    print("C-type definition:")
    print(safe_poly.print_c_arrays())

    # Export polytope search results
    if export_path is not None:
        safe_poly.export_csv(export_path)


if __name__ == "__main__":
    test_code()
