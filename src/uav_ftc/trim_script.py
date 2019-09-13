# Trim the aircraft model to find the domain where a trim input set exists

import numpy as np
from numpy import cos, sin
from scipy.optimize import minimize
from scipy.spatial import ConvexHull
import xarray as xr
import itertools as it

# import xyzpy as xyz
import os
import time
import matplotlib as mpl
import matplotlib.pyplot as plt
import click
import nlopt
import imp

import uav_model as mdl  # Import UAV model library
import plot_utils as plu  # Import plotting utilities
import polytope_utils as poly  # Used for SafeConvexPoly class
# tcpp = imp.load_source('trimmer.py', '/home/george/ros_workspaces/uav_ftc/src/last_letter/last_letter_lib/src/', 'trimmer.py')
tcpp = imp.load_source('trimmer', '/home/george/ros_workspaces/uav_ftc/src/last_letter/last_letter_lib/src/trimmer.py')


# Raise an error when invalid floating-point operations occur
np.seterr(invalid="raise")


class OptimResult:

    def __init__(self, x_star, cost, success):
        self.x = x_star
        self.fun = cost
        self.success = success


class Trimmer:

    x0 = None
    dx0 = None
    u0 = None
    x_des = None
    x_dot_des = None
    ix = None  # Indices of trim states
    idx = None  # Indices of trim derivatives
    iu = None

    bound_phi = (np.deg2rad(-5), np.deg2rad(5))  # Allow only upright flight
    bound_theta = (np.deg2rad(-5), np.deg2rad(5))
    bound_Va = (13, 15)
    bound_alpha = (np.deg2rad(0), np.deg2rad(2))
    bound_beta = (np.deg2rad(0), np.deg2rad(0))
    bound_p = (0, 0)
    bound_q = (0, 0)
    bound_r = (0, 0)
    bound_deltaa = (-1, 1)
    bound_deltae = (-1, 1)
    bound_deltat = (0, 1)
    bound_deltar = (-1, 1)

    optim_bounds = None

    # optim_library = 'scipy'
    optim_library = 'nlopt'
    optim_algorithm = None
    optim_cost_threshold = None
    nlopt_obj = None
    optim_result = None
    optim_time_accumulator = 0 # Store the total time spend finding trim points

    def __init__(
        self,
        model=mdl.get_derivatives,
        x0=mdl.aircraft_state(),
        u0=mdl.Inputs(),
        dx0=mdl.aircraft_state(),
        optim_lib = 'nlopt'
    ):
        self.model = model
        self.set_init_values(x0, u0, dx0)

        # Set default values
        self.ix = [3, 4, 6, 7, 8, 11]  # Indices of trim states
        self.idx = slice(2, 11)  # Indices of trim derivatives
        self.iu = []

        # Setup optimization method
        self.optim_library = optim_lib
        if self.optim_library == 'scipy':
            print("Building scipy minimize for trim calculations")
            self.optim_algorithm = self.scipy_minimize
            self.optim_cost_threshold = 50000
        elif self.optim_library == 'nlopt':
            print("Building nlopt for trim calculations")
            # self.nlopt_obj = nlopt.opt(nlopt.LN_COBYLA, 4)
            # self.nlopt_obj = nlopt.opt(nlopt.LN_BOBYQA, 4)
            self.nlopt_obj = nlopt.opt(nlopt.LN_PRAXIS, 4)
            # self.nlopt_obj = nlopt.opt(nlopt.LN_NELDERMEAD, 4)
            # self.nlopt_obj = nlopt.opt(nlopt.LN_SBPLX, 4)
            self.nlopt_obj.set_min_objective(self.nlopt_objective)
            self.optim_cost_threshold = 50000

            self.nlopt_obj.set_lower_bounds([-np.inf, -np.inf, 0, -np.inf])
            self.nlopt_obj.set_upper_bounds([ np.inf,  np.inf, 1,  np.inf])
            self.nlopt_obj.set_xtol_abs(4*[0.01])

            self.optim_algorithm = self.nlopt_optimize
            print("Using nlopt version {}.{}".format(nlopt.version_major(), nlopt.version_minor()))
        elif self.optim_library == 'nlopt_cpp':
            print("Building nlopt_cpp for trim calculations")
            self.nlopt_cpp_trimmer = tcpp.Trimmer('skywalker_2013')
            self.optim_algorithm = self.nlopt_cpp_optimize
            self.optim_cost_threshold = 10000
        else:
            raise ValueError('Undefined minimization algorithm')

    def nlopt_objective(self, x, grad):
        self.optim_result = x
        return self.cost_function_input_wrapper(x)

    def nlopt_optimize(self, x):
        try:
            trim_input = self.nlopt_obj.optimize(x.reshape(-1))
        except nlopt.RoundoffLimited:
            print("Roundoff error occurred. Stopping optimization...")
            trim_input = self.optim_result
        cost = self.nlopt_obj.last_optimum_value()
        success = self.nlopt_obj.last_optimize_result > 0
        # print(trim_input, cost, success)
        return OptimResult(trim_input, cost, success)

    def nlopt_cpp_optimize(self, u0):
        # Disreagard initializing input since it cannot be set in trimmer_cpp
        trim_state = np.array([self.x_des.att.x,
                               self.x_des.att.y,
                               self.x_des.airdata.x,
                               self.x_des.airdata.y,
                               self.x_des.airdata.z,
                               self.x_des.ang_vel.z])
        trim_ctrls = self.nlopt_cpp_trimmer.find_trim_input(trim_state)
        success = trim_ctrls[5]>0.1
        return OptimResult(trim_ctrls[0:4], trim_ctrls[4], success)

    def set_init_values(self, x0=None, u0=None, dx0=None):
        if x0 is not None:
            self.x0 = x0
        if u0 is not None:
            self.u0 = u0
        if dx0 is not None:
            self.dx0 = dx0

    def setup_trim_trajectory(self, Va, gamma, R):
        # Setup initial seed
        self.x0 = mdl.aircraft_state()
        self.x0.att.y = gamma
        self.x0.airdata.x = Va

        self.u0 = mdl.Inputs(0, 0, 0.5, 0)

        self.x_des = mdl.aircraft_state()
        self.x_des.airdata.x = Va
        self.x_des.airdata.z = 0
        self.ix = [6, 8]  # Indices of trim states which should be satisfied

        self.x_dot_des = mdl.aircraft_state()
        self.x_dot_des.pos.z = -Va * sin(gamma)
        self.x_dot_des.att.z = Va / R * cos(gamma)
        # Indices of trim derivatives which should be satisfied
        self.idx = slice(2, 12)

        self.iu = [2]  # Indices of input for cost reduction

    def setup_trim_states(self, phi, theta, Va, alpha, beta, r, verbose=False):

        self.x_des = mdl.aircraft_state()

        # Calculate dependent state elements
        k = r / (cos(phi) * cos(theta))  # Calculate Va/R*cos(gamma)
        p = -k * sin(theta)
        q = k * sin(phi) * cos(theta)

        # Fix state requirements
        self.x_des.att.x = phi
        self.x_des.att.y = theta
        self.x_des.airdata.x = Va
        self.x_des.airdata.y = alpha
        self.x_des.airdata.z = beta
        self.x_des.ang_vel.x = p
        self.x_des.ang_vel.y = q
        self.x_des.ang_vel.z = r

        # Calculate derived quantities
        gamma = theta - alpha
        try:
            R = Va / r * cos(gamma) * cos(phi) * cos(theta)
        except ZeroDivisionError:
            R = np.inf

        self.x_dot_des = mdl.aircraft_state()
        self.x_dot_des.pos.z = -Va * sin(gamma)
        self.x_dot_des.att.z = k  # k=Va/R*cos(gamma)

        # Set error indices
        self.ix = []  # Indices of trim states
        self.idx = slice(2, 12)  # Indices of trim derivatives
        self.iu = []

        if verbose:
            print(
                """
                Requested trajectory:\n
                Airspeed: {} m/s\n
                Flight Path Angle: {} degrees\n
                Turn Radius: {} m
                """.format(
                    Va, np.rad2deg(gamma), R
                )
            )

    def find_trim_state(self, verbose=False):
        # Find a trim state which satisfies the trim trajectory requirement
        # Returns a tuple with the trim states and trim inputs
        if self.x_dot_des is None or self.x_des is None:
            raise ValueError("Target model derivatives or states not set")

        ix_argument = [3, 4, 6, 7, 8, 9, 10, 11]
        # arguments: phi, theta, Va, alpha, beta, p, q, r, da, de, dt, dr
        init_vector = np.concatenate(
            (self.x0.to_array()[ix_argument], self.u0.to_array()), axis=0
        )
        self.optim_bounds = (
            self.bound_phi,
            self.bound_theta,
            self.bound_Va,
            self.bound_alpha,
            self.bound_beta,
            self.bound_p,
            self.bound_q,
            self.bound_r,
            self.bound_deltaa,
            self.bound_deltae,
            self.bound_deltat,
            self.bound_deltar,
        )
        res = minimize(
            self.cost_function_state_input_wrapper,
            init_vector,
            method="SLSQP",
            options={"disp": True},
            #    method='L-BFGS-B',
            bounds=self.optim_bounds,
            callback=self.optim_callback,
        )
        if res.success:
            optim_result_s = "SUCCESS"
        else:
            optim_result_s = "FAILURE"

        if verbose:
            print("Optimization result: {}\n {}".format(optim_result_s, res.message))
            print(
                """
                Optimization ended in {} iterations\n
                Optimization error: {}
                """.format(
                    res.nit, res.fun
                )
            )

        if res.success:
            trim_state = mdl.aircraft_state()
            trim_state.att.x = res.x[0]
            trim_state.att.y = res.x[1]
            trim_state.airdata.x = res.x[2]
            trim_state.airdata.y = res.x[3]
            trim_state.airdata.z = res.x[4]
            trim_state.ang_vel.x = res.x[5]
            trim_state.ang_vel.y = res.x[6]
            trim_state.ang_vel.z = res.x[7]

            trim_inputs = mdl.Inputs(*res.x[8:12])
            return (trim_state, trim_inputs)
        else:
            return (None, None)

    # Indicator function for use with polytope discovery module
    def indicator(self, point):
        phi, theta, Va, alpha, beta, r = tuple(point.reshape(-1))
        self.setup_trim_states(phi, theta, Va, alpha, beta, r)
        # inputs, cost, success = self.find_trim_input(verbose=True)
        inputs, cost, success = self.find_trim_input()

        optim_mask = success
        cost_mask = cost < self.optim_cost_threshold

        da_mask = (inputs.delta_a >= self.bound_deltaa[0]) * (
            inputs.delta_a <= self.bound_deltaa[1]
        )
        de_mask = (inputs.delta_e >= self.bound_deltae[0]) * (
            inputs.delta_e <= self.bound_deltae[1]
        )
        dt_mask = (inputs.delta_t >= self.bound_deltat[0]) * (
            inputs.delta_t <= self.bound_deltat[1]
        )
        dr_mask = (inputs.delta_r >= self.bound_deltar[0]) * (
            inputs.delta_r <= self.bound_deltar[1]
        )
        input_mask = da_mask * de_mask * dt_mask * dr_mask
        overall_mask = optim_mask * cost_mask * input_mask
        
        # if optim_mask and not cost_mask:
            # print('Rejected solution due to cost')

        # if optim_mask and not input_mask:
            # print('Rejected solution due to input constraint violation')

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
            state = np.zeros((len(list_defaults), 1))
            idx2 = 0
            for idx in range(len(list_defaults)):
                if list_fixed[idx]:
                    state[idx, 0] = list_defaults[idx]
                else:
                    state[idx, 0] = point[idx2]
                    idx2 += 1
            return self.indicator(state)

        return indicator_wrapper

    def scipy_minimize(self, init_vector):
        res = minimize(
            self.cost_function_input_wrapper,
            init_vector,
            #   method='SLSQP', options={'disp': True},
            method="L-BFGS-B",
            #    method='TNC'
            #   bounds=self.optim_bounds,
        )
        return res

    def find_trim_input(self, verbose=False):
        # Find a trim input wich satisfies the trim state
        # Returns: The optimal trim input
        # The final optimization cost
        # The success flag
        if self.x_des is None:
            raise ValueError("Target state not set")

        # arguments: phi, theta, Va, alpha, beta, p, q, r, da, de, dt, dr
        init_vector = self.u0.to_array()
        self.optim_bounds = (
            self.bound_deltaa,
            self.bound_deltae,
            self.bound_deltat,
            self.bound_deltar,
        )
        
        # Perform the optimization step and time execution
        t_start = time.time()
        res = self.optim_algorithm(init_vector)
        t_end = time.time()
        self.optim_time_accumulator = self.optim_time_accumulator + (t_end-t_start)

        if res.success:
            optim_result_s = "SUCCESS"
        else:
            optim_result_s = "FAILURE"

        trim_inputs = mdl.Inputs(*res.x)

        if verbose:
            print("Optimization result: {}\n {}".format(optim_result_s, res.message))
            print(
                """
                Optimization ended in {} iterations\n
                Optimization error: {}
                """.format(
                    res.nit, res.fun
                )
            )
            print("Optimization result:\n{}".format(trim_inputs))

        return (trim_inputs, res.fun, res.success)

    def find_trim_input_cost(self, verbose=False):
        _, cost, success = self.find_trim_input(verbose)
        return cost

    def cost_function_input_wrapper(self, arguments):
        # arguments: da, de, dt, dr
        inputs = mdl.Inputs(*arguments)

        return self.cost_function(self.x_des, inputs)

    def cost_function_state_input_wrapper(self, arguments):
        # arguments: phi, theta, Va, alpha, beta, p, q, r, da, de, dt, dr
        phi, theta = arguments[0:2]
        Va, alpha, beta = arguments[2:5]
        p, q, r = arguments[5:8]
        da, de, dt, dr = arguments[8:12]

        # Fill out cost function arguments
        state = mdl.aircraft_state()
        state.att = mdl.Vector3(phi, theta, 0)  # Yaw isn't optimized
        state.airdata = mdl.Vector3(Va, alpha, beta)
        state.ang_vel = mdl.Vector3(p, q, r)
        inputs = mdl.Inputs(da, de, dt, dr)

        return self.cost_function(state, inputs)

    def cost_function(self, state, inputs):
        # Penalize derivatives and states errors as well as input

        x_dot = self.model(state, inputs).to_array()
        x_dot_err = self.x_dot_des.to_array()[self.idx] - x_dot[self.idx]
        der_error_weight = 1000 * np.eye(x_dot_err.shape[0])
        x_dot_term = reduce(np.matmul, [x_dot_err.T, der_error_weight, x_dot_err])

        x_err = self.x_des.to_array()[self.ix] - state.to_array()[self.ix]
        x_error_weight = 10 * np.eye(x_err.shape[0])
        x_term = reduce(np.matmul, [x_err.T, x_error_weight, x_err])

        input_vec = inputs.to_array()[self.iu]
        input_weight = 10 * np.eye(input_vec.shape[0])
        input_term = reduce(np.matmul, [input_vec.T, input_weight, input_vec])

        cost = x_dot_term + x_term + input_term

        return cost.item()

    def optim_callback(self, arguments):
        # Optimization callback for state+inputs trim set search
        # arguments: phi, theta, Va, alpha, beta, p, q, r, da, de, dt, dr
        phi, theta = arguments[0:2]
        Va, alpha, beta = arguments[2:5]
        p, q, r = arguments[5:8]
        da, de, dt, dr = arguments[8:12]

        # Fill out cost function arguments
        state = mdl.aircraft_state()
        state.att = mdl.Vector3(phi, theta, 0)  # Yaw isn't optimized
        state.airdata = mdl.Vector3(Va, alpha, beta)
        state.ang_vel = mdl.Vector3(p, q, r)
        inputs = mdl.Inputs(da, de, dt, dr)

        print("Current optimization step: {}, {}".format(state, inputs))


# class FlightEnvelope:

#     axes_dict = None
#     axes_names = None
#     trimmer = None
#     static_trim = None

#     def __init__(self, axes_dict, trimmer):
#         self.axes_dict = axes_dict
#         self.axes_names = ["Va", "alpha", "beta", "phi", "theta", "r"]
#         self.trimmer = trimmer

#         # Verify axes_dict has the correct axes names
#         if not set(self.axes_dict.keys()) == set(self.axes_names):
#             raise KeyError("Not all state parameters provided")

#     def find_static_trim(self):
#         # Get data values dimension
#         # domain_tuples = self.axes_dict.items()
#         runner = xyz.Runner(
#             build_fe_element,
#             var_names=["delta_a", "delta_e", "delta_t", "delta_r", "cost", "success"],
#             # var_dims={'cost': [self.axes_names]},  # This duplicates the arguments for some reason
#             # 'cost': self.axes_names,
#             #   'success': self.axes_names},
#             resources={"trimmer": self.trimmer},
#             fn_args=["phi", "theta", "Va", "alpha", "beta", "r", "trimmer"],
#         )
#         self.static_trim = runner.run_combos(self.axes_dict)


def build_envelope_ndarray(trimmer, fl_env_dimension, axes_dict):
    # build state polyhedron
    fl_env = np.fromfunction(  # Function must support vector inputs
        np.vectorize(build_fe_element, excluded=("axes_dict", "trimmer")),
        fl_env_dimension,
        axes_dict=axes_dict,
        trimmer=trimmer,
    )
    # # iterate over all points
    # # calculate trim inputs for each point

    return fl_env


# def build_fe_element(*indices, axes_dict=None, trimmer=Trimmer()):
def build_fe_element(phi, theta, Va, alpha, beta, r, trimmer=Trimmer()):
    trimmer.setup_trim_states(phi, theta, Va, alpha, beta, r)
    trim_inputs, cost, success = trimmer.find_trim_input()
    return (
        trim_inputs.delta_a,
        trim_inputs.delta_e,
        trim_inputs.delta_t,
        trim_inputs.delta_r,
        cost,
        success,
    )


def find_nav_trim(points_vector):
    # Use self.static_trim to calculate navigation Va, R, gamma triplet
    # f = np.vectorize(state_to_nav_vec, otypes=[float, float, float])
    # return f(points_vector)
    return np.apply_along_axis(state_to_nav_vec, 1, points_vector)


def state_to_nav(Va, alpha, beta, phi, theta, r):
    # Convert from aircraft trim state to navigation triplet Va, R, gamma
    R_max = 1000

    gamma = theta - alpha  # Disregards wind
    R = Va / r * cos(gamma) * cos(phi) * cos(theta)  # Disregards wind
    if np.isinf(R):
        R = R_max

    return Va, gamma, R


def state_to_nav_vec(states):
    return state_to_nav(*tuple(states))


def build_feasible_fe(fe):
    # Mask infeasible optimizations
    # fe: a FlightEnvelope object
    # ds_f: a xarray.DataSet
    ds_f = fe.copy(deep=True)
    optim_mask = ds_f.success == 1
    cost_mask = ds_f.cost < self.optim_cost_threshold
    dt_mask = (ds_f.delta_t.data >= fe.trimmer.bound_deltat[0]) * (
        ds_f.delta_t.data <= fe.trimmer.bound_deltat[1]
    )
    da_mask = (ds_f.delta_a.data >= fe.trimmer.bound_deltaa[0]) * (
        ds_f.delta_a.data <= fe.trimmer.bound_deltaa[1]
    )
    de_mask = (ds_f.delta_e.data >= fe.trimmer.bound_deltae[0]) * (
        ds_f.delta_e.data <= fe.trimmer.bound_deltae[1]
    )
    dr_mask = (ds_f.delta_r.data >= fe.trimmer.bound_deltar[0]) * (
        ds_f.delta_r.data <= fe.trimmer.bound_deltar[1]
    )
    overall_mask = optim_mask * cost_mask * da_mask * de_mask * dt_mask * dr_mask

    ds_f["delta_t"] = ds_f.delta_t.where(overall_mask)
    ds_f["delta_e"] = ds_f.delta_e.where(overall_mask)
    ds_f["delta_a"] = ds_f.delta_a.where(overall_mask)
    ds_f["delta_r"] = ds_f.delta_r.where(overall_mask)
    ds_f["cost"] = ds_f.cost.where(overall_mask)
    ds_f["success"] = ds_f.success.where(overall_mask)

    ds_f = ds_f.where(overall_mask, drop=True)

    return ds_f


def build_convex_hull(ds):
    # Build the convex hull of the flight envelope datapoints
    # fe: a xarray.DataSet

    # Create a list of points
    ds_s = ds.stack(t=list(ds.coords.keys()))  # Create a new stacked dimension
    coords = ds_s.t.values  # Get the N x 1 np.array of coordinates
    point_vals = ds.delta_t.values.reshape(coords.shape[0])
    point_vec = np.array(
        [coord for (value, coord) in zip(point_vals, coords) if not np.isnan(value)]
    )

    return ConvexHull(
        point_vec, qhull_options="QJ"
    )  # Juggle the input because most of the points are coplanar
    # resulting in numerical issues


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
    "-o",
    "--optimizer",
    type=click.Choice(["scipy", "nlopt", "nlopt_cpp"]),
    default="nlopt",
    help="Select optimizer backend."
)
def test_code(plot, interactive, optimizer):
    trimmer = Trimmer(optim_lib=optimizer)

    # Trajectory Trimming
    # Va_des = 15
    # gamma_des = np.deg2rad(0)
    # R_des = 100
    # trimmer.setup_trim_trajectory(Va_des, gamma_des, R_des)
    # (trim_state, trim_inputs) = trimmer.find_trim_state()

    # Provide default values
    phi_des = np.deg2rad(0)
    theta_des = np.deg2rad(0)
    Va_des = 15
    alpha_des = np.deg2rad(2)
    beta_des = np.deg2rad(0)
    r_des = np.deg2rad(5)
    list_defaults = [phi_des, theta_des, Va_des, alpha_des, beta_des, r_des]

    # Set search degrees of freedom
    # Theta, Va, alpha
    # fix_phi = True
    # fix_theta = False
    # fix_Va = False
    # fix_alpha = False
    # fix_beta = True
    # fix_r = True

    # More dimensions
    fix_phi = False
    fix_theta = False
    fix_Va = False
    fix_alpha = False
    fix_beta = False
    fix_r = True

    list_fixed = [fix_phi, fix_theta, fix_Va, fix_alpha, fix_beta, fix_r]

    # Provide domain bounds
    phi_domain = (np.deg2rad(-20), np.deg2rad(20))
    theta_domain = (np.deg2rad(-45), np.deg2rad(45))
    Va_domain = (5, 30)
    alpha_domain = (np.deg2rad(-10), np.deg2rad(35))
    beta_domain = (np.deg2rad(-5), np.deg2rad(5))
    r_domain = [-0.5, 0.5]
    domain = np.array(
        [phi_domain, theta_domain, Va_domain, alpha_domain, beta_domain, r_domain]
    )
    current_domain = domain[np.invert(list_fixed)]
    n_dim = len(current_domain)

    # Set trimmer optimization bounds
    trimmer.bound_phi = phi_domain
    trimmer.bound_theta = theta_domain
    trimmer.bound_Va = Va_domain
    trimmer.bound_alpha = alpha_domain
    trimmer.bound_beta = beta_domain
    trimmer.bound_r = r_domain

    # Create indicator function
    indicator = trimmer.get_indicator(list_defaults, list_fixed)

    # Select eps values
    eps_phi = np.deg2rad(2)
    eps_theta = np.deg2rad(3)
    eps_Va = 2
    eps_alpha = np.deg2rad(3)
    eps_beta = np.deg2rad(1)
    eps_r = 0.1
    eps = np.array([eps_phi, eps_theta, eps_Va, eps_alpha, eps_beta, eps_r])
    current_eps = eps[np.invert(list_fixed)]

    # Initialize starting time
    t_start = time.time()

    # Initialize the polytope
    safe_poly = poly.SafeConvexPolytope(indicator, current_domain, current_eps)

    # Pass the variable strings
    string_phi = 'Phi'
    string_theta = 'Theta'
    string_Va = 'Va'
    string_alpha = 'Alpha'
    string_beta = 'Beta'
    string_r = 'r'
    string_list = [string_phi, string_theta, string_Va, string_alpha, string_beta, string_r]
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
    safe_poly.cluster(2 ** n_dim)

    t_end = time.time()
    print("Total script time: {}".format(t_end - t_start))

    # Print human-readable polytope equations
    unscaled_polytope = safe_poly._reduced_polytope.scale(safe_poly.eps.reshape(safe_poly._n_dim, 1))
    print("Polytope equations:")
    print("Variables:")
    print("Polytope equations:")
    print("Variables:")
    header = safe_poly.axis_label_list
    header.append(str(0))
    print(' '.join('{:>15s}'.format(v) for v in header))
    print(unscaled_polytope.get_equations_normalized(1))

    if plot:
        print("Plotting")
        safe_poly.plot()
        plt.show()


    # # Static Flight envelope construction
    # domain = {
    #     "Va": np.linspace(5, 30, 8),
    #     "alpha": np.linspace(np.deg2rad(-10), np.deg2rad(45), 8),
    #     "beta": [0],
    #     "phi": [0],
    #     "theta": np.linspace(np.deg2rad(-45), np.deg2rad(45), 8),
    #     "r": [0],
    # }

    # flight_envelope = FlightEnvelope(domain, trimmer)
    # flight_envelope.find_static_trim()
    # # plt_figure = flight_envelope.static_trim.xyz.heatmap(x='Va', y='alpha', z='delta_t', colors=True, colormap='viridis')
    # # flight_envelope.static_trim.xyz.heatmap(x='Va', y='alpha', z='delta_t', colors=True, colormap='viridis')

    # # Create a new flight_envelope with only feasible trim points
    # ds_f = build_feasible_fe(flight_envelope.static_trim)

    # # Create the convex hull of the flight envelope
    # convex_hull = build_convex_hull(ds_f)
    # # Plot a 3D slice
    # plu.plot3_points(convex_hull.points[:, [0, 1, 4]], ["Va", "alpha", "theta"])

    # # Build the Va, R, gamma variables for each trim point
    # flight_envelope.nav_trim = find_nav_trim(convex_hull.points)
    # # Plot the navigation envelope
    # plu.plot3_points(flight_envelope.nav_trim, ["Va", "gamma", "R"])


if __name__ == "__main__":
    test_code()
