# Trim the aircraft model to find the domain where a trim input set exists

import numpy as np
from numpy import cos, sin
from scipy.optimize import minimize
from scipy.spatial import ConvexHull
import xarray as xr
import xyzpy as xyz
import timeit

import uav_model as mdl  # Import UAV model library


# Raise an error when invalid floating-point operations occur
np.seterr(invalid='raise')


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

    def __init__(self,
                 model=mdl.get_derivatives,
                 x0=mdl.aircraft_state(),
                 u0=mdl.Inputs(),
                 dx0=mdl.aircraft_state()
                 ):
        self.model = model
        self.set_init_values(x0, u0, dx0)

        # Set default values
        self.ix = [3, 4, 6, 7, 8, 11]  # Indices of trim states
        self.idx = slice(2, 11)  # Indices of trim derivatives
        self.iu = []

    def set_init_values(self,
                        x0=None,
                        u0=None,
                        dx0=None
                        ):
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
        self.x_dot_des.pos.z = -Va*sin(gamma)
        self.x_dot_des.att.z = Va/R*cos(gamma)
        # Indices of trim derivatives which should be satisfied
        self.idx = slice(2, 12)

        self.iu = [2]  # Indices of input for cost reduction

    def setup_trim_states(self, phi, theta, Va, alpha, beta, r, verbose=False):

        self.x_des = mdl.aircraft_state()

        # Calculate dependent state elements
        k = r/(cos(phi)*cos(theta))  # Calculate Va/R*cos(gamma)
        p = -k*sin(theta)
        q = k*sin(phi)*cos(theta)

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
            R = Va/r*cos(gamma)*cos(phi)*cos(theta)
        except ZeroDivisionError:
            R = np.inf

        self.x_dot_des = mdl.aircraft_state()
        self.x_dot_des.pos.z = -Va*sin(gamma)
        self.x_dot_des.att.z = k  # k=Va/R*cos(gamma)

        # Set error indices
        self.ix = []  # Indices of trim states
        self.idx = slice(2, 12)  # Indices of trim derivatives
        self.iu = []

        if verbose:
            print(f'Requested trajectory:\n'
                  f'Airspeed: {Va} m/s\n'
                  f'Flight Path Angle: {np.rad2deg(gamma)} degrees\n'
                  f'Turn Radius: {R} m')

    def find_trim_state(self, verbose=False):
        # Find a trim state which satisfies the trim trajectory requirement
        # Returns a tuple with the trim states and trim inputs
        if self.x_dot_des is None or self.x_des is None:
            raise ValueError('Target model derivatives or states not set')

        ix_argument = [3, 4, 6, 7, 8, 9, 10, 11]
        # arguments: phi, theta, Va, alpha, beta, p, q, r, da, de, dt, dr
        init_vector = np.concatenate((self.x0.to_array()[ix_argument],
                                      self.u0.to_array()), axis=0)
        self.optim_bounds = (self.bound_phi, self.bound_theta,
                             self.bound_Va, self.bound_alpha, self.bound_beta,
                             self.bound_p, self.bound_q, self.bound_r,
                             self.bound_deltaa, self.bound_deltae,
                             self.bound_deltat, self.bound_deltar
                             )
        res = minimize(self.cost_function_state_input_wrapper,
                       init_vector,
                       method='SLSQP', options={'disp': True},
                       #    method='L-BFGS-B',
                       bounds=self.optim_bounds,
                       callback=self.optim_callback
                       )
        if res.success:
            optim_result_s = 'SUCCESS'
        else:
            optim_result_s = 'FAILURE'

        if verbose:
            print(f'Optimization result: {optim_result_s}\n {res.message}')
            print(f' Optimization ended in {res.nit} iterations\n'
                  f' Optimization error: {res.fun}')

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

    def find_trim_input(self, verbose=False):
        # Find a trim input wich satisfies the trim state
        # Returns: The optimal trim input
        # The final optimization cost
        # The success flag
        if self.x_des is None:
            raise ValueError('Target state not set')

        # arguments: phi, theta, Va, alpha, beta, p, q, r, da, de, dt, dr
        init_vector = self.u0.to_array()
        self.optim_bounds = (self.bound_deltaa, self.bound_deltae,
                             self.bound_deltat, self.bound_deltar)
        res = minimize(self.cost_function_input_wrapper,
                       init_vector,
                       #   method='SLSQP', options={'disp': True},
                       method='L-BFGS-B',
                       #    method='TNC'
                       #   bounds=self.optim_bounds,
                       )
        if res.success:
            optim_result_s = 'SUCCESS'
        else:
            optim_result_s = 'FAILURE'

        if verbose:
            print(f'Optimization result: {optim_result_s}\n {res.message}')
            print(f' Optimization ended in {res.nit} iterations\n'
                  f' Optimization error: {res.fun}')

        # if res.success:
        trim_inputs = mdl.Inputs(*res.x)
        # else:
            # trim_inputs = mdl.Inputs(None, None, None, None)

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

    def cost_function(self, state: mdl.aircraft_state, inputs: mdl.Inputs):
        # Penalize derivatives and states errors as well as input

        x_dot = self.model(state, inputs).to_array()
        x_dot_err = self.x_dot_des.to_array()[self.idx] - x_dot[self.idx]
        der_error_weight = 1000 * np.eye(x_dot_err.shape[0])
        x_dot_term = x_dot_err.T @ der_error_weight @ x_dot_err

        x_err = self.x_des.to_array()[self.ix] - state.to_array()[self.ix]
        x_error_weight = 10 * np.eye(x_err.shape[0])
        x_term = x_err.T @ x_error_weight @ x_err

        input_vec = inputs.to_array()[self.iu]
        input_weight = 10 * np.eye(input_vec.shape[0])
        input_term = input_vec.T @ input_weight @ input_vec

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

        print(f'Current optimization step:'
              f'{state}',
              f'{inputs}')


class FlightEnvelope:

    axes_dict = None
    axes_names = None
    trimmer = None
    static_trim = None

    def __init__(self, axes_dict: dict, trimmer: Trimmer):
        self.axes_dict = axes_dict
        self.axes_names = ['Va', 'alpha', 'beta', 'phi', 'theta', 'r']
        self.trimmer = trimmer

        # Verify axes_dict has the correct axes names
        if not set(self.axes_dict.keys()) == set(self.axes_names):
            raise KeyError('Not all state parameters provided')

    def find_static_trim(self):
        # Get data values dimension
        # domain_tuples = self.axes_dict.items()
        runner = xyz.Runner(build_fe_element,
                            var_names=['delta_a', 'delta_e', 'delta_t', 'delta_r', 'cost', 'success'],
                            # var_dims={'cost': [self.axes_names]},  # This duplicates the arguments for some reason
                            # 'cost': self.axes_names,
                            #   'success': self.axes_names},
                            resources={'trimmer': self.trimmer},
                            fn_args=['phi', 'theta', 'Va', 'alpha', 'beta', 'r', 'trimmer']
                            )
        self.static_trim = runner.run_combos(self.axes_dict)


def build_envelope_ndarray(trimmer: Trimmer, fl_env_dimension, axes_dict):
    # build state polyhedron
    fl_env = np.fromfunction(  # Function must support vector inputs
        np.vectorize(build_fe_element, excluded=('axes_dict', 'trimmer')), fl_env_dimension, axes_dict=axes_dict, trimmer=trimmer)
    # # iterate over all points
    # # calculate trim inputs for each point

    return fl_env


# def build_fe_element(*indices, axes_dict=None, trimmer=Trimmer()):
def build_fe_element(phi, theta, Va, alpha, beta, r, trimmer=Trimmer()):
    trimmer.setup_trim_states(phi, theta, Va, alpha, beta, r)
    trim_inputs, cost, success = trimmer.find_trim_input()
    return trim_inputs.delta_a, trim_inputs.delta_e, trim_inputs.delta_t, trim_inputs.delta_r, cost, success

def build_feasible_fe(fe):
    # Mask infeasible optimizations
    # fe: a FlightEnvelope object
    # ds_f: a xarray.DataSet
    ds_f = fe.copy(deep=True)
    optim_mask = ds_f.success==1
    cost_mask = ds_f.cost < 50000
    dt_mask = (ds_f.delta_t.data >= trimmer.bound_deltat[0]) * (ds_f.delta_t.data <= trimmer.bound_deltat[1])
    da_mask = (ds_f.delta_a.data >= trimmer.bound_deltaa[0]) * (ds_f.delta_a.data <= trimmer.bound_deltaa[1])
    de_mask = (ds_f.delta_e.data >= trimmer.bound_deltae[0]) * (ds_f.delta_e.data <= trimmer.bound_deltae[1])
    dr_mask = (ds_f.delta_r.data >= trimmer.bound_deltar[0]) * (ds_f.delta_r.data <= trimmer.bound_deltar[1])
    overall_mask = optim_mask * cost_mask * da_mask * de_mask * dt_mask * dr_mask

    ds_f['delta_t'] = ds_f.delta_t.where(overall_mask)
    ds_f['delta_e'] = ds_f.delta_e.where(overall_mask)
    ds_f['delta_a'] = ds_f.delta_a.where(overall_mask)
    ds_f['delta_r'] = ds_f.delta_r.where(overall_mask)
    ds_f['cost'] = ds_f.cost.where(overall_mask)
    ds_f['success'] = ds_f.success.where(overall_mask)

    ds_f = ds_f.where(overall_mask, drop=True)

    return ds_f

def build_convex_hull(ds):
    # Build the convex hull of the flight envelope datapoints
    # fe: a xarray.DataSet

    # Create a list of points
    ds_f_s = ds_f.stack(t=list(ds_f.dims.keys()))  # Create a new stacked dimension
    coords = ds_f_s.t.values  # Get the N x 1 np.array of coordinates
    point_vals = ds_f.delta_t.values.reshape(coords.shape[0],)
    point_vec = np.array([coord for (value, coord) in zip(point_vals, coords) if not np.isnan(value)])

    return ConvexHull(point_vec, qhull_options='QJ')  # Juggle the input because most of the points are coplanar
    # resulting in numerical issues


if __name__ == "__main__":

    trimmer = Trimmer()

    # Trajectory Trimming
    # Va_des = 15
    # gamma_des = np.deg2rad(0)
    # R_des = 100
    # trimmer.setup_trim_trajectory(Va_des, gamma_des, R_des)
    # (trim_state, trim_inputs) = trimmer.find_trim_state()

    # State trimming
    # Va_des = 15
    # alpha_des = np.deg2rad(2)
    # beta_des = np.deg2rad(0)
    # phi_des = np.deg2rad(30)
    # theta_des = np.deg2rad(15)
    # r_des = np.deg2rad(5)
    # Va_des = 15
    # alpha_des = np.deg2rad(1.5)
    # beta_des = np.deg2rad(0)
    # phi_des = np.deg2rad(0)
    # theta_des = np.deg2rad(1.5)
    # r_des = np.deg2rad(0)
    # trimmer.setup_trim_states(
    #     phi_des, theta_des, Va_des, alpha_des, beta_des, r_des)
    # trim_inputs = trimmer.find_trim_input()

    # Static Flight envelope construction
    domain = {'Va': np.linspace(5, 30, 8),
              'alpha': np.linspace(np.deg2rad(0), np.deg2rad(10), 8),
              'beta': [0],
              'phi': [0],
              'theta': np.linspace(np.deg2rad(-10), np.deg2rad(10), 8),
              'r': [0]}

    flight_envelope = FlightEnvelope(domain, trimmer)
    flight_envelope.find_static_trim()
    # plt_figure = flight_envelope.static_trim.xyz.heatmap(x='Va', y='alpha', z='delta_t', colors=True, colormap='viridis')
    # flight_envelope.static_trim.xyz.heatmap(x='Va', y='alpha', z='delta_t', colors=True, colormap='viridis')

    #Create a new flight_envelope with only feasible trim points
    ds_f = build_feasible_fe(flight_envelope.static_trim)

    # Create the convex hull of the flight envelope
    convex_hull = build_convex_hull(ds_f)
