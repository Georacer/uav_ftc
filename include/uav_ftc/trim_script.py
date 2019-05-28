# Trim the aircraft model to find the domain where a trim input set exists

import numpy as np
from numpy import cos, sin
from scipy.optimize import minimize
import timeit

import uav_model as mdl  # Import UAV model library


# Raise an error when invalid floating-point operations occur
np.seterr(invalid='raise')


class Trimmer:

    x0 = None
    u0 = None
    x_des = None
    x_dot_des = None
    ix = None  # Indices of trim states
    idx = None  # Indices of trim derivatives
    iu = None

    bound_phi = (np.deg2rad(-90), np.deg2rad(90))  # Allow only upright flight
    bound_theta = (np.deg2rad(-90), np.deg2rad(90))
    bound_Va = (5, 50)
    bound_alpha = (np.deg2rad(-20), np.deg2rad(90))
    bound_beta = (np.deg2rad(-90), np.deg2rad(90))
    bound_p = (-5, 5)
    bound_q = (-5, 5)
    bound_r = (-1, 1)
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

    def setup_trim_trajectory(self,
                              Va=15,
                              gamma=0,
                              R=np.inf
                              ):
        self.x0 = mdl.aircraft_state()
        self.x0.att.y = gamma
        self.x0.airdata.Va = Va
        self.u0 = mdl.Inputs(0, 0, 0.5, 0)

        self.x_des = mdl.aircraft_state()
        self.x_des.airdata.Va = Va
        self.x_des.airdata.beta = 0
        self.x_dot_des = mdl.aircraft_state()
        self.x_dot_des.pos.z = -Va*sin(gamma)
        self.x_dot_des.att.z = Va/R*cos(gamma)

        self.ix = [6, 8]  # Indices of trim states
        self.idx = slice(2, 12)  # Indices of trim derivatives
        self.iu = [3]

    def setup_trim_states(self, phi, theta, Va, alpha, beta, r):
        self.x_des = mdl.aircraft_state()

        # Calculate dependent state elements
        k = r/(cos(phi)*cos(theta))  # Calculate Va/R*cos(gamma)
        p = -k*sin(theta)
        q = k*sin(phi)*cos(theta)

        # Setup directly selected quantities
        self.x_des.att.x = phi
        self.x_des.att.y = theta
        self.x_des.airdata.Va = Va
        self.x_des.airdata.alpha = alpha
        self.x_des.airdata.beta = beta
        self.x_des.ang_vel.x = p
        self.x_des.ang_vel.y = q
        self.x_des.ang_vel.z = r

        # Setup derived quantities
        gamma = theta - alpha
        R = Va/r*cos(gamma)*cos(phi)*cos(theta)
        self.x_des.ang_vel.x = -Va/R*cos(gamma)*sin(theta)
        self.x_des.ang_vel.y = Va/R*cos(gamma)*sin(phi)*cos(theta)

        self.x_dot_des = mdl.aircraft_state()
        self.x_dot_des.pos.z = -Va*sin(gamma)
        self.x_dot_des.att.z = Va/R*cos(gamma)

        # Set error indices
        self.ix = []  # Indices of trim states
        self.idx = slice(2, 12)  # Indices of trim derivatives
        self.iu = [3]

    def find_trim_state(self):
        # Find a trim state which satisfies the trim trajectory requirement
        if self.x_dot_des is None or self.x_des is None:
            raise ValueError('Target model derivatives or states not set')

        ix_argument = [3, 4, 6, 7, 8, 9, 10, 11]
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
                       method='SLSQP',
                       bounds = self.optim_bounds
                       )

        print(f'Optimization result:\n {res.message}')
        if res.success:
            return res.x
        else:
            return None

    def find_trim_input(self):
        # Find a trim input wich satisfies the trim state
        if self.x_des is None:
            raise ValueError('Target state not set')

    def cost_function_input_wrapper(self, arguments):
        # arguments: da, de, dt, dr
        inputs = mdl.Inputs()
        inputs.from_array(arguments)

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
        # Penalize derivatives as well as input

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

        return cost


if __name__ == "__main__":

    Va_des = 15
    gamma_des = 0
    R_des = np.inf

    trimmer = Trimmer()
    trimmer.setup_trim_trajectory(Va_des, gamma_des, R_des)
    trim_state = trimmer.find_trim_state()
