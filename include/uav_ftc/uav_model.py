# %%
import numpy as np
from numpy import sin, cos, tan
from dataclasses import dataclass, field
from typing import Any

# %%


@dataclass
class Vector3:
    x: float = 0
    y: float = 0
    z: float = 0

    def to_array(self):
        return np.transpose(np.array([[self.x, self.y, self.z]]))

    @classmethod
    def from_array(cls, arr):
        # Convert a numpy array to Vector3
        if len(arr) == 1:  # arr is a 1-dimensional array
            x = arr[0]
            y = arr[1]
            z = arr[2]
        else:  # Arr is a 2-dimensional vertical vector array
            x = arr[0, 0]
            y = arr[1, 0]
            z = arr[2, 0]
        return cls(x,y,z)


@dataclass
class Inputs:
    delta_a: float = 0
    delta_e: float = 0
    delta_t: float = 0
    delta_r: float = 0

    def __repr__(self):
        s = (f'{self.__class__.__name__}:\n'
             f'delta_a={self.delta_a}\n'
             f'delta_e={self.delta_e}\n'
             f'delta_t={self.delta_t}\n'
             f'delta_r={self.delta_r}'
             )
        return s

    def to_array(self):
        return np.transpose(np.array([[self.delta_a,
                                       self.delta_e,
                                       self.delta_t,
                                       self.delta_r]]))
    @classmethod
    def from_array(cls, arr):
        # Convert a numpy array to Vector3
        if len(arr) == 1:  # arr is a 1-dimensional array
            delta_a = arr[0]
            delta_e = arr[1]
            delta_t = arr[2]
            delta_r = arr[3]
        else:  # Arr is a 2-dimensional vertical vector array
            delta_a = arr[0, 0]
            delta_e = arr[1, 0]
            delta_t = arr[2, 0]
            delta_r = arr[3, 0]
        return cls(delta_a, delta_e, delta_t, delta_r)


@dataclass
class aircraft_state:
    pos: Vector3 = field(default_factory=Vector3)  # NED position
    att: Vector3 = field(default_factory=Vector3)  # Euler angles attitude
    airdata: Vector3 = field(default_factory=Vector3)  # Airdata triplet
    # Body-frame angular velocity
    ang_vel: Vector3 = field(default_factory=Vector3)

    def __repr__(self):
        s = (f'{self.__class__.__name__}:\n'
             f'pos={repr(self.pos)}\n'
             f'att={repr(self.att)}\n'
             f'airdata={repr(self.airdata)}\n'
             f'ang_vel={repr(self.ang_vel)}'
             )
        return s

    def to_array(self):
        # Convert data object to a 12 x 1 numpy array
        output = np.zeros([12, 1])
        output[0, 0] = self.pos.x
        output[1, 0] = self.pos.y
        output[2, 0] = self.pos.z
        output[3, 0] = self.att.x
        output[4, 0] = self.att.y
        output[5, 0] = self.att.z
        output[6, 0] = self.airdata.x
        output[7, 0] = self.airdata.y
        output[8, 0] = self.airdata.z
        output[9, 0] = self.ang_vel.x
        output[10, 0] = self.ang_vel.y
        output[11, 0] = self.ang_vel.z

        return output


def eul_to_rot_mat(att: Vector3):
    # Convert Euler angle (roll, pitch, yaw) vector to rotation matrix from
    # Earth to Body frame
    phi = att.x
    theta = att.y
    psi = att.z

    sphi = sin(phi)
    cphi = cos(phi)
    sth = sin(theta)
    cth = cos(theta)
    spsi = sin(psi)
    cpsi = cos(psi)

    R = np.array([
        [cth*cpsi,                  cth*spsi,                 -sth],
        [sphi*sth*cpsi-cphi*spsi,   sphi*sth*spsi+cphi*cpsi,  sphi*cth],
        [cphi*sth*cpsi+sphi*spsi,    cphi*sth*spsi-sphi*cpsi, cphi*cth]
    ])
    return R


def eul_to_e(att: Vector3):
    # Create E matrix from Euler angle (roll, pitch, yaw) vector.
    # E matrix premultiplies angular rates to produce Euler angle derivatives
    # Returns a numpy 3x3 ndarray
    phi = att.x
    theta = att.y
    psi = att.z

    sphi = sin(phi)
    cphi = cos(phi)
    cth = cos(theta)
    tth = tan(theta)

    E = np.array([
        [1, sphi*tth, cphi*tth],
        [0, cphi, -sphi],
        [0, sphi/cth, cphi/cth]
    ])
    return E


def airdata_to_s(alpha, beta):
    # Create rotation matrix S, which transforms from body frame to wind frame
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)

    S = np.array([
        [ca*cb, sb, sa*cb],
        [-ca*sb, cb, -sa*sb],
        [-sa, 0, ca]
    ])
    return S


def airdata_to_u(airdata: Vector3) -> Vector3:
    # Convert airdata triplet to Body frame velocities
    Va = airdata.x
    alpha = airdata.y
    beta = airdata.z

    u = Va*cos(alpha)*cos(beta)
    v = Va*sin(beta)
    w = Va*sin(alpha)*cos(beta)

    return Vector3(u, v, w)


def get_qbar(V_a: float) -> float:
    # Return dynamic pressure
    rho = 1.225
    return 0.5*rho*V_a**2


def get_wrench_gravity(state: aircraft_state):
    # Returns a wrench tuple (F, M)
    # F is a Vector3 with forces in Body frame
    # M is a Vector3 with moments in Body frame

    par_i = inertial_parameters()
    m = par_i['m']

    euler_angles = state.att
    R_i_b = eul_to_rot_mat(euler_angles)
    g = 9.81
    Fg_i = np.array([[0, 0, m*g]]).T
    F = Vector3.from_array(R_i_b @ Fg_i)
    M = Vector3.from_array(np.zeros([3, 1]))
    return (F, M)


def get_wrench_prop(state: aircraft_state, inputs: Inputs):
    # Returns a wrench tuple (F, M)
    # F is a Vector3 with forces in the Body frame
    # M is a Vector3 with moments in Body frame

    s_prop = 0.2027
    c_prop = 1.0
    k_m = 40.0

    v_a = state.airdata.x
    deltat = inputs.delta_t
    thrust = 0.5*1.225*s_prop*((k_m*deltat)**2 - v_a**2)
    F = Vector3(thrust, 0, 0)
    M = Vector3()

    return (F, M)


def get_wrench_aerodynamics(state: aircraft_state, inputs: Inputs):
    # Returns a wrench tuple (F, M)
    # F is a Vector3 with forces in the Wind frame
    # M is a Vector3 with moments in Body frame

    # Write down aerodynamic parameters
    s = 0.45
    b = 1.88
    c = 0.24

    c_lift_0 = 0.56
    c_lift_deltae = 0.0
    c_lift_a = 6.9
    c_lift_q = 0
    mcoeff = 50
    oswald = 0.9
    alpha_stall = 0.4712
    c_drag_0 = 0.2
    c_drag_a = 3.0
    c_drag_q = 0
    c_drag_deltae = 0.0
    c_drag_p = 0.1
    c_y_0 = 0
    c_y_b = -0.98
    c_y_p = 0
    c_y_r = 0
    c_y_deltaa = 0
    c_y_deltar = -0.2  # opposite sign than c_n_deltar

    c_l_0 = 0
    c_l_p = -1.0
    c_l_b = -0.12
    c_l_r = 0.14
    c_l_deltaa = 0.25
    c_l_deltar = -0.037
    c_m_0 = 0.045
    c_m_a = -0.7
    c_m_q = -20
    c_m_deltae = 1.0
    c_n_0 = 0
    c_n_b = 0.25
    c_n_p = 0.022
    c_n_r = -1
    c_n_deltaa = 0.00
    c_n_deltar = 0.1  # opposite sign than c_y_deltar

    deltaa_max = 0.3491
    deltae_max = 0.3491
    deltar_max = 0.3491

    # Read needed states
    airdata = state.airdata
    Va = airdata.x
    alpha = airdata.y
    if (alpha > alpha_stall):
        alpha = alpha_stall
    beta = airdata.z

    omega = state.ang_vel
    p = omega.x
    q = omega.y
    r = omega.z

    deltaa = deltaa_max*inputs.delta_a
    deltae = deltae_max*inputs.delta_e
    deltar = deltar_max*inputs.delta_r

    qbar = get_qbar(Va)
    F_lift = qbar*s*(c_lift_0 + c_lift_a*alpha + c_lift_q*c/(2*Va)*q
                     + c_lift_deltae*deltae)
    F_drag = qbar*s*(c_drag_0 + c_drag_a*alpha + c_drag_q*c/(2*Va)*q
                     + c_drag_deltae*deltae)
    F_Y = qbar*s*(c_y_0 + c_y_b*beta + b/(2*Va)*(c_y_p*p + c_y_r*r)
                  + c_y_deltaa*deltaa + c_y_deltar*deltar)

    l = qbar*s*b*(c_l_0 + c_l_b*beta + b/(2*Va)*(c_l_p*p + c_l_r*r)
                  + c_l_deltaa*deltaa + c_l_deltar*deltar)
    m = qbar*s*c*(c_m_0 + c_m_a*alpha + c_m_q*c/(2*Va)*q + c_m_deltae*deltae)
    n = qbar*s*b*(c_n_0 + c_n_b*beta + b/(2*Va)*(c_n_p*p + c_n_r*r)
                  + c_n_deltaa*deltaa + c_n_deltar*deltar)

    F = Vector3(-F_drag, F_Y, -F_lift)
    M = Vector3(l, m, n)
    return (F, M)


def get_force_dervivatives(state: aircraft_state, inputs: Inputs) -> Vector3:
    # Returns tuple with derivatives of Va, alpha, beta
    par_i = inertial_parameters()
    Va = state.airdata.x
    alpha = state.airdata.y
    beta = state.airdata.z

    # Construct the forces in the wind frame
    F_aero, _ = get_wrench_aerodynamics(state, inputs)
    S = airdata_to_s(alpha, beta)
    F_prop_b, _ = get_wrench_prop(state, inputs)
    F_prop_arr = S @ F_prop_b.to_array()
    F_prop = Vector3.from_array(F_prop_arr)
    F_grav_b, _ = get_wrench_gravity(state)
    F_grav_arr = S @ F_grav_b.to_array()
    F_grav = Vector3.from_array(F_grav_arr)

    # Convert angular rates in wind frame
    vel_ang_w = S @ state.ang_vel.to_array()
    q_w = vel_ang_w[1, 0]
    r_w = vel_ang_w[2, 0]

    dot_va = 1/par_i['m']*(F_prop.x + F_aero.x + F_grav.x)
    dot_beta = 1/(par_i['m']*Va)*(F_prop.y + F_aero.y - par_i['m']*Va*r_w
                                  + F_grav.y)
    dot_alpha = 1/(par_i['m']*Va*cos(beta))*(F_prop.z + F_aero.z + par_i['m']*Va*q_w
                                             + F_grav.z)

    airdata_der = Vector3(dot_va, dot_alpha, dot_beta)
    return airdata_der


def get_moment_derivatives(state: aircraft_state, inputs: Inputs) -> Vector3:
    # Returns derivatives of p, q, r
    par_i = inertial_parameters()

    omega = state.ang_vel
    p = omega.x
    q = omega.y
    r = omega.z

    # Construct the moments in the body frame
    _, M_aero = get_wrench_aerodynamics(state, inputs)
    _, M_prop = get_wrench_prop(state, inputs)
    _, M_grav = get_wrench_gravity(state)

    # Construct composite inertial coefficients
    j_x = par_i['j_x']
    j_y = par_i['j_y']
    j_z = par_i['j_z']
    j_xz = par_i['j_xz']

    gamma = j_x*j_z - j_xz**2
    c_1 = ((j_y - j_z)*j_z - j_xz**2)/gamma
    c_2 = ((j_x - j_y + j_z)*j_xz)/gamma
    c_3 = j_z/gamma
    c_4 = j_xz/gamma
    c_5 = (j_z-j_x)/j_y
    c_6 = j_xz/j_y
    c_7 = 1/j_y
    c_8 = (j_x*(j_x-j_y) + j_xz**2)/gamma
    c_9 = j_x/gamma

    dot_p = (c_1*r + c_2*p)*q + c_3*(M_aero.x + M_prop.x + M_grav.x) + \
        + c_4*(M_aero.z + M_prop.z + M_grav.z)
    dot_q = c_5*p*r - c_6*(p**2 - r**2) + c_7*(M_aero.y + M_prop.y + M_grav.y)
    dot_r = (c_8*p - c_2*r)*q + c_4*(M_aero.x + M_prop.x + M_grav.x) \
        + c_9*(M_aero.z + M_prop.z + M_grav.z)

    moment_der = Vector3(dot_p, dot_q, dot_r)
    return moment_der


def get_attitude_derivatives(state: aircraft_state) -> Vector3:
    # Returns derivatives of Euler angles
    der_arr = eul_to_e(state.att) @ state.ang_vel.to_array()
    der_vec = Vector3.from_array(der_arr)
    return der_vec


def get_navigation_derivatives(state: aircraft_state) -> Vector3:
    B = eul_to_rot_mat(state.att)
    lin_vel_b = airdata_to_u(state.airdata)
    der_arr = B.T @ lin_vel_b.to_array()
    der_vec = Vector3.from_array(der_arr)
    return der_vec


def get_derivatives(state: aircraft_state, inputs: Inputs) -> aircraft_state:
    der_vec = aircraft_state()
    der_vec.airdata = get_force_dervivatives(state, inputs)
    der_vec.ang_vel = get_moment_derivatives(state, inputs)
    der_vec.att = get_attitude_derivatives(state)
    der_vec.pos = get_navigation_derivatives(state)
    return der_vec


def inertial_parameters():
    p = dict()
    p['m'] = 2.0
    p['j_x'] = 0.8244
    p['j_y'] = 1.135
    p['j_z'] = 1.759
    p['j_xz'] = 0.120

    return p


if __name__ == "__main__":

    current_state = aircraft_state()
    current_state.airdata.x = 15
    current_state.att.x = np.deg2rad(30)

    inputs = Inputs()
    inputs.delta_a = 0.5
    inputs.delta_e = 0.1
    inputs.delta_t = 0.5
    inputs.delta_r = 0.5

    derivatives = get_derivatives(current_state, inputs)
    print(derivatives)
    print('Successfully ran')


# %%
