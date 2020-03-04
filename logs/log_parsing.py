from numpy import arctan2 as atan2
from numpy import arcsin as asin
from numpy import pi

# Struct to hold log data to be pickled
class LogData:

    def __init__(self):
        self.time_databus = None
        self.p_n = None
        self.p_e = None
        self.p_d = None
        self.airspeed = None
        self.alpha = None
        self.beta = None
        self.phi = None
        self.theta = None
        self.psi = None
        self.p = None
        self.q = None
        self.r = None
        self.gamma = None
        self.psi_dot = None

        self.time_refRates = None
        self.ref_p = None
        self.ref_q = None
        self.ref_r = None

        self.time_refTrajectory = None
        self.ref_Va = None
        self.ref_gamma = None
        self.ref_psi_dot = None

def quat2euler2(x, y, z, w):
    q01 = w*x
    q02 = w*y
    q03 = w*z
    q11 = x*x
    q12 = x*y
    q13 = x*z
    q22 = y*y
    q23 = y*z
    q33 = z*z
    psi = atan2(2.0 * (q03 + q12), 1.0 - 2.0 * (q22 - q33))
    if psi < 0.0:
        psi += 2.0*pi

    theta = asin(2.0 * (q02 - q13))
    phi = atan2(2.0 * (q01 + q23), 1.0 - 2.0 * (q11 + q22))

    return (phi, theta, psi)
