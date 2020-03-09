import numpy as np

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

        self.el_A = None
        self.el_B = None
        self.el_C = None
        self.el_D = None
        self.el_E = None
        self.el_F = None
        self.el_G = None
        self.el_H = None
        self.el_I = None
        self.el_J = None

def filter_log_data(log_data, t_start, t_end):
    # log_attributes = [attr in dir(log_data) if not attr.startswith('__')]
    # for attr in log_attributes:
    #     new_dataseries = getattr(log_data, attr)
    #     new_dataseries = new_dataseries[]
    #     setattr(log_data, attr, )
    databus_start_idx = np.where(log_data.time_databus > t_start)[0][0]
    databus_end_idx = np.where(log_data.time_databus < t_end)[0][-1]
    log_data.time_databus = log_data.time_databus[databus_start_idx:databus_end_idx]
    log_data.p_n = log_data.p_n[databus_start_idx:databus_end_idx]
    log_data.p_e = log_data.p_e[databus_start_idx:databus_end_idx]
    log_data.p_d = log_data.p_d[databus_start_idx:databus_end_idx]
    log_data.airspeed = log_data.airspeed[databus_start_idx:databus_end_idx]
    log_data.alpha = log_data.alpha[databus_start_idx:databus_end_idx]
    log_data.beta = log_data.beta[databus_start_idx:databus_end_idx]
    log_data.phi = log_data.phi[databus_start_idx:databus_end_idx]
    log_data.theta = log_data.theta[databus_start_idx:databus_end_idx]
    log_data.psi = log_data.psi[databus_start_idx:databus_end_idx]
    log_data.p = log_data.p[databus_start_idx:databus_end_idx]
    log_data.q = log_data.q[databus_start_idx:databus_end_idx]
    log_data.r = log_data.r[databus_start_idx:databus_end_idx]
    log_data.gamma = log_data.gamma[databus_start_idx:databus_end_idx]
    log_data.psi_dot = log_data.psi_dot[databus_start_idx:databus_end_idx]

    refRates_start_idx = np.where(log_data.time_refRates > t_start)[0][0]
    refRates_end_idx = np.where(log_data.time_refRates < t_end)[0][-1]
    log_data.time_refRates = log_data.time_refRates[refRates_start_idx:refRates_end_idx]
    log_data.ref_p = log_data.ref_p[refRates_start_idx:refRates_end_idx]
    log_data.ref_q = log_data.ref_q[refRates_start_idx:refRates_end_idx]
    log_data.ref_r = log_data.ref_r[refRates_start_idx:refRates_end_idx]

    refTrajectory_start_idx = np.where(log_data.time_refTrajectory > t_start)[0][0]
    refTrajectory_end_idx = np.where(log_data.time_refTrajectory < t_end)[0][-1]
    log_data.time_refTrajectory = log_data.time_refTrajectory[refTrajectory_start_idx:refTrajectory_end_idx]
    log_data.ref_Va = log_data.ref_Va[refTrajectory_start_idx:refTrajectory_end_idx]
    log_data.ref_gamma = log_data.ref_gamma[refTrajectory_start_idx:refTrajectory_end_idx]
    log_data.ref_psi_dot = log_data.ref_psi_dot[refTrajectory_start_idx:refTrajectory_end_idx]

    if len(log_data.time_fe)>0:
        fe_start_idx = np.where(log_data.time_fe > t_start)[0][0]
        fe_end_idx = np.where(log_data.time_fe < t_end)[0][-1]
        log_data.el_A = log_data.el_A[fe_start_idx:fe_end_idx]
        log_data.el_B = log_data.el_B[fe_start_idx:fe_end_idx]
        log_data.el_C = log_data.el_C[fe_start_idx:fe_end_idx]
        log_data.el_D = log_data.el_D[fe_start_idx:fe_end_idx]
        log_data.el_E = log_data.el_E[fe_start_idx:fe_end_idx]
        log_data.el_F = log_data.el_F[fe_start_idx:fe_end_idx]
        log_data.el_G = log_data.el_G[fe_start_idx:fe_end_idx]
        log_data.el_H = log_data.el_H[fe_start_idx:fe_end_idx]
        log_data.el_I = log_data.el_I[fe_start_idx:fe_end_idx]
        log_data.el_J = log_data.el_J[fe_start_idx:fe_end_idx]

    return log_data
