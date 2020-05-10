import numpy as np

log_databus_attrs = [
    'time_databus',
    'gps_lat',
    'gps_lon',
    'gps_alt',
    'p_n',
    'p_e',
    'p_d',
    'v_n',
    'v_e',
    'v_d',
    'airspeed',
    'alpha',
    'beta',
    'qx',
    'qy',
    'qz',
    'qw',
    'phi',
    'theta',
    'psi',
    'p',
    'q',
    'r',
    'ax',
    'ay',
    'az',
    'wind_n',
    'wind_e',
    'wind_d',
    't_air',
    't_imu',
    'p_abs',
    'qbar',
    'rho',
    'g',
    'rps_motor',
    'rcin_1',
    'rcin_2',
    'rcin_3',
    'rcin_4',
    'rcin_5',
    'rcin_6',
    'rcin_7',
    'rcin_8',
    'rcout_1',
    'rcout_2',
    'rcout_3',
    'rcout_4',
    'gamma',
    'psi_dot',
]

log_ref_traj_attrs = [
    'time_refTrajectory',
    'ref_Va',
    'ref_gamma',
    'ref_psi_dot',
]

log_ref_rates_attrs = [
    'time_refRates',
    'ref_p',
    'ref_q',
    'ref_r',
]

log_fe_attrs = [
    'time_fe',
    'el_A',
    'el_B',
    'el_C',
    'el_D',
    'el_E',
    'el_F',
    'el_G',
    'el_H',
    'el_I',
    'el_J',
]

log_mission_attrs = [
    'waypoints',
    'obstacles',
    'ref_path',
]

log_attrs_groups = [
    log_databus_attrs,
    log_ref_rates_attrs,
    log_ref_traj_attrs,
    log_fe_attrs,
    log_mission_attrs,
]


# Struct to hold log data to be pickled
class LogData:

    def __init__(self):
        # Databus information
        for attr_name in log_databus_attrs:
            setattr(self, attr_name, None)

        # Reference angular rates
        for attr_name in log_ref_rates_attrs:
            setattr(self, attr_name, None)

        # Reference Trajectory
        for attr_name in log_ref_traj_attrs:
            setattr(self, attr_name, None)

        # Flight Envelope parameters
        for attr_name in log_fe_attrs:
            setattr(self, attr_name, None)

        # Waypoint information
        for attr_name in log_mission_attrs:
            setattr(self, attr_name, None)

    def __str__(self):
        s = []
        s.append('LogData object with contents:')
        s.append('-Databus messages: {}'.format(self.time_databus.shape[0]))
        s.append('-Trajectory reference messages: {}'.format(self.time_refTrajectory.shape[0]))
        s.append('-Angular rates reference messages: {}'.format(self.time_refRates.shape[0]))
        if self.time_fe is None:
            num_msgs = 0
        else:
            num_msgs = self.time_fe.shape[0]
        s.append('-Flight envelope messages: {}'.format(num_msgs))
        if self.waypoints is None:
            num_msgs = 0
        else:
            num_msgs = self.waypoints.shape[1]
        s.append('-Waypoint messages: {}'.format(num_msgs))
        if self.obstacles is None:
            num_msgs = 0
        else:
            num_msgs = self.obstacles.shape[1]
        s.append('-Obstacle messages: {}'.format(num_msgs))
        if self.ref_path is None:
            num_msgs = 0
        else:
            num_msgs = self.ref_path.shape[1]
        s.append('-Path messages: {}'.format(num_msgs))
        return '\n'.join(s)


def filter_log_data(log_data, t_start, t_end):
    log_data_filt = LogData()
    databus_start_idx = np.where(log_data.time_databus > t_start)[0][0]
    databus_end_idx = np.where(log_data.time_databus < t_end)[0][-1]
    for attr_name in log_databus_attrs:
        filtered_data = getattr(log_data, attr_name)[databus_start_idx:databus_end_idx+1]
        setattr(log_data_filt, attr_name, filtered_data)

    if len(log_data.time_refRates)>0:
        refRates_start_idx = np.where(log_data.time_refRates > t_start)[0][0]
        refRates_end_idx = np.where(log_data.time_refRates < t_end)[0][-1]
        for attr_name in log_ref_rates_attrs:
            filtered_data = getattr(log_data, attr_name)[refRates_start_idx:refRates_end_idx+1]
            setattr(log_data_filt, attr_name, filtered_data)

    if len(log_data.time_refTrajectory)>0:
        refTrajectory_start_idx = np.where(log_data.time_refTrajectory > t_start)[0][0]
        refTrajectory_end_idx = np.where(log_data.time_refTrajectory < t_end)[0][-1]
        for attr_name in log_ref_traj_attrs:
            filtered_data = getattr(log_data, attr_name)[refTrajectory_start_idx:refTrajectory_end_idx+1]
            setattr(log_data_filt, attr_name, filtered_data)

    if len(log_data.time_fe)>0:
        fe_start_idx = np.where(log_data.time_fe > t_start)[0][0]
        fe_end_idx = np.where(log_data.time_fe < t_end)[0][-1]
        for attr_name in log_fe_attrs:
            filtered_data = getattr(log_data, attr_name)[fe_start_idx:fe_end_idx+1]
            setattr(log_data_filt, attr_name, filtered_data)

    # Pass mission data directly
    log_data_filt.waypoints = log_data.waypoints
    log_data_filt.obstacles = log_data.obstacles
    log_data_filt.ref_path = log_data.ref_path

    return log_data_filt

def merge_log_data(log_data_iter):
    log_data = LogData()

    for log_attrs in log_attrs_groups:
        for attr_name in log_attrs:
            value_iter = []
            for log_data in log_data_iter:
                series = getattr(log_data, attr_name)
                if (
                    series is not None
                    and len(series)>0
                ):
                    value_iter.append(series)
            if len(value_iter)>0:
                setattr(log_data, attr_name, np.concatenate(value_iter))

    return log_data