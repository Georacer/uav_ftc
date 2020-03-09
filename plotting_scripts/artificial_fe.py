parameters_for_randomization = [
    (3, 'm', 2.0),
    (3, 'j_x', 0.8244),
    (3, 'j_y', 1.135),
    (3, 'j_z', 1.759),
    (3, 'j_xz', 0.1204),

    (4, 'airfoil1/s', 0.45),
    (4, 'airfoil1/b', 1.88),
    (4, 'airfoil1/c', 0.24),
    (4, 'airfoil1/c_L_0', 0.4),
    (4, 'airfoil1/c_L_deltae', 0.0),
    (4, 'airfoil1/c_L_alpha', 6.5),
    (4, 'airfoil1/c_L_qn', 0),
    (4, 'airfoil1/mcoeff', 50),
    (4, 'airfoil1/oswald', 0.9),
    (4, 'airfoil1/alpha_stall', 0.4712),
    (4, 'airfoil1/c_D_qn', 0),
    (4, 'airfoil1/c_D_deltae', 0.0),
    (4, 'airfoil1/c_D_0', 0.09),
    (4, 'airfoil1/c_D_alpha', 0.14),
    (4, 'airfoil1/c_Y_0', 0),
    (4, 'airfoil1/c_Y_beta', -0.98),
    (4, 'airfoil1/c_Y_pn', 0),
    (4, 'airfoil1/c_Y_rn', 0),
    (4, 'airfoil1/c_Y_deltaa', 0),
    (4, 'airfoil1/c_Y_deltar', -0.2), #opposite sign than c_n_deltar
    (4, 'airfoil1/c_l_0', 0),
    (4, 'airfoil1/c_l_pn', -1.0),
    (4, 'airfoil1/c_l_beta', -0.12),
    (4, 'airfoil1/c_l_rn', 0.14),
    (4, 'airfoil1/c_l_deltaa', 0.25),
    (4, 'airfoil1/c_l_deltar', -0.037),
    (4, 'airfoil1/c_m_0', 0.01),
    (4, 'airfoil1/c_m_alpha', -1.3),
    (4, 'airfoil1/c_m_qn', -20),
    (4, 'airfoil1/c_m_deltae', 1.0),
    (4, 'airfoil1/c_n_0', 0),
    (4, 'airfoil1/c_n_beta', 0.25),
    (4, 'airfoil1/c_n_pn', 0.022),
    (4, 'airfoil1/c_n_rn', -1),
    (4, 'airfoil1/c_n_deltaa', 0.00),
    (4, 'airfoil1/c_n_deltar', 0.1), #opposite sign than c_y_deltar
    (4, 'airfoil1/deltaa_max', 0.3491),
    (4, 'airfoil1/deltae_max', 0.3491),
    (4, 'airfoil1/deltar_max', 0.3491),
    (4, 'airfoil1/deltaa_max_nominal', 0.3491),
    (4, 'airfoil1/deltae_max_nominal', 0.3491),
    (4, 'airfoil1/deltar_max_nominal', 0.3491),

    (5, 'motor1/propDiam', 0.28),
]


def randomize_parameter(fe, parameter_tuple):
    param_type, param_name, param_value = parameter_tuple
    sign = np.random.choice(np.array([-1, 1]))

    change_prct = 0.05
    if param_value == 0:
        new_value = sign*0.05
    else:
        new_value = param_value + sign*param_value*change_prct

    result = fe.set_model_parameter( param_type, param_name, new_value)  
    if result:
        print("Succeeded changing {}".format(param_name))
    else:
        print("Failed changing {}".format(param_name))


def randomize_parameter_list(fe, parameter_list):
    for parameter_tuple in parameter_list:
        randomize_parameter(fe, parameter_tuple)


def get_image_name(fault_idx):
    if fault_idx == 0:
        img_name = 'nominal'
    if fault_idx == 1:
        img_name = 'fault_motor'
    if fault_idx == 2:
        img_name = 'fault_aileron'
    if fault_idx == 3:
        img_name = 'fault_flap'
    if fault_idx == 4:
        img_name = 'param_variation'
    return img_name


# Currently unused. May be useful, but have yet to find its use.
def get_flight_envelope(model_name, fault_idx):
    flight_envelope = fe.FlightEnvelope(model_name)

    # Set the limits of the search domain
    flight_envelope.set_domain_Va((5, 25))
    flight_envelope.set_domain_gamma((-30, 30))
    flight_envelope.set_R_min(100)
    flight_envelope.initialize(model_name)

    flag_plot_points = True
    img_name = get_image_name(fault_idx)
    if fault_idx == 0:
        print('Flight Envelope for the nominal model')
    if fault_idx == 1:
        print('Inducing engine fault')
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
        result = flight_envelope.set_model_parameter( 5, "motor1/omega_max", 10)  # Zero-out propeller efficiency
        if result:
            print("Succeeded")
        else:
            print("Failed")
        flight_envelope.update_model()  # Register model changes
        flag_plot_points = False  # FE too small and poitns hide it

    elif fault_idx == 2:
        print('Inducing aileron fault')
        result = flight_envelope.set_model_parameter( 4, "airfoil1/c_l_deltaa", 0.125)  # 50% of the original  
        if result:
            print("Succeeded")
        else:
            print("Failed")
        flight_envelope.update_model()  # Register model changes

    elif fault_idx == 3:
        print('Inducing flap fault')
        result = flight_envelope.set_model_parameter( 4, "airfoil1/c_l_0", 0.09)  
        if result:
            print("Succeeded")
        else:
            print("Failed")
        result = flight_envelope.set_model_parameter( 4, "airfoil1/c_n_0", 0.02)
        if result:
            print("Succeeded")
        else:
            print("Failed")
        flight_envelope.update_model()  # Register model changes
        flag_plot_points = False  # FE too small and poitns hide it

    elif fault_idx == 4:
        print('Inducing parameter variations')
        randomize_parameter_list(flight_envelope, parameters_for_randomization)  # Batch randomization
        flight_envelope.update_model()  # Register model changes
        flag_plot_points = True  # FE too small and points hide it

    # Calculate flight envelope
    safe_poly = flight_envelope.find_flight_envelope()

    return safe_poly


# Accepts a SafeConvexPolytope instance
def get_fe_ellipsoid(flight_envelope):
    return flight_envelope.ellipsoid_fit()