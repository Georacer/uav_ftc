#!/usr/bin/python

import rospy
import rospkg
import dynamic_reconfigure.client as dr_client

from trim_traj_fe import FlightEnvelope


class FlightEnvelopeROS:
    flight_envelope = None
    flag_needs_update = True
    safe_poly = None

    _dr_client = None

    def __init__(self):
        # Read the flight envelope box constraints from the parameter server
        uav_name = rospy.get_param("trimmer_uav_name")
        self.Va_max = rospy.get_param("Va_max", 25)
        self.Va_min = rospy.get_param("Va_min", 5)
        self.gamma_max_deg = rospy.get_param("gamma_max_deg", 30)
        self.gamma_min_deg = rospy.get_param("gamma_min_deg", -30)
        self.R_min = rospy.get_param("R_min", 100)

        self.flight_envelope = FlightEnvelope(uav_name)

        self.flight_envelope.set_domain_Va((self.Va_min, self.Va_max))
        self.flight_envelope.set_domain_gamma((self.gamma_min_deg, self.gamma_max_deg))
        self.flight_envelope.set_R_min(self.R_min)
        self.flight_envelope.initialize(uav_name)

        # Raise a dynamic reconfigure server to communicate changing flight envelope constraints
        self._dr_client = dr_client.Client("flight_envelope", timeout=2)

        # Set a topic callback for receiving parameter changes on the UAV model

    def update(self):
        self.safe_poly = self.flight_envelope.find_flight_envelope()
        # Approximate by ellipsoid
        self.safe_poly.ellipsoid_fit()
        self.send_fe_params()
        self.flag_needs_update = False
        rospy.loginfo("Sent new Flight Envelope parameters")

    def send_fe_params(self):
        ellipsoid_coeffs = self.safe_poly._el_v
        param_dict = dict()
        param_dict["Va_max"] = self.Va_max
        param_dict["Va_min"] = self.Va_max
        param_dict["gamma_max"] = self.gamma_max_deg
        param_dict["gamma_min"] = self.gamma_min_deg
        param_dict["R_min"] = self.R_min
        param_dict["el_A"] = ellipsoid_coeffs[0]
        param_dict["el_B"] = ellipsoid_coeffs[1]
        param_dict["el_C"] = ellipsoid_coeffs[2]
        param_dict["el_D"] = ellipsoid_coeffs[3]
        param_dict["el_E"] = ellipsoid_coeffs[4]
        param_dict["el_F"] = ellipsoid_coeffs[5]
        param_dict["el_G"] = ellipsoid_coeffs[6]
        param_dict["el_H"] = ellipsoid_coeffs[7]
        param_dict["el_I"] = ellipsoid_coeffs[8]
        param_dict["el_J"] = ellipsoid_coeffs[9]
        self._dr_client.update_configuration(param_dict)

    def parameter_callback(self, msg):
        self.flight_envelope.set_model_parameter(msg.name, msg.value)
        self.flag_needs_update = True


if __name__ == "__main__":

    rospy.init.node("Flight Envelope Generator", anonymous=True)

    flight_envelope = FlightEnvelopeROS()
    r = rospy.Rate(2)

    rospy.loginfo("Flight Envelope generator node up")

    while not rospy.is_shutdown():
        if flight_envelope.flag_needs_update:
            flight_envelope.update()
        r.sleep()
