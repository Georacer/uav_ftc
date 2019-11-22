#!/usr/bin/python

import rospy
import rospkg

from uav_ftc.trim_traj_fe import FlightEnvelope

from uav_ftc.msg import FlightEnvelopeEllipsoid
from last_letter_msgs.msg import Parameter


class FlightEnvelopeROS:
    flight_envelope = None
    flag_needs_update = True
    safe_poly = None

    _dr_client = None
    _pub = None

    def __init__(self):
        # Read the flight envelope box constraints from the parameter server
        uav_name = rospy.get_param("~uav_name")
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

        # Set a topic publisher for the flight envelope description
        self._pub = rospy.Publisher('flight_envelope', FlightEnvelopeEllipsoid, queue_size=10)

        # Set a topic callback for receiving parameter changes on the UAV model
        param_topic_name = 'estimated_parameters'
        rospy.Subscriber(param_topic_name, Parameter, self.parameter_callback)

    def update(self):
        self.flight_envelope.update_model() # Write any new model parameters received so far
        self.safe_poly = self.flight_envelope.find_flight_envelope()
        # Approximate by ellipsoid
        self.safe_poly.ellipsoid_fit()
        self.send_fe_params()
        self.flag_needs_update = False
        rospy.loginfo("Sent new Flight Envelope parameters")

    def send_fe_params(self):
        fe = FlightEnvelopeEllipsoid()

        ellipsoid_coeffs = self.safe_poly._el_v
        fe.Va_max = self.Va_max
        fe.Va_min = self.Va_min
        fe.gamma_max = self.gamma_max_deg
        fe.gamma_min = self.gamma_min_deg
        fe.R_min = self.R_min
        fe.el_A = ellipsoid_coeffs[0]
        fe.el_B = ellipsoid_coeffs[1]
        fe.el_C = ellipsoid_coeffs[2]
        fe.el_D = ellipsoid_coeffs[3]
        fe.el_E = ellipsoid_coeffs[4]
        fe.el_F = ellipsoid_coeffs[5]
        fe.el_G = ellipsoid_coeffs[6]
        fe.el_H = ellipsoid_coeffs[7]
        fe.el_I = ellipsoid_coeffs[8]
        fe.el_J = ellipsoid_coeffs[9]
        self._pub.publish(fe)

    def parameter_callback(self, msg):
        self.flight_envelope.set_model_parameter(msg.type, msg.name, msg.value)
        self.flag_needs_update = True


if __name__ == "__main__":

    rospy.init_node("Flight Envelope Generator", anonymous=True)

    flight_envelope = FlightEnvelopeROS()
    r = rospy.Rate(2)

    rospy.loginfo("Flight Envelope generator node up")

    while not rospy.is_shutdown():
        if flight_envelope.flag_needs_update:
            flight_envelope.update()
        r.sleep()
