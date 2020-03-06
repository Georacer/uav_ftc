#!/usr/bin/env python

# Create and publish a static message of a Flight Envelope
# Good for FE testing purposes.

import os
import json

import rospy
import rospkg

from uav_ftc.msg import FlightEnvelopeEllipsoid


def build_flight_envelope(fe_dict):
    msg = FlightEnvelopeEllipsoid()
    msg.header.stamp = rospy.Time.now()
    msg.Va_max = fe_dict["Va_max"]
    msg.Va_min = fe_dict["Va_min"]
    msg.gamma_max = fe_dict["gamma_max"]
    msg.gamma_min = fe_dict["gamma_min"]
    msg.R_min = fe_dict["R_min"]
    msg.el_A = fe_dict["el_A"]
    msg.el_B = fe_dict["el_B"]
    msg.el_C = fe_dict["el_C"]
    msg.el_D = fe_dict["el_D"]
    msg.el_E = fe_dict["el_E"]
    msg.el_F = fe_dict["el_F"]
    msg.el_G = fe_dict["el_G"]
    msg.el_H = fe_dict["el_H"]
    msg.el_I = fe_dict["el_I"]
    msg.el_J = fe_dict["el_J"]

    return msg


if __name__ == '__main__':

    rospy.init_node('flight_envelope_pub', anonymous=True)
    rospy.loginfo('Flight Envelope publisher node up')
    r = rospy.Rate(1)
    fe_pub = rospy.Publisher('flight_envelope', FlightEnvelopeEllipsoid, queue_size=1) # Setup FE publisher

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('uav_ftc')
    rospy.loginfo('Package found at {0}'.format(package_path))

    # Specify flight envelope to use
    fe_file = rospy.get_param('~fe', 'none')


    if fe_file != 'none':

        rospy.loginfo('Using Flight Envelope: {0}'.format(fe_file))
        full_filename = os.path.join(package_path, 'data/flight_envelopes', fe_file+'.json')
        rospy.loginfo('Full file name: {0}'.format(full_filename))

        with open(full_filename) as fh:
            try:
                json_dict = json.load(fh)
            except ValueError as e:
                rospy.logerr('Malformed json Flight Envelope file')

        fe = build_flight_envelope(json_dict)

        while not rospy.is_shutdown():
            fe_pub.publish(fe)
            r.sleep()

    else:
        rospy.logwarn('No flight envelope description passed')
