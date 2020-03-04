#/usr/bin/python

import os
from collections import OrderedDict
import json

import rospy
import rospkg

from last_letter_msgs.msg import Parameter

class FaultGenerator:

    def __init__(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('uav_ftc')
        self.fault_dir = os.path.join(package_path, 'data/faults')
        # rospy.loginfo('Package found at {0}'.format(package_path))

    def generate_fault(self, fault_name):
        fault_file = os.path.join(self.fault_dir, fault_name+'.json')
        result = []
        with open(fault_file) as fh:
            try:
                json_dict = json.load(fh, object_pairs_hook=OrderedDict)
            except ValueError as e:
                rospy.logerr('Malformed json fault file')

            for param_name, param_data in json_dict.iteritems():
                msg = Parameter()
                msg.name = param_name
                msg.type = param_data['type']
                msg.value = param_data['value']
                result.append(msg)
            return result


if __name__ == '__main__':
    fg = FaultGenerator()
