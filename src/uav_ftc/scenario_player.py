#!/usr/bin/python

import os
from collections import OrderedDict
import json

import rospy
import rospkg
from geometry_msgs.msg import Vector3Stamped

class Scheduler:

    scenario = None
    pending_item = None
    reference = None
    end_reached = False
    start_time = None
    relative_time = False

    def __init__(self, scenario_file):
        self.start_time = rospy.Time.now()  # Store initialization instance
        self.pub = rospy.Publisher('refTrajectory', Vector3Stamped, queue_size=10)

        with open(scenario_file) as fh:
            try:
                json_dict = json.load(fh, object_pairs_hook=OrderedDict)
            except ValueError as e:
                rospy.logerr('Malformed json scenario file')

            self.reference = Vector3Stamped()

            self.scenario = json_dict
            # Read if there is a relative_time specification
            self.relative_time = self.scenario.pop('relative_time', False)
            if self.relative_time:
                rospy.loginfo('Mission specified relative timing')

            try:
                self.popitem()
            except StopIteration:
                self.end_reached = True

    def check_time(self):
        now = rospy.Time.now()
        if self.relative_time:
            time = now - self.start_time
        else:
            time = now
        # print('Time: {}'.format(time.secs))

        if self.pending_item['time'] <= time.secs:
            self.parse_next_reference()

    def parse_next_reference(self):
        # Apply the reference
        try:
            self.apply_reference(self.pending_item)
        except AttributeError as e:
            print(e)
        # Pop the next item
        try:
            self.popitem()
        except StopIteration:
            self.end_reached = True

    def apply_reference(self, ref_item):
        for attribute, value in ref_item.iteritems():
            if attribute == 'time':
                self.reference.header.stamp = rospy.Time.now()
                continue

            if attribute == 'airspeed':
                v_attr_name = 'x'
            elif attribute == 'gamma':
                v_attr_name = 'y'
            elif attribute == 'turn_radius':
                v_attr_name = 'z'
            else:
                raise AttributeError('Invalid reference attribute {}'.format(attribute))
            setattr(self.reference.vector, v_attr_name, value)
        rospy.loginfo("New reference:\n {}".format(self.reference))
        self.pub.publish(self.reference)

        
    def popitem(self):
        while True:
            try:
                # print(self.scenario)
                _, self.pending_item = self.scenario.popitem(last=False)
                # print(self.pending_item)
                if 'time' not in self.pending_item.keys():
                    rospy.logwarn('Scenario item did not specify time attribute. Popping the next one')
                    raise AttributeError
                break
            except AttributeError:
                continue
            except KeyError:  # OrderedDict has been exhausted
                raise StopIteration
            # else:
            #     break


if __name__ == '__main__':
    rospy.init_node('scenario_player', anonymous=True)
    rospy.loginfo('Scenario player node up')
    r = rospy.Rate(10)

    scenario_file = rospy.get_param('~scenario')

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('uav_ftc')
    rospy.loginfo('Package found at {0}'.format(package_path))

    if scenario_file != 'none':

        rospy.loginfo('Playing scenario {0}'.format(scenario_file))
        full_filename = os.path.join(package_path, 'data/scenarios', scenario_file+'.json')
        rospy.loginfo('Full file name: {0}'.format(full_filename))

        scheduler = Scheduler(full_filename)

        while not rospy.is_shutdown() and not scheduler.end_reached:
            scheduler.check_time()
            r.sleep()

    rospy.loginfo("Scenario ended.")