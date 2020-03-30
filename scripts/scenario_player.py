#!/usr/bin/python

import os
from collections import OrderedDict
import json

import rospy
import rospkg
from geometry_msgs.msg import Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray

from last_letter_msgs.msg import Parameter
import uav_ftc.fault_generator as fg

class Scheduler:

    scenario = None
    pending_item = None
    reference = None
    wp_list = None
    wp_count = 0
    timestamp = None
    end_reached = False
    start_time = None
    relative_time = False

    def __init__(self, scenario_file):
        self.start_time = rospy.Time.now()  # Store initialization instance
        self.ref_rates_pub = rospy.Publisher('refRates', Vector3Stamped, queue_size=10) # Setup rates reference publisher
        self.ref_traj_pub = rospy.Publisher('refTrajectory', Vector3Stamped, queue_size=10) # Setup trajectory reference publisher
        self.ref_wp_pub = rospy.Publisher('waypoints', MarkerArray, queue_size=10) # Setup waypoint reference publisher
        # Setup fault publisher
        self.fg = fg.FaultGenerator()
        self.fault_pub = rospy.Publisher('parameter_changes', Parameter, queue_size=100)

        with open(scenario_file) as fh:
            try:
                json_dict = json.load(fh, object_pairs_hook=OrderedDict)
            except ValueError as e:
                rospy.logerr('Malformed json scenario file')

            self.reference = Vector3Stamped()
            self.wp_list = MarkerArray()

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
            self.parse_next_point()

    def parse_next_point(self):
        # Apply the reference
        try:
            self.parse_point(self.pending_item)
        except AttributeError as e:
            print(e)
        # Pop the next item
        try:
            self.popitem()
        except StopIteration:
            self.end_reached = True

    def parse_point(self, ref_item):
        for attribute, value in ref_item.iteritems():
            if attribute == 'time':
            # set the timestamp
                self.timestamp = rospy.Time.now()
                continue

            if attribute == 'angular_rates':
            # Read and publish a new reference rates tuple
                rospy.loginfo('Publishing new reference rates')
                for ref_type, ref_value in value.iteritems():
                    if ref_type == 'p':
                        v_attr_name = 'x'
                    elif ref_type == 'q':
                        v_attr_name = 'y'
                    elif ref_type == 'r':
                        v_attr_name = 'z'
                    else:
                        raise AttributeError('Invalid reference attribute {}'.format(ref_type))
                    setattr(self.reference.vector, v_attr_name, ref_value)
                rospy.loginfo("New reference:\n {}".format(self.reference))
                self.reference.header.stamp = self.timestamp
                self.ref_rates_pub.publish(self.reference)

            if attribute == 'trajectory':
            # Read and publish a new reference trajectory
                rospy.loginfo('Publishing new reference trajectory')
                for ref_type, ref_value in value.iteritems():
                    if ref_type == 'airspeed':
                        v_attr_name = 'x'
                    elif ref_type == 'gamma':
                        v_attr_name = 'y'
                    elif ref_type == 'psi_dot':
                        v_attr_name = 'z'
                    else:
                        raise AttributeError('Invalid reference attribute {}'.format(ref_type))
                    setattr(self.reference.vector, v_attr_name, ref_value)
                rospy.loginfo("New reference:\n {}".format(self.reference))
                self.reference.header.stamp = self.timestamp
                self.ref_traj_pub.publish(self.reference)

            if attribute == 'waypoint':
            # Read and a new reference waypoint
                rospy.loginfo('Reading new reference waypoint')
                flush = False

                new_wp = Marker()
                new_wp.header.stamp = self.timestamp
                new_wp.header.frame_id = 'map'
                new_wp.action = new_wp.ADD
                new_wp.type = new_wp.SPHERE
                new_wp.id = self.wp_count
                new_wp.pose.orientation.x = 0
                new_wp.pose.orientation.y = 0
                new_wp.pose.orientation.z = 0
                new_wp.pose.orientation.w = 1
                new_wp.color.r = 0
                new_wp.color.g = 1
                new_wp.color.b = 0
                new_wp.color.a = 1

                self.wp_count += 1
                for ref_type, ref_value in value.iteritems():
                    if ref_type == 'n':
                        new_wp.pose.position.x = ref_value
                    elif ref_type == 'e':
                        new_wp.pose.position.y = ref_value
                    elif ref_type == 'd':
                        new_wp.pose.position.z = ref_value
                    elif ref_type == 'r':
                        new_wp.scale.x = 2*ref_value
                        new_wp.scale.y = 2*ref_value
                        new_wp.scale.z = 2*ref_value
                    elif ref_type == 'flush':
                        flush = ref_value
                    else:
                        raise AttributeError('Invalid reference attribute {}'.format(ref_type))
                rospy.loginfo("New waypoint reference:\n {}".format(new_wp))
                self.wp_list.markers.append(new_wp)
                if flush:
                    self.ref_wp_pub.publish(self.wp_list)

            if attribute == 'fault':
            # Signal a new fault from a predetermined set of faults
                rospy.loginfo('Publishing new fault')
                message_list = self.fg.generate_fault(value)
                for msg in message_list:
                    msg.header.stamp = self.timestamp
                    self.fault_pub.publish(msg)

        
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