#!/usr/bin/python

# Use
# rosbag filter ~/.ros/waypoints.bag ~/.ros/waypoints_filtered.bag "topic in ('/skywalker_2013_mod/waypoints', '/skywalker_2013_mod/obstacles', '/skywalker_2013_mod/rrt_path')"
# To generate a filtered bag

import os
from copy import deepcopy

import rosbag
from rospy import Time
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

in_path = os.path.expanduser('~/.ros/waypoints.bag')
out_path = os.path.expanduser('~/.ros/waypoints_filtered_synched.bag')

in_bag = rosbag.Bag(in_path)
out_bag = rosbag.Bag(out_path, 'w')

# did_find_start_time = False
start_time = None

# Find earliest message time
for topic, msg, t in in_bag.read_messages(topics=[
    '/skywalker_2013_mod/waypoints',
    '/skywalker_2013_mod/obstacles',
    '/skywalker_2013_mod/rrt_path',
    ]):
    if msg.__class__.__name__.endswith('Path'):
        msg_time = msg.header.stamp
    if msg.__class__.__name__.endswith('MarkerArray'):
        msg_time = msg.markers[0].header.stamp

    if start_time == None:
        start_time = msg_time
    elif start_time.secs > t.secs or (start_time.secs == t.secs and start_time.nsecs > t.nsecs):
        start_time = msg_time
print('Set start time at {}.{}'.format(start_time.secs, start_time.nsecs))


try:
    for topic, msg, t in in_bag.read_messages(topics=[
        '/skywalker_2013_mod/waypoints',
        '/skywalker_2013_mod/obstacles',
        '/skywalker_2013_mod/rrt_path',
        ]):

        # if type(msg) is Path:
        if msg.__class__.__name__.endswith('Path'):
            msg_time = msg.header.stamp
            new_time = msg_time - start_time
            msg.header.stamp = Time(new_time.secs, new_time.nsecs)

        # if type(msg) is MarkerArray:
        if msg.__class__.__name__.endswith('MarkerArray'):
            msg_time = msg.markers[0].header.stamp
            for idx in range(len(msg.markers)):
                new_time = msg.markers[idx].header.stamp - start_time
                msg.markers[idx].header.stamp = Time(new_time.secs, new_time.nsecs)

        out_bag.write(topic, msg, new_time)
finally:
    out_bag.close()
