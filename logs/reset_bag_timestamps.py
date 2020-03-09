#!/usr/bin/python

# Filter specific messages and shift their timestamps to 0
# Create a new bag file with them

import os
from copy import deepcopy

import click
import rosbag
from rospy import Time
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

topics_to_filter = [
    '/skywalker_2013_mod/waypoints',
    '/skywalker_2013_mod/obstacles',
    '/skywalker_2013_mod/rrt_path',
]


@click.command()
@click.option(
    '-i',
    '--input-file',
    help='Bag file to edit timestamps'
)
@click.option(
    '-o',
    '--outpout-file',
    default=None,
    help='Output filename'
)
def main_code(input_file, output_file):
    in_path = os.path.expanduser(input_file)
    print('Reading log {}'.format(in_path))
    if output_file is None:
        in_fileparts = os.path.split(in_path)
        in_filename = in_fileparts[-1]
        in_filename_head = os.path.splitext(in_filename)
        out_path = os.path.join(in_fileparts[0], in_filename_head + 'shifted_filtered.bag')
    else:
        out_path = os.path.expanduser(output_file)
    print('Writing to log {}'.format(out_path))

    in_bag = rosbag.Bag(in_path)
    out_bag = rosbag.Bag(out_path, 'w')

    # Find earliest message time
    start_time = None # Select starting time (to crop log if needed)
    for topic, msg, t in in_bag.read_messages(topics=topics_to_filter):
        # Depending on the message type, catpure the msg time
        if msg.__class__.__name__.endswith('Path'):
            msg_time = msg.header.stamp
        elif msg.__class__.__name__.endswith('MarkerArray'):
            msg_time = msg.markers[0].header.stamp
        else:
            raise TypeError('Topic type not as expected')

        if start_time == None:
            start_time = msg_time
            break
        #TODO: Should I use msg_time instead of t to compare time?
        elif start_time.secs > t.secs or (start_time.secs == t.secs and start_time.nsecs > t.nsecs):
            start_time = msg_time
            break
    print('Set start time at {}.{}'.format(start_time.secs, start_time.nsecs))

    # Parse and filter messages and shift their time
    try:
        for topic, msg, t in in_bag.read_messages(topics=topics_to_filter):

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

            # Forward the shifted messages into the new bag file
            out_bag.write(topic, msg, new_time)
    finally:
        out_bag.close()


if __name__ == '__main__':
    main_code()