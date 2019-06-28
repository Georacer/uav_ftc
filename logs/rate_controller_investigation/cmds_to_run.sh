rosbag filter $1/UAV_recording.bag $1/filtered.bag "'/skywalker_2013/' in topic"
rostopic echo -b $1/filtered.bag -p /skywalker_2013/states > $1/states.csv
rostopic echo -b $1/filtered.bag -p /skywalker_2013/refCmds > $1/ref.csv
rostopic echo -b $1/filtered.bag -p /skywalker_2013/ctrlSurfaceCmds > $1/input.csv
