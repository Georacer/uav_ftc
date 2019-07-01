mkdir -p $1
echo "Copying log file..."
cp ~/.ros/UAV_recording.bag $1/UAV_recording.bag
echo "Filtering log file..."
rosbag filter $1/UAV_recording.bag $1/filtered.bag "'/skywalker_2013/' in topic"
echo "Extracting topic to .csv files..."
rostopic echo -b $1/filtered.bag -p /skywalker_2013/states > $1/states.csv
rostopic echo -b $1/filtered.bag -p /skywalker_2013/refRates > $1/ref.csv
rostopic echo -b $1/filtered.bag -p /skywalker_2013/ctrlSurfaceCmds > $1/input.csv
echo "Running plotting script"
./plot_rates.py $1
