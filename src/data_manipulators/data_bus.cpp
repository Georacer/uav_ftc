#include "data_bus.hpp"

#include "data_from_last_letter.cpp"


DataBus::DataBus(ros::NodeHandle n, uint data_source)
{
    if (data_source == DATA_SOURCE_LL)
    {
        sub_handler_ = SubHandlerLL(n);
    }
    data_pub_ = n.advertise<uav_ftc::BusData>("dataBus", 1000);
}

void DataBus::publish_data()
{
    bus_data_ = sub_handler_.bus_data;
    data_pub_.publish(bus_data_);
}

void DataBus::set_pub_rate(double rate)
{
    pub_rate_ = rate;
}

void DataBus::run()
{
	ros::Rate spinner(pub_rate_);

	while (ros::ok())
	{
		ros::spinOnce();
		spinner.sleep();
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_bus");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
    int pub_rate;
	np.param("pub_rate", pub_rate, 100); //publication rate in Hz
    int data_source;
	if (!np.getParam("data_source", data_source))
    {
        ROS_ERROR("Could not read 'data_source' parameter");
        ros::shutdown();
    } //publication rate in Hz

    DataBus data_bus(n, data_source);
    data_bus.set_pub_rate(pub_rate);
	ROS_INFO("DataBus node up");

    data_bus.run();
}