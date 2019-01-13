#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <boost/thread/recursive_mutex.hpp>

#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <string>

// For CLion to update/capture the updated parameter and message types, open the "build" folder and run "make"

/* Include generated Dynamic Reconfigure parameters */
#include <kugle_driver/ParametersConfig.h>

/* Include generated Services */
#include <kugle_srvs/AddTwoInts.h>

/* Include generated Message Types */
#include <kugle_msgs/Test.h>

void TestServiceClient(ros::NodeHandle &n);

dynamic_reconfigure::Server<kugle_driver::ParametersConfig> * serverPtr;
/* Initialize the parameters at once, otherwise the values will be random */
kugle_driver::ParametersConfig config;

void paramChangeCallback(kugle_driver::ParametersConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %f %s %s %d",
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
}

bool add(kugle_srvs::AddTwoInts::Request  &req,
		 kugle_srvs::AddTwoInts::Response &res)
{
	res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);

    // As a test, update int_param with the result
    //ros::NodeHandle n("~");
    //n.setParam("int_param", (int)res.sum);
    config.int_param = res.sum;
    serverPtr->updateConfig(config);

	return true;
}

int main(int argc, char **argv) {
	std::string nodeName = "driver";
	ros::init(argc, argv, nodeName.c_str());
	ros::NodeHandle n("~"); // default/current namespace node handle

	// Enable reconfigurable parameters - note that any parameters set on the node by roslaunch <param> tags will be seen by a dynamically reconfigurable node just as it would have been by a conventional node.
    boost::recursive_mutex configMutex;
	dynamic_reconfigure::Server<kugle_driver::ParametersConfig> server(configMutex);
	dynamic_reconfigure::Server<kugle_driver::ParametersConfig>::CallbackType f;
	f = boost::bind(&paramChangeCallback, _1, _2);
	server.setCallback(f);
    serverPtr = &server;

    server.getConfigDefault(config);
    server.updateConfig(config);

	// Create service
	ros::ServiceServer service = n.advertiseService("add_two_ints", add);

	// Create custom message publisher
    ros::Publisher pub = n.advertise<kugle_msgs::Test>("test", 1000);
	kugle_msgs::Test testMsg;
	testMsg.first_name = "Thomas";
    testMsg.last_name = "Jespersen";
	testMsg.age = 10;
	testMsg.score = 99;

	ros::Rate loop_rate(10); // 10 hz loop rate
	while (ros::ok())
	{
        pub.publish(testMsg);
		ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)
		loop_rate.sleep();
	}
}

// Note that the service call (client) can not be performed from within the same node from where the service is provided
void TestServiceClient(ros::NodeHandle &n)
{
	ros::ServiceClient client = n.serviceClient<kugle_srvs::AddTwoInts>("add_two_ints");
	kugle_srvs::AddTwoInts srv;
	srv.request.a = 10;
	srv.request.b = 20;
	if (client.call(srv))
	{
		ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
	}
}
