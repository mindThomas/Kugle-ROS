/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include <ros/ros.h>
#include <kugle_misc/QuaternionVelocityControl.h>

int main(int argc, char **argv) {
	std::string nodeName = "velocity_control";
	ros::init(argc, argv, nodeName.c_str());
	ros::NodeHandle n("~");

	double rate;
	n.param("rate", rate, double(10));

	kugle_misc::QuaternionVelocityControl velocityControl(rate);
	ros::spin();
}
