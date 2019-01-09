/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

#include <string>

ros::Publisher referencePub;
ros::Subscriber joystickSub;
double maximum_linear_velocity;
double maximum_angular_velocity;
bool controlModeIsInertial;

void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Remap joystick message into twist (velocity) message
	geometry_msgs::TwistStamped velocityMsg;

	if (controlModeIsInertial)
		velocityMsg.header.frame_id = "world"; // control inertial velocity
	else
    	velocityMsg.header.frame_id = "robot"; // control velocity in heading frame

	velocityMsg.twist.linear.x = maximum_linear_velocity * msg->axes[1]; // left stick Y
    velocityMsg.twist.linear.y = maximum_linear_velocity * msg->axes[0]; // left stick X
    velocityMsg.twist.angular.z = maximum_angular_velocity * msg->axes[2]; // right stick X
	/*
	    this->buttonSq = joy->buttons[0];
        this->buttonX = joy->buttons[1];
        this->buttonO = joy->buttons[2];
        this->buttonTr = joy->buttons[3];
        this->buttonTouch = joy->buttons[13];
        this->l1 = joy->buttons[4];
        this->r1 = joy->buttons[5];

        this->arrowsX = joy->axes[9];
        this->arrowsY = joy->axes[10];
        this->l2 = joy->axes[3];
        this->r2 = joy->axes[4];

        this->leftStickX = joy->axes[0];
        this->leftStickY = joy->axes[1];
        this->rightStickX = joy->axes[2];
        this->rightStickY = joy->axes[5];
	 */

	referencePub.publish(velocityMsg);
}

int main(int argc, char **argv) {
	std::string nodeName = "joystick_velocity_mapper";
	ros::init(argc, argv, nodeName.c_str());
	ros::NodeHandle n; // default/current namespace node handle
	ros::NodeHandle nParam("~"); // private node handle

	// Publish to reference topic
	std::string reference_topic;
	nParam.param("reference_topic", reference_topic, std::string("cmd_velocity"));
	referencePub = n.advertise<geometry_msgs::TwistStamped>(reference_topic, 1000);

	// Subscribe to joystick topic
	std::string joystick_topic;
	nParam.param("joystick_topic", joystick_topic, std::string("joy"));
	joystickSub = n.subscribe(joystick_topic, 1000, &joystickCallback);

	// Get maximum velocity
    nParam.param("maximum_linear_velocity", maximum_linear_velocity, double(0.5));
    nParam.param("maximum_angular_velocity", maximum_angular_velocity, double(0.5));

    // Get control mode
	std::string control_mode;
	nParam.param("control_mode", control_mode, std::string("body")); // defaults to body velocity references
	if (!control_mode.compare("inertial")) // mode is inertial
		controlModeIsInertial = true;
	else
		controlModeIsInertial = false;

	ros::spin();
	/*
	while (ros::ok())
	{
		ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)
		loop_rate.sleep();
	}
	*/
}
