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

struct {
    ros::Time time;
    double x;
    double y;
    double yawVel;
} VelocityReference;

void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    VelocityReference.time = ros::Time::now();
    VelocityReference.x = maximum_linear_velocity * msg->axes[1]; // left stick Y
    VelocityReference.y = maximum_linear_velocity * msg->axes[0]; // left stick X
    VelocityReference.yawVel = maximum_angular_velocity * msg->axes[2]; // right stick X

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
}

void PublishVelocity(double timeout)
{
    geometry_msgs::Twist velocityMsg;
    ros::Time current_ros_time = ros::Time::now();
    ros::Duration diff = current_ros_time - VelocityReference.time;

    velocityMsg.linear.x = 0;
    velocityMsg.linear.y = 0;
    velocityMsg.angular.x = 0;
    velocityMsg.angular.y = 0;
    velocityMsg.angular.z = 0;

    if (diff.toSec() < timeout) { // update message with joystick-based velocity reference
        velocityMsg.linear.x = VelocityReference.x;
        velocityMsg.linear.y = VelocityReference.y;
        velocityMsg.angular.z = VelocityReference.yawVel;
    }

    referencePub.publish(velocityMsg);
}

int main(int argc, char **argv) {
	std::string nodeName = "joystick_velocity_mapper";
	ros::init(argc, argv, nodeName.c_str());
	ros::NodeHandle n; // default/current namespace node handle
	ros::NodeHandle nParam("~"); // private node handle

	// Publish to reference topic
	referencePub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	// Subscribe to joystick topic
	joystickSub = n.subscribe("joy", 1000, &joystickCallback);

	// Get maximum velocity
    nParam.param("maximum_linear_velocity", maximum_linear_velocity, double(0.5));
    nParam.param("maximum_angular_velocity", maximum_angular_velocity, double(0.5));

    // Get publish rate
    int publish_rate;
    nParam.param("publish_rate", publish_rate, int(10));

	//ros::spin();
	ros::Rate loop_rate(publish_rate); // publish at specified publish rate
	while (ros::ok())
	{
		ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)
        PublishVelocity(1.0/publish_rate);
		loop_rate.sleep();
	}
}
