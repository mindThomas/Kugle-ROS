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
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <string>

void Quaternion_Integration_Body(tf::Quaternion &q, const double omega_body[3], const double dt);
void Quaternion_Integration_Inertial(tf::Quaternion &q, const double omega_inertial[3], const double dt);

double angularVelocityReference[3];
bool angularVelocityReferenceInBody = true;
ros::Time lastReferenceUpdateTime;

void omegaBodyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	//ROS_INFO_STREAM("Angular velocity: (" << msg->angular.x << ", " << msg->angular.y << ", " << msg->angular.z << ")");
	angularVelocityReference[0] = msg->angular.x;
	angularVelocityReference[1] = msg->angular.y;
	angularVelocityReference[2] = msg->angular.z;
    angularVelocityReferenceInBody = true;
	lastReferenceUpdateTime = ros::Time::now();
}

void omegaInertialCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Angular velocity: (" << msg->angular.x << ", " << msg->angular.y << ", " << msg->angular.z << ")");
    angularVelocityReference[0] = msg->angular.x;
    angularVelocityReference[1] = msg->angular.y;
    angularVelocityReference[2] = msg->angular.z;
    angularVelocityReferenceInBody = false;
    lastReferenceUpdateTime = ros::Time::now();
}

int main(int argc, char **argv) {
	std::string nodeName = "quaternion_integrator";
	ros::init(argc, argv, nodeName.c_str());
	ros::NodeHandle n; // default/current namespace node handle
	ros::NodeHandle nParam("~"); // private node handle

	// Configure angular velocity topic to subscribe to
	std::string angular_velocity_body_topic;
	if (!nParam.getParam("angular_velocity_body_topic", angular_velocity_body_topic)) {
		angular_velocity_body_topic = "cmd_vel_angular";
		ROS_WARN_STREAM("[" << nodeName << "] angular_velocity_body_topic not set. Defaults to: " << angular_velocity_body_topic);
	}
	// Alternative is
	//nParam.param("omega_topic", omega_topic, std::string("cmd_omega"));
	ros::Subscriber sub_body = n.subscribe(angular_velocity_body_topic, 1000, omegaBodyCallback);

	// Configure angular velocity topic to subscribe to
	std::string angular_velocity_inertial_topic;
	if (!nParam.getParam("angular_velocity_inertial_topic", angular_velocity_inertial_topic)) {
        angular_velocity_inertial_topic = "cmd_vel_angular_inertial";
		ROS_WARN_STREAM("[" << nodeName << "] angular_velocity_inertial_topic not set. Defaults to: " << angular_velocity_inertial_topic);
	}
	// Alternative is
	//nParam.param("omega_topic", omega_topic, std::string("cmd_omega"));
	ros::Subscriber sub_inertial = n.subscribe(angular_velocity_inertial_topic, 1000, omegaInertialCallback);

	// Configure quaternion topic to publish to
	std::string quaternion_topic;
	if (!nParam.getParam("quaternion_topic", quaternion_topic)) {
		quaternion_topic = "cmd_quaternion";
		ROS_WARN_STREAM("[" << nodeName << "] quaternion_topic not set. Defaults to: " << quaternion_topic);
	}
	ros::Publisher pub = n.advertise<geometry_msgs::Quaternion>(quaternion_topic, 1000);

	// Configure timeout
	double timeout;
	if (!nParam.getParam("timeout", timeout)) {
		timeout = 1;
		ROS_WARN_STREAM("[" << nodeName << "] timeout not set. Defaults to: " << timeout << " second");
	}

	// Configure publish rate
	double rate;
	if (!nParam.getParam("rate", rate)) {
		rate = 200;
		ROS_WARN_STREAM("[" << nodeName << "] publish rate not set. Defaults to: " << rate);
	}
	ros::Rate loop_rate(rate);
	double dt = 1.0 / rate;

	// Initialize current angular velocity reference setpoint
	angularVelocityReference[0] = 0.0;
	angularVelocityReference[1] = 0.0;
	angularVelocityReference[2] = 0.0;
	lastReferenceUpdateTime = ros::Time::now();

	// Initialize quaternion as a unit quaternion
	tf::Quaternion quaternion(0,0,0,1);

	while (ros::ok())
	{
		ros::Time currentTime = ros::Time::now();
		ros::Duration timeDiff = currentTime - lastReferenceUpdateTime;
		if (timeDiff.toSec() > timeout) { // reset angular velocity due to timeout
			angularVelocityReference[0] = 0.0;
			angularVelocityReference[1] = 0.0;
			angularVelocityReference[2] = 0.0;
			lastReferenceUpdateTime = ros::Time::now();

            double yaw, pitch, roll;
            tf::Matrix3x3 mat(quaternion);
            mat.getEulerYPR(yaw, pitch, roll);
            quaternion.setRPY(0,0,yaw); // reset to only current heading
		}

		if (angularVelocityReferenceInBody)
		    Quaternion_Integration_Body(quaternion, angularVelocityReference, dt);
		else
            Quaternion_Integration_Body(quaternion, angularVelocityReference, dt);

        geometry_msgs::Quaternion pubQuaternion;
        quaternionTFToMsg(quaternion, pubQuaternion);
		pub.publish(pubQuaternion);
		ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)
		loop_rate.sleep();
	}
}

void Quaternion_Integration_Body(tf::Quaternion &q, const double omega_body[3], const double dt)
{
	/* Quaternion Exponential method
     * q_out = q o exp(1/2*dt*q_omeg)
     * q_omeg = [0,omeg_x,omeg_y,omeg_z]
     */
	double omega_norm = sqrt(omega_body[0]*omega_body[0] + omega_body[1]*omega_body[1] + omega_body[2]*omega_body[2]);
	double q_exp[4];

	if (omega_norm > 0) {
		double sinOmeg = sinf(0.5f * dt * omega_norm);
		q_exp[0] = cosf(0.5f * dt * omega_norm); // scalar part
		q_exp[1] = sinOmeg * omega_body[0] / omega_norm;
		q_exp[2] = sinOmeg * omega_body[1] / omega_norm;
		q_exp[3] = sinOmeg * omega_body[2] / omega_norm;
	} else {
		// unit quaternion since the angular velocity is zero (no movement)
		q_exp[0] = 1.0f;
		q_exp[1] = 0.0f;
		q_exp[2] = 0.0f;
		q_exp[3] = 0.0f;
	}

	// q_out = Phi(q) o q_exp
	double q_in[4] = {q.w(), q.x(), q.y(), q.z()};
	q.setW( q_in[0]*q_exp[0] - q_in[1]*q_exp[1] - q_in[2]*q_exp[2] - q_in[3]*q_exp[3] );
    q.setX( q_in[1]*q_exp[0] + q_in[0]*q_exp[1] - q_in[3]*q_exp[2] + q_in[2]*q_exp[3] );
    q.setY( q_in[2]*q_exp[0] + q_in[3]*q_exp[1] + q_in[0]*q_exp[2] - q_in[1]*q_exp[3] );
	q.setZ( q_in[3]*q_exp[0] - q_in[2]*q_exp[1] + q_in[1]*q_exp[2] + q_in[0]*q_exp[3] );
}

void Quaternion_Integration_Inertial(tf::Quaternion &q, const double omega_inertial[3], const double dt)
{
	/* Quaternion Exponential method
     * q_out = exp(1/2*dt*q_omeg) o q
     * q_omeg = [0,omeg_x,omeg_y,omeg_z]
     */
	double omega_norm = sqrtf(omega_inertial[0]*omega_inertial[0] + omega_inertial[1]*omega_inertial[1] + omega_inertial[2]*omega_inertial[2]);
	double q_exp[4];

	if (omega_norm > 0) {
		double sinOmeg = sinf(0.5f * dt * omega_norm);
		q_exp[0] = cosf(0.5f * dt * omega_norm); // scalar part
		q_exp[1] = sinOmeg * omega_inertial[0] / omega_norm;
		q_exp[2] = sinOmeg * omega_inertial[1] / omega_norm;
		q_exp[3] = sinOmeg * omega_inertial[2] / omega_norm;
	} else {
		// unit quaternion since the angular velocity is zero (no movement)
		q_exp[0] = 1.0f;
		q_exp[1] = 0.0f;
		q_exp[2] = 0.0f;
		q_exp[3] = 0.0f;
	}

	// q_out = Gamma(q) o q_exp
	double q_in[4] = {q.w(), q.x(), q.y(), q.z()};
    q.setW( q_exp[0]*q_in[0] - q_exp[1]*q_in[1] - q_exp[2]*q_in[2] - q_exp[3]*q_in[3] );
    q.setX( q_exp[1]*q_in[0] + q_exp[0]*q_in[1] + q_exp[3]*q_in[2] - q_exp[2]*q_in[3] );
    q.setY( q_exp[2]*q_in[0] - q_exp[3]*q_in[1] + q_exp[0]*q_in[2] + q_exp[1]*q_in[3] );
    q.setZ( q_exp[3]*q_in[0] + q_exp[2]*q_in[1] - q_exp[1]*q_in[2] + q_exp[0]*q_in[3] );
}