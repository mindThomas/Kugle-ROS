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

#include <kugle_misc/QuaternionVelocityControl.h>

#include <math.h>
#include <stdlib.h>
#include <cmath>

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <kugle_misc/Quaternion.h>
#include <kugle_misc/FirstOrderLPF.h>

namespace kugle_misc {

QuaternionVelocityControl::QuaternionVelocityControl(double Rate, double ReferenceSmoothingTau) : rate_(Rate), dx_ref_filt_(1.0/Rate, ReferenceSmoothingTau), dy_ref_filt_(1.0/Rate, ReferenceSmoothingTau)
{
    ros::NodeHandle nParam("~");

	// Reset integral quaternion to the unit quaternion (no integral)
	q_tilt_integral_[0] = 1;
	q_tilt_integral_[1] = 0;
	q_tilt_integral_[2] = 0;
	q_tilt_integral_[3] = 0;

	// Publish to quaternion reference topic
	std::string quaternion_topic;
    nParam.param("quaternion_topic", quaternion_topic, std::string("cmd_quaternion"));
    quaternionPub_ = nh_.advertise<geometry_msgs::Quaternion>(quaternion_topic, 1000);

	// Subscribe to odometry topic
	std::string odometry_topic;
    nParam.param("odometry_topic", odometry_topic, std::string("odom_raw"));
    odometrySub_ = nh_.subscribe(odometry_topic, 1000, &QuaternionVelocityControl::OdometryCallback, this);

    // Subscribe to velocity reference
    std::string reference_topic;
    nParam.param("reference_topic", reference_topic, std::string("cmd_velocity"));
    referenceSub_ = nh_.subscribe(reference_topic, 1000, &QuaternionVelocityControl::VelocityReferenceCallback, this);

    // Load parameters
    nParam.param("velocity_error_clamp", params_.VelocityErrorClamp, params_.VelocityErrorClamp);
    nParam.param("max_tilt", params_.MaxTilt, params_.MaxTilt);
    nParam.param("integral_gain", params_.IntegralGain, params_.IntegralGain);
    nParam.param("max_integral_correction", params_.MaxIntegralCorrection, params_.MaxIntegralCorrection);

    // Reset internal module states
    currentAttitude_.w = 1;
    currentAttitude_.x = 0;
    currentAttitude_.y = 0;
    currentAttitude_.z = 0;
    currentVelocity_.x = 0;
    currentVelocity_.y = 0;
    velocityRefGivenInHeadingFrame_ = true;
    desiredVelocity_.x = 0;
    desiredVelocity_.y = 0;
    desiredAngularYawVelocity_ = 0;
    lastReferenceUpdateTime_ = ros::Time::now();

    shouldExit_ = false;
    thread_ = boost::thread(boost::bind(&QuaternionVelocityControl::Thread, this));
}

QuaternionVelocityControl::~QuaternionVelocityControl()
{
    shouldExit_ = true;
    thread_.join();
}

void QuaternionVelocityControl::Thread()
{
    ros::Rate loop_rate(rate_); // 10 hz loop rate
    double dt = 1.0 / rate_;

    geometry_msgs::Quaternion quaternionMsg;
    double desiredYaw = 0;
    double q[4];
    double dxy[2];
    double velocityRef[2];
    double q_ref[4];

    while (ros::ok() && !shouldExit_)
    {
        q[0] = currentAttitude_.w;
        q[1] = currentAttitude_.x;
        q[2] = currentAttitude_.y;
        q[3] = currentAttitude_.z;

        dxy[0] = currentVelocity_.x; // notice that this velocity is in heading frame
        dxy[1] = currentVelocity_.y;

        velocityRef[0] = desiredVelocity_.x;
        velocityRef[1] = desiredVelocity_.y;

        desiredYaw += dt * desiredAngularYawVelocity_;

        Step(q, dxy, velocityRef, velocityRefGivenInHeadingFrame_, desiredYaw, dt, q_ref);

        quaternionMsg.w = q_ref[0];
        quaternionMsg.x = q_ref[1];
        quaternionMsg.y = q_ref[2];
        quaternionMsg.z = q_ref[3];

        quaternionPub_.publish(quaternionMsg);
        ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)
        loop_rate.sleep();
    }
}

void QuaternionVelocityControl::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Received odometry, translational velocity: (" << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << ")");
    currentAttitude_ = msg->pose.pose.orientation;
    currentVelocity_ = msg->twist.twist.linear; // notice that this velocity is in heading frame
}

void QuaternionVelocityControl::VelocityReferenceCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    desiredVelocity_ = msg->twist.linear;
    desiredAngularYawVelocity_ = msg->twist.angular.z;
    if (msg->header.frame_id.find("robot") != std::string::npos) // frame id (name) contains "robot"
        velocityRefGivenInHeadingFrame_ = true;
    else
        velocityRefGivenInHeadingFrame_ = false;
    lastReferenceUpdateTime_ = ros::Time::now();
}

void QuaternionVelocityControl::Step(const double q[4], const double dxy[2], const double velocityRef[2], const bool velocityRefGivenInHeadingFrame, const double headingRef, const double dt, double q_ref_out[4])
{
	//const double Velocity_Inertial_q[4] = {0, dx_filt.sample(fmin(fmax(*dx, -CLAMP_VELOCITY), CLAMP_VELOCITY)), dy_filt.sample(fmin(fmax(*dy, -CLAMP_VELOCITY), CLAMP_VELOCITY)), 0};
	double Velocity_Heading_q[4] = {0, dxy[0], dxy[1], 0};
	double Velocity_Heading[2];

	double Velocity_Reference_Filtered[2];
	Velocity_Reference_Filtered[0] = dx_ref_filt_.Filter(velocityRef[0]);
	Velocity_Reference_Filtered[1] = dy_ref_filt_.Filter(velocityRef[1]);

	if (velocityRefGivenInHeadingFrame) { // reference given in inertial frame
		// Calculate velocity error
        Velocity_Heading_q[1] -= Velocity_Reference_Filtered[0];
        Velocity_Heading_q[2] -= Velocity_Reference_Filtered[1];
	}
	else  // reference given in heading frame
    {
        double Velocity_Ref_Inertial_q[4] = {0, Velocity_Reference_Filtered[0], Velocity_Reference_Filtered[1], 0};
        double Velocity_Ref_Heading_q[4];
        //Velocity_Ref_Heading = [0,1,0,0;0,0,1,0] * Phi(q)' * Gamma(q) * [0;Velocity_Ref;0];
        double tmp_q[4];
        Quaternion_Gamma(q, Velocity_Ref_Inertial_q, tmp_q); // Gamma(q) * [0;Velocity_Ref;0];
        Quaternion_PhiT(q, tmp_q, Velocity_Ref_Heading_q); // Phi(q)' * Gamma(q) * [0;velocityRef;0];

        Velocity_Heading_q[1] -= Velocity_Ref_Heading_q[1];
        Velocity_Heading_q[2] -= Velocity_Ref_Heading_q[2];
    }

    // OBS. The above could also be replaced with a Quaternion transformation function that takes in a 3-dimensional vector

	Velocity_Heading[0] = std::min(fmax(Velocity_Heading_q[1], -params_.VelocityErrorClamp), params_.VelocityErrorClamp);
	Velocity_Heading[1] = std::min(fmax(Velocity_Heading_q[2], -params_.VelocityErrorClamp), params_.VelocityErrorClamp);

	/*Velocity_Heading_Integral[0] += INTEGRAL_GAIN/_SampleRate * fmin(fmax(Velocity_Heading[0], -CLAMP_VELOCITY), CLAMP_VELOCITY); // INTEGRAL_GAIN * dt * velocity
	Velocity_Heading_Integral[1] += INTEGRAL_GAIN/_SampleRate * fmin(fmax(Velocity_Heading[1], -CLAMP_VELOCITY), CLAMP_VELOCITY);

	Velocity_Heading[0] += Velocity_Heading_Integral[0]; // correct with integral error
	Velocity_Heading[1] += Velocity_Heading_Integral[1];*/

	double q_heading[4]; // heading reference quaternion based on heading reference
	q_heading[0] = cos(headingRef/2);
	q_heading[1] = 0;
	q_heading[2] = 0;
	q_heading[3] = sin(headingRef/2);

	double normVelocity_Heading = sqrt(Velocity_Heading[0]*Velocity_Heading[0] + Velocity_Heading[1]*Velocity_Heading[1]);
	if (normVelocity_Heading == 0) {
		// Return upright quaternion (with heading reference) if we have no velocity, since we are then where we are supposed to be
		q_ref_out[0] = q_heading[0];
		q_ref_out[1] = q_heading[1];
		q_ref_out[2] = q_heading[2];
		q_ref_out[3] = q_heading[3];
		return;
	}

	//CorrectionDirection = [0, 1; -1, 0] * Velocity_Heading / normVelocity_Heading;
	double CorrectionDirection[2] = {Velocity_Heading[1] / normVelocity_Heading,
																 -Velocity_Heading[0] / normVelocity_Heading};

	// CorrectionAmountRadian = min(max((normVelocity_Heading / 5), -1), 1) * deg2rad(30); % max 5 m/s resulting in 30 degree tilt <= this is the proportional gain
	double CorrectionAmountRadian = std::min(std::max(normVelocity_Heading / params_.VelocityErrorClamp, -1.0), 1.0) * deg2rad(params_.MaxTilt);

	double q_tilt[4]; // tilt reference quaternion defined in heading frame
	q_tilt[0] = cos(CorrectionAmountRadian/2);
	q_tilt[1] = sin(CorrectionAmountRadian/2)*CorrectionDirection[0];
	q_tilt[2] = sin(CorrectionAmountRadian/2)*CorrectionDirection[1];
	q_tilt[3] = 0;

	if (velocityRef[0] != 0 || velocityRef[1] != 0) {
		CorrectionAmountRadian = 0; // only do integral action when velocity reference is zero
	}

	double q_tilt_integral__incremental[4];
	q_tilt_integral__incremental[0] = cos(CorrectionAmountRadian/2 * params_.IntegralGain * dt);
	q_tilt_integral__incremental[1] = sin(CorrectionAmountRadian/2 * params_.IntegralGain * dt)*CorrectionDirection[0];
	q_tilt_integral__incremental[2] = sin(CorrectionAmountRadian/2 * params_.IntegralGain * dt)*CorrectionDirection[1];
	q_tilt_integral__incremental[3] = 0;

	double q_tilt_integral__old[4];
	for (int i = 0; i < 4; i++) q_tilt_integral__old[i] = q_tilt_integral_[i];

	Quaternion_Phi(q_tilt_integral__old, q_tilt_integral__incremental, q_tilt_integral_);
	Quaternion_AngleClamp(q_tilt_integral_, deg2rad(params_.MaxIntegralCorrection), q_tilt_integral_);

	double q_tilt_withintegral[4];
	Quaternion_Phi(q_tilt, q_tilt_integral_, q_tilt_withintegral);

	// Add heading to quaternion reference = q_heading o q_tilt
	Quaternion_Phi(q_heading, q_tilt_withintegral, q_ref_out);
}

} // end namespace kugle_misc
