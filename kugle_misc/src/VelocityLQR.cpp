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

#include <kugle_misc/VelocityLQR.h>

#include <math.h>
#include <stdlib.h>
#include <cmath>

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <kugle_misc/Quaternion.h>
#include <kugle_misc/FirstOrderLPF.h>

namespace kugle_misc {

VelocityLQR::VelocityLQR(double Rate) : rate_(Rate), _omega_x_ref_filt(1.0/Rate, params_.OmegaLPFtau), _omega_y_ref_filt(1.0/Rate, params_.OmegaLPFtau), gainMatrix(Eigen::Map<const Eigen::Matrix<double, 2, 10, Eigen::RowMajor>>(const_cast<double*>(gainMatrixCoefficients)))
{
    ros::NodeHandle nParam("~");

    // Publish to quaternion reference topic
    quaternionPub_ = nh_.advertise<geometry_msgs::Quaternion>("cmd_quaternion", 1000);

    // Subscribe to state estimate topic
	stateEstimateSub_ = nh_.subscribe("StateEstimate", 1000, &VelocityLQR::StateEstimateCallback, this);

    // Subscribe to velocity reference
    referenceSub_ = nh_.subscribe("cmd_vel", 1000, &VelocityLQR::VelocityReferenceCallback, this);

    // Subscribe to velocity reference
    referenceInertialSub_ = nh_.subscribe("cmd_vel_inertial", 1000, &VelocityLQR::VelocityReferenceInertialCallback, this);

    // Load parameters
    /*nParam.param("velocity_error_clamp", params_.VelocityErrorClamp, params_.VelocityErrorClamp);
    nParam.param("max_tilt", params_.MaxTilt, params_.MaxTilt);
    nParam.param("integral_gain", params_.IntegralGain, params_.IntegralGain);
    nParam.param("max_integral_correction", params_.MaxIntegralCorrection, params_.MaxIntegralCorrection);*/

    _omega_x_ref_filt.ChangeTimeconstant(params_.OmegaLPFtau);
    _omega_y_ref_filt.ChangeTimeconstant(params_.OmegaLPFtau);

    // Reset internal module states
    currentStateEstimate.q.w = 1;
    currentStateEstimate.q.x = 0;
    currentStateEstimate.q.y = 0;
    currentStateEstimate.q.z = 0;
    currentStateEstimate.dq.w = 0;
    currentStateEstimate.dq.x = 0;
    currentStateEstimate.dq.y = 0;
    currentStateEstimate.dq.z = 0;
    currentStateEstimate.position[0] = 0;
    currentStateEstimate.position[1] = 0;
    currentStateEstimate.velocity[0] = 0;
    currentStateEstimate.velocity[1] = 0;
    velocityRefGivenInHeadingFrame_ = true;
    desiredVelocity_.x = 0;
    desiredVelocity_.y = 0;
    desiredAngularYawVelocity_ = 0;
    lastReferenceUpdateTime_ = ros::Time::now();

    // Reset roll and pitch reference variables (used to integrate the computed angular velocity reference)
    roll_ref = 0;
    pitch_ref = 0;

    position_control_enabled = false;

    // Reset position reference used for station keeping
    position_reference[0] = 0;
    position_reference[1] = 0;

    // Reset velocity error integral
    velocity_error_integral[0] = 0;
    velocity_error_integral[1] = 0;

    VelocityIntegralAfterPowerup = true;
    VelocityIntegralInitializationTime = 5; // wait 5 seconds in the beginning for integrator to settle (and before allowing manual movement)

    shouldExit_ = false;
    thread_ = boost::thread(boost::bind(&VelocityLQR::Thread, this));
}

VelocityLQR::~VelocityLQR()
{
    shouldExit_ = true;
    thread_.join();
}

void VelocityLQR::Thread()
{
    ros::Rate loop_rate(rate_); // 10 hz loop rate
    double dt = 1.0 / rate_;

    geometry_msgs::Quaternion quaternionMsg;
    double desiredYaw = 0;
    double q[4];
	double dq[4];
    double xy[2];
    double dxy[2];
    double velocityRef[2];
    double q_ref[4];

    while (ros::ok() && !shouldExit_)
    {
        q[0] = currentStateEstimate.q.w;
        q[1] = currentStateEstimate.q.x;
        q[2] = currentStateEstimate.q.y;
        q[3] = currentStateEstimate.q.z;

		dq[0] = currentStateEstimate.dq.w;
		dq[1] = currentStateEstimate.dq.x;
		dq[2] = currentStateEstimate.dq.y;
		dq[3] = currentStateEstimate.dq.z;

        xy[0] = currentStateEstimate.position[0];
		xy[1] = currentStateEstimate.position[1];

        dxy[0] = currentStateEstimate.velocity[0]; // velocity in inertial frame
        dxy[1] = currentStateEstimate.velocity[1];

        velocityRef[0] = desiredVelocity_.x;
        velocityRef[1] = desiredVelocity_.y;

        desiredYaw += dt * desiredAngularYawVelocity_;

		Step(xy, q, dxy, dq, velocityRef, velocityRefGivenInHeadingFrame_, desiredYaw, params_.VelocityClamp, params_.AngularVelocityClamp, params_.AccelerationLimit, params_.MaxTilt, params_.IntegralEnabled, params_.PositionControlAtZeroVelocityReference, params_.PositionControlAtZeroVelocityReference_MaximumKickinVelocity, params_.StabilizationDetectionVelocity, dt, q_ref);

        quaternionMsg.w = q_ref[0];
        quaternionMsg.x = q_ref[1];
        quaternionMsg.y = q_ref[2];
        quaternionMsg.z = q_ref[3];

        quaternionPub_.publish(quaternionMsg);
        ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)
        loop_rate.sleep();
    }
}

void VelocityLQR::StateEstimateCallback(const kugle_msgs::StateEstimate::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Received odometry, translational velocity: (" << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << ")");
    currentStateEstimate.q = msg->q;
	currentStateEstimate.dq = msg->dq;
	currentStateEstimate.position = msg->position;
	currentStateEstimate.velocity = msg->velocity;
}

void VelocityLQR::VelocityReferenceCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    desiredVelocity_ = msg->linear;
    desiredAngularYawVelocity_ = msg->angular.z;
    velocityRefGivenInHeadingFrame_ = true;
    lastReferenceUpdateTime_ = ros::Time::now();
}

void VelocityLQR::VelocityReferenceInertialCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    desiredVelocity_ = msg->linear;
    desiredAngularYawVelocity_ = msg->angular.z;
    velocityRefGivenInHeadingFrame_ = false;
    lastReferenceUpdateTime_ = ros::Time::now();
}

void VelocityLQR::Step(const double xy[2], const double q[4], const double dxy[2], const double dq[4], const double velocityRef[2], const bool velocityRefGivenInHeadingFrame, const double headingRef, const double velocity_error_clamp, const double angular_velocity_clamp, const double acceleration_limit, const double MaxTilt, const bool VelocityIntegralEnabled, const bool PositionControlAtZeroVelocityReference, const double PositionControlAtZeroVelocityReference_MaximumKickinVelocity, const double StabilizationDetectionVelocity, const double dt, double q_ref_out[4])
{
	/* Form error state vector */
	// The error state is linearized in the heading frame. Hence both velocity error, quaternions etc. has to be transformed into heading frame (rotated with heading)
	// X_err = [dx_err_integral, dy_err_integral, q2, q3, dx_err, dy_err, dq2, dq3, q2_ref_prev, q3_ref_prev]
	Eigen::Matrix<double, 10, 1> X_err;
	double * pos_err_heading = &X_err(0);
	double * vel_err_integral_heading = &X_err(0);
	double * q_tilt_xy = &X_err(2);
	double * vel_err_heading = &X_err(4);
	double * dq_tilt_xy = &X_err(6);
	double * q_ref_tilt_xy = &X_err(8);

	/* Extract current heading */
	double heading = HeadingFromQuaternion(q);
	double q_heading[4];
	HeadingQuaternion(q, q_heading);

	/* Convert quaternion parts into heading frame */
	/* Compute previously set reference quaternion */
	double q_ref_tilt[4];
	Quaternion_eul2quat_zyx(0, pitch_ref, roll_ref, q_ref_tilt);

	/* Extract tilt quaternion */
	double q_tilt[4]; // current tilt
	/* q_tilt = q_heading* o q; */
	Quaternion_PhiT(q_heading, q, q_tilt);

	/* Extract body angular velocity */
	double omega_body[3];
	Quaternion_GetAngularVelocity_Body(q, dq, omega_body);

	/* Compute quaternion derivative in heading frame */
	double dq_tilt[4];
	/* dq_tilt = 1/2 * Phi(q_tilt) * [0;omega_body]; */
	Quaternion_GetDQ_FromBody(q_tilt, omega_body, dq_tilt);

	/* Set state vector with quaternion states */
	q_ref_tilt_xy[0] = q_ref_tilt[1];
	q_ref_tilt_xy[1] = q_ref_tilt[2];
	q_tilt_xy[0] = q_tilt[1];
	q_tilt_xy[1] = q_tilt[2];
	dq_tilt_xy[0] = dq_tilt[1];
	dq_tilt_xy[1] = dq_tilt[2];

	/* Update position reference if we are supposed to be moving */
	if (!position_control_enabled) {
		position_reference[0] = xy[0];
		position_reference[1] = xy[1];
	}

	/* Enable position control (station keeping) if velocity reference is zero */
	if (!VelocityIntegralAfterPowerup && PositionControlAtZeroVelocityReference && velocityRef[0] == 0 && velocityRef[1] == 0 && (dxy[0]*dxy[0] + dxy[1]*dxy[1] < PositionControlAtZeroVelocityReference_MaximumKickinVelocity*PositionControlAtZeroVelocityReference_MaximumKickinVelocity)) {
		position_control_enabled = true;
	} else {
		position_control_enabled = false;
	}

	if (VelocityIntegralInitializationTime > 0) {
		VelocityIntegralInitializationTime -= dt;
		if (VelocityIntegralInitializationTime < 0) VelocityIntegralInitializationTime = 0;

		if (velocityRef[0] != 0 || velocityRef[1] != 0) { // velocity reference demands movement
			VelocityIntegralInitializationTime = 0;
			VelocityIntegralAfterPowerup = false;
		}
	}
	else if (VelocityIntegralAfterPowerup) {
		if (velocityRef[0] != 0 || velocityRef[1] != 0 ||
			(dxy[0]*dxy[0] + dxy[1]*dxy[1] > StabilizationDetectionVelocity*StabilizationDetectionVelocity)) { // manual movement of velocity reference requires the initialization integral to be disabled
			VelocityIntegralInitializationTime = 0;
			VelocityIntegralAfterPowerup = false;
		}
	}

	/* Compute position error */
	double pos_err[2];
	pos_err[0] = xy[0] - position_reference[0];
	pos_err[1] = xy[1] - position_reference[1];

	/* Rotate position error from inertial frame into heading frame */
	Math_Rotate2D(pos_err, -heading, pos_err_heading);

	/* Compute velocity error */
	double vel_err[2];
	vel_err[0] = dxy[0];
	vel_err[1] = dxy[1];

	if (!velocityRefGivenInHeadingFrame) {
		vel_err[0] -= velocityRef[0];
		vel_err[1] -= velocityRef[1];
	}

	/* Rotation velocity error from inertial frame into heading frame */
	Math_Rotate2D(vel_err, -heading, vel_err_heading);

	if (velocityRefGivenInHeadingFrame) {
		vel_err_heading[0] -= velocityRef[0];
		vel_err_heading[1] -= velocityRef[1];
	}

	/* Compute reference in different frames for debugging */
	if (velocityRefGivenInHeadingFrame) {
		Velocity_Reference_Filtered_Heading[0] = velocityRef[0];
		Velocity_Reference_Filtered_Heading[1] = velocityRef[1];
		Math_Rotate2D(velocityRef, heading, Velocity_Reference_Filtered_Inertial);
	} else {
		Velocity_Reference_Filtered_Inertial[0] = velocityRef[0];
		Velocity_Reference_Filtered_Inertial[1] = velocityRef[1];
		Math_Rotate2D(velocityRef, -heading, Velocity_Reference_Filtered_Heading);
	}

	/* Clamp velocity error */
	vel_err_heading[0] = fminf(fmaxf(vel_err_heading[0], -velocity_error_clamp), velocity_error_clamp);
	vel_err_heading[1] = fminf(fmaxf(vel_err_heading[1], -velocity_error_clamp), velocity_error_clamp);

	/* Include integral term as current integral value */
	vel_err_integral_heading[0] += velocity_error_integral[0];
	vel_err_integral_heading[1] += velocity_error_integral[1];

	/* Update integral */
	if (VelocityIntegralEnabled || VelocityIntegralAfterPowerup) {
		// Rate limit the integration
		velocity_error_integral[0] += dt * fmaxf(fminf(vel_err_heading[0], 0.05), -0.05);
		velocity_error_integral[1] += dt * fmaxf(fminf(vel_err_heading[1], 0.05), -0.05);
	}

	/* Compute angular velocity reference by matrix multiplication with LQR gain */
	Eigen::Matrix<double, 2, 1> omega_ref_control;
	omega_ref_control = -gainMatrix * X_err;

	/* Saturate angular velocity output */
	omega_ref_control(0) = fminf(fmaxf(omega_ref_control(0), -angular_velocity_clamp), angular_velocity_clamp);
	omega_ref_control(1) = fminf(fmaxf(omega_ref_control(1), -angular_velocity_clamp), angular_velocity_clamp);

	/* Filter angular velocity before setting as output */
	omega_ref_control(0) = _omega_x_ref_filt.Filter(omega_ref_control(0));
	omega_ref_control(1) = _omega_y_ref_filt.Filter(omega_ref_control(1));

	/* Integrate angle reference */
	roll_ref += dt * omega_ref_control(0);
	pitch_ref += dt * omega_ref_control(1);

	/* Saturate roll and pitch references */
	roll_ref = fminf(fmaxf(roll_ref, -MaxTilt), MaxTilt);
	pitch_ref = fminf(fmaxf(pitch_ref, -MaxTilt), MaxTilt);

	/* Compute and set output quaternion */
	Quaternion_eul2quat_zyx(headingRef, pitch_ref, roll_ref, q_ref_out);
}

} // end namespace kugle_misc
