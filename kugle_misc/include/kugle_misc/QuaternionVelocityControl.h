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

#ifndef MODULES_CONTROLLERS_QUATERNIONVELOCITYCONTROL_H
#define MODULES_CONTROLLERS_QUATERNIONVELOCITYCONTROL_H

#include <stddef.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <kugle_misc/Quaternion.h>
#include <kugle_misc/FirstOrderLPF.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace kugle_misc {

class QuaternionVelocityControl
{
	public:
		QuaternionVelocityControl(double Rate, double ReferenceSmoothingTau = 0.1);
		~QuaternionVelocityControl();

	private:
        void Thread();
        void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void VelocityReferenceCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void VelocityReferenceInertialCallback(const geometry_msgs::Twist::ConstPtr& msg);
		void Step(const double q[4], const double dxy[2], const double velocityRef[2], const bool velocityRefGivenInHeadingFrame, const double headingRef, const double dt, double q_ref_out[4]);

	private:
        double rate_;
        boost::thread thread_;
        bool shouldExit_;
		ros::NodeHandle nh_;

        ros::Publisher quaternionPub_;
        ros::Subscriber odometrySub_;
        ros::Subscriber referenceSub_;
        ros::Subscriber referenceInertialSub_;
        tf::TransformListener tfListener_;

        geometry_msgs::Quaternion currentAttitude_;
        geometry_msgs::Vector3 currentVelocity_;
        geometry_msgs::Vector3 desiredVelocity_;
        double desiredAngularYawVelocity_;
        ros::Time lastReferenceUpdateTime_;
        bool velocityRefGivenInHeadingFrame_;

		FirstOrderLPF dx_ref_filt_;
		FirstOrderLPF dy_ref_filt_;

		double q_tilt_integral_[4];

		struct {
			double VelocityErrorClamp = 0.15; // velocity clamp for the proportional gain - note that at this velocity MaxTilt will be set [meters pr. second]
			double MaxTilt= 5.0; // max tilt that velocity controller can set [degrees]
			double IntegralGain = 0.3; // integral gain, which corresponds to the incremental compensation rate (1/gain is the number of seconds it takes the integral to reach a constant offset value)
			double MaxIntegralCorrection = 8.0; // max tilt integral effect can compensate with [degrees]
		} params_;
};
	
	
} // end namespace kugle_misc

#endif
