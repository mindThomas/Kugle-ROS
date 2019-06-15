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

#ifndef MODULES_CONTROLLERS_VELOCITYLQR_H
#define MODULES_CONTROLLERS_VELOCITYLQR_H

#include <stddef.h>
#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <kugle_misc/Quaternion.h>
#include <kugle_misc/FirstOrderLPF.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <kugle_msgs/StateEstimate.h>

namespace kugle_misc {

class VelocityLQR
{
	public:
		VelocityLQR(double Rate);
		~VelocityLQR();

	private:
        void Thread();
		void StateEstimateCallback(const kugle_msgs::StateEstimate::ConstPtr& msg);
        void VelocityReferenceCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void VelocityReferenceInertialCallback(const geometry_msgs::Twist::ConstPtr& msg);
		void Step(const double xy[2], const double q[4], const double dxy[2], const double dq[4], const double velocityRef[2], const bool velocityRefGivenInHeadingFrame, const double headingRef, const double velocity_error_clamp, const double angular_velocity_clamp, const double acceleration_limit, const double MaxTilt, const bool VelocityIntegralEnabled, const bool PositionControlAtZeroVelocityReference, const double PositionControlAtZeroVelocityReference_MaximumKickinVelocity, const double StabilizationDetectionVelocity, const double dt, double q_ref_out[4]);

	private:
        double rate_;
        boost::thread thread_;
        bool shouldExit_;
		ros::NodeHandle nh_;

        ros::Publisher quaternionPub_;
        ros::Subscriber stateEstimateSub_;
        ros::Subscriber referenceSub_;
        ros::Subscriber referenceInertialSub_;
        tf::TransformListener tfListener_;

		kugle_msgs::StateEstimate currentStateEstimate;
        geometry_msgs::Vector3 desiredVelocity_;
        double desiredAngularYawVelocity_;
        ros::Time lastReferenceUpdateTime_;
        bool velocityRefGivenInHeadingFrame_;

		Eigen::Map<const Eigen::Matrix<double, 2, 10, Eigen::RowMajor>> gainMatrix;

		FirstOrderLPF _omega_x_ref_filt;
		FirstOrderLPF _omega_y_ref_filt;

		double roll_ref;
		double pitch_ref;

		bool position_control_enabled;
		double position_reference[2]; // used when 'PositionControlAtZeroVelocityReference' is enabled
		double velocity_error_integral[2];

		/* For debugging purposes */
		double Velocity_Reference_Filtered_Inertial[2];
		double Velocity_Reference_Filtered_Heading[2];

		double VelocityIntegralInitializationTime;
		bool VelocityIntegralAfterPowerup;

		struct {
            double MaxTilt = 3.0; // max tilt that velocity controller can set [degrees]
            double VelocityClamp = 0.5;
            double AccelerationLimit = 1.0;
            double AngularVelocityClamp = 0.5;
            double OmegaLPFtau = 0.3;
            bool IntegralEnabled = false;
            bool PositionControlAtZeroVelocityReference = true;
            double PositionControlAtZeroVelocityReference_MaximumKickinVelocity = 0.1;
            double IntegratorPowerupStabilizeTime = 3.0; // wait 3 seconds in the beginning for integrator to settle (and before allowing manual movement)
            double StabilizationDetectionVelocity = 0.2; // if the robot is pushed with a velocity of more than 0.2 m/s after the initialization time the initialization integrator will be disabled allowing manual movement
        } params_;

        const double gainMatrixCoefficients[2*10] = {
                    1.20495276485036e-14,	-0.999999999999943,	9.57622655412988,	1.07269782092211e-13,	1.81156418200928e-14,	-1.56367340001856,	3.55048271315795,	3.60191361345669e-14,	4.55367707656021,	3.35649664687253e-14,
                    0.999999999999976,	-4.93444326794843e-15,	9.16997798798626e-14,	9.58330393966576,	1.56400044281033,	-1.36239574252401e-14,	2.90922217285368e-14,	3.5561021496512,	2.37666072559232e-14,	4.55403037855776
        };
};


} // end namespace kugle_misc

#endif
