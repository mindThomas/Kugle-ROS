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
 
#ifndef LIBRARY_MPC_H
#define LIBRARY_MPC_H

#include <cmath>

#define deg2rad(x) (M_PI*x/180.f)
#define rad2deg(x) (180.f*x/M_PI)

#include <stdlib.h>
#include <limits>

#include "acado_common.h"
#include "acado_indices.h"

#include "Path.h"
#include "Trajectory.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/math/quaternion.hpp> // see https://www.boost.org/doc/libs/1_66_0/libs/math/doc/html/quaternions.html

/* For visualization/plotting only */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace MPC {

    static float inf = std::numeric_limits<float>::infinity();

class MPC
{
	public:
		static const unsigned int HorizonLength = ACADO_N;		// 'WNmat' needs to be a double matrix of size [ACADO_NY x ACADO_NY]

	private:
		static ACADO_t& ACADO;
		static const unsigned int num_StateVariables = ACADO_NX;	// 'xInit' needs to be a double vector of size [ACADO_NX x 1]
		static const unsigned int num_Inputs = ACADO_NU;		// 'uInit' needs to be a double vector of size [ACADO_NU x 1]		
		static const unsigned int num_Outputs = ACADO_NY;		// 'Wmat' needs to be a double matrix of size [ACADO_NY x ACADO_NY]
		static const unsigned int num_FinalOutputs = ACADO_NYN;		// 'WNmat' needs to be a double matrix of size [ACADO_NYN x ACADO_NYN]
		static const unsigned int num_OnlineData = ACADO_NOD;		// 'odInit' needs to be a double matrix of size [ACADO_N+1 x ACADO_NOD]
										// 'refInit' needs to be a double matrix of size [ACADO_N+1 x ACADO_NY]

#if 0
	/* ACADO Variables overview */
	acadoVariables.x[ACADO_N + 1, ACADO_NX]
	acadoVariables.u[ACADO_N, ACADO_NU]
	#if ACADO_NXA	
	acadoVariables.z[ACADO_N, ACADO_NXA]
	#endif // ACADO_NXA	
	#if ACADO_NOD
	acadoVariables.od[ACADO_N + 1, ACADO_NOD]
	#endif // ACADO_NOD
	acadoVariables.y[ACADO_N, ACADO_NY]	
	#if ACADO_NYN
	acadoVariables.yN[1, ACADO_NYN]
	#endif // ACADO_NYN
	#if ACADO_INITIAL_STATE_FIXED
	acadoVariables.x0[1, ACADO_NX]
	#endif // ACADO_INITIAL_STATE_FIXED

	#if ACADO_WEIGHTING_MATRICES_TYPE == 1
	acadoVariables.W[ACADO_NY, ACADO_NY]
	acadoVariables.WN[ACADO_NYN, ACADO_NYN]
	#elif ACADO_WEIGHTING_MATRICES_TYPE == 2
	acadoVariables.W[ACADO_N * ACADO_NY, ACADO_NY]
	acadoVariables.WN[ACADO_NYN, ACADO_NYN]
	#endif // ACADO_WEIGHTING_MATRICES_TYPE

	#if ACADO_USE_LINEAR_TERMS == 1
	#if ACADO_WEIGHTING_MATRICES_TYPE == 1
	acadoVariables.Wlx[ACADO_NX, 1]
	acadoVariables.Wlu[ACADO_NU, 1]
	#elif ACADO_WEIGHTING_MATRICES_TYPE == 2
	acadoVariables.Wlx[(ACADO_N+1) * ACADO_NX, 1]
	acadoVariables.Wlu[ACADO_N * ACADO_NU, 1]
	#endif // ACADO_WEIGHTING_MATRICES_TYPE
	#endif // ACADO_USE_LINEAR_TERMS

	/* Bounds - only if they are not hardcoded (ACADO_HARDCODED_CONSTRAINT_VALUES == 0) */
	#if ACADO_INITIAL_STATE_FIXED == 1
	acadoVariables.lbValues[ACADO_N * ACADO_NU, 1]
	acadoVariables.ubValues[ACADO_N * ACADO_NU, 1]
	#else
	acadoVariables.lbValues[ACADO_NX + ACADO_N * ACADO_NU, 1]
	acadoVariables.ubValues[ACADO_NX + ACADO_N * ACADO_NU, 1]
	#endif /* ACADO_INITIAL_STATE_FIXED == 0 */

	#if ACADO_USE_ARRIVAL_COST == 1
	acadoVariables.xAC[ACADO_NX, 1]
	acadoVariables.SAC[ACADO_NX, ACADO_NX]
	acadoVariables.WL[ACADO_NX, ACADO_NX]
	#endif
#endif

    public:
        typedef struct state_t
        {
            boost::math::quaternion<double> quaternion;
            Eigen::Vector2d position;
            Eigen::Vector2d velocity;
            double pathVelocity;
        } state_t;

	    typedef enum orientation_selection_t : uint8_t
        {
	        INERTIAL_FRAME = 0,
	        HEADING_FRAME = 1,
	        VELOCITY_FRAME = 2
        } orientation_selection_t;

	    typedef enum status_t
        {
            SUCCESS = 0,
            ITERATION_LIMIT_REACHED = 1,
            INFEASIBLE = -2,
            UNBOUNDED = -3
        } status_t;

	public:
		MPC();
		~MPC();

		void Reset();
		void Step();

        void setPath(Path& path, const Eigen::Vector2d& origin, const Eigen::Vector2d& currentPosition);
		void setVelocityBounds(double min_velocity, double max_velocity);
		void setDesiredVelocity(double velocity);

        double getWindowAngularVelocityX(void);
        double getWindowAngularVelocityY(void);
        double getWindowAngularVelocityZ(void);
        Eigen::Vector2d getInertialAngularVelocity(void);

		void setCurrentState(const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q);
        void setControlLimits(double maxAngularVelocity, double maxAngle);
        void setWeights(const Eigen::MatrixXd& W, const Eigen::MatrixXd& WN);

        void setTrajectory(Trajectory& trajectory, const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q);
        void ExtractWindowTrajectory(Trajectory& trajectory, Trajectory& extractedWindowTrajectory, const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q, double ExtractionDistance = 0, orientation_selection_t OrientationSelection = INERTIAL_FRAME);

        void PlotPredictedTrajectory(cv::Mat& image);
        void PlotRobot(cv::Mat& image, cv::Scalar color, bool drawXup, double x_min, double y_min, double x_max, double y_max);
        Trajectory getPredictedTrajectory(void);
        state_t getHorizonState(unsigned int horizonIndex = 1);

        Trajectory getCurrentTrajectory(void);
        Path getCurrentPath(void);
		double getClosestPointOnPath(void);

		double extractHeading(const boost::math::quaternion<double>& q);

		double getSampleTime(void) const;

    private:
        void Initialize();
        void initStates(void);
        void setReferences(void);
        void shiftStates(void);

	private:
        Trajectory currentTrajectory_;

        double desiredVelocity_;
        double minVelocity_;
        double maxVelocity_;
        double maxAngle_;
        double maxAngularVelocity_;

        Eigen::Vector3d controlBodyAngularVelocity_; // this is the computed output of the MPC

        double WindowWidth_;
        double WindowHeight_;
        Eigen::Vector2d WindowOffset_;
        double WindowOrientation_; // heading/yaw of the path window (in inertial frame)
        Path windowPath_;
        double windowPathLength_;
        Eigen::Vector2d windowPathOrigin_; // origin of path in inertial frame - hence what (0,0) of path corresponds to in inertial coordinates
        double closestPositionOnCurrentPath_; // this corresponds to the initialization value of the 's' parameter - however this is actually input as OnlineData instead
		unsigned int pathApproximationOrder_;
        orientation_selection_t WindowOrientationSelection_;

		/* Both position and velocity is defined in inertial frame */
        Eigen::Vector2d position_;
        Eigen::Vector2d velocity_;
		boost::math::quaternion<double> quaternion_;

		/* Solver status and outputs */
		status_t SolverStatus_;
		double SolverKKT_;
		double SolverCostValue_;
		int SolverIterations_;
    };
	
}
	
#endif
