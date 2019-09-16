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
 
#ifndef LIBRARY_MPC_H
#define LIBRARY_MPC_H

#include <cmath>

#define deg2rad(x) (M_PI*x/180.f)
#define rad2deg(x) (180.f*x/M_PI)

#include <stdlib.h>
#include <limits>
#include <vector>

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

class Obstacle
{
    public:
        double x;
        double y;
        double radius;

        Obstacle() : x(999), y(999), radius(0.01) {}; // neglible obstacle
        Obstacle(double x_, double y_, double radius_) : x(x_), y(y_), radius(radius_) {};

        double proximity(const Eigen::Vector2d& position) const
        {
            return sqrt( (x-position[0])*(x-position[0]) + (y-position[1])*(y-position[1]) ) - radius;
        }

        /* Operator used for sorting */
        static bool proximitySorting(const Eigen::Vector2d& position, const Obstacle& obstacle1, const Obstacle& obstacle2)
        {
            return (obstacle1.proximity(position) < obstacle2.proximity(position));
        }
};

class MPC
{
	public:
		static const unsigned int HorizonLength = ACADO_N;		// 'WNmat' needs to be a double matrix of size [ACADO_NY x ACADO_NY]
        static constexpr double RobotRadius = 0.1; // radius of robot in meters

		// Weight definitions
        static const double WPathFollow;
        static const double WLateral;
        static const double WVelocity;
        static const double WProgress;
        static const double WSmoothness;
        static const double WOmega;
        static const double WAngle;
        static const double WObstacles;
        static const double ProximityOffset;
        static const double ProximityScale;

        static const double Wdiag[ACADO_NY]; // Horizon weight matrix (cost)
        static const double WNdiag[ACADO_NYN]; // Final state weight matrix (cost)

	private:
		static ACADO_t& ACADO;
		static constexpr unsigned int num_StateVariables = ACADO_NX;	// 'xInit' needs to be a double vector of size [ACADO_NX x 1]
		static constexpr unsigned int num_Inputs = ACADO_NU;			// 'uInit' needs to be a double vector of size [ACADO_NU x 1]
		static constexpr unsigned int num_Outputs = ACADO_NY;			// 'Wmat' needs to be a double matrix of size [ACADO_NY x ACADO_NY]
		static constexpr unsigned int num_FinalOutputs = ACADO_NYN;		// 'WNmat' needs to be a double matrix of size [ACADO_NYN x ACADO_NYN]
		static constexpr unsigned int num_OnlineData = ACADO_NOD;		// 'odInit' needs to be a double matrix of size [ACADO_N+1 x ACADO_NOD]
                                                                        // 'refInit' needs to be a double matrix of size [ACADO_N+1 x ACADO_NY]

    public:
        typedef struct state_t
        {
            boost::math::quaternion<double> quaternion;
            Eigen::Vector2d position;
            Eigen::Vector2d velocity;
            double pathDistance;
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
            UNBOUNDED = -3,
			OTHER = -1
        } status_t;

	public:
		MPC();
		~MPC();

		void Reset();
		void Step();

        void setPath(Path& path, const Eigen::Vector2d& origin, const Eigen::Vector2d& currentPosition);
        void setXYreferencePosition(double x, double y);
        void setObstacles(std::vector<Obstacle>& obstacles);
		void setVelocityBounds(double min_velocity, double max_velocity);
        void setObstacleProximityParameters(double proximityOffset, double proximityScale);
		void setDesiredVelocity(double velocity);

        double getWindowAngularVelocityX(void);
        double getWindowAngularVelocityY(void);
        double getWindowAngularVelocityZ(void);
        Eigen::Vector2d getInertialAngularVelocity(void);
        Eigen::Vector3d getBodyAngularVelocity(double yawAngularVelocity);
        std::vector<std::pair<double,double>> getInertialAngularVelocityHorizon(void);

		void setCurrentState(const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q);
        void setControlLimits(double maxAngle, double maxAngularVelocity, double maxAngularAcceleration);
        void setWeights(const Eigen::MatrixXd& W, const Eigen::MatrixXd& WN);

        void setTrajectory(Trajectory& trajectory, const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q);
        void ExtractWindowTrajectory(Trajectory& trajectory, Trajectory& extractedWindowTrajectory, const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q, double ExtractionDistance = 0, bool DoWindowFilteringBeforeExtractionDistance = false, orientation_selection_t OrientationSelection = INERTIAL_FRAME);

		void PlotPredictedTrajectory(cv::Mat& image, double x_min, double y_min, double x_max, double y_max);
        void PlotPredictedTrajectoryInWindow(cv::Mat& image, double x_min, double y_min, double x_max, double y_max);
        void PlotRobot(cv::Mat& image, cv::Scalar color, bool drawXup, double x_min, double y_min, double x_max, double y_max);
        void PlotRobotInWindow(cv::Mat& image, cv::Scalar color, bool drawXup, double x_min, double y_min, double x_max, double y_max);
        void PlotObstacles(cv::Mat& image, cv::Scalar obstacleColor, cv::Scalar consideredColor, bool drawXup, double x_min, double y_min, double x_max, double y_max);
        void PlotObstaclesInWindow(cv::Mat& image, cv::Scalar color, bool drawXup, double x_min, double y_min, double x_max, double y_max);
        Trajectory getPredictedTrajectory(void);
        state_t getHorizonState(unsigned int horizonIndex = 1);
        double getHorizonPathLength() const;

        Trajectory getCurrentTrajectory(void);
        Path getCurrentPath(void);
		double getClosestPointOnPath(void);

		double extractHeading(const boost::math::quaternion<double>& q);

		double getSampleTime() const;
		double getSolveTime() const;
        double getCurrentPathPosition() const;
        status_t getStatus() const;
	double SolverCostValue_;
    private:
        void resetACADO();
        void resetStates(void);
        void setReferences(void);
        void shiftStates(void);

	private:
        Trajectory currentTrajectory_;
        std::vector<Obstacle> currentObstacles_;
        std::vector<Obstacle> consideredObstacles_;

        double desiredVelocity_;
        double minVelocity_;
        double maxVelocity_;
        double maxAngle_;
        double maxAngularVelocity_;
        double maxAngularAcceleration_;
        double proximityOffset_;
        double proximityScale_;

        Eigen::Vector2d windowAngularVelocity_ref_; // this is the computed output of the MPC

        double WindowWidth_;
        double WindowHeight_;
        Eigen::Vector2d WindowOffset_;
        double WindowOrientation_; // heading/yaw of the path window (in inertial frame)
        Path windowPath_;
        double windowPathLength_;
        Eigen::Vector2d windowPathOrigin_; // origin of path in inertial frame - hence what (0,0) of path corresponds to in inertial coordinates
        double closestPositionOnCurrentPathToOrigin_; // this corresponds to the initialization value of the 's' parameter - however this is actually input as OnlineData instead
		unsigned int pathApproximationOrder_;
        orientation_selection_t WindowOrientationSelection_;

		/* Both position and velocity is defined in inertial frame */
        Eigen::Vector2d position_;
        Eigen::Vector2d velocity_;
		boost::math::quaternion<double> quaternion_;

		/* Solver status and outputs */
		status_t SolverStatus_;
		double SolverKKT_;
		int SolverIterations_;
		double SolveTime_;
    };
	
}
	
#endif
