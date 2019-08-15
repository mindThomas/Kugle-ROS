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
 
#include "MPC.h"

#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "acado_indices.h"


/** Instance of ACADO data structure */
extern "C" {
    ACADOvariables acadoVariables;
    ACADOworkspace acadoWorkspace;
}

namespace MPC {

    ACADO_t& MPC::ACADO = *((ACADO_t *)&acadoVariables);

    const double MPC::WPathFollow = 9999;  // weight on longitudinal tracking error - should be high to achieve accurate path following
    const double MPC::WLateral = 25;  // weight on lateral tracking error
    const double MPC::WVelocity = 80;  // weight on error between desired velocity and actual longitudinal velocity
    const double MPC::WProgress = 0;  // punishment of being far away from the end of the fitted path
    const double MPC::WSmoothness = 100;  // punishment of changes in angular velocity reference (hence weight on d_omega_ref)
    const double MPC::WOmega = 20;  // weight on angular velocity reference (omega_ref)
    const double MPC::WAngle = 10;  // weight on angle reference
    const double MPC::WObstacles = 20;  // weight on the exponential obstacle proximity, defined by an offset and scale value
    const double MPC::ProximityOffset = 0.3;  // exponential obstacle offset
    const double MPC::ProximityScale = 10;  // exponential obstacle scale

    // Horizon weight matrix (cost)
    const double MPC::Wdiag[ACADO_NY] = {MPC::WPathFollow, MPC::WLateral,  MPC::WAngle,MPC::WAngle,  MPC::WOmega,MPC::WOmega,  MPC::WVelocity,MPC::WProgress,  MPC::WSmoothness,MPC::WSmoothness,  MPC::WObstacles,  99999.0,9999.0,9999.0}; // { lag_error; lateral_deviation;  q2;q3;  omega_ref_x;omega_ref_y;  velocity_error; away_from_end_error;   domega_ref_x;domega_ref_y;   obstacle_proximity;   velocity_slack;angle_slack;proximity_slack }

    // Final state weight matrix (cost)
    const double MPC::WNdiag[ACADO_NYN] = {MPC::WPathFollow, 10*MPC::WLateral,  MPC::WAngle,MPC::WAngle,  MPC::WOmega,MPC::WOmega,  MPC::WVelocity,MPC::WProgress,  MPC::WObstacles}; // { lag_error; lateral_deviation;  q2;q3;  omega_ref_x;omega_ref_y;  velocity_error; away_from_end_error;   obstacle_proximity }


    MPC::MPC()
    {
        if (sizeof(ACADO_t) != sizeof(ACADOvariables)) {
            std::cout << "[ERROR] MPC ACADO variable typedefinition does not match generated!" << std::endl;
            throw;
        }

        // Initialize ACADO
        resetACADO();
        resetStates();

        // Set window and path parameters
        WindowWidth_ = 5.0;
        WindowHeight_ = 5.0;
        WindowOffset_ << 0, 0;
        WindowOrientation_ = 0;
        pathApproximationOrder_ = 8;
        WindowOrientationSelection_ = INERTIAL_FRAME;

        // Set weights
        Eigen::Matrix<double, ACADO_NY, 1> Wdiag_(Wdiag);
        Eigen::Matrix<double, ACADO_NYN, 1> WNdiag_(WNdiag);
        setWeights(Wdiag_.asDiagonal().toDenseMatrix(), WNdiag_.asDiagonal().toDenseMatrix());

        // Set internal to default values
        setVelocityBounds(0.0, 10.0);
        setDesiredVelocity(1.0);
        setControlLimits(deg2rad(10), deg2rad(15), deg2rad(30));
        setObstacleProximityParameters(ProximityOffset, ProximityScale);
        setReferences();

        // Set an empty reference path
        Path emptyPath;
        setPath(emptyPath, Eigen::Vector2d(0,0), Eigen::Vector2d(0,0));

        // Set no obstacles
        std::vector<Obstacle> noObstacles;
        setObstacles(noObstacles);
    }


    MPC::~MPC()
    {

    }

    void MPC::Reset(void)
    {
        resetACADO();
        resetStates();

        // Set internal parameters
        setVelocityBounds(minVelocity_, maxVelocity_);
        setDesiredVelocity(desiredVelocity_);
        setControlLimits(maxAngle_, maxAngularVelocity_, maxAngularAcceleration_);
        setObstacleProximityParameters(proximityOffset_, proximityScale_);
        setReferences();

        // Set current path and states
        //setPath(windowPath_, windowPathOrigin_, windowPathOrigin_);
        //setCurrentState(position_, velocity_, quaternion_);

        // Set an empty reference path
        Path emptyPath;
        setPath(emptyPath, Eigen::Vector2d(0,0), Eigen::Vector2d(0,0));

        // Set no obstacles
        std::vector<Obstacle> noObstacles;
        setObstacles(noObstacles);
    }

    void MPC::setTrajectory(Trajectory& trajectory, const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q)
    {
        double PreviousHorizonLength = getHorizonPathLength();
        double ExtractionLength = 1.3*PreviousHorizonLength; // 1.3*HorizonLength*ACADO_TS*desiredVelocity_
        if (ExtractionLength < 3.0)
            ExtractionLength = 3.0;

        ExtractWindowTrajectory(trajectory, currentTrajectory_, position, velocity, q, ExtractionLength, false, WindowOrientationSelection_); // change the last parameter/flag to:  0=window in inertial frame,  1=window in heading frame,  2=window in velocity direction frame
        //currentTrajectory_.print(); currentTrajectory_.plot(true); cv::waitKey(0);

        bool StopAtEnd = currentTrajectory_.includesGoal();
        if (StopAtEnd)
            std::cout << "MPC trajectory includes goal" << std::endl;

        Path windowPath(currentTrajectory_, pathApproximationOrder_, StopAtEnd, true, true);
        //windowPath.print(); windowPath.plot(true); cv::waitKey(0);
        setPath(windowPath, position, position);
        //setCurrentState(position, velocity, q);
    }

    Trajectory MPC::getCurrentTrajectory(void)
    {
        return currentTrajectory_;
    }

    Path MPC::getCurrentPath(void)
    {
        return windowPath_;
    }

    double MPC::getClosestPointOnPath(void)
    {
        return closestPositionOnCurrentPathToOrigin_;
    }

    void MPC::ExtractWindowTrajectory(Trajectory& trajectory, Trajectory& extractedWindowTrajectory, const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q, double ExtractionDistance, bool DoWindowFilteringBeforeExtractionDistance, orientation_selection_t OrientationSelection)
    {
        // Extract a window trajectory given a full trajectory, the current location of the robot and an approximate desired length of the extracted trajectory
        if (OrientationSelection == INERTIAL_FRAME) // inertial frame
            WindowOrientation_ = 0;
        else if (OrientationSelection == HEADING_FRAME) // robot yaw (heading)
            WindowOrientation_ = extractHeading(q);
        else if (OrientationSelection == VELOCITY_FRAME) // velocity direction
            WindowOrientation_ = atan2(velocity[1], velocity[0]);
        else
            WindowOrientation_ = 0;

        /* Extract window trajectory in robot frame - hence a robot-centric trajectory */
        Trajectory RobotCentricTrajectory;
        if (DoWindowFilteringBeforeExtractionDistance) {
            // Extract a robot-centric window of the input trajectory and filter away any points outside this window
            trajectory.WindowExtract(RobotCentricTrajectory, position, WindowOrientation_, WindowWidth_, WindowHeight_, WindowOffset_);
        } else {
            // Make a robot-centric trajectory but include all trajectory points - leave the discarding of points to the later "ApproxCurveLengthExtract"
            RobotCentricTrajectory = trajectory;
            RobotCentricTrajectory.move(-position);
            RobotCentricTrajectory.rotate(-WindowOrientation_);
        }

        Trajectory WindowTrajectory;
        RobotCentricTrajectory.FixSequenceOrder(WindowTrajectory, 0, trajectory.GetLastSequenceID()); // an advanced type of sorting

        // Find continuous sequence of points closest to robot
        Trajectory SequenceTrajectory;
        WindowTrajectory.ExtractContinuousClosestSequence(SequenceTrajectory);

        // Extract trajectory matching desired approximate length/distance
        extractedWindowTrajectory.clear();
        if (ExtractionDistance > 0)
            SequenceTrajectory.ApproxCurveLengthExtract(extractedWindowTrajectory, ExtractionDistance);
        else
            extractedWindowTrajectory = SequenceTrajectory;
    }

    void MPC::setPath(Path& path, const Eigen::Vector2d& origin, const Eigen::Vector2d& currentPosition)
    {
        windowPath_ = path;
        windowPathOrigin_ = origin;
        windowPathLength_ = path.length();

        if (path.order() > 9) {
            std::cout << "ERROR: Path order not supported by MPC (too high)" << std::endl;
            return;
        }

        // Based on current position, compute the closest position (s-value) on this new path
        closestPositionOnCurrentPathToOrigin_ = windowPath_.FindClosestPoint(currentPosition - windowPathOrigin_);

        std::cout << "MPC path updated:" << std::endl;
        std::cout << "   length = " << windowPathLength_ << std::endl;
        std::cout << "   closest point = " << closestPositionOnCurrentPathToOrigin_ << std::endl;
        windowPath_.print();

        // Set path for full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].cx0 = windowPath_.getXcoefficient(0);
            ACADO.od[i].cx1 = windowPath_.getXcoefficient(1);
            ACADO.od[i].cx2 = windowPath_.getXcoefficient(2);
            ACADO.od[i].cx3 = windowPath_.getXcoefficient(3);
            ACADO.od[i].cx4 = windowPath_.getXcoefficient(4);
            ACADO.od[i].cx5 = windowPath_.getXcoefficient(5);
            ACADO.od[i].cx6 = windowPath_.getXcoefficient(6);
            ACADO.od[i].cx7 = windowPath_.getXcoefficient(7);
            ACADO.od[i].cx8 = windowPath_.getXcoefficient(8);
            ACADO.od[i].cx9 = windowPath_.getXcoefficient(9);

            ACADO.od[i].cy0 = windowPath_.getYcoefficient(0);
            ACADO.od[i].cy1 = windowPath_.getYcoefficient(1);
            ACADO.od[i].cy2 = windowPath_.getYcoefficient(2);
            ACADO.od[i].cy3 = windowPath_.getYcoefficient(3);
            ACADO.od[i].cy4 = windowPath_.getYcoefficient(4);
            ACADO.od[i].cy5 = windowPath_.getYcoefficient(5);
            ACADO.od[i].cy6 = windowPath_.getYcoefficient(6);
            ACADO.od[i].cy7 = windowPath_.getYcoefficient(7);
            ACADO.od[i].cy8 = windowPath_.getYcoefficient(8);
            ACADO.od[i].cy9 = windowPath_.getYcoefficient(9);

            ACADO.od[i].trajectoryLength = windowPathLength_;
            ACADO.od[i].trajectoryStart = closestPositionOnCurrentPathToOrigin_;
            ACADO.od[i].desiredVelocity = desiredVelocity_;
        }

        // Reset horizon predictions for the movement on the path (s-value)
        //real_t s_reset = ACADO.x[0].s;
        real_t s_reset = ACADO.x[1].s; // resetting such that the value 1 step in the horizon becomes 0, since the states has not been shifted yet
        real_t x_reset = ACADO.x[1].x;
        real_t y_reset = ACADO.x[1].y;
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.x[i].s = ACADO.x[i].s - s_reset;
            ACADO.x[i].x = ACADO.x[i].x - x_reset;
            ACADO.x[i].y = ACADO.x[i].y - y_reset;
        }
        // This results in ACADO.x[0].s = 0 and the path value predictions are then increasing from there

        if ((ACADO.x[ACADO_N].s + closestPositionOnCurrentPathToOrigin_) > windowPathLength_) {
            // Current MPC horizon initialization is running outside of new path. Reset to evenly spaced s-values along path
            double s_value = 0;
            double s_spacing = (windowPathLength_ - closestPositionOnCurrentPathToOrigin_) / (ACADO_N-1);
            for (unsigned int i = 1; i < (ACADO_N+1); i++) {
                ACADO.x[i].s = s_value;
                s_value += s_spacing;
            }
        }
    }

    void MPC::setXYreferencePosition(double x, double y)
    {
        // OBS. Sending discrete position references to the path following controller turns it into a "trajectory tracking" controller
        Path emptyPath;
        windowPath_ = emptyPath;
        windowPathOrigin_ = Eigen::Vector2d(x,y);
        windowPathLength_ = 99; // arbitrary length, since it is just a constant point

        std::cout << "MPC static position enabled:" << std::endl;
        std::cout << "   x = " << x << std::endl;
        std::cout << "   y = " << y << std::endl;

        // Set path for full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].cx0 = 0;
            ACADO.od[i].cx1 = 0.0001; // OBS. Even though we just want to hold the position, we still need to set a valid path due to the way the MPC tries to take the derivative
            ACADO.od[i].cx2 = 0;
            ACADO.od[i].cx3 = 0;
            ACADO.od[i].cx4 = 0;
            ACADO.od[i].cx5 = 0;
            ACADO.od[i].cx6 = 0;
            ACADO.od[i].cx7 = 0;
            ACADO.od[i].cx8 = 0;
            ACADO.od[i].cx9 = 0;

            ACADO.od[i].cy0 = 0;
            ACADO.od[i].cy1 = 0;
            ACADO.od[i].cy2 = 0;
            ACADO.od[i].cy3 = 0;
            ACADO.od[i].cy4 = 0;
            ACADO.od[i].cy5 = 0;
            ACADO.od[i].cy6 = 0;
            ACADO.od[i].cy7 = 0;
            ACADO.od[i].cy8 = 0;
            ACADO.od[i].cy9 = 0;

            ACADO.od[i].trajectoryLength = windowPathLength_;
            ACADO.od[i].trajectoryStart = 0;
            ACADO.od[i].desiredVelocity = 0;
        }

        // Reset horizon predictions for the movement on the path (s-value)
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.x[i].s = 0;
            ACADO.x[i].x = 0;
            ACADO.x[i].y = 0;
        }
    }

    void MPC::setObstacles(std::vector<Obstacle>& obstacles)
    {
        currentObstacles_ = obstacles;

        // Sort the obstacles to extract and use the nearest obstacles
        Eigen::Vector2d horizonEndPosition = getHorizonState(ACADO_N).position;
        std::sort(currentObstacles_.begin(), currentObstacles_.end(), std::bind(&Obstacle::proximitySorting, horizonEndPosition, std::placeholders::_1, std::placeholders::_2));

        consideredObstacles_.clear();

        // Set obstacles throughout full horizon
        Eigen::Vector2d obstacle, obstacle_in_window;
        if (currentObstacles_.size() >= 1) {
            consideredObstacles_.push_back(currentObstacles_.at(0));
            obstacle[0] = currentObstacles_.at(0).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(0).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            obstacle_in_window = rot * obstacle;
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs1_x = obstacle_in_window[0];
                ACADO.od[i].obs1_y = obstacle_in_window[1];
                ACADO.od[i].obs1_r = currentObstacles_.at(0).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs1_x = 99;
                ACADO.od[i].obs1_y = 99;
                ACADO.od[i].obs1_r = 0.001;
            }
        }
        if (currentObstacles_.size() >= 2) {
            consideredObstacles_.push_back(currentObstacles_.at(1));
            obstacle[0] = currentObstacles_.at(1).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(1).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            obstacle_in_window = rot * obstacle;
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs2_x = obstacle_in_window[0];
                ACADO.od[i].obs2_y = obstacle_in_window[1];
                ACADO.od[i].obs2_r = currentObstacles_.at(1).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs2_x = 99;
                ACADO.od[i].obs2_y = 99;
                ACADO.od[i].obs2_r = 0.001;
            }
        }
        if (obstacles.size() >= 3) {
            consideredObstacles_.push_back(currentObstacles_.at(2));
            obstacle[0] = currentObstacles_.at(2).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(2).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            obstacle_in_window = rot * obstacle;
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs3_x = obstacle_in_window[0];
                ACADO.od[i].obs3_y = obstacle_in_window[1];
                ACADO.od[i].obs3_r = currentObstacles_.at(2).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs3_x = 99;
                ACADO.od[i].obs3_y = 99;
                ACADO.od[i].obs3_r = 0.001;
            }
        }
        if (currentObstacles_.size() >= 4) {
            consideredObstacles_.push_back(currentObstacles_.at(3));
            obstacle[0] = currentObstacles_.at(3).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(3).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            obstacle_in_window = rot * obstacle;
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs4_x = obstacle_in_window[0];
                ACADO.od[i].obs4_y = obstacle_in_window[1];
                ACADO.od[i].obs4_r = currentObstacles_.at(3).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs4_x = 99;
                ACADO.od[i].obs4_y = 99;
                ACADO.od[i].obs4_r = 0.001;
            }
        }
        if (currentObstacles_.size() >= 5) {
            consideredObstacles_.push_back(currentObstacles_.at(4));
            obstacle[0] = currentObstacles_.at(4).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(4).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            obstacle_in_window = rot * obstacle;
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs5_x = obstacle_in_window[0];
                ACADO.od[i].obs5_y = obstacle_in_window[1];
                ACADO.od[i].obs5_r = currentObstacles_.at(4).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs5_x = 99;
                ACADO.od[i].obs5_y = 99;
                ACADO.od[i].obs5_r = 0.001;
            }
        }
    }

    void MPC::setVelocityBounds(double min_velocity, double max_velocity)
    {
        minVelocity_ = min_velocity;
        maxVelocity_ = max_velocity;

        // Set bounds for the full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            //ACADO.od[i].minVelocity = minVelocity_; // minimum velocity ignored for now
            ACADO.od[i].maxVelocity = maxVelocity_;
        }
    }

    void MPC::setDesiredVelocity(double velocity)
    {
        desiredVelocity_ = velocity;

        // Set the desired velocity for the full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].desiredVelocity = desiredVelocity_;
        }
    }

    void MPC::setControlLimits(double maxAngle, double maxAngularVelocity, double maxAngularAcceleration)
    {
        maxAngularAcceleration_ = maxAngularAcceleration;
        maxAngularVelocity_ = maxAngularVelocity;
        maxAngle_ = maxAngle;

        // Set bounds for the full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].maxAngle = maxAngle_;
            ACADO.od[i].maxOmegaRef = maxAngularVelocity_;
            ACADO.od[i].maxdOmegaRef = maxAngularAcceleration_;
        }
    }

    void MPC::setObstacleProximityParameters(double proximityOffset, double proximityScale)
    {
        proximityOffset_ = proximityOffset;
        proximityScale_ = proximityScale;

        // Set bounds for the full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].proximityOffset = proximityOffset_;
            ACADO.od[i].proximityScale = proximityScale_;
        }
    }

    void MPC::setWeights(const Eigen::MatrixXd& W, const Eigen::MatrixXd& WN)
    {
        if (W.rows() != ACADO_NY || W.cols() != ACADO_NY) {
            std::cout << "[ERROR] Incorrect W weight matrix for MPC" << std::endl;
            throw;
            return; // Error, weights of incorrect dimension
        }
        if (WN.rows() != ACADO_NYN || WN.cols() != ACADO_NYN) {
            std::cout << "[ERROR] Incorrect WN weight matrix for MPC" << std::endl;
            throw;
            return; // Error, weights of incorrect dimension
        }

        // Weights are stored in Row major format
        for (unsigned int i = 0; i < ACADO_NY; i++) {
            for (unsigned int j = 0; j < ACADO_NY; j++) {
                ACADO.W[ACADO_NY*i + j] = W(i,j);
            }
        }

        for (unsigned int i = 0; i < ACADO_NYN; i++) {
            for (unsigned int j = 0; j < ACADO_NYN; j++) {
                ACADO.WN[ACADO_NYN*i + j] = WN(i,j);
            }
        }
    }

    void MPC::setReferences(void)
    {
        for (unsigned int i = 0; i < ACADO_N; i++) {
            ACADO.y[i].lag_error = 0;
            ACADO.y[i].lateral_deviation = 0;
            ACADO.y[i].q2 = 0;
            ACADO.y[i].q3 = 0;
            ACADO.y[i].omega_ref_x = 0;
            ACADO.y[i].omega_ref_y = 0;
            ACADO.y[i].velocity_error = 0;
            ACADO.y[i].away_from_end_error = 0;
            ACADO.y[i].domega_ref_x = 0;
            ACADO.y[i].domega_ref_y = 0;
            ACADO.y[i].obstacle_proximity = 0;
            ACADO.y[i].velocity_slack = 0;
            ACADO.y[i].angle_slack = 0;
            ACADO.y[i].proximity_slack = 0;
        }
        ACADO.yN.lag_error = 0;
        ACADO.yN.lateral_deviation = 0;
        ACADO.yN.q2 = 0;
        ACADO.yN.q3 = 0;
        ACADO.yN.omega_ref_x = 0;
        ACADO.yN.omega_ref_y = 0;
        ACADO.yN.velocity_error = 0;
        ACADO.yN.away_from_end_error = 0;
        ACADO.yN.obstacle_proximity = 0;
    }

    double MPC::extractHeading(const boost::math::quaternion<double>& q)
    {
        boost::math::quaternion<double> p = q * boost::math::quaternion<double>(0,1,0,0) * conj(q); // Phi(q)*Gamma(q)'*[0,1,0,0]'
        return atan2(p.R_component_3(), p.R_component_2()); //  atan2(p(3),p(2));
    }

    /* Position and velocity is given in inertial frame */
    void MPC::setCurrentState(const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q)
    {
        position_ = position;
        velocity_ = velocity;
        quaternion_ = q;

        /* Extract current robot heading/yaw from attitude quaternion */
        double RobotYaw = extractHeading(q);
        boost::math::quaternion<double> q_heading(cos(RobotYaw/2),0,0,sin(RobotYaw/2)); // w,x,y,z
        boost::math::quaternion<double> q_tilt = conj(q_heading) * q; // tilt defined in heading frame
        if (q_tilt.R_component_1() < 0)
            q_tilt = -q_tilt; // invert q_tilt to take shortest rotation

        /* Compute rotation matrix for path window orientation - rotates a vector from inertial frame to window frame*/
        Eigen::Matrix2d R_window = Eigen::Rotation2Dd(WindowOrientation_).toRotationMatrix().transpose();

        /* Compute velocity in window frame */
        Eigen::Vector2d WindowVelocity = R_window * velocity;

        /* Compute approximated tilt quaternion in the window frame, for use in the linearly approximated MPC model
         * - where acceleration is defined in the heading frame independently by the q2 (x) and q3 (y) components of the tilt quaternion*/
        boost::math::quaternion<double> q_tilt_inertial = q * conj(q_heading); // tilt defined in inertial frame
        Eigen::Vector2d q_tiltWindow_q2q3 = R_window * Eigen::Vector2d(q_tilt_inertial.R_component_2(), q_tilt_inertial.R_component_3());

        /* Compute window position */
        Eigen::Vector2d WindowPosition = R_window * (position - windowPathOrigin_);

        /* Set ACADO initialization states */
        ACADO.x0.q2 = q_tiltWindow_q2q3[0]; // q2
        ACADO.x0.q3 = q_tiltWindow_q2q3[1]; // q3
        ACADO.x0.x = WindowPosition[0];
        ACADO.x0.y = WindowPosition[1];
        ACADO.x0.dx = WindowVelocity[0];
        ACADO.x0.dy = WindowVelocity[1];

        /* Should not modify path parameter, since this is kept static by the shifting strategy, as closestPositionOnCurrentPathToOrigin defines the starting point on the current path */
        ACADO.x0.s = ACADO.x[1].s; // set the initial guess to the shifted horizon value of the path parameter, s
        //ACADO.x[0].s = ACADO.x0.s = 0;
        //ACADO.x[0].ds = ACADO.x0.ds = 0;

        /* Set current angular velocity - this could also be set using the quaternion derivative. For now we just use the previously set value */
        ACADO.x0.omega_ref_x = windowAngularVelocity_ref_[0];
        ACADO.x0.omega_ref_y = windowAngularVelocity_ref_[1];
    }

    double MPC::getCurrentPathPosition() const
    {
        return ACADO.x0.s;
    }

    void MPC::resetStates(void)
    {
        ACADO.x0.q2 = 0;
        ACADO.x0.q3 = 0;
        ACADO.x0.x = 0;
        ACADO.x0.y = 0;
        ACADO.x0.dx = 0.001; // dx, dy and ds has to be initialized with a very small value apparently - otherwise the solver is infeasible from the get-go
        ACADO.x0.dy = 0.001;
        ACADO.x0.s = 0;
        ACADO.x0.ds = 0.001;
        ACADO.x0.omega_ref_x = 0;
        ACADO.x0.omega_ref_y = 0;

        /* Initialize horizon states to same values */
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.x[i].q2 = ACADO.x0.q2;
            ACADO.x[i].q3 = ACADO.x0.q3;
            ACADO.x[i].x = ACADO.x0.x;
            ACADO.x[i].y = ACADO.x0.y;
            ACADO.x[i].dx = ACADO.x0.dx;
            ACADO.x[i].dy = ACADO.x0.dy;
            ACADO.x[i].s = ACADO.x0.s;
            ACADO.x[i].ds = ACADO.x0.ds;
            ACADO.x[i].omega_ref_x = ACADO.x0.omega_ref_x;
            ACADO.x[i].omega_ref_y = ACADO.x0.omega_ref_y;
        }

        for (unsigned int i = 0; i < ACADO_N; i++) {
            ACADO.u[i].domega_ref_x = 0;
            ACADO.u[i].domega_ref_y = 0;
            ACADO.u[i].dds = 0;
            ACADO.u[i].velocity_slack = 0;
            ACADO.u[i].angle_slack = 0;
            ACADO.u[i].proximity_slack = 0;
        }

        windowAngularVelocity_ref_[0] = 0;
        windowAngularVelocity_ref_[1] = 0;
    }

    void MPC::shiftStates(void)
    {
        // Shift the states and controls by discarding the first step in the horizon (being the current step) and shifting all later steps one timestep forward.
        // Set the new final state (end state of horizon) to the same value as the shifted final state
        // OBS. This should be done as one of the first things before running the solver - by doing it here instead of after running the solver, the predicted horizon will be kept in the the ACADO variables until next step
        for (unsigned int i = 0; i < ACADO_N; i++) {
            ACADO.x[i].q2 = ACADO.x[i+1].q2;
            ACADO.x[i].q3 = ACADO.x[i+1].q3;
            ACADO.x[i].x = ACADO.x[i+1].x;
            ACADO.x[i].y = ACADO.x[i+1].y;
            ACADO.x[i].dx = ACADO.x[i+1].dx;
            ACADO.x[i].dy = ACADO.x[i+1].dy;
            ACADO.x[i].s = ACADO.x[i+1].s;
            ACADO.x[i].ds = ACADO.x[i+1].ds;
            ACADO.x[i].omega_ref_x = ACADO.x[i+1].omega_ref_x;
            ACADO.x[i].omega_ref_y = ACADO.x[i+1].omega_ref_y;
        }

        for (unsigned int i = 0; i < (ACADO_N-1); i++) {
            ACADO.u[i].domega_ref_x = ACADO.u[i+1].domega_ref_x;
            ACADO.u[i].domega_ref_y = ACADO.u[i+1].domega_ref_y;
            ACADO.u[i].dds = ACADO.u[i+1].dds;
            ACADO.u[i].velocity_slack = ACADO.u[i+1].velocity_slack;
            ACADO.u[i].angle_slack = ACADO.u[i+1].angle_slack;
            ACADO.u[i].proximity_slack = ACADO.u[i+1].proximity_slack;
        }

        ACADO.x[0].q2 = ACADO.x0.q2;
        ACADO.x[0].q3 = ACADO.x0.q3;
        ACADO.x[0].x = ACADO.x0.x;
        ACADO.x[0].y = ACADO.x0.y;
        ACADO.x[0].dx = ACADO.x0.dx;
        ACADO.x[0].dy = ACADO.x0.dy;
        ACADO.x[0].s = ACADO.x0.s;
        ACADO.x[0].ds = ACADO.x0.ds;
        ACADO.x[0].omega_ref_x = ACADO.x0.omega_ref_x;
        ACADO.x[0].omega_ref_y = ACADO.x0.omega_ref_y;
    }

    double MPC::getWindowAngularVelocityX(void) { return std::fmax(std::fmin(windowAngularVelocity_ref_[0], maxAngularVelocity_), -maxAngularVelocity_); }
    double MPC::getWindowAngularVelocityY(void) { return std::fmax(std::fmin(windowAngularVelocity_ref_[1], maxAngularVelocity_), -maxAngularVelocity_); }

    Eigen::Vector2d MPC::getInertialAngularVelocity(void)
    {
        // convert MPC W_omega_ref to K_omega_ref   (from window frame, which the MPC is working within, to inertial frame)
        Eigen::Vector2d W_omega_ref_2D;
        W_omega_ref_2D << getWindowAngularVelocityX(), getWindowAngularVelocityY(); // omega_ref in heading frame
        Eigen::Matrix2d R_window = Eigen::Rotation2Dd(WindowOrientation_).toRotationMatrix().transpose(); // the transpose here is used to get R_window in the same form as everywhere else:   R_window = [cos(Y), sin(Y);  -sin(Y), cos(Y)]
        Eigen::Vector2d I_omega_ref_2D = R_window.transpose() * W_omega_ref_2D;
        return I_omega_ref_2D;
    }

    Eigen::Vector3d MPC::getBodyAngularVelocity(double yawAngularVelocity)
    {
        Eigen::Vector3d I_omega_ref;
        I_omega_ref << getInertialAngularVelocity(), yawAngularVelocity; // omega_ref in inertial frame

        // omega_ref_B = devec * Phi(q)' * Gamma(q) * vec * omega_ref_I;
        boost::math::quaternion<double> q_I_omega_ref(0, I_omega_ref(0), I_omega_ref(1), I_omega_ref(2));
        boost::math::quaternion<double> q_B_omega_ref = conj(quaternion_) * q_I_omega_ref * quaternion_;

        Eigen::Vector3d B_omega_ref;
        B_omega_ref << q_B_omega_ref.R_component_2(), q_B_omega_ref.R_component_3(), q_B_omega_ref.R_component_4();

        return B_omega_ref;
    }

    std::vector<std::pair<double,double>> MPC::getInertialAngularVelocityHorizon(void)
    {
        std::vector<std::pair<double,double>> angularVelocities;
        Eigen::Matrix2d R_window = Eigen::Rotation2Dd(WindowOrientation_).toRotationMatrix().transpose(); // the transpose here is used to get R_window in the same form as everywhere else:   R_window = [cos(Y), sin(Y);  -sin(Y), cos(Y)]

        for (unsigned int i = (ACADO_N+1); i >= 1; i--) {
            Eigen::Vector2d H_omega_ref_2D(std::fmax(std::fmin(ACADO.x[i].omega_ref_x, maxAngularVelocity_), -maxAngularVelocity_),
                                           std::fmax(std::fmin(ACADO.x[i].omega_ref_y, maxAngularVelocity_), -maxAngularVelocity_));
            Eigen::Vector2d I_omega_ref_2D = R_window.transpose() * H_omega_ref_2D;

            std::pair<double,double> angularVelocity;
            angularVelocity.first = I_omega_ref_2D[0];
            angularVelocity.second = I_omega_ref_2D[1];
            angularVelocities.push_back(angularVelocity);
        }

        return angularVelocities;
    }

    /*void MPC::EnableDebugOutput()
    {
        // See "QPOASES_PRINTLEVEL" flag in "acado_qpoases3_interface.h" - eg. set to PL_HIGH instead of PL_NONE
    }*/

    void MPC::resetACADO()
    {
        /* Reset ACADO workspace */
        // Either we can just initialize the solver cleanly
        acado_initializeSolver(); // basically does   memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
        // Or we can initialize it by forward simulation  // requires acadoVariables to be initialized
        //acado_initializeNodesByForwardSimulation();

        #if ACADO_USE_ARRIVAL_COST == 1
        acado_updateArrivalCost( 1 );   // what does this do? And does the argument need to be 1 or 0?
        #endif // ACADO_USE_ARRIVAL_COST == 1
    }

    void MPC::Step()
    {
        acado_timer t;          // to measure solvetime of solver call
        acado_tic(&t);

        /* Shift states - done here instead of after, to keep both the current and predicted MPC values in the ACADO variable */
        shiftStates();

        /* Prediction step */
        acado_preparationStep();

        /* Feedback step */
        SolverStatus_ = (status_t)acado_feedbackStep();

        /* Get solver status and debug info */
        SolverKKT_ = acado_getKKT();
        SolverCostValue_ = acado_getObjective();
        SolverIterations_ = acado_getNWSR();

        /* Shifting */
        #if ACADO_USE_ARRIVAL_COST == 1
        acado_updateArrivalCost( 0 );
        #endif // ACADO_USE_ARRIVAL_COST == 1

        //acado_shiftStates(strategy, xEnd, uEnd);
        // Shifting strategy: Move all samples forward, discarding the first/current element and 1) Initialize last node with xEnd or 2) Initialize node 51 by forward simulation.
        /*acado_shiftStates(1, 0, 0); // shift samples and keep the last value the same
        acado_shiftControls( 0 );*/

        /* return outputs and prepare next iteration */
        //for ( int i = 0; i < ACADO_NU; i++ ) out_u0[i] = acadoVariables.u[i];

        /*windowAngularVelocity_ref_[0] = ACADO.u[0].omega_ref_x;
        windowAngularVelocity_ref_[1] = ACADO.u[0].omega_ref_y;*/

        /* The MPC is controlling the angular acceleration, thus the current angular velocity output is taken as the state one step ahead
         * (gives the same as integrating the current domega_ref_x and domega_ref_y output from the MPC) */
        windowAngularVelocity_ref_[0] = ACADO.x[1].omega_ref_x;
        windowAngularVelocity_ref_[1] = ACADO.x[1].omega_ref_y;

        if ((ACADO.x[ACADO_N].s + closestPositionOnCurrentPathToOrigin_) > windowPathLength_) {
            std::cout << "[WARN] MPC horizon is running outside of input path - Path length=" << windowPathLength_ << " vs s[N]=" << (ACADO.x[ACADO_N].s + closestPositionOnCurrentPathToOrigin_) << std::endl;
        }

        /* calculate compute time */
        SolveTime_ = static_cast<float>(acado_toc(&t));

        std::cout << "Solve time: " << SolveTime_ << std::endl;
    }

    void MPC::PlotPredictedTrajectory(cv::Mat& image, double x_min, double y_min, double x_max, double y_max)
    {
        Trajectory predictedTrajectory;
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            auto state = getHorizonState(i);
            double velocity = sqrt(state.velocity(0)*state.velocity(0) + state.velocity(1)*state.velocity(1));
            double RobotYaw = extractHeading(state.quaternion);
            predictedTrajectory.AddPoint(state.position, false, RobotYaw, velocity);
        }
        // Note that this predicted trajectory is in inertial frame
        predictedTrajectory.plot(image, cv::Scalar(255, 0, 0), false, false, x_min, y_min, x_max, y_max);
    }

    void MPC::PlotPredictedTrajectoryInWindow(cv::Mat& image, double x_min, double y_min, double x_max, double y_max)
    {
        Trajectory predictedTrajectory;
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            double velocity = sqrt(ACADO.x[i].dx*ACADO.x[i].dx + ACADO.x[i].dy*ACADO.x[i].dy);
            //double velocity = ACADO.x[i].ds;
            predictedTrajectory.AddPoint(ACADO.x[i].x, ACADO.x[i].y, false, 0, velocity);
        }
        // Note that this predicted trajectory is in window frame
        predictedTrajectory.plot(image, cv::Scalar(255, 0, 0), true, true, x_min, y_min, x_max, y_max);
    }

    Trajectory MPC::getPredictedTrajectory(void)
    {
        Trajectory predictedTrajectory;

        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            double velocity = sqrt(ACADO.x[i].dx*ACADO.x[i].dx + ACADO.x[i].dy*ACADO.x[i].dy);
            //double velocity = ACADO.x[i].ds;
            predictedTrajectory.AddPoint(ACADO.x[i].x, ACADO.x[i].y, 0, velocity);
        }

        return predictedTrajectory;
    }

    MPC::state_t MPC::getHorizonState(unsigned int horizonIndex)
    {
        MPC::state_t state;
        if (horizonIndex >= (ACADO_N+1)) return state; // return an empty/zero state, since we are outside the horizon

        /* Rotate quaternion into inertial frame - the MPC quaternion elements are in window frame */
        Eigen::Matrix2d R_window = Eigen::Rotation2Dd(WindowOrientation_).toRotationMatrix().transpose();
        Eigen::Vector2d q_tilt_inertial_q2q3 = R_window.transpose() * Eigen::Vector2d(ACADO.x[horizonIndex].q2, ACADO.x[horizonIndex].q3);

        /* Compute heading quaternion */
        double RobotYaw = extractHeading(quaternion_);
        boost::math::quaternion<double> q_heading(cos(RobotYaw/2),0,0,sin(RobotYaw/2)); // w,x,y,z

        /* Construct tilt quaternion in heading frame */
        Eigen::Matrix2d R_heading = Eigen::Rotation2Dd(RobotYaw).toRotationMatrix().transpose();
        Eigen::Vector2d q_tilt_heading_q2q3 = R_heading * q_tilt_inertial_q2q3;
        double q1 = sqrt(1 - q_tilt_heading_q2q3[0]*q_tilt_heading_q2q3[0] - q_tilt_heading_q2q3[1]*q_tilt_heading_q2q3[1]);
        boost::math::quaternion<double> q_tilt_heading(q1, q_tilt_heading_q2q3[0], q_tilt_heading_q2q3[1], 0);

        /* Construct resulting quaternion by combining the tilt and heading */
        boost::math::quaternion<double> q = q_heading * q_tilt_heading;
        q /= norm(q); // this step can maybe be left out, since the quaternions being multiplied should already be unit quaternions

        /* Compute inertial frame position */
        Eigen::Vector2d InertialPosition = R_window.transpose() * Eigen::Vector2d(ACADO.x[horizonIndex].x, ACADO.x[horizonIndex].y) + windowPathOrigin_;

        /* Rotate estimated velocity from window frame to inertial frame */
        Eigen::Vector2d InertialVelocity = R_window.transpose() * Eigen::Vector2d(ACADO.x[horizonIndex].dx, ACADO.x[horizonIndex].dy);

        /* Prepare output variable */
        state.quaternion = q;
        state.position = InertialPosition;
        state.velocity = InertialVelocity;
        state.pathDistance = ACADO.x[horizonIndex].s;
        state.pathVelocity = ACADO.x[horizonIndex].ds;

        return state;
    }

    double MPC::getHorizonPathLength() const
    {
        return ACADO.x[ACADO_N].s;
    }

    MPC::status_t MPC::getStatus() const
    {
        return SolverStatus_;
    }

    double MPC::getSolveTime() const
    {
        return SolveTime_;
    }

    void MPC::PlotRobot(cv::Mat& image, cv::Scalar color, bool drawXup, double x_min, double y_min, double x_max, double y_max)
    {
        double xres = image.cols;
        double yres = image.rows;

        // Scale range (x_min:x_max) and (y_min:y_max) to (0:499)
        double scale_x = xres / (x_max - x_min);
        double scale_y = yres / (y_max - y_min);
        double center_x = (x_min + x_max) / 2.0;
        double center_y = (y_min + y_max) / 2.0;

        float x = (position_[0]-x_min) * scale_x;
        float y = (position_[1]-y_min) * scale_y;

        if (x >= 0 && x < xres && y >= 0 && y < yres) {
            cv::Point center;
            if (drawXup) // draw with robot x-axis pointing up in plot
                center = cv::Point(yres - y, xres - x);
            else
                center = cv::Point(x, yres - y);

            //cv::drawMarker(image, point, color, cv::MARKER_STAR, 6, 2, 8);
            cv::circle(image, center, scale_x*RobotRadius, color, 1, 8);

            /* Compute and plot robot heading (based on quaternion) */
            double plotHeading = extractHeading(quaternion_);
            cv::Point pHeading;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pHeading = center + cv::Point(-scale_x*RobotRadius*sin(plotHeading), -scale_x*RobotRadius*cos(plotHeading));
            else
                pHeading = center + cv::Point(scale_x*RobotRadius*cos(plotHeading), -scale_x*RobotRadius*sin(plotHeading));
            cv::line(image, center, pHeading, color, 1, 8);

            /* Compute and plot velocity in window frame */
            /*Eigen::Matrix2d R_window = Eigen::Rotation2Dd(WindowOrientation_).toRotationMatrix().transpose();
            Eigen::Vector2d WindowVelocity = R_window * velocity_;
            cv::Point pVelocity;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pVelocity = center + cv::Point(-20*WindowVelocity[1], -20*WindowVelocity[0]);
            else
                pVelocity = center + cv::Point(20*WindowVelocity[0], -20*WindowVelocity[1]);
            cv::line(image, center, pVelocity, cv::Scalar(255, 0, 0), 2, 8);*/

            /* Plot velocity in inertial frame */
            cv::Point pVelocity;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pVelocity = center + cv::Point(-2*scale_x*RobotRadius*velocity_[1], -2*scale_x*RobotRadius*velocity_[0]);
            else
                pVelocity = center + cv::Point(2*scale_x*RobotRadius*velocity_[0], -2*scale_x*RobotRadius*velocity_[1]);
            cv::line(image, center, pVelocity, cv::Scalar(255, 0, 0), 2, 8);

            /* Compute and plot tilt direction */
            /* Compute Tilt direction */
            boost::math::quaternion<double> q_z = quaternion_ * boost::math::quaternion<double>(0,0,0,1) * conj(quaternion_); // Phi(q)*Gamma(q)'*[0,0,0,1]'
            Eigen::Vector2d tiltDirection = Eigen::Vector2d(q_z.R_component_2(), q_z.R_component_3()); // R_window * [0,1,0,0;0,0,1,0] * Phi(q)*Gamma(q)'*[0,0,0,1]';
            cv::Point pTilt;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pTilt = center + cv::Point(-15*scale_x*RobotRadius*tiltDirection[1], -15*scale_x*RobotRadius*tiltDirection[0]);
            else
                pTilt = center + cv::Point(15*scale_x*RobotRadius*tiltDirection[0], -15*scale_x*RobotRadius*tiltDirection[1]);
            cv::line(image, center, pTilt, cv::Scalar(0, 255, 0), 2, 8);
        }
    }

    void MPC::PlotRobotInWindow(cv::Mat& image, cv::Scalar color, bool drawXup, double x_min, double y_min, double x_max, double y_max)
    {
        double xres = image.cols;
        double yres = image.rows;

        // Scale range (x_min:x_max) and (y_min:y_max) to (0:499)
        double scale_x = xres / (x_max - x_min);
        double scale_y = yres / (y_max - y_min);
        double center_x = (x_min + x_max) / 2.0;
        double center_y = (y_min + y_max) / 2.0;

        Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
        Eigen::Vector2d WindowPosition = rot * (position_ - windowPathOrigin_);

        float x = (WindowPosition[0]-x_min) * scale_x;
        float y = (WindowPosition[1]-y_min) * scale_y;

        if (x >= 0 && x < xres && y >= 0 && y < yres) {
            cv::Point center;
            if (drawXup) // draw with robot x-axis pointing up in plot
                center = cv::Point(yres - y, xres - x);
            else
                center = cv::Point(x, yres - y);

            //cv::drawMarker(image, point, color, cv::MARKER_STAR, 6, 2, 8);
            cv::circle(image, center, scale_x*RobotRadius, color, 1, 8);

            /* Compute and plot robot heading (based on quaternion) */
            double plotHeading = extractHeading(quaternion_);
            cv::Point pHeading;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pHeading = center + cv::Point(-scale_x*RobotRadius*sin(plotHeading-WindowOrientation_), -scale_x*RobotRadius*cos(plotHeading-WindowOrientation_));
            else
                pHeading = center + cv::Point(scale_x*RobotRadius*cos(plotHeading-WindowOrientation_), -scale_x*RobotRadius*sin(plotHeading-WindowOrientation_));
            cv::line(image, center, pHeading, color, 1, 8);

            /* Compute and plot velocity in window frame */
            /*Eigen::Matrix2d R_window = Eigen::Rotation2Dd(WindowOrientation_).toRotationMatrix().transpose();
            Eigen::Vector2d WindowVelocity = R_window * velocity_;
            cv::Point pVelocity;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pVelocity = center + cv::Point(-20*WindowVelocity[1], -20*WindowVelocity[0]);
            else
                pVelocity = center + cv::Point(20*WindowVelocity[0], -20*WindowVelocity[1]);
            cv::line(image, center, pVelocity, cv::Scalar(255, 0, 0), 2, 8);*/

            /* Plot velocity in inertial frame */
            cv::Point pVelocity;
            Eigen::Vector2d WindowVelocity = rot * velocity_;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pVelocity = center + cv::Point(-2*scale_x*RobotRadius*WindowVelocity[1], -2*scale_x*RobotRadius*WindowVelocity[0]);
            else
                pVelocity = center + cv::Point(2*scale_x*RobotRadius*WindowVelocity[0], -2*scale_x*RobotRadius*WindowVelocity[1]);
            cv::line(image, center, pVelocity, cv::Scalar(255, 0, 0), 2, 8);

            /* Compute and plot tilt direction */
            /* Compute Tilt direction */
            boost::math::quaternion<double> q_z = quaternion_ * boost::math::quaternion<double>(0,0,0,1) * conj(quaternion_); // Phi(q)*Gamma(q)'*[0,0,0,1]'
            Eigen::Vector2d tiltDirection = Eigen::Vector2d(q_z.R_component_2(), q_z.R_component_3()); // R_window * [0,1,0,0;0,0,1,0] * Phi(q)*Gamma(q)'*[0,0,0,1]';
            Eigen::Vector2d WindowTiltDirection = rot * tiltDirection;
            cv::Point pTilt;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pTilt = center + cv::Point(-15*scale_x*RobotRadius*WindowTiltDirection[1], -15*scale_x*RobotRadius*WindowTiltDirection[0]);
            else
                pTilt = center + cv::Point(15*scale_x*RobotRadius*WindowTiltDirection[0], -15*scale_x*RobotRadius*WindowTiltDirection[1]);
            cv::line(image, center, pTilt, cv::Scalar(0, 255, 0), 2, 8);
        }
    }


    void MPC::PlotObstacles(cv::Mat& image, cv::Scalar obstacleColor, cv::Scalar consideredColor, bool drawXup, double x_min, double y_min, double x_max, double y_max)
    {
        double xres = image.cols;
        double yres = image.rows;

        // Scale range (x_min:x_max) and (y_min:y_max) to (0:499)
        double scale_x = xres / (x_max - x_min);
        double scale_y = yres / (y_max - y_min);
        double center_x = (x_min + x_max) / 2.0;
        double center_y = (y_min + y_max) / 2.0;

        for (auto obstacle : currentObstacles_) {
            float x = (obstacle.x - x_min) * scale_x;
            float y = (obstacle.y - y_min) * scale_y;

            if (x >= 0 && x < xres && y >= 0 && y < yres) {
                cv::Point center;
                if (drawXup) // draw with robot x-axis pointing up in plot
                    center = cv::Point(yres - y, xres - x);
                else
                    center = cv::Point(x, yres - y);

                //cv::drawMarker(image, point, color, cv::MARKER_STAR, 6, 2, 8);
                cv::circle(image, center, scale_x*obstacle.radius, obstacleColor, 1, 8);
            }
        }

        for (auto obstacle : consideredObstacles_) {
            float x = (obstacle.x - x_min) * scale_x;
            float y = (obstacle.y - y_min) * scale_y;

            if (x >= 0 && x < xres && y >= 0 && y < yres) {
                cv::Point center;
                if (drawXup) // draw with robot x-axis pointing up in plot
                    center = cv::Point(yres - y, xres - x);
                else
                    center = cv::Point(x, yres - y);

                //cv::drawMarker(image, point, color, cv::MARKER_STAR, 6, 2, 8);
                cv::circle(image, center, scale_x*obstacle.radius, consideredColor, 1, 8);
            }
        }
    }

    void MPC::PlotObstaclesInWindow(cv::Mat& image, cv::Scalar color, bool drawXup, double x_min, double y_min, double x_max, double y_max)
    {
        double xres = image.cols;
        double yres = image.rows;

        // Scale range (x_min:x_max) and (y_min:y_max) to (0:499)
        double scale_x = xres / (x_max - x_min);
        double scale_y = yres / (y_max - y_min);
        double center_x = (x_min + x_max) / 2.0;
        double center_y = (y_min + y_max) / 2.0;

        int endIdx = currentObstacles_.size() - 1;
        if (endIdx > 4) endIdx = 4;

        for (int i = 0; i <= endIdx; i++) {
            Eigen::Vector2d p_obstacle, p_obstacle_in_window;
            p_obstacle[0] = currentObstacles_.at(i).x - windowPathOrigin_[0];
            p_obstacle[1] = currentObstacles_.at(i).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            p_obstacle_in_window = rot * p_obstacle;

            float x = (p_obstacle_in_window[0] - x_min) * scale_x;
            float y = (p_obstacle_in_window[1] - y_min) * scale_y;

            if (x >= 0 && x < xres && y >= 0 && y < yres) {
                cv::Point center;
                if (drawXup) // draw with robot x-axis pointing up in plot
                    center = cv::Point(yres - y, xres - x);
                else
                    center = cv::Point(x, yres - y);

                //cv::drawMarker(image, point, color, cv::MARKER_STAR, 6, 2, 8);
                cv::circle(image, center, scale_x*currentObstacles_.at(i).radius, color, 1, 8);
            }
        }
    }


    double MPC::getSampleTime() const
    {
        return ACADO_TS;
    }

}
