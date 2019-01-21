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

    MPC::MPC()
    {
        Reset();
    }


    MPC::~MPC()
    {

    }

    void MPC::Reset(void)
    {
        // Initialize ACADO
        Initialize();
        initStates();

        // Set window and path parameters
        WindowWidth_ = 5.0;
        WindowHeight_ = 5.0;
        WindowOffset_ << 0, 0;
        pathApproximationOrder_ = 5;
        WindowOrientationSelection_ = INERTIAL_FRAME;

        // Set weights
        Eigen::Matrix<double, 5, 1> Wdiag;
        Eigen::Matrix<double, 3, 1> WNdiag;
        Wdiag << 100, 100, 1, 3, 3;
        WNdiag << 100, 100, 1;
        setWeights(Wdiag.asDiagonal().toDenseMatrix(), WNdiag.asDiagonal().toDenseMatrix());

        // Set internal to default values
        setVelocityBounds(0.0, 3.0);
        setDesiredVelocity(1.0);
        setControlLimits(deg2rad(10), deg2rad(5));
        setReferences();

        // Set an empty reference path
        Path emptyPath;
        setPath(emptyPath, Eigen::Vector2d(0,0), Eigen::Vector2d(0,0));
    }

    void MPC::setTrajectory(Trajectory& trajectory, const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q)
    {
        ExtractWindowTrajectory(trajectory, currentTrajectory_, position, velocity, q, 1.1*HorizonLength*ACADO_TS*desiredVelocity_, WindowOrientationSelection_); // change the last parameter/flag to:  0=window in inertial frame,  1=window in heading frame,  2=window in velocity direction frame
        //currentTrajectory_.print(); currentTrajectory_.plot(true); cv::waitKey(0);

        Path windowPath(currentTrajectory_, pathApproximationOrder_, true, true);
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
        return closestPositionOnCurrentPath_;
    }

    void MPC::ExtractWindowTrajectory(Trajectory& trajectory, Trajectory& extractedWindowTrajectory, const Eigen::Vector2d& position, const Eigen::Vector2d& velocity, const boost::math::quaternion<double>& q, double ExtractionDistance, orientation_selection_t OrientationSelection)
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
        trajectory.WindowExtract(RobotCentricTrajectory, position, WindowOrientation_, WindowWidth_, WindowHeight_, WindowOffset_);

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

        if (path.order() > 7) {
            std::cout << "ERROR: Path order not supported by MPC (too high)" << std::endl;
            return;
        }

        // Based on current position, compute the closest position (s-value) on this new path
        closestPositionOnCurrentPath_ = windowPath_.FindClosestPoint(currentPosition - windowPathOrigin_);

        std::cout << "MPC path updated:" << std::endl;
        std::cout << "   length = " << windowPathLength_ << std::endl;
        std::cout << "   closest point = " << closestPositionOnCurrentPath_ << std::endl;
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

            ACADO.od[i].cy0 = windowPath_.getYcoefficient(0);
            ACADO.od[i].cy1 = windowPath_.getYcoefficient(1);
            ACADO.od[i].cy2 = windowPath_.getYcoefficient(2);
            ACADO.od[i].cy3 = windowPath_.getYcoefficient(3);
            ACADO.od[i].cy4 = windowPath_.getYcoefficient(4);
            ACADO.od[i].cy5 = windowPath_.getYcoefficient(5);
            ACADO.od[i].cy6 = windowPath_.getYcoefficient(6);
            ACADO.od[i].cy7 = windowPath_.getYcoefficient(7);

            ACADO.od[i].trajectoryLength = windowPathLength_;
            ACADO.od[i].trajectoryStart = closestPositionOnCurrentPath_;
        }

        // Reset horizon predictions for the movement on the path (s-value)
        real_t s_reset = ACADO.x[0].s;
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.x[i].s = ACADO.x[i].s - s_reset;
        }
        // This results in ACADO.x[0].s = 0 and the path value predictions are then increasing from there
    }

    void MPC::setVelocityBounds(double min_velocity, double max_velocity)
    {
        minVelocity_ = min_velocity;
        maxVelocity_ = max_velocity;

        // Set bounds for the full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].minVelocity = minVelocity_;
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

    void MPC::setControlLimits(double maxAngularVelocity, double maxAngle)
    {
        maxAngularVelocity_ = maxAngularVelocity;
        maxAngle_ = maxAngle;

        // Set bounds for the full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].maxAngle = maxAngle_;
            ACADO.od[i].maxOmegaRef = maxAngularVelocity_;
        }
    }

    void MPC::setWeights(const Eigen::MatrixXd& W, const Eigen::MatrixXd& WN)
    {
        if (W.rows() != ACADO_NY || W.cols() != ACADO_NY) return; // Error, weights of incorrect dimension
        if (WN.rows() != ACADO_NYN || WN.cols() != ACADO_NYN) return; // Error, weights of incorrect dimension

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
            ACADO.y[i].x_err = 0;
            ACADO.y[i].y_err = 0;
            ACADO.y[i].ds_err = 0;
            ACADO.y[i].omega_ref_x = 0;
            ACADO.y[i].omega_ref_y = 0;
        }
        ACADO.yN.x_err = 0;
        ACADO.yN.y_err = 0;
        ACADO.yN.ds_err = 0;
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

        /* Compute rotation matrix for path window orientation */
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

        /* Should not modify path parameter, since this is kept static by the shifting strategy, as closestPositionOnCurrentPath defines the starting point on the current path */
        ACADO.x0.s = ACADO.x[1].s; // set the initial guess to the shifted horizon value of the path parameter, s
        //ACADO.x[0].s = ACADO.x0.s = 0;
        //ACADO.x[0].ds = ACADO.x0.ds = 0;
    }

    void MPC::initStates(void)
    {
        ACADO.x0.q2 = 0;
        ACADO.x0.q3 = 0;
        ACADO.x0.x = 0;
        ACADO.x0.y = 0;
        ACADO.x0.dx = 0;
        ACADO.x0.dy = 0;
        ACADO.x0.s = 0;
        ACADO.x0.ds = 0;

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
        }

        for (unsigned int i = 0; i < ACADO_N; i++) {
            ACADO.u[i].omega_ref_x = 0;
            ACADO.u[i].omega_ref_y = 0;
            ACADO.u[i].s_acceleration = 0;
        }
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
        }

        for (unsigned int i = 0; i < (ACADO_N-1); i++) {
            ACADO.u[i].omega_ref_x = ACADO.u[i+1].omega_ref_x;
            ACADO.u[i].omega_ref_y = ACADO.u[i+1].omega_ref_y;
            ACADO.u[i].s_acceleration = ACADO.u[i+1].s_acceleration;
        }

        ACADO.x[0].q2 = ACADO.x0.q2;
        ACADO.x[0].q3 = ACADO.x0.q3;
        ACADO.x[0].x = ACADO.x0.x;
        ACADO.x[0].y = ACADO.x0.y;
        ACADO.x[0].dx = ACADO.x0.dx;
        ACADO.x[0].dy = ACADO.x0.dy;
        ACADO.x[0].s = ACADO.x0.s;
        ACADO.x[0].ds = ACADO.x0.ds;
    }


    double MPC::getWindowAngularVelocityX(void) { return controlBodyAngularVelocity_[0]; }
    double MPC::getWindowAngularVelocityY(void) { return controlBodyAngularVelocity_[1]; }
    double MPC::getWindowAngularVelocityZ(void) { return controlBodyAngularVelocity_[2]; }

    Eigen::Vector2d MPC::getInertialAngularVelocity(void)
    {
        // convert MPC W_omega_ref to K_omega_ref   (from window frame, which the MPC is working within, to inertial frame)
        Eigen::Vector2d H_omega_ref_2D;
        H_omega_ref_2D << getWindowAngularVelocityX(), getWindowAngularVelocityY(); // omega_ref in heading frame
        Eigen::Matrix2d R_window = Eigen::Rotation2Dd(WindowOrientation_).toRotationMatrix().transpose();
        Eigen::Vector2d I_omega_ref_2D = R_window.transpose() * H_omega_ref_2D;
        return I_omega_ref_2D;
    }


    /*void MPC::EnableDebugOutput()
    {
        // See "QPOASES_PRINTLEVEL" flag in "acado_qpoases3_interface.h" - eg. set to PL_HIGH instead of PL_NONE
    }*/

    void MPC::Initialize()
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

        /* Shift states - done here instead of after, to keep the predicted MPC values in the ACADO variable */
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

        controlBodyAngularVelocity_[0] = ACADO.u[0].omega_ref_x;
        controlBodyAngularVelocity_[1] = ACADO.u[0].omega_ref_y;
        controlBodyAngularVelocity_[2] = 0;

        if (ACADO.x[ACADO_N].s > windowPathLength_) {
            std::cout << "[WARN] MPC horizon is running outside of input path" << std::endl;
        }

        /* calculate compute time */
        float solve_time = static_cast<float>(acado_toc(&t));

        std::cout << "Solve time: " << solve_time << std::endl;
    }

    #if 0
    void Acado_Initialize(double * xInit, double * uInit, double * Wmat, double * WNmat, double * refInit, double * odInit)
    {
        int i, j;

        /* Initialize the solver. */
        acado_initializeSolver();

        for( i=0; i < ACADO_N+1; ++i ) {
            for( j=0; j < ACADO_NX; ++j ) acadoVariables.x[i*ACADO_NX+j] = xInit[j];
        }
        /*#if ACADO_NXA > 0   // algebraic variables
        for( i=0; i < ACADO_N; ++i ) {
            for( j=0; j < ACADO_NXA; ++j ) acadoVariables.z[i*ACADO_NXA+j] = zInit[j];
        }
        #endif*/
        for( i=0; i < ACADO_N; ++i ) {
            for( j=0; j < ACADO_NU; ++j ) acadoVariables.u[i*ACADO_NU+j] = uInit[j];
        }

        for( i=0; i < ACADO_N; ++i ) {
            for( j=0; j < ACADO_NY; ++j ) acadoVariables.y[i*ACADO_NY+j] = refInit[j*(ACADO_N+1)+i];
        }
        for( j=0; j < ACADO_NYN; ++j ) acadoVariables.yN[j] = refInit[j*(ACADO_N+1) + ACADO_N];

        #if ACADO_NOD
        for( i=0; i < ACADO_N+1; ++i ) {
            for( j=0; j < ACADO_NOD; ++j ) acadoVariables.od[i*ACADO_NOD+j] = odInit[j*(ACADO_N+1)+i];
        }
        #endif

        /* ACADO Matrices are stored in row-major order
        uint i,j;
        for( i=0; i < getNumRows(); i++ ){
            for( j=0; j < getNumCols(); j++ ){
                if( types[i][j] != SBMT_ZERO ){
                    types   [i][j]  = SBMT_DENSE;
                    elements[i][j] *= scalar    ;
                }
            }
        }
        */

        for( i = 0; i < (ACADO_NYN); ++i )  {
            for( j = 0; j < ACADO_NYN; ++j ) {
                acadoVariables.WN[i*ACADO_NYN+j] = WNmat[i*ACADO_NYN+j];
            }
        }
        for( i = 0; i < (ACADO_NY); ++i )  {
            for( j = 0; j < ACADO_NY; ++j ) {
                acadoVariables.W[i*ACADO_NY+j] = Wmat[i*ACADO_NY+j];
            }
        }

        /*#if VARYING_BOUNDS
        for( i = 0; i < QPOASES_NVMAX; ++i )  {
            acadoVariables.lbValues[i] = bValues[i];
            acadoVariables.ubValues[i] = bValues[QPOASES_NVMAX+QPOASES_NCMAX+i];
        }

        for( i = 0; i < QPOASES_NCMAX; ++i )  {
            acadoVariables.lbAValues[i] = bValues[QPOASES_NVMAX+i];
            acadoVariables.ubAValues[i] = bValues[QPOASES_NVMAX+QPOASES_NCMAX+QPOASES_NVMAX+i];
        }
        #endif*/

        /* Prepare first step */
        acado_preparationStep();
    }

    void Acado_Step(
        double *in_x, double *in_ref, double *in_od, double in_nIter,
        double *out_u0, double *out_xTraj, double *out_uTraj, double *out_kktTol, double *out_status, double *out_nIter, double *out_objVal
    )
    {
        int i, j, status;
        double sumIter;
        real_t kkt;

        //real_t *in_x, *in_ref, *in_W, *in_WN, *in_bValues, *in_od, *in_nIter;
        //real_t *out_u0, *out_xTraj, *out_uTraj, *out_kktTol, *out_cpuTime, *out_status, *out_nIter, *out_objVal, *xInit, *zInit, *uInit;

        #if RESET_WHEN_NAN == 1
            real_t x_prev[ (ACADO_N+1)*ACADO_NX ];
            #if ACADO_NXA > 0
            real_t z_prev[ ACADO_N*ACADO_NXA ];
            #endif
            real_t u_prev[ ACADO_N*ACADO_NU ];
        #endif

        for( i=0; i < ACADO_NX; ++i ) acadoVariables.x0[i] = in_x[i];

        for( i=0; i < ACADO_N; ++i ) {
            for( j=0; j < ACADO_NY; ++j ) acadoVariables.y[i*ACADO_NY+j] = in_ref[j*(ACADO_N+1)+i];
        }
        for( j=0; j < ACADO_NYN; ++j ) acadoVariables.yN[j] = in_ref[j*(ACADO_N+1) + ACADO_N];

        #if ACADO_NOD
        for( i=0; i < ACADO_N+1; ++i ) {
            for( j=0; j < ACADO_NOD; ++j ) acadoVariables.od[i*ACADO_NOD+j] = in_od[j*(ACADO_N+1)+i];
        }
        #endif

        /* Do not update weight matrices */
        /*for( i=0; i < ACADO_NY; ++i ) {
            for( j=0; j < ACADO_NY; ++j ) acadoVariables.W[i*ACADO_NY+j] = (double)(*in_W[i*ACADO_NY+j]);
        }
        for( i=0; i < ACADO_NYN; ++i ) {
            for( j=0; j < ACADO_NYN; ++j ) acadoVariables.WN[i*ACADO_NYN+j] = (double)(*in_WN[i*ACADO_NYN+j]);
        }*/

        /*#if VARYING_BOUNDS
        for( i = 0; i < QPOASES_NVMAX; ++i )  {
            acadoVariables.lbValues[i] = (double)(*in_bValues[i]);
            acadoVariables.ubValues[i] = (double)(*in_bValues[QPOASES_NVMAX+QPOASES_NCMAX+i]);
        }

        for( i = 0; i < QPOASES_NCMAX; ++i )  {
            acadoVariables.lbAValues[i] = (double)(*in_bValues[QPOASES_NVMAX+i]);
            acadoVariables.ubAValues[i] = (double)(*in_bValues[QPOASES_NVMAX+QPOASES_NCMAX+QPOASES_NVMAX+i]);
        }
        #endif*/

        #if RESET_WHEN_NAN == 1
            for( i = 0; i < (ACADO_N+1)*ACADO_NX; ++i ) x_prev[ i ] = acadoVariables.x[ i ];
            #if ACADO_NXA > 0
            for( i = 0; i < ACADO_N*ACADO_NXA; ++i ) z_prev[ i ] = acadoVariables.z[ i ];
            #endif
            for( i = 0; i < ACADO_N*ACADO_NU; ++i ) u_prev[ i ] = acadoVariables.u[ i ];
        #endif

        status = acado_feedbackStep( );
        sumIter = acado_getNWSR();

        for( i = 1; i < in_nIter; i++ ) {
            acado_preparationStep( );
            status = acado_feedbackStep( );
            sumIter += acado_getNWSR();
        }

        #if RESET_WHEN_NAN == 1
            kkt = acado_getKKT();
            if( kkt < KKT_MIN || kkt > KKT_MAX || kkt != kkt ) {
                for( i = 0; i < (ACADO_N+1)*ACADO_NX; ++i ) acadoVariables.x[ i ] = x_prev[ i ];
                #if ACADO_NXA > 0
                for( i = 0; i < ACADO_N*ACADO_NXA; ++i ) acadoVariables.z[ i ] = z_prev[ i ];
                #endif
                for( i = 0; i < ACADO_N*ACADO_NU; ++i ) acadoVariables.u[ i ] = u_prev[ i ];

                status = -30; // PERFORMED RESET TO PREVIOUS TRAJECTORIES BECAUSE OF NAN
            }
        #endif

        for( i=0; i < ACADO_N+1; ++i ) {
            for( j = 0; j < ACADO_NX; ++j ) {
                out_xTraj[j*(ACADO_N+1)+i] = acadoVariables.x[i*ACADO_NX+j];
            }
        }
        for( i=0; i < ACADO_N; ++i ) {
            for( j = 0; j < ACADO_NU; ++j ) {
                out_uTraj[j*(ACADO_N)+i] = acadoVariables.u[i*ACADO_NU+j];
            }
        }

        /* return outputs and prepare next iteration */
        for( i=0; i < ACADO_NU; ++i ) out_u0[i] = acadoVariables.u[i];
        *out_kktTol = acado_getKKT( );
        *out_status = status;
        *out_nIter = (double) sumIter;
        *out_objVal = acado_getObjective( );

        /* Shift the initialization (look at acado_common.h). */
        acado_shiftStates(1, 0, 0); // Shifting strategy: 1. Initialize node 51 with xEnd. 2. Initialize node 51 by forward simulation.
        acado_shiftControls( 0 );

        /* Prepare for the next step. */
        acado_preparationStep();
    }
    #endif

    void Matrix_Print_RowMajor(double * matrix, int rows, int cols)
    {
      for (int m = 0; m < rows; m++) {
        printf(" ");
        for (int n = 0; n < cols; n++) {
            printf("%8.4f ", matrix[cols*m + n]);
        }
        printf("\n");
      }
    }

    void Matrix_Print_ColMajor(double * matrix, int rows, int cols)
    {
      for (int m = 0; m < rows; m++) {
        printf(" ");
        for (int n = 0; n < cols; n++) {
            printf("%8.4f ", matrix[rows*n + m]);
        }
        printf("\n");
      }
    }

    void MPC::PlotPredictedTrajectory(cv::Mat& image)
    {
        Trajectory predictedTrajectory;
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            double velocity = sqrt(ACADO.x[i].dx*ACADO.x[i].dx + ACADO.x[i].dy*ACADO.x[i].dy);
            //double velocity = ACADO.x[i].ds;
            predictedTrajectory.AddPoint(ACADO.x[i].x, ACADO.x[i].y, 0, velocity);
        }
        // Note that this predicted trajectory is in window frame
        predictedTrajectory.plot(image, cv::Scalar(255, 0, 0), true, true);
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
        double q1 = sqrt(1 - q_tilt_inertial_q2q3[0]*q_tilt_inertial_q2q3[0] - q_tilt_inertial_q2q3[1]*q_tilt_inertial_q2q3[1]);
        boost::math::quaternion<double> q_tilt_inertial(q1, q_tilt_inertial_q2q3[0], q_tilt_inertial_q2q3[1], 0);

        /* Compute heading quaternion */
        double RobotYaw = extractHeading(quaternion_);
        boost::math::quaternion<double> q_heading(cos(RobotYaw/2),0,0,sin(RobotYaw/2)); // w,x,y,z

        /* Construct resulting quaternion by combining the tilt and heading */
        boost::math::quaternion<double> q = q_tilt_inertial * q_heading; // tilt defined in inertial frame
        q /= norm(q); // this step can maybe be left out, since the quaternions being multiplied should already be unit quaternions

        /* Compute inertial frame position */
        Eigen::Vector2d InertialPosition = R_window.transpose() * Eigen::Vector2d(ACADO.x[horizonIndex].x, ACADO.x[horizonIndex].y) + windowPathOrigin_;

        /* Rotate estimated velocity from window frame to inertial frame */
        Eigen::Vector2d InertialVelocity = R_window.transpose() * Eigen::Vector2d(ACADO.x[horizonIndex].dx, ACADO.x[horizonIndex].dy);

        /* Prepare output variable */
        state.quaternion = q;
        state.position = InertialPosition;
        state.velocity = InertialVelocity;
        state.pathVelocity = ACADO.x[horizonIndex].ds;

        return state;
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
            cv::circle(image, center, 10, color, 1, 8);

            /* Compute and plot robot heading (based on quaternion) */
            double plotHeading = extractHeading(quaternion_);
            cv::Point pHeading;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pHeading = center + cv::Point(-10*sin(plotHeading), -10*cos(plotHeading));
            else
                pHeading = center + cv::Point(10*cos(plotHeading), -10*sin(plotHeading));
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
                pVelocity = center + cv::Point(-20*velocity_[1], -20*velocity_[0]);
            else
                pVelocity = center + cv::Point(20*velocity_[0], -20*velocity_[1]);
            cv::line(image, center, pVelocity, cv::Scalar(255, 0, 0), 2, 8);

            /* Compute and plot tilt direction */
            /* Compute Tilt direction */
            boost::math::quaternion<double> q_z = quaternion_ * boost::math::quaternion<double>(0,0,0,1) * conj(quaternion_); // Phi(q)*Gamma(q)'*[0,0,0,1]'
            Eigen::Vector2d tiltDirection = Eigen::Vector2d(q_z.R_component_2(), q_z.R_component_3()); // R_window * [0,1,0,0;0,0,1,0] * Phi(q)*Gamma(q)'*[0,0,0,1]';
            cv::Point pTilt;
            if (drawXup) // draw with robot x-axis pointing up in plot
                pTilt = center + cv::Point(-150*tiltDirection[1], -150*tiltDirection[0]);
            else
                pTilt = center + cv::Point(150*tiltDirection[0], -150*tiltDirection[1]);
            cv::line(image, center, pTilt, cv::Scalar(0, 255, 0), 2, 8);
        }
    }

    double MPC::getSampleTime(void) const
    {
        return ACADO_TS;
    }

}
