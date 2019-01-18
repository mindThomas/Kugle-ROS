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
#include "Path.h"

#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"


/** Instance of ACADO data structure */
extern "C" {
    ACADOvariables acadoVariables;
    ACADOworkspace acadoWorkspace;
}

namespace MPC {

    MPC::MPC()
    {
        Initialize();
    }


    MPC::~MPC()
    {

    }

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

        /* Prediction step */
        acado_preparationStep();

        /* Feedback step */
        auto status = (real_t)acado_feedbackStep();

        auto kktValue = acado_getKKT();
        auto objValue = acado_getObjective();

        auto nIterations = (real_t)acado_getNWSR();

        /* Shifting */
        #if ACADO_USE_ARRIVAL_COST == 1
        acado_updateArrivalCost( 0 );
        #endif // ACADO_USE_ARRIVAL_COST == 1

        //acado_shiftStates(strategy, xEnd, uEnd);
        acado_shiftStates(1, 0, 0); // Shifting strategy: Move all samples forward, discarding the first/current element and 1) Initialize last node with xEnd or 2) Initialize node 51 by forward simulation.
        acado_shiftControls( 0 );

        /* return outputs and prepare next iteration */
        //for ( int i = 0; i < ACADO_NU; i++ ) out_u0[i] = acadoVariables.u[i];

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

}
