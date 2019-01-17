
#define S_FUNCTION_NAME   acado_solver_sfun
#define S_FUNCTION_LEVEL  2

#define MDL_START

#define VARYING_BOUNDS 0
#define RESET_WHEN_NAN 1   // no reset ( = 0 ), reset with previous traj ( = 1 )
#define KKT_MIN -1
#define KKT_MAX 1e15

#include "acado_common.h"
// #include "acado_auxiliary_functions.h"
#include "simstruc.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

#define SAMPLINGTIME -1


static void mdlInitializeSizes (SimStruct *S)
{
    /* Specify the number of continuous and discrete states */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);

    /* Specify the number of intput ports */
    if ( !ssSetNumInputPorts(S, 7) )
        return;

    /* Specify the number of output ports */
    if ( !ssSetNumOutputPorts(S, 8) )
        return;

    /* Specify the number of parameters */
    ssSetNumSFcnParams(S, 6);
    if ( ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S) )
        return;

    /* Specify dimension information for the input ports */
    ssSetInputPortVectorDimension(S, 0, ACADO_NX);
    ssSetInputPortVectorDimension(S, 1, ACADO_N*ACADO_NY+ACADO_NYN);
    ssSetInputPortVectorDimension(S, 2, ACADO_NY*ACADO_NY);
    ssSetInputPortVectorDimension(S, 3, ACADO_NYN*ACADO_NYN);
    #if VARYING_BOUNDS
    ssSetInputPortMatrixDimensions(S, 4, QPOASES_NVMAX+QPOASES_NCMAX, 2);
    #else
    ssSetInputPortVectorDimension(S, 4, 1);
    #endif
    #if ACADO_NOD
    ssSetInputPortVectorDimension(S, 5, (ACADO_N+1)*ACADO_NOD);
    #else
    ssSetInputPortVectorDimension(S, 5, 1);
    #endif
    ssSetInputPortVectorDimension(S, 6, 1);

    /* Specify dimension information for the output ports */
    ssSetOutputPortVectorDimension(S, 0, ACADO_NU );
    ssSetOutputPortMatrixDimensions(S, 1, ACADO_N+1, ACADO_NX );
    ssSetOutputPortMatrixDimensions(S, 2, ACADO_N, ACADO_NU );
    ssSetOutputPortVectorDimension(S, 3, 1 );
    ssSetOutputPortVectorDimension(S, 4, 1 );
    ssSetOutputPortVectorDimension(S, 5, 1 );
    ssSetOutputPortVectorDimension(S, 6, 1 );
    ssSetOutputPortVectorDimension(S, 7, 1 );

    /* Specify the direct feedthrough status */
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 6, 1);

    /* One sample time */
    ssSetNumSampleTimes(S, 1);
    }


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

    #endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    int i, j, k;

    InputRealPtrsType in_ref, in_od;
    double *xInit, *zInit, *uInit, *Wmat, *WNmat, *bValues;

    /* get inputs and perform feedback step */
    in_ref = ssGetInputPortRealSignalPtrs(S, 1);
    in_od = ssGetInputPortRealSignalPtrs(S, 5);

    xInit = mxGetPr( ssGetSFcnParam(S, 0) );
    zInit = mxGetPr( ssGetSFcnParam(S, 1) );
    uInit = mxGetPr( ssGetSFcnParam(S, 2) );

    for( i=0; i < ACADO_N+1; ++i ) {
        for( j=0; j < ACADO_NX; ++j ) acadoVariables.x[i*ACADO_NX+j] = xInit[j];
    }
    #if ACADO_NXA > 0
    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NXA; ++j ) acadoVariables.z[i*ACADO_NXA+j] = zInit[j];
    }
    #endif
    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NU; ++j ) acadoVariables.u[i*ACADO_NU+j] = uInit[j];
    }

    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NY; ++j ) acadoVariables.y[i*ACADO_NY+j] = (double)(*in_ref[i*ACADO_NY+j]);
    }
    for( i=0; i < ACADO_NYN; ++i ) acadoVariables.yN[i] = (double)(*in_ref[ACADO_N*ACADO_NY+i]);
    
    #if ACADO_NOD
    for( i=0; i < ACADO_N+1; ++i ) {
        for( j=0; j < ACADO_NOD; ++j ) acadoVariables.od[i*ACADO_NOD+j] = (double)(*in_od[i*ACADO_NOD+j]);
    }
    #endif
    
    Wmat = mxGetPr( ssGetSFcnParam(S, 3) );
    WNmat = mxGetPr( ssGetSFcnParam(S, 4) );
    bValues = mxGetPr( ssGetSFcnParam(S, 5) );
    
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
    
    #if VARYING_BOUNDS
    for( i = 0; i < QPOASES_NVMAX; ++i )  {
        acadoVariables.lbValues[i] = bValues[i];
        acadoVariables.ubValues[i] = bValues[QPOASES_NVMAX+QPOASES_NCMAX+i];
    }
    
    for( i = 0; i < QPOASES_NCMAX; ++i )  {
        acadoVariables.lbAValues[i] = bValues[QPOASES_NVMAX+i];
        acadoVariables.ubAValues[i] = bValues[QPOASES_NVMAX+QPOASES_NCMAX+QPOASES_NVMAX+i];
    }
    #endif

    acado_initializeSolver();
    acado_preparationStep( );
}

// double timeFdb, timePrep;

static void mdlOutputs(SimStruct *S, int_T tid)
{
    int i, j, status, sumIter;
//     timer t;
    real_t kkt;

    InputRealPtrsType in_x, in_ref, in_W, in_WN, in_bValues, in_od, in_nIter;
    real_t *out_u0, *out_xTraj, *out_uTraj, *out_kktTol, *out_cpuTime, *out_status, *out_nIter, *out_objVal, *xInit, *zInit, *uInit;
    
    #if RESET_WHEN_NAN == 1
        real_t x_prev[ (ACADO_N+1)*ACADO_NX ];
        #if ACADO_NXA > 0
        real_t z_prev[ ACADO_N*ACADO_NXA ];
        #endif
        real_t u_prev[ ACADO_N*ACADO_NU ];
    #endif
    
    out_u0     = ssGetOutputPortRealSignal(S, 0);
    out_xTraj  = ssGetOutputPortRealSignal(S, 1);
    out_uTraj  = ssGetOutputPortRealSignal(S, 2);
    out_status = ssGetOutputPortRealSignal(S, 3);
    out_kktTol = ssGetOutputPortRealSignal(S, 4);
    out_cpuTime = ssGetOutputPortRealSignal(S, 5);
    out_nIter = ssGetOutputPortRealSignal(S, 6);
    out_objVal = ssGetOutputPortRealSignal(S, 7);

    /* get inputs and perform feedback step */
    in_x    = ssGetInputPortRealSignalPtrs(S, 0);
    in_ref  = ssGetInputPortRealSignalPtrs(S, 1);
    in_W  = ssGetInputPortRealSignalPtrs(S, 2);
    in_WN  = ssGetInputPortRealSignalPtrs(S, 3);
    in_bValues  = ssGetInputPortRealSignalPtrs(S, 4);
    in_od   = ssGetInputPortRealSignalPtrs(S, 5);
    in_nIter   = ssGetInputPortRealSignalPtrs(S, 6);
    
    for( i=0; i < ACADO_NX; ++i ) acadoVariables.x0[i] = (double)(*in_x[i]);
    
    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NY; ++j ) acadoVariables.y[i*ACADO_NY+j] = (double)(*in_ref[i*ACADO_NY+j]);
    }
    for( i=0; i < ACADO_NYN; ++i ) acadoVariables.yN[i] = (double)(*in_ref[ACADO_N*ACADO_NY+i]);

    #if ACADO_NOD
    for( i=0; i < ACADO_N+1; ++i ) {
        for( j=0; j < ACADO_NOD; ++j ) acadoVariables.od[i*ACADO_NOD+j] = (double)(*in_od[i*ACADO_NOD+j]);
    }
    #endif
    
    for( i=0; i < ACADO_NY; ++i ) {
        for( j=0; j < ACADO_NY; ++j ) acadoVariables.W[i*ACADO_NY+j] = (double)(*in_W[i*ACADO_NY+j]);
    }
    for( i=0; i < ACADO_NYN; ++i ) {
        for( j=0; j < ACADO_NYN; ++j ) acadoVariables.WN[i*ACADO_NYN+j] = (double)(*in_WN[i*ACADO_NYN+j]);
    }
    
    #if VARYING_BOUNDS
    for( i = 0; i < QPOASES_NVMAX; ++i )  {
        acadoVariables.lbValues[i] = (double)(*in_bValues[i]);
        acadoVariables.ubValues[i] = (double)(*in_bValues[QPOASES_NVMAX+QPOASES_NCMAX+i]);
    }
    
    for( i = 0; i < QPOASES_NCMAX; ++i )  {
        acadoVariables.lbAValues[i] = (double)(*in_bValues[QPOASES_NVMAX+i]);
        acadoVariables.ubAValues[i] = (double)(*in_bValues[QPOASES_NVMAX+QPOASES_NCMAX+QPOASES_NVMAX+i]);
    }
    #endif
    
    #if RESET_WHEN_NAN == 1
        for( i = 0; i < (ACADO_N+1)*ACADO_NX; ++i ) x_prev[ i ] = acadoVariables.x[ i ];
        #if ACADO_NXA > 0
        for( i = 0; i < ACADO_N*ACADO_NXA; ++i ) z_prev[ i ] = acadoVariables.z[ i ];
        #endif
        for( i = 0; i < ACADO_N*ACADO_NU; ++i ) u_prev[ i ] = acadoVariables.u[ i ];
    #endif
        
//     tic( &t );
    status = acado_feedbackStep( );
    sumIter = (int)acado_getNWSR();     
    
    for( i = 1; i < (int)(*in_nIter[0]); i++ ) {
        acado_preparationStep( );
        status = acado_feedbackStep( );
        sumIter += (int)acado_getNWSR();     
    }
//     timeFdb = toc( &t );
    
    #if RESET_WHEN_NAN == 1
        kkt = acado_getKKT();
        if( kkt < KKT_MIN || kkt > KKT_MAX || kkt != kkt ) {
            for( i = 0; i < (ACADO_N+1)*ACADO_NX; ++i ) acadoVariables.x[ i ] = x_prev[ i ];
            #if ACADO_NXA > 0
            for( i = 0; i < ACADO_N*ACADO_NXA; ++i ) acadoVariables.z[ i ] = z_prev[ i ];
            #endif
            for( i = 0; i < ACADO_N*ACADO_NU; ++i ) acadoVariables.u[ i ] = u_prev[ i ];
            
            status = -30; // PERFORMED RESET TO PREVIOUS TRAJECTORIES BECAUSE OF NOTANUMBER
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
    
//     printf("Timing RTI iteration (MPC feedback, error value: %d):      %.3g ms   \n", status, timeFdb*1e3);

    /* return outputs and prepare next iteration */

    for( i=0; i < ACADO_NU; ++i ) out_u0[i] = acadoVariables.u[i];
    out_kktTol[0] = acado_getKKT( );
    out_status[0] = status;
    out_cpuTime[0] = -1;
    out_nIter[0] = (double) sumIter;
    out_objVal[0] = acado_getObjective( );
}


#define MDL_UPDATE 
static void mdlUpdate(SimStruct *S, int_T tid) 
{ 
//     timer t;
	real_t* xEnd = NULL;
	real_t* uEnd = NULL;
    
    // Shift the state and control trajectories:
 	acado_shiftStates(0, xEnd, uEnd);
 	acado_shiftControls(uEnd);
    
//     tic( &t );
    acado_preparationStep( );
//     timePrep = toc( &t );
    
//     printf("Timing RTI iteration (MPC preparation):                   %.3g ms   \n", timePrep*1e3);
//     printf("---------------------------------------------------------------\n");
} 


static void mdlTerminate(SimStruct *S)
{
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

