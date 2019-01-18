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

namespace MPC {

    static float inf = std::numeric_limits<float>::infinity();

class MPC
{
	public:
		static const unsigned int HorizonLength = ACADO_N;		// 'WNmat' needs to be a double matrix of size [ACADO_NY x ACADO_NY]
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
		MPC();
		~MPC();

		void Initialize();
		void Step();
};
	
}
	
#endif
