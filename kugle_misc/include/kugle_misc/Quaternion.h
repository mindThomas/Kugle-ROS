/* Copyright (C) 2018 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  thomasj@tkjelectronics.com
 * ------------------------------------------
 */
 
#ifndef MISC_QUATERNION_H
#define MISC_QUATERNION_H

#include <stddef.h>
#include <stdlib.h>

#define deg2rad(x) (M_PI*x/180.f)
#define rad2deg(x) (180.f*x/M_PI)
#define devec(x) (&x[1]) // devectorize from dimension 4 to dimension 3 (take bottom 3 elements) - works only on vector (double array)

// Quaternion class for computations with quaternions of the format
//   q = {q0, q1, q2, q3} = {w, x, y, z} = {w, v}
// Hence a 4-dimensional vector where the scalar value is first and the vector part are the 3 bottom elements
//   w = q0
//   v = {q1, q2, q3}

namespace kugle_misc {

class Quaternion
{

public:
	Quaternion();	
	Quaternion(const double q[4]);
	Quaternion(double q0, double q1, double q2, double q3);
	~Quaternion();

private:
	union q
	{
		double q[4];
		struct {
			double scalar;
			double vector[3];
		};
	};		
};
	

extern void Quaternion_Phi(const double q[4], const double p[4], double result[4]); // result = q o p = Phi(q)*p
extern void Quaternion_devecPhi(const double q[4], const double p[4], double result[3]); // result = V * q o p = V*Phi(q)*p
extern void Quaternion_PhiT(const double q[4], const double p[4], double result[4]); // result = q* o p = Phi(q)^T*p
extern void Quaternion_devecPhiT(const double q[4], const double p[4], double result[3]); // result = V * q* o p = V*Phi(q)^T*p
extern void Quaternion_mat_PhiT(const double q[4], double mat[4*4]);
extern void Quaternion_mat_devecPhiT(const double q[4], double mat[3*4]);
extern void Quaternion_Gamma(const double p[4], const double q[4], double result[4]); // result = q o p = Gamma(p)*q
extern void Quaternion_GammaT(const double p[4], const double q[4], double result[4]); // result = q o p* = Gamma(p)^T*q
extern void Quaternion_mat_GammaT(const double p[4], double mat[4*4]); // mat = Gamma(p)'
extern void Quaternion_mat_devecGammaT(const double p[4], double mat[3*4]); // mat = devec*Gamma(p)'
extern void Quaternion_Conjugate(const double q[4], double result[4]); // result = q*
extern void Quaternion_Conjugate(double q[4]); // q = q*
extern void Quaternion_Print(const double q[4]);
extern void Quaternion_Normalize(const double q[4], double q_out[4]);
extern void Quaternion_Normalize(double q[4]);
extern void Quaternion_eul2quat_zyx(const double yaw, const double pitch, const double roll, double q[4]);
extern void Quaternion_quat2eul_zyx(const double q[4], double yaw_pitch_roll[3]);
extern void Quaternion_AngleClamp(const double q[4], const double angleMax, double q_clamped[4]);
extern void Quaternion_Integration_Body(const double q[4], const double omega_body[3], const double dt, double q_out[4]);
extern void Quaternion_Integration_Inertial(const double q[4], const double omega_inertial[3], const double dt, double q_out[4]);

extern void HeadingIndependentReferenceManual(const double q_ref[4], const double q[4], double q_ref_out[4]);
extern void HeadingIndependentQdot(const double dq[4], const double q[4], double q_dot_out[4]);

extern double invSqrt(double x);

} // end namespace kugle_misc

#endif
