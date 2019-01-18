/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 8;

/* Compute outputs: */
out[0] = ((real_t)(5.0000000000000000e-01)*u[0]);
out[1] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[2] = xd[4];
out[3] = xd[5];
out[4] = ((real_t)(1.3710000000000001e+01)*xd[1]);
out[5] = ((real_t)(-1.3710000000000001e+01)*xd[0]);
out[6] = xd[7];
out[7] = u[2];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(5.0000000000000000e-01);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(5.0000000000000000e-01);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(1.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(1.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(1.3710000000000001e+01);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(-1.3710000000000001e+01);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(1.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(1.0000000000000000e+00);
}



void acado_solve_dim8_triangular( real_t* const A, real_t* const b )
{

b[7] = b[7]/A[63];
b[6] -= + A[55]*b[7];
b[6] = b[6]/A[54];
b[5] -= + A[47]*b[7];
b[5] -= + A[46]*b[6];
b[5] = b[5]/A[45];
b[4] -= + A[39]*b[7];
b[4] -= + A[38]*b[6];
b[4] -= + A[37]*b[5];
b[4] = b[4]/A[36];
b[3] -= + A[31]*b[7];
b[3] -= + A[30]*b[6];
b[3] -= + A[29]*b[5];
b[3] -= + A[28]*b[4];
b[3] = b[3]/A[27];
b[2] -= + A[23]*b[7];
b[2] -= + A[22]*b[6];
b[2] -= + A[21]*b[5];
b[2] -= + A[20]*b[4];
b[2] -= + A[19]*b[3];
b[2] = b[2]/A[18];
b[1] -= + A[15]*b[7];
b[1] -= + A[14]*b[6];
b[1] -= + A[13]*b[5];
b[1] -= + A[12]*b[4];
b[1] -= + A[11]*b[3];
b[1] -= + A[10]*b[2];
b[1] = b[1]/A[9];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim8_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 8; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (7); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*8+i]);
	for( j=(i+1); j < 8; j++ ) {
		temp = fabs(A[j*8+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 8; ++k)
{
	acadoWorkspace.rk_dim8_swap = A[i*8+k];
	A[i*8+k] = A[indexMax*8+k];
	A[indexMax*8+k] = acadoWorkspace.rk_dim8_swap;
}
	acadoWorkspace.rk_dim8_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim8_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*8+i];
	for( j=i+1; j < 8; j++ ) {
		A[j*8+i] = -A[j*8+i]/A[i*8+i];
		for( k=i+1; k < 8; k++ ) {
			A[j*8+k] += A[j*8+i] * A[i*8+k];
		}
		b[j] += A[j*8+i] * b[i];
	}
}
det *= A[63];
det = fabs(det);
acado_solve_dim8_triangular( A, b );
return det;
}

void acado_solve_dim8_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim8_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim8_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim8_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim8_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim8_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim8_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim8_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim8_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim8_bPerm[1] += A[8]*acadoWorkspace.rk_dim8_bPerm[0];

acadoWorkspace.rk_dim8_bPerm[2] += A[16]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[2] += A[17]*acadoWorkspace.rk_dim8_bPerm[1];

acadoWorkspace.rk_dim8_bPerm[3] += A[24]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[3] += A[25]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[3] += A[26]*acadoWorkspace.rk_dim8_bPerm[2];

acadoWorkspace.rk_dim8_bPerm[4] += A[32]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[4] += A[33]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[4] += A[34]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[4] += A[35]*acadoWorkspace.rk_dim8_bPerm[3];

acadoWorkspace.rk_dim8_bPerm[5] += A[40]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[5] += A[41]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[5] += A[42]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[5] += A[43]*acadoWorkspace.rk_dim8_bPerm[3];
acadoWorkspace.rk_dim8_bPerm[5] += A[44]*acadoWorkspace.rk_dim8_bPerm[4];

acadoWorkspace.rk_dim8_bPerm[6] += A[48]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[6] += A[49]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[6] += A[50]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[6] += A[51]*acadoWorkspace.rk_dim8_bPerm[3];
acadoWorkspace.rk_dim8_bPerm[6] += A[52]*acadoWorkspace.rk_dim8_bPerm[4];
acadoWorkspace.rk_dim8_bPerm[6] += A[53]*acadoWorkspace.rk_dim8_bPerm[5];

acadoWorkspace.rk_dim8_bPerm[7] += A[56]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[7] += A[57]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[7] += A[58]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[7] += A[59]*acadoWorkspace.rk_dim8_bPerm[3];
acadoWorkspace.rk_dim8_bPerm[7] += A[60]*acadoWorkspace.rk_dim8_bPerm[4];
acadoWorkspace.rk_dim8_bPerm[7] += A[61]*acadoWorkspace.rk_dim8_bPerm[5];
acadoWorkspace.rk_dim8_bPerm[7] += A[62]*acadoWorkspace.rk_dim8_bPerm[6];


acado_solve_dim8_triangular( A, acadoWorkspace.rk_dim8_bPerm );
b[0] = acadoWorkspace.rk_dim8_bPerm[0];
b[1] = acadoWorkspace.rk_dim8_bPerm[1];
b[2] = acadoWorkspace.rk_dim8_bPerm[2];
b[3] = acadoWorkspace.rk_dim8_bPerm[3];
b[4] = acadoWorkspace.rk_dim8_bPerm[4];
b[5] = acadoWorkspace.rk_dim8_bPerm[5];
b[6] = acadoWorkspace.rk_dim8_bPerm[6];
b[7] = acadoWorkspace.rk_dim8_bPerm[7];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 1.2500000000000001e-02 };


/* Fixed step size:0.025 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[8] = rk_eta[96];
acadoWorkspace.rk_xxx[9] = rk_eta[97];
acadoWorkspace.rk_xxx[10] = rk_eta[98];
acadoWorkspace.rk_xxx[11] = rk_eta[99];
acadoWorkspace.rk_xxx[12] = rk_eta[100];
acadoWorkspace.rk_xxx[13] = rk_eta[101];
acadoWorkspace.rk_xxx[14] = rk_eta[102];
acadoWorkspace.rk_xxx[15] = rk_eta[103];
acadoWorkspace.rk_xxx[16] = rk_eta[104];
acadoWorkspace.rk_xxx[17] = rk_eta[105];
acadoWorkspace.rk_xxx[18] = rk_eta[106];
acadoWorkspace.rk_xxx[19] = rk_eta[107];
acadoWorkspace.rk_xxx[20] = rk_eta[108];
acadoWorkspace.rk_xxx[21] = rk_eta[109];
acadoWorkspace.rk_xxx[22] = rk_eta[110];
acadoWorkspace.rk_xxx[23] = rk_eta[111];
acadoWorkspace.rk_xxx[24] = rk_eta[112];
acadoWorkspace.rk_xxx[25] = rk_eta[113];
acadoWorkspace.rk_xxx[26] = rk_eta[114];
acadoWorkspace.rk_xxx[27] = rk_eta[115];
acadoWorkspace.rk_xxx[28] = rk_eta[116];
acadoWorkspace.rk_xxx[29] = rk_eta[117];
acadoWorkspace.rk_xxx[30] = rk_eta[118];
acadoWorkspace.rk_xxx[31] = rk_eta[119];
acadoWorkspace.rk_xxx[32] = rk_eta[120];
acadoWorkspace.rk_xxx[33] = rk_eta[121];

for (run = 0; run < 2; ++run)
{
if( run > 0 ) {
for (i = 0; i < 8; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 11] = rk_eta[i * 8 + 8];
acadoWorkspace.rk_diffsPrev2[i * 11 + 1] = rk_eta[i * 8 + 9];
acadoWorkspace.rk_diffsPrev2[i * 11 + 2] = rk_eta[i * 8 + 10];
acadoWorkspace.rk_diffsPrev2[i * 11 + 3] = rk_eta[i * 8 + 11];
acadoWorkspace.rk_diffsPrev2[i * 11 + 4] = rk_eta[i * 8 + 12];
acadoWorkspace.rk_diffsPrev2[i * 11 + 5] = rk_eta[i * 8 + 13];
acadoWorkspace.rk_diffsPrev2[i * 11 + 6] = rk_eta[i * 8 + 14];
acadoWorkspace.rk_diffsPrev2[i * 11 + 7] = rk_eta[i * 8 + 15];
acadoWorkspace.rk_diffsPrev2[i * 11 + 8] = rk_eta[i * 3 + 72];
acadoWorkspace.rk_diffsPrev2[i * 11 + 9] = rk_eta[i * 3 + 73];
acadoWorkspace.rk_diffsPrev2[i * 11 + 10] = rk_eta[i * 3 + 74];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 8; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 88 ]) );
for (j = 0; j < 8; ++j)
{
tmp_index1 = (run1 * 8) + (j);
acadoWorkspace.rk_A[tmp_index1 * 8] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 7)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 8] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 8 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 8 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 8 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 8 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 8 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 8 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 8 + 7] = acadoWorkspace.rk_kkk[run1 + 7] - acadoWorkspace.rk_rhsTemp[7];
}
det = acado_solve_dim8_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 8];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 8 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 8 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 8 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 8 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 8 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 8 + 6];
acadoWorkspace.rk_kkk[j + 7] += acadoWorkspace.rk_b[j * 8 + 7];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 8; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 8] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 8 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 8 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 8 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 8 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 8 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 8 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 8 + 7] = acadoWorkspace.rk_kkk[run1 + 7] - acadoWorkspace.rk_rhsTemp[7];
}
acado_solve_dim8_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 8];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 8 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 8 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 8 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 8 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 8 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 8 + 6];
acadoWorkspace.rk_kkk[j + 7] += acadoWorkspace.rk_b[j * 8 + 7];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 8; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 88 ]) );
for (j = 0; j < 8; ++j)
{
tmp_index1 = (run1 * 8) + (j);
acadoWorkspace.rk_A[tmp_index1 * 8] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 88) + (j * 11 + 7)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 8; ++run1)
{
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_b[i * 8] = - acadoWorkspace.rk_diffsTemp2[(i * 88) + (run1)];
acadoWorkspace.rk_b[i * 8 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 88) + (run1 + 11)];
acadoWorkspace.rk_b[i * 8 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 88) + (run1 + 22)];
acadoWorkspace.rk_b[i * 8 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 88) + (run1 + 33)];
acadoWorkspace.rk_b[i * 8 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 88) + (run1 + 44)];
acadoWorkspace.rk_b[i * 8 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 88) + (run1 + 55)];
acadoWorkspace.rk_b[i * 8 + 6] = - acadoWorkspace.rk_diffsTemp2[(i * 88) + (run1 + 66)];
acadoWorkspace.rk_b[i * 8 + 7] = - acadoWorkspace.rk_diffsTemp2[(i * 88) + (run1 + 77)];
}
if( 0 == run1 ) {
det = acado_solve_dim8_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
}
 else {
acado_solve_dim8_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 8];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 8 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 8 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 8 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 8 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 8 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 8 + 6];
acadoWorkspace.rk_diffK[i + 7] = acadoWorkspace.rk_b[i * 8 + 7];
}
for (i = 0; i < 8; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 11) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 11) + (run1)] += + acadoWorkspace.rk_diffK[i]*(real_t)2.5000000000000001e-02;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 8; ++j)
{
tmp_index1 = (i * 8) + (j);
tmp_index2 = (run1) + (j * 11);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 88) + (tmp_index2 + 8)];
}
}
acado_solve_dim8_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 8];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 8 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 8 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 8 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 8 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 8 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 8 + 6];
acadoWorkspace.rk_diffK[i + 7] = acadoWorkspace.rk_b[i * 8 + 7];
}
for (i = 0; i < 8; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 11) + (run1 + 8)] = + acadoWorkspace.rk_diffK[i]*(real_t)2.5000000000000001e-02;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)2.5000000000000001e-02;
rk_eta[1] += + acadoWorkspace.rk_kkk[1]*(real_t)2.5000000000000001e-02;
rk_eta[2] += + acadoWorkspace.rk_kkk[2]*(real_t)2.5000000000000001e-02;
rk_eta[3] += + acadoWorkspace.rk_kkk[3]*(real_t)2.5000000000000001e-02;
rk_eta[4] += + acadoWorkspace.rk_kkk[4]*(real_t)2.5000000000000001e-02;
rk_eta[5] += + acadoWorkspace.rk_kkk[5]*(real_t)2.5000000000000001e-02;
rk_eta[6] += + acadoWorkspace.rk_kkk[6]*(real_t)2.5000000000000001e-02;
rk_eta[7] += + acadoWorkspace.rk_kkk[7]*(real_t)2.5000000000000001e-02;
if( run == 0 ) {
for (i = 0; i < 8; ++i)
{
for (j = 0; j < 8; ++j)
{
tmp_index2 = (j) + (i * 8);
rk_eta[tmp_index2 + 8] = acadoWorkspace.rk_diffsNew2[(i * 11) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 72] = acadoWorkspace.rk_diffsNew2[(i * 11) + (j + 8)];
}
}
}
else {
for (i = 0; i < 8; ++i)
{
for (j = 0; j < 8; ++j)
{
tmp_index2 = (j) + (i * 8);
rk_eta[tmp_index2 + 8] = + acadoWorkspace.rk_diffsNew2[i * 11]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 8] += + acadoWorkspace.rk_diffsNew2[i * 11 + 1]*acadoWorkspace.rk_diffsPrev2[j + 11];
rk_eta[tmp_index2 + 8] += + acadoWorkspace.rk_diffsNew2[i * 11 + 2]*acadoWorkspace.rk_diffsPrev2[j + 22];
rk_eta[tmp_index2 + 8] += + acadoWorkspace.rk_diffsNew2[i * 11 + 3]*acadoWorkspace.rk_diffsPrev2[j + 33];
rk_eta[tmp_index2 + 8] += + acadoWorkspace.rk_diffsNew2[i * 11 + 4]*acadoWorkspace.rk_diffsPrev2[j + 44];
rk_eta[tmp_index2 + 8] += + acadoWorkspace.rk_diffsNew2[i * 11 + 5]*acadoWorkspace.rk_diffsPrev2[j + 55];
rk_eta[tmp_index2 + 8] += + acadoWorkspace.rk_diffsNew2[i * 11 + 6]*acadoWorkspace.rk_diffsPrev2[j + 66];
rk_eta[tmp_index2 + 8] += + acadoWorkspace.rk_diffsNew2[i * 11 + 7]*acadoWorkspace.rk_diffsPrev2[j + 77];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 72] = acadoWorkspace.rk_diffsNew2[(i * 11) + (j + 8)];
rk_eta[tmp_index2 + 72] += + acadoWorkspace.rk_diffsNew2[i * 11]*acadoWorkspace.rk_diffsPrev2[j + 8];
rk_eta[tmp_index2 + 72] += + acadoWorkspace.rk_diffsNew2[i * 11 + 1]*acadoWorkspace.rk_diffsPrev2[j + 19];
rk_eta[tmp_index2 + 72] += + acadoWorkspace.rk_diffsNew2[i * 11 + 2]*acadoWorkspace.rk_diffsPrev2[j + 30];
rk_eta[tmp_index2 + 72] += + acadoWorkspace.rk_diffsNew2[i * 11 + 3]*acadoWorkspace.rk_diffsPrev2[j + 41];
rk_eta[tmp_index2 + 72] += + acadoWorkspace.rk_diffsNew2[i * 11 + 4]*acadoWorkspace.rk_diffsPrev2[j + 52];
rk_eta[tmp_index2 + 72] += + acadoWorkspace.rk_diffsNew2[i * 11 + 5]*acadoWorkspace.rk_diffsPrev2[j + 63];
rk_eta[tmp_index2 + 72] += + acadoWorkspace.rk_diffsNew2[i * 11 + 6]*acadoWorkspace.rk_diffsPrev2[j + 74];
rk_eta[tmp_index2 + 72] += + acadoWorkspace.rk_diffsNew2[i * 11 + 7]*acadoWorkspace.rk_diffsPrev2[j + 85];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 5.0000000000000000e-01;
}
for (i = 0; i < 8; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



