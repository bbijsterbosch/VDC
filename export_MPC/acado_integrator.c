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
const real_t* u = in + 7;
/* Vector of auxiliary variables; number of elements: 4. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[3]));
a[1] = (sin(xd[3]));
a[2] = (cos(xd[3]));
a[3] = (sin(xd[3]));

/* Compute outputs: */
out[0] = (xd[2]*xd[1]);
out[1] = (((((real_t)(-2.2463768115942028e+02)/xd[0])*xd[1])+(((real_t)(7.3231884057971016e+01)*xd[2])/xd[0]))+((real_t)(8.6956521739130437e+01)*xd[6]));
out[2] = ((((real_t)(3.8360220155627253e+01)/xd[0])*xd[1])-(((real_t)(2.2981725564623270e+02)*xd[2])/xd[0]));
out[3] = xd[2];
out[4] = ((a[0]*xd[0])-(a[1]*xd[1]));
out[5] = ((a[2]*xd[1])+(a[3]*xd[0]));
out[6] = u[0];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 16. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((real_t)(1.0000000000000000e+00)/xd[0]);
a[1] = (a[0]*a[0]);
a[2] = ((real_t)(1.0000000000000000e+00)/xd[0]);
a[3] = (a[2]*a[2]);
a[4] = ((real_t)(1.0000000000000000e+00)/xd[0]);
a[5] = (a[4]*a[4]);
a[6] = ((real_t)(1.0000000000000000e+00)/xd[0]);
a[7] = (a[6]*a[6]);
a[8] = (cos(xd[3]));
a[9] = (sin(xd[3]));
a[10] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[11] = (cos(xd[3]));
a[12] = (sin(xd[3]));
a[13] = (cos(xd[3]));
a[14] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[15] = (cos(xd[3]));

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = xd[2];
out[2] = xd[1];
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = ((((real_t)(0.0000000000000000e+00)-((real_t)(-2.2463768115942028e+02)*a[1]))*xd[1])+((real_t)(0.0000000000000000e+00)-(((real_t)(7.3231884057971016e+01)*xd[2])*a[3])));
out[9] = ((real_t)(-2.2463768115942028e+02)/xd[0]);
out[10] = ((real_t)(7.3231884057971016e+01)*a[2]);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(8.6956521739130437e+01);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = ((((real_t)(0.0000000000000000e+00)-((real_t)(3.8360220155627253e+01)*a[5]))*xd[1])-((real_t)(0.0000000000000000e+00)-(((real_t)(2.2981725564623270e+02)*xd[2])*a[7])));
out[17] = ((real_t)(3.8360220155627253e+01)/xd[0]);
out[18] = ((real_t)(0.0000000000000000e+00)-((real_t)(2.2981725564623270e+02)*a[6]));
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
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
out[32] = a[8];
out[33] = ((real_t)(0.0000000000000000e+00)-a[9]);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = ((a[10]*xd[0])-(a[11]*xd[1]));
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = a[12];
out[41] = a[13];
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = ((a[14]*xd[1])+(a[15]*xd[0]));
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(1.0000000000000000e+00);
}



void acado_solve_dim7_triangular( real_t* const A, real_t* const b )
{

b[6] = b[6]/A[48];
b[5] -= + A[41]*b[6];
b[5] = b[5]/A[40];
b[4] -= + A[34]*b[6];
b[4] -= + A[33]*b[5];
b[4] = b[4]/A[32];
b[3] -= + A[27]*b[6];
b[3] -= + A[26]*b[5];
b[3] -= + A[25]*b[4];
b[3] = b[3]/A[24];
b[2] -= + A[20]*b[6];
b[2] -= + A[19]*b[5];
b[2] -= + A[18]*b[4];
b[2] -= + A[17]*b[3];
b[2] = b[2]/A[16];
b[1] -= + A[13]*b[6];
b[1] -= + A[12]*b[5];
b[1] -= + A[11]*b[4];
b[1] -= + A[10]*b[3];
b[1] -= + A[9]*b[2];
b[1] = b[1]/A[8];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim7_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 7; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (6); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*7+i]);
	for( j=(i+1); j < 7; j++ ) {
		temp = fabs(A[j*7+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 7; ++k)
{
	acadoWorkspace.rk_dim7_swap = A[i*7+k];
	A[i*7+k] = A[indexMax*7+k];
	A[indexMax*7+k] = acadoWorkspace.rk_dim7_swap;
}
	acadoWorkspace.rk_dim7_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim7_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*7+i];
	for( j=i+1; j < 7; j++ ) {
		A[j*7+i] = -A[j*7+i]/A[i*7+i];
		for( k=i+1; k < 7; k++ ) {
			A[j*7+k] += A[j*7+i] * A[i*7+k];
		}
		b[j] += A[j*7+i] * b[i];
	}
}
det *= A[48];
det = fabs(det);
acado_solve_dim7_triangular( A, b );
return det;
}

void acado_solve_dim7_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim7_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim7_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim7_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim7_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim7_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim7_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim7_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim7_bPerm[1] += A[7]*acadoWorkspace.rk_dim7_bPerm[0];

acadoWorkspace.rk_dim7_bPerm[2] += A[14]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[2] += A[15]*acadoWorkspace.rk_dim7_bPerm[1];

acadoWorkspace.rk_dim7_bPerm[3] += A[21]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[3] += A[22]*acadoWorkspace.rk_dim7_bPerm[1];
acadoWorkspace.rk_dim7_bPerm[3] += A[23]*acadoWorkspace.rk_dim7_bPerm[2];

acadoWorkspace.rk_dim7_bPerm[4] += A[28]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[4] += A[29]*acadoWorkspace.rk_dim7_bPerm[1];
acadoWorkspace.rk_dim7_bPerm[4] += A[30]*acadoWorkspace.rk_dim7_bPerm[2];
acadoWorkspace.rk_dim7_bPerm[4] += A[31]*acadoWorkspace.rk_dim7_bPerm[3];

acadoWorkspace.rk_dim7_bPerm[5] += A[35]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[5] += A[36]*acadoWorkspace.rk_dim7_bPerm[1];
acadoWorkspace.rk_dim7_bPerm[5] += A[37]*acadoWorkspace.rk_dim7_bPerm[2];
acadoWorkspace.rk_dim7_bPerm[5] += A[38]*acadoWorkspace.rk_dim7_bPerm[3];
acadoWorkspace.rk_dim7_bPerm[5] += A[39]*acadoWorkspace.rk_dim7_bPerm[4];

acadoWorkspace.rk_dim7_bPerm[6] += A[42]*acadoWorkspace.rk_dim7_bPerm[0];
acadoWorkspace.rk_dim7_bPerm[6] += A[43]*acadoWorkspace.rk_dim7_bPerm[1];
acadoWorkspace.rk_dim7_bPerm[6] += A[44]*acadoWorkspace.rk_dim7_bPerm[2];
acadoWorkspace.rk_dim7_bPerm[6] += A[45]*acadoWorkspace.rk_dim7_bPerm[3];
acadoWorkspace.rk_dim7_bPerm[6] += A[46]*acadoWorkspace.rk_dim7_bPerm[4];
acadoWorkspace.rk_dim7_bPerm[6] += A[47]*acadoWorkspace.rk_dim7_bPerm[5];


acado_solve_dim7_triangular( A, acadoWorkspace.rk_dim7_bPerm );
b[0] = acadoWorkspace.rk_dim7_bPerm[0];
b[1] = acadoWorkspace.rk_dim7_bPerm[1];
b[2] = acadoWorkspace.rk_dim7_bPerm[2];
b[3] = acadoWorkspace.rk_dim7_bPerm[3];
b[4] = acadoWorkspace.rk_dim7_bPerm[4];
b[5] = acadoWorkspace.rk_dim7_bPerm[5];
b[6] = acadoWorkspace.rk_dim7_bPerm[6];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 1.6666666666666668e-03 };


/* Fixed step size:0.00333333 */
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
acadoWorkspace.rk_xxx[7] = rk_eta[63];

for (run = 0; run < 3; ++run)
{
if( run > 0 ) {
for (i = 0; i < 7; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 8] = rk_eta[i * 7 + 7];
acadoWorkspace.rk_diffsPrev2[i * 8 + 1] = rk_eta[i * 7 + 8];
acadoWorkspace.rk_diffsPrev2[i * 8 + 2] = rk_eta[i * 7 + 9];
acadoWorkspace.rk_diffsPrev2[i * 8 + 3] = rk_eta[i * 7 + 10];
acadoWorkspace.rk_diffsPrev2[i * 8 + 4] = rk_eta[i * 7 + 11];
acadoWorkspace.rk_diffsPrev2[i * 8 + 5] = rk_eta[i * 7 + 12];
acadoWorkspace.rk_diffsPrev2[i * 8 + 6] = rk_eta[i * 7 + 13];
acadoWorkspace.rk_diffsPrev2[i * 8 + 7] = rk_eta[i + 56];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 56 ]) );
for (j = 0; j < 7; ++j)
{
tmp_index1 = (run1 * 7) + (j);
acadoWorkspace.rk_A[tmp_index1 * 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 6)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 7) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 7] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 7 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 7 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 7 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 7 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 7 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 7 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
}
det = acado_solve_dim7_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 7];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 7 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 7 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 7 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 7 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 7 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 7 + 6];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 7] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 7 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 7 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 7 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 7 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 7 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 7 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
}
acado_solve_dim7_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 7];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 7 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 7 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 7 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 7 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 7 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 7 + 6];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 7; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 56 ]) );
for (j = 0; j < 7; ++j)
{
tmp_index1 = (run1 * 7) + (j);
acadoWorkspace.rk_A[tmp_index1 * 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 7 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 56) + (j * 8 + 6)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 7) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 7; ++run1)
{
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_b[i * 7] = - acadoWorkspace.rk_diffsTemp2[(i * 56) + (run1)];
acadoWorkspace.rk_b[i * 7 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 56) + (run1 + 8)];
acadoWorkspace.rk_b[i * 7 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 56) + (run1 + 16)];
acadoWorkspace.rk_b[i * 7 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 56) + (run1 + 24)];
acadoWorkspace.rk_b[i * 7 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 56) + (run1 + 32)];
acadoWorkspace.rk_b[i * 7 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 56) + (run1 + 40)];
acadoWorkspace.rk_b[i * 7 + 6] = - acadoWorkspace.rk_diffsTemp2[(i * 56) + (run1 + 48)];
}
if( 0 == run1 ) {
det = acado_solve_dim7_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
}
 else {
acado_solve_dim7_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 7];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 7 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 7 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 7 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 7 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 7 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 7 + 6];
}
for (i = 0; i < 7; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 8) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 8) + (run1)] += + acadoWorkspace.rk_diffK[i]*(real_t)3.3333333333333335e-03;
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index1 = (i * 7) + (j);
tmp_index2 = (run1) + (j * 8);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 56) + (tmp_index2 + 7)];
}
}
acado_solve_dim7_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim7_perm );
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 7];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 7 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 7 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 7 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 7 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 7 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 7 + 6];
}
for (i = 0; i < 7; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 8) + (run1 + 7)] = + acadoWorkspace.rk_diffK[i]*(real_t)3.3333333333333335e-03;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)3.3333333333333335e-03;
rk_eta[1] += + acadoWorkspace.rk_kkk[1]*(real_t)3.3333333333333335e-03;
rk_eta[2] += + acadoWorkspace.rk_kkk[2]*(real_t)3.3333333333333335e-03;
rk_eta[3] += + acadoWorkspace.rk_kkk[3]*(real_t)3.3333333333333335e-03;
rk_eta[4] += + acadoWorkspace.rk_kkk[4]*(real_t)3.3333333333333335e-03;
rk_eta[5] += + acadoWorkspace.rk_kkk[5]*(real_t)3.3333333333333335e-03;
rk_eta[6] += + acadoWorkspace.rk_kkk[6]*(real_t)3.3333333333333335e-03;
if( run == 0 ) {
for (i = 0; i < 7; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index2 = (j) + (i * 7);
rk_eta[tmp_index2 + 7] = acadoWorkspace.rk_diffsNew2[(i * 8) + (j)];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 56] = acadoWorkspace.rk_diffsNew2[(i * 8) + (j + 7)];
}
}
}
else {
for (i = 0; i < 7; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index2 = (j) + (i * 7);
rk_eta[tmp_index2 + 7] = + acadoWorkspace.rk_diffsNew2[i * 8]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 8 + 1]*acadoWorkspace.rk_diffsPrev2[j + 8];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 8 + 2]*acadoWorkspace.rk_diffsPrev2[j + 16];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 8 + 3]*acadoWorkspace.rk_diffsPrev2[j + 24];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 8 + 4]*acadoWorkspace.rk_diffsPrev2[j + 32];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 8 + 5]*acadoWorkspace.rk_diffsPrev2[j + 40];
rk_eta[tmp_index2 + 7] += + acadoWorkspace.rk_diffsNew2[i * 8 + 6]*acadoWorkspace.rk_diffsPrev2[j + 48];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 56] = acadoWorkspace.rk_diffsNew2[(i * 8) + (j + 7)];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 8]*acadoWorkspace.rk_diffsPrev2[j + 7];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 8 + 1]*acadoWorkspace.rk_diffsPrev2[j + 15];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 8 + 2]*acadoWorkspace.rk_diffsPrev2[j + 23];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 8 + 3]*acadoWorkspace.rk_diffsPrev2[j + 31];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 8 + 4]*acadoWorkspace.rk_diffsPrev2[j + 39];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 8 + 5]*acadoWorkspace.rk_diffsPrev2[j + 47];
rk_eta[tmp_index2 + 56] += + acadoWorkspace.rk_diffsNew2[i * 8 + 6]*acadoWorkspace.rk_diffsPrev2[j + 55];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 3.3333333333333331e-01;
}
for (i = 0; i < 7; ++i)
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



