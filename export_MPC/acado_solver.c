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




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 7];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 7 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 7 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 7 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 7 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 7 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 7 + 6];

acadoWorkspace.state[63] = acadoVariables.u[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 7] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 7 + 7];
acadoWorkspace.d[lRun1 * 7 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 7 + 8];
acadoWorkspace.d[lRun1 * 7 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 7 + 9];
acadoWorkspace.d[lRun1 * 7 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 7 + 10];
acadoWorkspace.d[lRun1 * 7 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 7 + 11];
acadoWorkspace.d[lRun1 * 7 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 7 + 12];
acadoWorkspace.d[lRun1 * 7 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 7 + 13];

acadoWorkspace.evGx[lRun1 * 49] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 49 + 1] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 49 + 2] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 49 + 3] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 49 + 4] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 49 + 5] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 49 + 6] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 49 + 7] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 49 + 8] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 49 + 9] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 49 + 10] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 49 + 11] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 49 + 12] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 49 + 13] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 49 + 14] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 49 + 15] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 49 + 16] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 49 + 17] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 49 + 18] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 49 + 19] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 49 + 20] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 49 + 21] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 49 + 22] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 49 + 23] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 49 + 24] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 49 + 25] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 49 + 26] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 49 + 27] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 49 + 28] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 49 + 29] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 49 + 30] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 49 + 31] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 49 + 32] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 49 + 33] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 49 + 34] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 49 + 35] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 49 + 36] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 49 + 37] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 49 + 38] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 49 + 39] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 49 + 40] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 49 + 41] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 49 + 42] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 49 + 43] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 49 + 44] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 49 + 45] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 49 + 46] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 49 + 47] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 49 + 48] = acadoWorkspace.state[55];

acadoWorkspace.evGu[lRun1 * 7] = acadoWorkspace.state[56];
acadoWorkspace.evGu[lRun1 * 7 + 1] = acadoWorkspace.state[57];
acadoWorkspace.evGu[lRun1 * 7 + 2] = acadoWorkspace.state[58];
acadoWorkspace.evGu[lRun1 * 7 + 3] = acadoWorkspace.state[59];
acadoWorkspace.evGu[lRun1 * 7 + 4] = acadoWorkspace.state[60];
acadoWorkspace.evGu[lRun1 * 7 + 5] = acadoWorkspace.state[61];
acadoWorkspace.evGu[lRun1 * 7 + 6] = acadoWorkspace.state[62];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 7;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 14. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (tan(xd[6]));
a[1] = (atan(((real_t)(5.0394265232974911e-01)*a[0])));
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = ((real_t)(1.0000000000000000e+00)/(pow((cos(xd[6])),2)));
a[9] = (real_t)(5.0394265232974911e-01);
a[10] = (a[8]*a[9]);
a[11] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow(((real_t)(5.0394265232974911e-01)*a[0]),2))));
a[12] = (a[10]*a[11]);
a[13] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[1];
out[1] = a[2];
out[2] = a[3];
out[3] = a[4];
out[4] = a[5];
out[5] = a[6];
out[6] = a[7];
out[7] = a[12];
out[8] = a[13];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[8];
tmpQ1[8] = + tmpQ2[9];
tmpQ1[9] = + tmpQ2[10];
tmpQ1[10] = + tmpQ2[11];
tmpQ1[11] = + tmpQ2[12];
tmpQ1[12] = + tmpQ2[13];
tmpQ1[13] = + tmpQ2[14];
tmpQ1[14] = + tmpQ2[16];
tmpQ1[15] = + tmpQ2[17];
tmpQ1[16] = + tmpQ2[18];
tmpQ1[17] = + tmpQ2[19];
tmpQ1[18] = + tmpQ2[20];
tmpQ1[19] = + tmpQ2[21];
tmpQ1[20] = + tmpQ2[22];
tmpQ1[21] = + tmpQ2[24];
tmpQ1[22] = + tmpQ2[25];
tmpQ1[23] = + tmpQ2[26];
tmpQ1[24] = + tmpQ2[27];
tmpQ1[25] = + tmpQ2[28];
tmpQ1[26] = + tmpQ2[29];
tmpQ1[27] = + tmpQ2[30];
tmpQ1[28] = + tmpQ2[32];
tmpQ1[29] = + tmpQ2[33];
tmpQ1[30] = + tmpQ2[34];
tmpQ1[31] = + tmpQ2[35];
tmpQ1[32] = + tmpQ2[36];
tmpQ1[33] = + tmpQ2[37];
tmpQ1[34] = + tmpQ2[38];
tmpQ1[35] = + tmpQ2[40];
tmpQ1[36] = + tmpQ2[41];
tmpQ1[37] = + tmpQ2[42];
tmpQ1[38] = + tmpQ2[43];
tmpQ1[39] = + tmpQ2[44];
tmpQ1[40] = + tmpQ2[45];
tmpQ1[41] = + tmpQ2[46];
tmpQ1[42] = + tmpQ2[48];
tmpQ1[43] = + tmpQ2[49];
tmpQ1[44] = + tmpQ2[50];
tmpQ1[45] = + tmpQ2[51];
tmpQ1[46] = + tmpQ2[52];
tmpQ1[47] = + tmpQ2[53];
tmpQ1[48] = + tmpQ2[54];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[56];
tmpR2[1] = +tmpObjS[57];
tmpR2[2] = +tmpObjS[58];
tmpR2[3] = +tmpObjS[59];
tmpR2[4] = +tmpObjS[60];
tmpR2[5] = +tmpObjS[61];
tmpR2[6] = +tmpObjS[62];
tmpR2[7] = +tmpObjS[63];
tmpR1[0] = + tmpR2[7];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = +tmpObjSEndTerm[36];
tmpQN2[37] = +tmpObjSEndTerm[37];
tmpQN2[38] = +tmpObjSEndTerm[38];
tmpQN2[39] = +tmpObjSEndTerm[39];
tmpQN2[40] = +tmpObjSEndTerm[40];
tmpQN2[41] = +tmpObjSEndTerm[41];
tmpQN2[42] = +tmpObjSEndTerm[42];
tmpQN2[43] = +tmpObjSEndTerm[43];
tmpQN2[44] = +tmpObjSEndTerm[44];
tmpQN2[45] = +tmpObjSEndTerm[45];
tmpQN2[46] = +tmpObjSEndTerm[46];
tmpQN2[47] = +tmpObjSEndTerm[47];
tmpQN2[48] = +tmpObjSEndTerm[48];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
tmpQN1[36] = + tmpQN2[36];
tmpQN1[37] = + tmpQN2[37];
tmpQN1[38] = + tmpQN2[38];
tmpQN1[39] = + tmpQN2[39];
tmpQN1[40] = + tmpQN2[40];
tmpQN1[41] = + tmpQN2[41];
tmpQN1[42] = + tmpQN2[42];
tmpQN1[43] = + tmpQN2[43];
tmpQN1[44] = + tmpQN2[44];
tmpQN1[45] = + tmpQN2[45];
tmpQN1[46] = + tmpQN2[46];
tmpQN1[47] = + tmpQN2[47];
tmpQN1[48] = + tmpQN2[48];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 40; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 7];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 7 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 7 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 7 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 7 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 7 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 7 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 8] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 8 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 8 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 8 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 8 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 8 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 8 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 8 + 7] = acadoWorkspace.objValueOut[7];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 49 ]), &(acadoWorkspace.Q2[ runObj * 56 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj ]), &(acadoWorkspace.R2[ runObj * 8 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[280];
acadoWorkspace.objValueIn[1] = acadoVariables.x[281];
acadoWorkspace.objValueIn[2] = acadoVariables.x[282];
acadoWorkspace.objValueIn[3] = acadoVariables.x[283];
acadoWorkspace.objValueIn[4] = acadoVariables.x[284];
acadoWorkspace.objValueIn[5] = acadoVariables.x[285];
acadoWorkspace.objValueIn[6] = acadoVariables.x[286];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[21] + Gx1[4]*Gx2[28] + Gx1[5]*Gx2[35] + Gx1[6]*Gx2[42];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[15] + Gx1[3]*Gx2[22] + Gx1[4]*Gx2[29] + Gx1[5]*Gx2[36] + Gx1[6]*Gx2[43];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[23] + Gx1[4]*Gx2[30] + Gx1[5]*Gx2[37] + Gx1[6]*Gx2[44];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[24] + Gx1[4]*Gx2[31] + Gx1[5]*Gx2[38] + Gx1[6]*Gx2[45];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[18] + Gx1[3]*Gx2[25] + Gx1[4]*Gx2[32] + Gx1[5]*Gx2[39] + Gx1[6]*Gx2[46];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[19] + Gx1[3]*Gx2[26] + Gx1[4]*Gx2[33] + Gx1[5]*Gx2[40] + Gx1[6]*Gx2[47];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[27] + Gx1[4]*Gx2[34] + Gx1[5]*Gx2[41] + Gx1[6]*Gx2[48];
Gx3[7] = + Gx1[7]*Gx2[0] + Gx1[8]*Gx2[7] + Gx1[9]*Gx2[14] + Gx1[10]*Gx2[21] + Gx1[11]*Gx2[28] + Gx1[12]*Gx2[35] + Gx1[13]*Gx2[42];
Gx3[8] = + Gx1[7]*Gx2[1] + Gx1[8]*Gx2[8] + Gx1[9]*Gx2[15] + Gx1[10]*Gx2[22] + Gx1[11]*Gx2[29] + Gx1[12]*Gx2[36] + Gx1[13]*Gx2[43];
Gx3[9] = + Gx1[7]*Gx2[2] + Gx1[8]*Gx2[9] + Gx1[9]*Gx2[16] + Gx1[10]*Gx2[23] + Gx1[11]*Gx2[30] + Gx1[12]*Gx2[37] + Gx1[13]*Gx2[44];
Gx3[10] = + Gx1[7]*Gx2[3] + Gx1[8]*Gx2[10] + Gx1[9]*Gx2[17] + Gx1[10]*Gx2[24] + Gx1[11]*Gx2[31] + Gx1[12]*Gx2[38] + Gx1[13]*Gx2[45];
Gx3[11] = + Gx1[7]*Gx2[4] + Gx1[8]*Gx2[11] + Gx1[9]*Gx2[18] + Gx1[10]*Gx2[25] + Gx1[11]*Gx2[32] + Gx1[12]*Gx2[39] + Gx1[13]*Gx2[46];
Gx3[12] = + Gx1[7]*Gx2[5] + Gx1[8]*Gx2[12] + Gx1[9]*Gx2[19] + Gx1[10]*Gx2[26] + Gx1[11]*Gx2[33] + Gx1[12]*Gx2[40] + Gx1[13]*Gx2[47];
Gx3[13] = + Gx1[7]*Gx2[6] + Gx1[8]*Gx2[13] + Gx1[9]*Gx2[20] + Gx1[10]*Gx2[27] + Gx1[11]*Gx2[34] + Gx1[12]*Gx2[41] + Gx1[13]*Gx2[48];
Gx3[14] = + Gx1[14]*Gx2[0] + Gx1[15]*Gx2[7] + Gx1[16]*Gx2[14] + Gx1[17]*Gx2[21] + Gx1[18]*Gx2[28] + Gx1[19]*Gx2[35] + Gx1[20]*Gx2[42];
Gx3[15] = + Gx1[14]*Gx2[1] + Gx1[15]*Gx2[8] + Gx1[16]*Gx2[15] + Gx1[17]*Gx2[22] + Gx1[18]*Gx2[29] + Gx1[19]*Gx2[36] + Gx1[20]*Gx2[43];
Gx3[16] = + Gx1[14]*Gx2[2] + Gx1[15]*Gx2[9] + Gx1[16]*Gx2[16] + Gx1[17]*Gx2[23] + Gx1[18]*Gx2[30] + Gx1[19]*Gx2[37] + Gx1[20]*Gx2[44];
Gx3[17] = + Gx1[14]*Gx2[3] + Gx1[15]*Gx2[10] + Gx1[16]*Gx2[17] + Gx1[17]*Gx2[24] + Gx1[18]*Gx2[31] + Gx1[19]*Gx2[38] + Gx1[20]*Gx2[45];
Gx3[18] = + Gx1[14]*Gx2[4] + Gx1[15]*Gx2[11] + Gx1[16]*Gx2[18] + Gx1[17]*Gx2[25] + Gx1[18]*Gx2[32] + Gx1[19]*Gx2[39] + Gx1[20]*Gx2[46];
Gx3[19] = + Gx1[14]*Gx2[5] + Gx1[15]*Gx2[12] + Gx1[16]*Gx2[19] + Gx1[17]*Gx2[26] + Gx1[18]*Gx2[33] + Gx1[19]*Gx2[40] + Gx1[20]*Gx2[47];
Gx3[20] = + Gx1[14]*Gx2[6] + Gx1[15]*Gx2[13] + Gx1[16]*Gx2[20] + Gx1[17]*Gx2[27] + Gx1[18]*Gx2[34] + Gx1[19]*Gx2[41] + Gx1[20]*Gx2[48];
Gx3[21] = + Gx1[21]*Gx2[0] + Gx1[22]*Gx2[7] + Gx1[23]*Gx2[14] + Gx1[24]*Gx2[21] + Gx1[25]*Gx2[28] + Gx1[26]*Gx2[35] + Gx1[27]*Gx2[42];
Gx3[22] = + Gx1[21]*Gx2[1] + Gx1[22]*Gx2[8] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[22] + Gx1[25]*Gx2[29] + Gx1[26]*Gx2[36] + Gx1[27]*Gx2[43];
Gx3[23] = + Gx1[21]*Gx2[2] + Gx1[22]*Gx2[9] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[23] + Gx1[25]*Gx2[30] + Gx1[26]*Gx2[37] + Gx1[27]*Gx2[44];
Gx3[24] = + Gx1[21]*Gx2[3] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[24] + Gx1[25]*Gx2[31] + Gx1[26]*Gx2[38] + Gx1[27]*Gx2[45];
Gx3[25] = + Gx1[21]*Gx2[4] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[25] + Gx1[25]*Gx2[32] + Gx1[26]*Gx2[39] + Gx1[27]*Gx2[46];
Gx3[26] = + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[26] + Gx1[25]*Gx2[33] + Gx1[26]*Gx2[40] + Gx1[27]*Gx2[47];
Gx3[27] = + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[20] + Gx1[24]*Gx2[27] + Gx1[25]*Gx2[34] + Gx1[26]*Gx2[41] + Gx1[27]*Gx2[48];
Gx3[28] = + Gx1[28]*Gx2[0] + Gx1[29]*Gx2[7] + Gx1[30]*Gx2[14] + Gx1[31]*Gx2[21] + Gx1[32]*Gx2[28] + Gx1[33]*Gx2[35] + Gx1[34]*Gx2[42];
Gx3[29] = + Gx1[28]*Gx2[1] + Gx1[29]*Gx2[8] + Gx1[30]*Gx2[15] + Gx1[31]*Gx2[22] + Gx1[32]*Gx2[29] + Gx1[33]*Gx2[36] + Gx1[34]*Gx2[43];
Gx3[30] = + Gx1[28]*Gx2[2] + Gx1[29]*Gx2[9] + Gx1[30]*Gx2[16] + Gx1[31]*Gx2[23] + Gx1[32]*Gx2[30] + Gx1[33]*Gx2[37] + Gx1[34]*Gx2[44];
Gx3[31] = + Gx1[28]*Gx2[3] + Gx1[29]*Gx2[10] + Gx1[30]*Gx2[17] + Gx1[31]*Gx2[24] + Gx1[32]*Gx2[31] + Gx1[33]*Gx2[38] + Gx1[34]*Gx2[45];
Gx3[32] = + Gx1[28]*Gx2[4] + Gx1[29]*Gx2[11] + Gx1[30]*Gx2[18] + Gx1[31]*Gx2[25] + Gx1[32]*Gx2[32] + Gx1[33]*Gx2[39] + Gx1[34]*Gx2[46];
Gx3[33] = + Gx1[28]*Gx2[5] + Gx1[29]*Gx2[12] + Gx1[30]*Gx2[19] + Gx1[31]*Gx2[26] + Gx1[32]*Gx2[33] + Gx1[33]*Gx2[40] + Gx1[34]*Gx2[47];
Gx3[34] = + Gx1[28]*Gx2[6] + Gx1[29]*Gx2[13] + Gx1[30]*Gx2[20] + Gx1[31]*Gx2[27] + Gx1[32]*Gx2[34] + Gx1[33]*Gx2[41] + Gx1[34]*Gx2[48];
Gx3[35] = + Gx1[35]*Gx2[0] + Gx1[36]*Gx2[7] + Gx1[37]*Gx2[14] + Gx1[38]*Gx2[21] + Gx1[39]*Gx2[28] + Gx1[40]*Gx2[35] + Gx1[41]*Gx2[42];
Gx3[36] = + Gx1[35]*Gx2[1] + Gx1[36]*Gx2[8] + Gx1[37]*Gx2[15] + Gx1[38]*Gx2[22] + Gx1[39]*Gx2[29] + Gx1[40]*Gx2[36] + Gx1[41]*Gx2[43];
Gx3[37] = + Gx1[35]*Gx2[2] + Gx1[36]*Gx2[9] + Gx1[37]*Gx2[16] + Gx1[38]*Gx2[23] + Gx1[39]*Gx2[30] + Gx1[40]*Gx2[37] + Gx1[41]*Gx2[44];
Gx3[38] = + Gx1[35]*Gx2[3] + Gx1[36]*Gx2[10] + Gx1[37]*Gx2[17] + Gx1[38]*Gx2[24] + Gx1[39]*Gx2[31] + Gx1[40]*Gx2[38] + Gx1[41]*Gx2[45];
Gx3[39] = + Gx1[35]*Gx2[4] + Gx1[36]*Gx2[11] + Gx1[37]*Gx2[18] + Gx1[38]*Gx2[25] + Gx1[39]*Gx2[32] + Gx1[40]*Gx2[39] + Gx1[41]*Gx2[46];
Gx3[40] = + Gx1[35]*Gx2[5] + Gx1[36]*Gx2[12] + Gx1[37]*Gx2[19] + Gx1[38]*Gx2[26] + Gx1[39]*Gx2[33] + Gx1[40]*Gx2[40] + Gx1[41]*Gx2[47];
Gx3[41] = + Gx1[35]*Gx2[6] + Gx1[36]*Gx2[13] + Gx1[37]*Gx2[20] + Gx1[38]*Gx2[27] + Gx1[39]*Gx2[34] + Gx1[40]*Gx2[41] + Gx1[41]*Gx2[48];
Gx3[42] = + Gx1[42]*Gx2[0] + Gx1[43]*Gx2[7] + Gx1[44]*Gx2[14] + Gx1[45]*Gx2[21] + Gx1[46]*Gx2[28] + Gx1[47]*Gx2[35] + Gx1[48]*Gx2[42];
Gx3[43] = + Gx1[42]*Gx2[1] + Gx1[43]*Gx2[8] + Gx1[44]*Gx2[15] + Gx1[45]*Gx2[22] + Gx1[46]*Gx2[29] + Gx1[47]*Gx2[36] + Gx1[48]*Gx2[43];
Gx3[44] = + Gx1[42]*Gx2[2] + Gx1[43]*Gx2[9] + Gx1[44]*Gx2[16] + Gx1[45]*Gx2[23] + Gx1[46]*Gx2[30] + Gx1[47]*Gx2[37] + Gx1[48]*Gx2[44];
Gx3[45] = + Gx1[42]*Gx2[3] + Gx1[43]*Gx2[10] + Gx1[44]*Gx2[17] + Gx1[45]*Gx2[24] + Gx1[46]*Gx2[31] + Gx1[47]*Gx2[38] + Gx1[48]*Gx2[45];
Gx3[46] = + Gx1[42]*Gx2[4] + Gx1[43]*Gx2[11] + Gx1[44]*Gx2[18] + Gx1[45]*Gx2[25] + Gx1[46]*Gx2[32] + Gx1[47]*Gx2[39] + Gx1[48]*Gx2[46];
Gx3[47] = + Gx1[42]*Gx2[5] + Gx1[43]*Gx2[12] + Gx1[44]*Gx2[19] + Gx1[45]*Gx2[26] + Gx1[46]*Gx2[33] + Gx1[47]*Gx2[40] + Gx1[48]*Gx2[47];
Gx3[48] = + Gx1[42]*Gx2[6] + Gx1[43]*Gx2[13] + Gx1[44]*Gx2[20] + Gx1[45]*Gx2[27] + Gx1[46]*Gx2[34] + Gx1[47]*Gx2[41] + Gx1[48]*Gx2[48];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1] + Gx1[2]*Gu1[2] + Gx1[3]*Gu1[3] + Gx1[4]*Gu1[4] + Gx1[5]*Gu1[5] + Gx1[6]*Gu1[6];
Gu2[1] = + Gx1[7]*Gu1[0] + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[4] + Gx1[12]*Gu1[5] + Gx1[13]*Gu1[6];
Gu2[2] = + Gx1[14]*Gu1[0] + Gx1[15]*Gu1[1] + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[3] + Gx1[18]*Gu1[4] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[6];
Gu2[3] = + Gx1[21]*Gu1[0] + Gx1[22]*Gu1[1] + Gx1[23]*Gu1[2] + Gx1[24]*Gu1[3] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[5] + Gx1[27]*Gu1[6];
Gu2[4] = + Gx1[28]*Gu1[0] + Gx1[29]*Gu1[1] + Gx1[30]*Gu1[2] + Gx1[31]*Gu1[3] + Gx1[32]*Gu1[4] + Gx1[33]*Gu1[5] + Gx1[34]*Gu1[6];
Gu2[5] = + Gx1[35]*Gu1[0] + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[2] + Gx1[38]*Gu1[3] + Gx1[39]*Gu1[4] + Gx1[40]*Gu1[5] + Gx1[41]*Gu1[6];
Gu2[6] = + Gx1[42]*Gu1[0] + Gx1[43]*Gu1[1] + Gx1[44]*Gu1[2] + Gx1[45]*Gu1[3] + Gx1[46]*Gu1[4] + Gx1[47]*Gu1[5] + Gx1[48]*Gu1[6];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol)] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + Gu1[3]*Gu2[3] + Gu1[4]*Gu2[4] + Gu1[5]*Gu2[5] + Gu1[6]*Gu2[6];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 41] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + Gu1[3]*Gu2[3] + Gu1[4]*Gu2[4] + Gu1[5]*Gu2[5] + Gu1[6]*Gu2[6] + R11[0];
acadoWorkspace.H[iRow * 41] += 1.0000000000000000e-04;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[7]*Gu1[1] + Gx1[14]*Gu1[2] + Gx1[21]*Gu1[3] + Gx1[28]*Gu1[4] + Gx1[35]*Gu1[5] + Gx1[42]*Gu1[6];
Gu2[1] = + Gx1[1]*Gu1[0] + Gx1[8]*Gu1[1] + Gx1[15]*Gu1[2] + Gx1[22]*Gu1[3] + Gx1[29]*Gu1[4] + Gx1[36]*Gu1[5] + Gx1[43]*Gu1[6];
Gu2[2] = + Gx1[2]*Gu1[0] + Gx1[9]*Gu1[1] + Gx1[16]*Gu1[2] + Gx1[23]*Gu1[3] + Gx1[30]*Gu1[4] + Gx1[37]*Gu1[5] + Gx1[44]*Gu1[6];
Gu2[3] = + Gx1[3]*Gu1[0] + Gx1[10]*Gu1[1] + Gx1[17]*Gu1[2] + Gx1[24]*Gu1[3] + Gx1[31]*Gu1[4] + Gx1[38]*Gu1[5] + Gx1[45]*Gu1[6];
Gu2[4] = + Gx1[4]*Gu1[0] + Gx1[11]*Gu1[1] + Gx1[18]*Gu1[2] + Gx1[25]*Gu1[3] + Gx1[32]*Gu1[4] + Gx1[39]*Gu1[5] + Gx1[46]*Gu1[6];
Gu2[5] = + Gx1[5]*Gu1[0] + Gx1[12]*Gu1[1] + Gx1[19]*Gu1[2] + Gx1[26]*Gu1[3] + Gx1[33]*Gu1[4] + Gx1[40]*Gu1[5] + Gx1[47]*Gu1[6];
Gu2[6] = + Gx1[6]*Gu1[0] + Gx1[13]*Gu1[1] + Gx1[20]*Gu1[2] + Gx1[27]*Gu1[3] + Gx1[34]*Gu1[4] + Gx1[41]*Gu1[5] + Gx1[48]*Gu1[6];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[1] + Q11[2]*Gu1[2] + Q11[3]*Gu1[3] + Q11[4]*Gu1[4] + Q11[5]*Gu1[5] + Q11[6]*Gu1[6] + Gu2[0];
Gu3[1] = + Q11[7]*Gu1[0] + Q11[8]*Gu1[1] + Q11[9]*Gu1[2] + Q11[10]*Gu1[3] + Q11[11]*Gu1[4] + Q11[12]*Gu1[5] + Q11[13]*Gu1[6] + Gu2[1];
Gu3[2] = + Q11[14]*Gu1[0] + Q11[15]*Gu1[1] + Q11[16]*Gu1[2] + Q11[17]*Gu1[3] + Q11[18]*Gu1[4] + Q11[19]*Gu1[5] + Q11[20]*Gu1[6] + Gu2[2];
Gu3[3] = + Q11[21]*Gu1[0] + Q11[22]*Gu1[1] + Q11[23]*Gu1[2] + Q11[24]*Gu1[3] + Q11[25]*Gu1[4] + Q11[26]*Gu1[5] + Q11[27]*Gu1[6] + Gu2[3];
Gu3[4] = + Q11[28]*Gu1[0] + Q11[29]*Gu1[1] + Q11[30]*Gu1[2] + Q11[31]*Gu1[3] + Q11[32]*Gu1[4] + Q11[33]*Gu1[5] + Q11[34]*Gu1[6] + Gu2[4];
Gu3[5] = + Q11[35]*Gu1[0] + Q11[36]*Gu1[1] + Q11[37]*Gu1[2] + Q11[38]*Gu1[3] + Q11[39]*Gu1[4] + Q11[40]*Gu1[5] + Q11[41]*Gu1[6] + Gu2[5];
Gu3[6] = + Q11[42]*Gu1[0] + Q11[43]*Gu1[1] + Q11[44]*Gu1[2] + Q11[45]*Gu1[3] + Q11[46]*Gu1[4] + Q11[47]*Gu1[5] + Q11[48]*Gu1[6] + Gu2[6];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[7]*w11[1] + Gx1[14]*w11[2] + Gx1[21]*w11[3] + Gx1[28]*w11[4] + Gx1[35]*w11[5] + Gx1[42]*w11[6] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[8]*w11[1] + Gx1[15]*w11[2] + Gx1[22]*w11[3] + Gx1[29]*w11[4] + Gx1[36]*w11[5] + Gx1[43]*w11[6] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[9]*w11[1] + Gx1[16]*w11[2] + Gx1[23]*w11[3] + Gx1[30]*w11[4] + Gx1[37]*w11[5] + Gx1[44]*w11[6] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[10]*w11[1] + Gx1[17]*w11[2] + Gx1[24]*w11[3] + Gx1[31]*w11[4] + Gx1[38]*w11[5] + Gx1[45]*w11[6] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[11]*w11[1] + Gx1[18]*w11[2] + Gx1[25]*w11[3] + Gx1[32]*w11[4] + Gx1[39]*w11[5] + Gx1[46]*w11[6] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[12]*w11[1] + Gx1[19]*w11[2] + Gx1[26]*w11[3] + Gx1[33]*w11[4] + Gx1[40]*w11[5] + Gx1[47]*w11[6] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[13]*w11[1] + Gx1[20]*w11[2] + Gx1[27]*w11[3] + Gx1[34]*w11[4] + Gx1[41]*w11[5] + Gx1[48]*w11[6] + w12[6];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[1]*w11[1] + Gu1[2]*w11[2] + Gu1[3]*w11[3] + Gu1[4]*w11[4] + Gu1[5]*w11[5] + Gu1[6]*w11[6];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + w12[0];
w13[1] = + Q11[7]*w11[0] + Q11[8]*w11[1] + Q11[9]*w11[2] + Q11[10]*w11[3] + Q11[11]*w11[4] + Q11[12]*w11[5] + Q11[13]*w11[6] + w12[1];
w13[2] = + Q11[14]*w11[0] + Q11[15]*w11[1] + Q11[16]*w11[2] + Q11[17]*w11[3] + Q11[18]*w11[4] + Q11[19]*w11[5] + Q11[20]*w11[6] + w12[2];
w13[3] = + Q11[21]*w11[0] + Q11[22]*w11[1] + Q11[23]*w11[2] + Q11[24]*w11[3] + Q11[25]*w11[4] + Q11[26]*w11[5] + Q11[27]*w11[6] + w12[3];
w13[4] = + Q11[28]*w11[0] + Q11[29]*w11[1] + Q11[30]*w11[2] + Q11[31]*w11[3] + Q11[32]*w11[4] + Q11[33]*w11[5] + Q11[34]*w11[6] + w12[4];
w13[5] = + Q11[35]*w11[0] + Q11[36]*w11[1] + Q11[37]*w11[2] + Q11[38]*w11[3] + Q11[39]*w11[4] + Q11[40]*w11[5] + Q11[41]*w11[6] + w12[5];
w13[6] = + Q11[42]*w11[0] + Q11[43]*w11[1] + Q11[44]*w11[2] + Q11[45]*w11[3] + Q11[46]*w11[4] + Q11[47]*w11[5] + Q11[48]*w11[6] + w12[6];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6];
w12[1] += + Gx1[7]*w11[0] + Gx1[8]*w11[1] + Gx1[9]*w11[2] + Gx1[10]*w11[3] + Gx1[11]*w11[4] + Gx1[12]*w11[5] + Gx1[13]*w11[6];
w12[2] += + Gx1[14]*w11[0] + Gx1[15]*w11[1] + Gx1[16]*w11[2] + Gx1[17]*w11[3] + Gx1[18]*w11[4] + Gx1[19]*w11[5] + Gx1[20]*w11[6];
w12[3] += + Gx1[21]*w11[0] + Gx1[22]*w11[1] + Gx1[23]*w11[2] + Gx1[24]*w11[3] + Gx1[25]*w11[4] + Gx1[26]*w11[5] + Gx1[27]*w11[6];
w12[4] += + Gx1[28]*w11[0] + Gx1[29]*w11[1] + Gx1[30]*w11[2] + Gx1[31]*w11[3] + Gx1[32]*w11[4] + Gx1[33]*w11[5] + Gx1[34]*w11[6];
w12[5] += + Gx1[35]*w11[0] + Gx1[36]*w11[1] + Gx1[37]*w11[2] + Gx1[38]*w11[3] + Gx1[39]*w11[4] + Gx1[40]*w11[5] + Gx1[41]*w11[6];
w12[6] += + Gx1[42]*w11[0] + Gx1[43]*w11[1] + Gx1[44]*w11[2] + Gx1[45]*w11[3] + Gx1[46]*w11[4] + Gx1[47]*w11[5] + Gx1[48]*w11[6];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6];
w12[1] += + Gx1[7]*w11[0] + Gx1[8]*w11[1] + Gx1[9]*w11[2] + Gx1[10]*w11[3] + Gx1[11]*w11[4] + Gx1[12]*w11[5] + Gx1[13]*w11[6];
w12[2] += + Gx1[14]*w11[0] + Gx1[15]*w11[1] + Gx1[16]*w11[2] + Gx1[17]*w11[3] + Gx1[18]*w11[4] + Gx1[19]*w11[5] + Gx1[20]*w11[6];
w12[3] += + Gx1[21]*w11[0] + Gx1[22]*w11[1] + Gx1[23]*w11[2] + Gx1[24]*w11[3] + Gx1[25]*w11[4] + Gx1[26]*w11[5] + Gx1[27]*w11[6];
w12[4] += + Gx1[28]*w11[0] + Gx1[29]*w11[1] + Gx1[30]*w11[2] + Gx1[31]*w11[3] + Gx1[32]*w11[4] + Gx1[33]*w11[5] + Gx1[34]*w11[6];
w12[5] += + Gx1[35]*w11[0] + Gx1[36]*w11[1] + Gx1[37]*w11[2] + Gx1[38]*w11[3] + Gx1[39]*w11[4] + Gx1[40]*w11[5] + Gx1[41]*w11[6];
w12[6] += + Gx1[42]*w11[0] + Gx1[43]*w11[1] + Gx1[44]*w11[2] + Gx1[45]*w11[3] + Gx1[46]*w11[4] + Gx1[47]*w11[5] + Gx1[48]*w11[6];
w12[0] += + Gu1[0]*U1[0];
w12[1] += + Gu1[1]*U1[0];
w12[2] += + Gu1[2]*U1[0];
w12[3] += + Gu1[3]*U1[0];
w12[4] += + Gu1[4]*U1[0];
w12[5] += + Gu1[5]*U1[0];
w12[6] += + Gu1[6]*U1[0];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol)] = acadoWorkspace.H[(iCol * 40) + (iRow)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7];
QDy1[1] = + Q2[8]*Dy1[0] + Q2[9]*Dy1[1] + Q2[10]*Dy1[2] + Q2[11]*Dy1[3] + Q2[12]*Dy1[4] + Q2[13]*Dy1[5] + Q2[14]*Dy1[6] + Q2[15]*Dy1[7];
QDy1[2] = + Q2[16]*Dy1[0] + Q2[17]*Dy1[1] + Q2[18]*Dy1[2] + Q2[19]*Dy1[3] + Q2[20]*Dy1[4] + Q2[21]*Dy1[5] + Q2[22]*Dy1[6] + Q2[23]*Dy1[7];
QDy1[3] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5] + Q2[30]*Dy1[6] + Q2[31]*Dy1[7];
QDy1[4] = + Q2[32]*Dy1[0] + Q2[33]*Dy1[1] + Q2[34]*Dy1[2] + Q2[35]*Dy1[3] + Q2[36]*Dy1[4] + Q2[37]*Dy1[5] + Q2[38]*Dy1[6] + Q2[39]*Dy1[7];
QDy1[5] = + Q2[40]*Dy1[0] + Q2[41]*Dy1[1] + Q2[42]*Dy1[2] + Q2[43]*Dy1[3] + Q2[44]*Dy1[4] + Q2[45]*Dy1[5] + Q2[46]*Dy1[6] + Q2[47]*Dy1[7];
QDy1[6] = + Q2[48]*Dy1[0] + Q2[49]*Dy1[1] + Q2[50]*Dy1[2] + Q2[51]*Dy1[3] + Q2[52]*Dy1[4] + Q2[53]*Dy1[5] + Q2[54]*Dy1[6] + Q2[55]*Dy1[7];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 40 + 1600) + (col)] = + Hx[0]*E[0] + Hx[1]*E[1] + Hx[2]*E[2] + Hx[3]*E[3] + Hx[4]*E[4] + Hx[5]*E[5] + Hx[6]*E[6];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5] + Hx[6]*tmpd[6];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 40 */
static const int xBoundIndices[ 40 ] = 
{ 13, 20, 27, 34, 41, 48, 55, 62, 69, 76, 83, 90, 97, 104, 111, 118, 125, 132, 139, 146, 153, 160, 167, 174, 181, 188, 195, 202, 209, 216, 223, 230, 237, 244, 251, 258, 265, 272, 279, 286 };
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 49 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 49 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 98 ]), &(acadoWorkspace.C[ 49 ]), &(acadoWorkspace.C[ 98 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 147 ]), &(acadoWorkspace.C[ 98 ]), &(acadoWorkspace.C[ 147 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 196 ]), &(acadoWorkspace.C[ 147 ]), &(acadoWorkspace.C[ 196 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 245 ]), &(acadoWorkspace.C[ 196 ]), &(acadoWorkspace.C[ 245 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 294 ]), &(acadoWorkspace.C[ 245 ]), &(acadoWorkspace.C[ 294 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 343 ]), &(acadoWorkspace.C[ 294 ]), &(acadoWorkspace.C[ 343 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 392 ]), &(acadoWorkspace.C[ 343 ]), &(acadoWorkspace.C[ 392 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.C[ 392 ]), &(acadoWorkspace.C[ 441 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 490 ]), &(acadoWorkspace.C[ 441 ]), &(acadoWorkspace.C[ 490 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 539 ]), &(acadoWorkspace.C[ 490 ]), &(acadoWorkspace.C[ 539 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 588 ]), &(acadoWorkspace.C[ 539 ]), &(acadoWorkspace.C[ 588 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 637 ]), &(acadoWorkspace.C[ 588 ]), &(acadoWorkspace.C[ 637 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 686 ]), &(acadoWorkspace.C[ 637 ]), &(acadoWorkspace.C[ 686 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 735 ]), &(acadoWorkspace.C[ 686 ]), &(acadoWorkspace.C[ 735 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 784 ]), &(acadoWorkspace.C[ 735 ]), &(acadoWorkspace.C[ 784 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 833 ]), &(acadoWorkspace.C[ 784 ]), &(acadoWorkspace.C[ 833 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 882 ]), &(acadoWorkspace.C[ 833 ]), &(acadoWorkspace.C[ 882 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 931 ]), &(acadoWorkspace.C[ 882 ]), &(acadoWorkspace.C[ 931 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 980 ]), &(acadoWorkspace.C[ 931 ]), &(acadoWorkspace.C[ 980 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1029 ]), &(acadoWorkspace.C[ 980 ]), &(acadoWorkspace.C[ 1029 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1078 ]), &(acadoWorkspace.C[ 1029 ]), &(acadoWorkspace.C[ 1078 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1127 ]), &(acadoWorkspace.C[ 1078 ]), &(acadoWorkspace.C[ 1127 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1176 ]), &(acadoWorkspace.C[ 1127 ]), &(acadoWorkspace.C[ 1176 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1225 ]), &(acadoWorkspace.C[ 1176 ]), &(acadoWorkspace.C[ 1225 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1274 ]), &(acadoWorkspace.C[ 1225 ]), &(acadoWorkspace.C[ 1274 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1323 ]), &(acadoWorkspace.C[ 1274 ]), &(acadoWorkspace.C[ 1323 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1372 ]), &(acadoWorkspace.C[ 1323 ]), &(acadoWorkspace.C[ 1372 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1421 ]), &(acadoWorkspace.C[ 1372 ]), &(acadoWorkspace.C[ 1421 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1470 ]), &(acadoWorkspace.C[ 1421 ]), &(acadoWorkspace.C[ 1470 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1519 ]), &(acadoWorkspace.C[ 1470 ]), &(acadoWorkspace.C[ 1519 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1568 ]), &(acadoWorkspace.C[ 1519 ]), &(acadoWorkspace.C[ 1568 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1617 ]), &(acadoWorkspace.C[ 1568 ]), &(acadoWorkspace.C[ 1617 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1666 ]), &(acadoWorkspace.C[ 1617 ]), &(acadoWorkspace.C[ 1666 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1715 ]), &(acadoWorkspace.C[ 1666 ]), &(acadoWorkspace.C[ 1715 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1764 ]), &(acadoWorkspace.C[ 1715 ]), &(acadoWorkspace.C[ 1764 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1813 ]), &(acadoWorkspace.C[ 1764 ]), &(acadoWorkspace.C[ 1813 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1862 ]), &(acadoWorkspace.C[ 1813 ]), &(acadoWorkspace.C[ 1862 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1911 ]), &(acadoWorkspace.C[ 1862 ]), &(acadoWorkspace.C[ 1911 ]) );
for (lRun2 = 0; lRun2 < 40; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 81)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 7 ]), &(acadoWorkspace.E[ lRun3 * 7 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 40; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (7)) * (7)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (7)) * (1)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (7)) * (1)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (40)) - (1)) * (7)) * (1)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 39; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 7 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 49 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 49 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (7)) * (1)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 ]), &(acadoWorkspace.evGu[ lRun2 * 7 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun2 = 0; lRun2 < 280; ++lRun2)
acadoWorkspace.sbar[lRun2 + 7] = acadoWorkspace.d[lRun2];

acadoWorkspace.lb[0] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-9.0666454649056083e-01 - acadoVariables.u[39];
acadoWorkspace.ub[0] = (real_t)4.0799904592075237e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)4.0799904592075237e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)4.0799904592075237e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)4.0799904592075237e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)4.0799904592075237e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)4.0799904592075237e-01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)4.0799904592075237e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)4.0799904592075237e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)4.0799904592075237e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)4.0799904592075237e-01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)4.0799904592075237e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)4.0799904592075237e-01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)4.0799904592075237e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)4.0799904592075237e-01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)4.0799904592075237e-01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)4.0799904592075237e-01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)4.0799904592075237e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)4.0799904592075237e-01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)4.0799904592075237e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)4.0799904592075237e-01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)4.0799904592075237e-01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)4.0799904592075237e-01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)4.0799904592075237e-01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)4.0799904592075237e-01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)4.0799904592075237e-01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)4.0799904592075237e-01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)4.0799904592075237e-01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)4.0799904592075237e-01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)4.0799904592075237e-01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)4.0799904592075237e-01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)4.0799904592075237e-01 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)4.0799904592075237e-01 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)4.0799904592075237e-01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)4.0799904592075237e-01 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)4.0799904592075237e-01 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)4.0799904592075237e-01 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)4.0799904592075237e-01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)4.0799904592075237e-01 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)4.0799904592075237e-01 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)4.0799904592075237e-01 - acadoVariables.u[39];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 7;
lRun4 = ((lRun3) / (7)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 79)) / (2)) + (lRun4)) - (1)) * (7)) + ((lRun3) % (7));
acadoWorkspace.A[(lRun1 * 40) + (lRun2)] = acadoWorkspace.E[lRun5];
}
}

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 7];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 7 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 7 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 7 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 7 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.x[lRun1 * 7 + 5];
acadoWorkspace.conValueIn[6] = acadoVariables.x[lRun1 * 7 + 6];
acadoWorkspace.conValueIn[7] = acadoVariables.u[lRun1];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1 * 7] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun1 * 7 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 7 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 7 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 7 + 4] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 7 + 5] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 7 + 6] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHu[lRun1] = acadoWorkspace.conValueOut[8];
}



for (lRun2 = 0; lRun2 < 39; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun3) * (lRun3 * -1 + 79)) / (2)) + (lRun2);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 7 + 7 ]), &(acadoWorkspace.E[ lRun4 * 7 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[1600] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1641] = acadoWorkspace.evHu[1];
acadoWorkspace.A[1682] = acadoWorkspace.evHu[2];
acadoWorkspace.A[1723] = acadoWorkspace.evHu[3];
acadoWorkspace.A[1764] = acadoWorkspace.evHu[4];
acadoWorkspace.A[1805] = acadoWorkspace.evHu[5];
acadoWorkspace.A[1846] = acadoWorkspace.evHu[6];
acadoWorkspace.A[1887] = acadoWorkspace.evHu[7];
acadoWorkspace.A[1928] = acadoWorkspace.evHu[8];
acadoWorkspace.A[1969] = acadoWorkspace.evHu[9];
acadoWorkspace.A[2010] = acadoWorkspace.evHu[10];
acadoWorkspace.A[2051] = acadoWorkspace.evHu[11];
acadoWorkspace.A[2092] = acadoWorkspace.evHu[12];
acadoWorkspace.A[2133] = acadoWorkspace.evHu[13];
acadoWorkspace.A[2174] = acadoWorkspace.evHu[14];
acadoWorkspace.A[2215] = acadoWorkspace.evHu[15];
acadoWorkspace.A[2256] = acadoWorkspace.evHu[16];
acadoWorkspace.A[2297] = acadoWorkspace.evHu[17];
acadoWorkspace.A[2338] = acadoWorkspace.evHu[18];
acadoWorkspace.A[2379] = acadoWorkspace.evHu[19];
acadoWorkspace.A[2420] = acadoWorkspace.evHu[20];
acadoWorkspace.A[2461] = acadoWorkspace.evHu[21];
acadoWorkspace.A[2502] = acadoWorkspace.evHu[22];
acadoWorkspace.A[2543] = acadoWorkspace.evHu[23];
acadoWorkspace.A[2584] = acadoWorkspace.evHu[24];
acadoWorkspace.A[2625] = acadoWorkspace.evHu[25];
acadoWorkspace.A[2666] = acadoWorkspace.evHu[26];
acadoWorkspace.A[2707] = acadoWorkspace.evHu[27];
acadoWorkspace.A[2748] = acadoWorkspace.evHu[28];
acadoWorkspace.A[2789] = acadoWorkspace.evHu[29];
acadoWorkspace.A[2830] = acadoWorkspace.evHu[30];
acadoWorkspace.A[2871] = acadoWorkspace.evHu[31];
acadoWorkspace.A[2912] = acadoWorkspace.evHu[32];
acadoWorkspace.A[2953] = acadoWorkspace.evHu[33];
acadoWorkspace.A[2994] = acadoWorkspace.evHu[34];
acadoWorkspace.A[3035] = acadoWorkspace.evHu[35];
acadoWorkspace.A[3076] = acadoWorkspace.evHu[36];
acadoWorkspace.A[3117] = acadoWorkspace.evHu[37];
acadoWorkspace.A[3158] = acadoWorkspace.evHu[38];
acadoWorkspace.A[3199] = acadoWorkspace.evHu[39];
acadoWorkspace.lbA[40] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[41] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[42] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[43] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[44] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[45] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[46] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[47] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[48] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[49] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[50] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[51] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[52] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[53] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[54] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[55] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[56] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[57] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[58] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[59] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[19];
acadoWorkspace.lbA[60] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[61] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[21];
acadoWorkspace.lbA[62] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[63] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[23];
acadoWorkspace.lbA[64] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[65] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[25];
acadoWorkspace.lbA[66] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[67] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[27];
acadoWorkspace.lbA[68] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[69] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[29];
acadoWorkspace.lbA[70] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[30];
acadoWorkspace.lbA[71] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[31];
acadoWorkspace.lbA[72] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[32];
acadoWorkspace.lbA[73] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[33];
acadoWorkspace.lbA[74] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[34];
acadoWorkspace.lbA[75] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[35];
acadoWorkspace.lbA[76] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[36];
acadoWorkspace.lbA[77] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[37];
acadoWorkspace.lbA[78] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[38];
acadoWorkspace.lbA[79] = (real_t)-1.7453292519943295e-01 - acadoWorkspace.evH[39];

acadoWorkspace.ubA[40] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[41] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[42] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[43] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[44] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[45] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[46] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[47] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[48] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[49] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[50] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[51] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[52] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[53] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[54] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[55] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[56] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[57] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[58] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[59] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[60] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[61] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[62] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[63] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[64] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[65] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[66] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[67] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[68] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[69] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[70] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[71] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[72] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[73] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[74] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[75] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[76] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[77] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[78] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[79] = (real_t)1.7453292519943295e-01 - acadoWorkspace.evH[39];

}

void acado_condenseFdb(  )
{
int lRun1;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
for (lRun1 = 0; lRun1 < 320; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 8 ]), &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 16 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 32 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 40 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 56 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 64 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 80 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 88 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 104 ]), &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 112 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 128 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 136 ]), &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 152 ]), &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.g[ 19 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 160 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 176 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 184 ]), &(acadoWorkspace.Dy[ 184 ]), &(acadoWorkspace.g[ 23 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 200 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.g[ 25 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 208 ]), &(acadoWorkspace.Dy[ 208 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 224 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 232 ]), &(acadoWorkspace.Dy[ 232 ]), &(acadoWorkspace.g[ 29 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 248 ]), &(acadoWorkspace.Dy[ 248 ]), &(acadoWorkspace.g[ 31 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 256 ]), &(acadoWorkspace.Dy[ 256 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 264 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 272 ]), &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 280 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.g[ 35 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 296 ]), &(acadoWorkspace.Dy[ 296 ]), &(acadoWorkspace.g[ 37 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 304 ]), &(acadoWorkspace.Dy[ 304 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 312 ]), &(acadoWorkspace.Dy[ 312 ]), &(acadoWorkspace.g[ 39 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 56 ]), &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.QDy[ 7 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 112 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 14 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 168 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 224 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 280 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 336 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 392 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 49 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 448 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 504 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 560 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 616 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.QDy[ 77 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 672 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 728 ]), &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.QDy[ 91 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 784 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 98 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 105 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 896 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.QDy[ 112 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 952 ]), &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.QDy[ 119 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1008 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1064 ]), &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.QDy[ 133 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1120 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1176 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 147 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1232 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.QDy[ 154 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1288 ]), &(acadoWorkspace.Dy[ 184 ]), &(acadoWorkspace.QDy[ 161 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1344 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 168 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1400 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.QDy[ 175 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1456 ]), &(acadoWorkspace.Dy[ 208 ]), &(acadoWorkspace.QDy[ 182 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1512 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 189 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1568 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.QDy[ 196 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1624 ]), &(acadoWorkspace.Dy[ 232 ]), &(acadoWorkspace.QDy[ 203 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1680 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 210 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1736 ]), &(acadoWorkspace.Dy[ 248 ]), &(acadoWorkspace.QDy[ 217 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1792 ]), &(acadoWorkspace.Dy[ 256 ]), &(acadoWorkspace.QDy[ 224 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1848 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.QDy[ 231 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1904 ]), &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.QDy[ 238 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1960 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.QDy[ 245 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2016 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.QDy[ 252 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2072 ]), &(acadoWorkspace.Dy[ 296 ]), &(acadoWorkspace.QDy[ 259 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2128 ]), &(acadoWorkspace.Dy[ 304 ]), &(acadoWorkspace.QDy[ 266 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2184 ]), &(acadoWorkspace.Dy[ 312 ]), &(acadoWorkspace.QDy[ 273 ]) );

acadoWorkspace.QDy[280] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[281] = + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[282] = + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[283] = + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[284] = + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[285] = + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[286] = + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[6];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 7 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 49 ]), &(acadoWorkspace.sbar[ 7 ]), &(acadoWorkspace.sbar[ 14 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 98 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 147 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 196 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 35 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 245 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 294 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 49 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 343 ]), &(acadoWorkspace.sbar[ 49 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 392 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 490 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 77 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 539 ]), &(acadoWorkspace.sbar[ 77 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 588 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 91 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 637 ]), &(acadoWorkspace.sbar[ 91 ]), &(acadoWorkspace.sbar[ 98 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 686 ]), &(acadoWorkspace.sbar[ 98 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 735 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 784 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 119 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 833 ]), &(acadoWorkspace.sbar[ 119 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 882 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 133 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 931 ]), &(acadoWorkspace.sbar[ 133 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 980 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1029 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 154 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1078 ]), &(acadoWorkspace.sbar[ 154 ]), &(acadoWorkspace.sbar[ 161 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1127 ]), &(acadoWorkspace.sbar[ 161 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1176 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 175 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1225 ]), &(acadoWorkspace.sbar[ 175 ]), &(acadoWorkspace.sbar[ 182 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1274 ]), &(acadoWorkspace.sbar[ 182 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1323 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 196 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1372 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.sbar[ 203 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1421 ]), &(acadoWorkspace.sbar[ 203 ]), &(acadoWorkspace.sbar[ 210 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1470 ]), &(acadoWorkspace.sbar[ 210 ]), &(acadoWorkspace.sbar[ 217 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1519 ]), &(acadoWorkspace.sbar[ 217 ]), &(acadoWorkspace.sbar[ 224 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1568 ]), &(acadoWorkspace.sbar[ 224 ]), &(acadoWorkspace.sbar[ 231 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1617 ]), &(acadoWorkspace.sbar[ 231 ]), &(acadoWorkspace.sbar[ 238 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1666 ]), &(acadoWorkspace.sbar[ 238 ]), &(acadoWorkspace.sbar[ 245 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1715 ]), &(acadoWorkspace.sbar[ 245 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1764 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 259 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1813 ]), &(acadoWorkspace.sbar[ 259 ]), &(acadoWorkspace.sbar[ 266 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1862 ]), &(acadoWorkspace.sbar[ 266 ]), &(acadoWorkspace.sbar[ 273 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1911 ]), &(acadoWorkspace.sbar[ 273 ]), &(acadoWorkspace.sbar[ 280 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[280] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[281] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[282] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[283] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[284] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[285] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[286] + acadoWorkspace.QDy[280];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[280] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[281] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[282] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[283] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[284] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[285] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[286] + acadoWorkspace.QDy[281];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[280] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[281] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[282] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[283] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[284] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[285] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[286] + acadoWorkspace.QDy[282];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[280] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[281] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[282] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[283] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[284] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[285] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[286] + acadoWorkspace.QDy[283];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[280] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[281] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[282] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[283] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[284] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[285] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[286] + acadoWorkspace.QDy[284];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[280] + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[281] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[282] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[283] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[284] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[285] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[286] + acadoWorkspace.QDy[285];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[280] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[281] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[282] + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[283] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[284] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[285] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[286] + acadoWorkspace.QDy[286];
acado_macBTw1( &(acadoWorkspace.evGu[ 273 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1911 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 273 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1911 ]), &(acadoWorkspace.sbar[ 273 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 266 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1862 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 266 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1862 ]), &(acadoWorkspace.sbar[ 266 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 259 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 37 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1813 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 259 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1813 ]), &(acadoWorkspace.sbar[ 259 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1764 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 252 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1764 ]), &(acadoWorkspace.sbar[ 252 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 245 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 35 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1715 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 245 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1715 ]), &(acadoWorkspace.sbar[ 245 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 238 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1666 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 238 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1666 ]), &(acadoWorkspace.sbar[ 238 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 231 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1617 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 231 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1617 ]), &(acadoWorkspace.sbar[ 231 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 224 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1568 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 224 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1568 ]), &(acadoWorkspace.sbar[ 224 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 217 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 31 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1519 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 217 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1519 ]), &(acadoWorkspace.sbar[ 217 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 210 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1470 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 210 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1470 ]), &(acadoWorkspace.sbar[ 210 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 203 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 29 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1421 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 203 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1421 ]), &(acadoWorkspace.sbar[ 203 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 196 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1372 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 196 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1372 ]), &(acadoWorkspace.sbar[ 196 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1323 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 189 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1323 ]), &(acadoWorkspace.sbar[ 189 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 182 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1274 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 182 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1274 ]), &(acadoWorkspace.sbar[ 182 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 175 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 25 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1225 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 175 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1225 ]), &(acadoWorkspace.sbar[ 175 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 168 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1176 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 168 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1176 ]), &(acadoWorkspace.sbar[ 168 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 161 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 23 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1127 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 161 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1127 ]), &(acadoWorkspace.sbar[ 161 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 154 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1078 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 154 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1078 ]), &(acadoWorkspace.sbar[ 154 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 147 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1029 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 147 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1029 ]), &(acadoWorkspace.sbar[ 147 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 140 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 980 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 140 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 980 ]), &(acadoWorkspace.sbar[ 140 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 133 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 19 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 931 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 133 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 931 ]), &(acadoWorkspace.sbar[ 133 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 882 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 882 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 119 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 17 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 833 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 119 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 833 ]), &(acadoWorkspace.sbar[ 119 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 784 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 112 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 784 ]), &(acadoWorkspace.sbar[ 112 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 105 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 735 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 105 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 735 ]), &(acadoWorkspace.sbar[ 105 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 98 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 686 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 98 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 686 ]), &(acadoWorkspace.sbar[ 98 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 91 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 13 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 637 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 91 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 637 ]), &(acadoWorkspace.sbar[ 91 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 588 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 588 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 77 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 11 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 539 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 77 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 539 ]), &(acadoWorkspace.sbar[ 77 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 490 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 70 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 490 ]), &(acadoWorkspace.sbar[ 70 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 63 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 441 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 441 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 392 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 392 ]), &(acadoWorkspace.sbar[ 56 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 49 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 7 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 343 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 49 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 343 ]), &(acadoWorkspace.sbar[ 49 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 42 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 294 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 294 ]), &(acadoWorkspace.sbar[ 42 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 35 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 5 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 245 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 35 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 245 ]), &(acadoWorkspace.sbar[ 35 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 28 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 196 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 196 ]), &(acadoWorkspace.sbar[ 28 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 21 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 147 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 147 ]), &(acadoWorkspace.sbar[ 21 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 98 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 14 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 98 ]), &(acadoWorkspace.sbar[ 14 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 7 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 1 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 49 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 7 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 49 ]), &(acadoWorkspace.sbar[ 7 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


tmp = acadoWorkspace.sbar[13] + acadoVariables.x[13];
acadoWorkspace.lbA[0] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[0] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[20] + acadoVariables.x[20];
acadoWorkspace.lbA[1] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[1] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[27] + acadoVariables.x[27];
acadoWorkspace.lbA[2] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[2] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[34] + acadoVariables.x[34];
acadoWorkspace.lbA[3] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[3] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[41] + acadoVariables.x[41];
acadoWorkspace.lbA[4] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[4] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[48] + acadoVariables.x[48];
acadoWorkspace.lbA[5] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[5] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[55] + acadoVariables.x[55];
acadoWorkspace.lbA[6] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[6] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[62] + acadoVariables.x[62];
acadoWorkspace.lbA[7] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[7] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[69] + acadoVariables.x[69];
acadoWorkspace.lbA[8] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[8] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[76] + acadoVariables.x[76];
acadoWorkspace.lbA[9] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[9] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[83] + acadoVariables.x[83];
acadoWorkspace.lbA[10] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[10] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[90] + acadoVariables.x[90];
acadoWorkspace.lbA[11] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[11] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[97] + acadoVariables.x[97];
acadoWorkspace.lbA[12] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[12] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[104] + acadoVariables.x[104];
acadoWorkspace.lbA[13] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[13] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[111] + acadoVariables.x[111];
acadoWorkspace.lbA[14] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[14] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[118] + acadoVariables.x[118];
acadoWorkspace.lbA[15] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[15] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[125] + acadoVariables.x[125];
acadoWorkspace.lbA[16] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[16] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[132] + acadoVariables.x[132];
acadoWorkspace.lbA[17] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[17] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[139] + acadoVariables.x[139];
acadoWorkspace.lbA[18] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[18] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[146] + acadoVariables.x[146];
acadoWorkspace.lbA[19] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[19] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[153] + acadoVariables.x[153];
acadoWorkspace.lbA[20] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[20] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[160] + acadoVariables.x[160];
acadoWorkspace.lbA[21] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[21] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[167] + acadoVariables.x[167];
acadoWorkspace.lbA[22] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[22] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[174] + acadoVariables.x[174];
acadoWorkspace.lbA[23] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[23] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[181] + acadoVariables.x[181];
acadoWorkspace.lbA[24] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[24] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[188] + acadoVariables.x[188];
acadoWorkspace.lbA[25] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[25] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[195] + acadoVariables.x[195];
acadoWorkspace.lbA[26] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[26] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[202] + acadoVariables.x[202];
acadoWorkspace.lbA[27] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[27] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[209] + acadoVariables.x[209];
acadoWorkspace.lbA[28] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[28] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[216] + acadoVariables.x[216];
acadoWorkspace.lbA[29] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[29] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[223] + acadoVariables.x[223];
acadoWorkspace.lbA[30] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[30] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[230] + acadoVariables.x[230];
acadoWorkspace.lbA[31] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[31] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[237] + acadoVariables.x[237];
acadoWorkspace.lbA[32] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[32] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[244] + acadoVariables.x[244];
acadoWorkspace.lbA[33] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[33] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[251] + acadoVariables.x[251];
acadoWorkspace.lbA[34] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[34] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[258] + acadoVariables.x[258];
acadoWorkspace.lbA[35] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[35] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[265] + acadoVariables.x[265];
acadoWorkspace.lbA[36] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[36] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[272] + acadoVariables.x[272];
acadoWorkspace.lbA[37] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[37] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[279] + acadoVariables.x[279];
acadoWorkspace.lbA[38] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[38] = (real_t)4.0799904592075237e-01 - tmp;
tmp = acadoWorkspace.sbar[286] + acadoVariables.x[286];
acadoWorkspace.lbA[39] = (real_t)-4.0799904592075237e-01 - tmp;
acadoWorkspace.ubA[39] = (real_t)4.0799904592075237e-01 - tmp;

acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 7 ]), &(acadoWorkspace.sbar[ 7 ]), &(acadoWorkspace.lbA[ 41 ]), &(acadoWorkspace.ubA[ 41 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 14 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 21 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.lbA[ 43 ]), &(acadoWorkspace.ubA[ 43 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 28 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.lbA[ 44 ]), &(acadoWorkspace.ubA[ 44 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 35 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.lbA[ 45 ]), &(acadoWorkspace.ubA[ 45 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.lbA[ 46 ]), &(acadoWorkspace.ubA[ 46 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 49 ]), &(acadoWorkspace.sbar[ 49 ]), &(acadoWorkspace.lbA[ 47 ]), &(acadoWorkspace.ubA[ 47 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 63 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.lbA[ 49 ]), &(acadoWorkspace.ubA[ 49 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.lbA[ 50 ]), &(acadoWorkspace.ubA[ 50 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 77 ]), &(acadoWorkspace.sbar[ 77 ]), &(acadoWorkspace.lbA[ 51 ]), &(acadoWorkspace.ubA[ 51 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.lbA[ 52 ]), &(acadoWorkspace.ubA[ 52 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 91 ]), &(acadoWorkspace.sbar[ 91 ]), &(acadoWorkspace.lbA[ 53 ]), &(acadoWorkspace.ubA[ 53 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 98 ]), &(acadoWorkspace.sbar[ 98 ]), &(acadoWorkspace.lbA[ 54 ]), &(acadoWorkspace.ubA[ 54 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 105 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.lbA[ 55 ]), &(acadoWorkspace.ubA[ 55 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.lbA[ 56 ]), &(acadoWorkspace.ubA[ 56 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 119 ]), &(acadoWorkspace.sbar[ 119 ]), &(acadoWorkspace.lbA[ 57 ]), &(acadoWorkspace.ubA[ 57 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 126 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.lbA[ 58 ]), &(acadoWorkspace.ubA[ 58 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 133 ]), &(acadoWorkspace.sbar[ 133 ]), &(acadoWorkspace.lbA[ 59 ]), &(acadoWorkspace.ubA[ 59 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.lbA[ 60 ]), &(acadoWorkspace.ubA[ 60 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 147 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.lbA[ 61 ]), &(acadoWorkspace.ubA[ 61 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 154 ]), &(acadoWorkspace.sbar[ 154 ]), &(acadoWorkspace.lbA[ 62 ]), &(acadoWorkspace.ubA[ 62 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 161 ]), &(acadoWorkspace.sbar[ 161 ]), &(acadoWorkspace.lbA[ 63 ]), &(acadoWorkspace.ubA[ 63 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.lbA[ 64 ]), &(acadoWorkspace.ubA[ 64 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 175 ]), &(acadoWorkspace.sbar[ 175 ]), &(acadoWorkspace.lbA[ 65 ]), &(acadoWorkspace.ubA[ 65 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 182 ]), &(acadoWorkspace.sbar[ 182 ]), &(acadoWorkspace.lbA[ 66 ]), &(acadoWorkspace.ubA[ 66 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 189 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.lbA[ 67 ]), &(acadoWorkspace.ubA[ 67 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 196 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.lbA[ 68 ]), &(acadoWorkspace.ubA[ 68 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 203 ]), &(acadoWorkspace.sbar[ 203 ]), &(acadoWorkspace.lbA[ 69 ]), &(acadoWorkspace.ubA[ 69 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.sbar[ 210 ]), &(acadoWorkspace.lbA[ 70 ]), &(acadoWorkspace.ubA[ 70 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 217 ]), &(acadoWorkspace.sbar[ 217 ]), &(acadoWorkspace.lbA[ 71 ]), &(acadoWorkspace.ubA[ 71 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.sbar[ 224 ]), &(acadoWorkspace.lbA[ 72 ]), &(acadoWorkspace.ubA[ 72 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 231 ]), &(acadoWorkspace.sbar[ 231 ]), &(acadoWorkspace.lbA[ 73 ]), &(acadoWorkspace.ubA[ 73 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 238 ]), &(acadoWorkspace.sbar[ 238 ]), &(acadoWorkspace.lbA[ 74 ]), &(acadoWorkspace.ubA[ 74 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 245 ]), &(acadoWorkspace.sbar[ 245 ]), &(acadoWorkspace.lbA[ 75 ]), &(acadoWorkspace.ubA[ 75 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.lbA[ 76 ]), &(acadoWorkspace.ubA[ 76 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 259 ]), &(acadoWorkspace.sbar[ 259 ]), &(acadoWorkspace.lbA[ 77 ]), &(acadoWorkspace.ubA[ 77 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 266 ]), &(acadoWorkspace.sbar[ 266 ]), &(acadoWorkspace.lbA[ 78 ]), &(acadoWorkspace.ubA[ 78 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 273 ]), &(acadoWorkspace.sbar[ 273 ]), &(acadoWorkspace.lbA[ 79 ]), &(acadoWorkspace.ubA[ 79 ]) );

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
for (lRun1 = 0; lRun1 < 280; ++lRun1)
acadoWorkspace.sbar[lRun1 + 7] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 7 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 49 ]), &(acadoWorkspace.evGu[ 7 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.sbar[ 7 ]), &(acadoWorkspace.sbar[ 14 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 98 ]), &(acadoWorkspace.evGu[ 14 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 147 ]), &(acadoWorkspace.evGu[ 21 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 196 ]), &(acadoWorkspace.evGu[ 28 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 35 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 245 ]), &(acadoWorkspace.evGu[ 35 ]), &(acadoWorkspace.x[ 5 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 294 ]), &(acadoWorkspace.evGu[ 42 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 49 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 343 ]), &(acadoWorkspace.evGu[ 49 ]), &(acadoWorkspace.x[ 7 ]), &(acadoWorkspace.sbar[ 49 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 392 ]), &(acadoWorkspace.evGu[ 56 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.evGu[ 63 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 490 ]), &(acadoWorkspace.evGu[ 70 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 77 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 539 ]), &(acadoWorkspace.evGu[ 77 ]), &(acadoWorkspace.x[ 11 ]), &(acadoWorkspace.sbar[ 77 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 588 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 91 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 637 ]), &(acadoWorkspace.evGu[ 91 ]), &(acadoWorkspace.x[ 13 ]), &(acadoWorkspace.sbar[ 91 ]), &(acadoWorkspace.sbar[ 98 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 686 ]), &(acadoWorkspace.evGu[ 98 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 98 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 735 ]), &(acadoWorkspace.evGu[ 105 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 784 ]), &(acadoWorkspace.evGu[ 112 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 119 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 833 ]), &(acadoWorkspace.evGu[ 119 ]), &(acadoWorkspace.x[ 17 ]), &(acadoWorkspace.sbar[ 119 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 882 ]), &(acadoWorkspace.evGu[ 126 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 133 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 931 ]), &(acadoWorkspace.evGu[ 133 ]), &(acadoWorkspace.x[ 19 ]), &(acadoWorkspace.sbar[ 133 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 980 ]), &(acadoWorkspace.evGu[ 140 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1029 ]), &(acadoWorkspace.evGu[ 147 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 154 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1078 ]), &(acadoWorkspace.evGu[ 154 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 154 ]), &(acadoWorkspace.sbar[ 161 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1127 ]), &(acadoWorkspace.evGu[ 161 ]), &(acadoWorkspace.x[ 23 ]), &(acadoWorkspace.sbar[ 161 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1176 ]), &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 175 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1225 ]), &(acadoWorkspace.evGu[ 175 ]), &(acadoWorkspace.x[ 25 ]), &(acadoWorkspace.sbar[ 175 ]), &(acadoWorkspace.sbar[ 182 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1274 ]), &(acadoWorkspace.evGu[ 182 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 182 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1323 ]), &(acadoWorkspace.evGu[ 189 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 196 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1372 ]), &(acadoWorkspace.evGu[ 196 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.sbar[ 203 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1421 ]), &(acadoWorkspace.evGu[ 203 ]), &(acadoWorkspace.x[ 29 ]), &(acadoWorkspace.sbar[ 203 ]), &(acadoWorkspace.sbar[ 210 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1470 ]), &(acadoWorkspace.evGu[ 210 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 210 ]), &(acadoWorkspace.sbar[ 217 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1519 ]), &(acadoWorkspace.evGu[ 217 ]), &(acadoWorkspace.x[ 31 ]), &(acadoWorkspace.sbar[ 217 ]), &(acadoWorkspace.sbar[ 224 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1568 ]), &(acadoWorkspace.evGu[ 224 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 224 ]), &(acadoWorkspace.sbar[ 231 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1617 ]), &(acadoWorkspace.evGu[ 231 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 231 ]), &(acadoWorkspace.sbar[ 238 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1666 ]), &(acadoWorkspace.evGu[ 238 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 238 ]), &(acadoWorkspace.sbar[ 245 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1715 ]), &(acadoWorkspace.evGu[ 245 ]), &(acadoWorkspace.x[ 35 ]), &(acadoWorkspace.sbar[ 245 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1764 ]), &(acadoWorkspace.evGu[ 252 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 259 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1813 ]), &(acadoWorkspace.evGu[ 259 ]), &(acadoWorkspace.x[ 37 ]), &(acadoWorkspace.sbar[ 259 ]), &(acadoWorkspace.sbar[ 266 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1862 ]), &(acadoWorkspace.evGu[ 266 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 266 ]), &(acadoWorkspace.sbar[ 273 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1911 ]), &(acadoWorkspace.evGu[ 273 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 273 ]), &(acadoWorkspace.sbar[ 280 ]) );
for (lRun1 = 0; lRun1 < 287; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 40; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 7];
acadoWorkspace.state[1] = acadoVariables.x[index * 7 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 7 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 7 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 7 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 7 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 7 + 6];
acadoWorkspace.state[63] = acadoVariables.u[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 7 + 7] = acadoWorkspace.state[0];
acadoVariables.x[index * 7 + 8] = acadoWorkspace.state[1];
acadoVariables.x[index * 7 + 9] = acadoWorkspace.state[2];
acadoVariables.x[index * 7 + 10] = acadoWorkspace.state[3];
acadoVariables.x[index * 7 + 11] = acadoWorkspace.state[4];
acadoVariables.x[index * 7 + 12] = acadoWorkspace.state[5];
acadoVariables.x[index * 7 + 13] = acadoWorkspace.state[6];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 40; ++index)
{
acadoVariables.x[index * 7] = acadoVariables.x[index * 7 + 7];
acadoVariables.x[index * 7 + 1] = acadoVariables.x[index * 7 + 8];
acadoVariables.x[index * 7 + 2] = acadoVariables.x[index * 7 + 9];
acadoVariables.x[index * 7 + 3] = acadoVariables.x[index * 7 + 10];
acadoVariables.x[index * 7 + 4] = acadoVariables.x[index * 7 + 11];
acadoVariables.x[index * 7 + 5] = acadoVariables.x[index * 7 + 12];
acadoVariables.x[index * 7 + 6] = acadoVariables.x[index * 7 + 13];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[280] = xEnd[0];
acadoVariables.x[281] = xEnd[1];
acadoVariables.x[282] = xEnd[2];
acadoVariables.x[283] = xEnd[3];
acadoVariables.x[284] = xEnd[4];
acadoVariables.x[285] = xEnd[5];
acadoVariables.x[286] = xEnd[6];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[280];
acadoWorkspace.state[1] = acadoVariables.x[281];
acadoWorkspace.state[2] = acadoVariables.x[282];
acadoWorkspace.state[3] = acadoVariables.x[283];
acadoWorkspace.state[4] = acadoVariables.x[284];
acadoWorkspace.state[5] = acadoVariables.x[285];
acadoWorkspace.state[6] = acadoVariables.x[286];
if (uEnd != 0)
{
acadoWorkspace.state[63] = uEnd[0];
}
else
{
acadoWorkspace.state[63] = acadoVariables.u[39];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[280] = acadoWorkspace.state[0];
acadoVariables.x[281] = acadoWorkspace.state[1];
acadoVariables.x[282] = acadoWorkspace.state[2];
acadoVariables.x[283] = acadoWorkspace.state[3];
acadoVariables.x[284] = acadoWorkspace.state[4];
acadoVariables.x[285] = acadoWorkspace.state[5];
acadoVariables.x[286] = acadoWorkspace.state[6];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 39; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[39] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39];
kkt = fabs( kkt );
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 80; ++index)
{
prd = acadoWorkspace.y[index + 40];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 8 */
real_t tmpDy[ 8 ];

/** Row vector of size: 7 */
real_t tmpDyN[ 7 ];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 7];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 7 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 7 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 7 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 7 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 7 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 7 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 8] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 8];
acadoWorkspace.Dy[lRun1 * 8 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 8 + 1];
acadoWorkspace.Dy[lRun1 * 8 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 8 + 2];
acadoWorkspace.Dy[lRun1 * 8 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 8 + 3];
acadoWorkspace.Dy[lRun1 * 8 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 8 + 4];
acadoWorkspace.Dy[lRun1 * 8 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 8 + 5];
acadoWorkspace.Dy[lRun1 * 8 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 8 + 6];
acadoWorkspace.Dy[lRun1 * 8 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 8 + 7];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[280];
acadoWorkspace.objValueIn[1] = acadoVariables.x[281];
acadoWorkspace.objValueIn[2] = acadoVariables.x[282];
acadoWorkspace.objValueIn[3] = acadoVariables.x[283];
acadoWorkspace.objValueIn[4] = acadoVariables.x[284];
acadoWorkspace.objValueIn[5] = acadoVariables.x[285];
acadoWorkspace.objValueIn[6] = acadoVariables.x[286];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 8]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 8 + 1]*acadoVariables.W[9];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 8 + 2]*acadoVariables.W[18];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 8 + 3]*acadoVariables.W[27];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 8 + 4]*acadoVariables.W[36];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 8 + 5]*acadoVariables.W[45];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 8 + 6]*acadoVariables.W[54];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 8 + 7]*acadoVariables.W[63];
objVal += + acadoWorkspace.Dy[lRun1 * 8]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 8 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 8 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 8 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 8 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 8 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 8 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 8 + 7]*tmpDy[7];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[8];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[16];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[24];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[32];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[40];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[48];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6];

objVal *= 0.5;
return objVal;
}

