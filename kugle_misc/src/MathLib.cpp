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
 
#include "kugle_misc/MathLib.h"
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memcpy
#include <stdlib.h>     /* malloc, free, rand */

namespace kugle_misc {

double Math_Round(double num, unsigned int dec)
{
	//double power = powf(10, dec);
	long long power = 1;
	for (unsigned int i = 0; i < dec; i++) {
		power *= 10;
	}

	return roundf(num * power) / power;
}

void Math_SymmetrizeSquareMatrix(double * mat, unsigned int rows)
{
	double * in = (double *)malloc(sizeof(double) * rows * rows);
	if (!in) return;

	memcpy(in, mat, sizeof(double) * rows * rows);

	for (unsigned int i = 0; i < rows; i++) {
		for (unsigned int j = 0; j < rows; j++) {
			mat[rows*i + j] = (in[rows*i + j] + in[rows*j + i]) / 2.0;
		}
	}

	free(in);
}

void Math_Rotate2D(const double V[2], const double theta, double Vr[2])
{
	Vr[0] = cosf(theta) * V[0] - sinf(theta) * V[1];
	Vr[1] = sinf(theta) * V[0] + cosf(theta) * V[1];
}

} // end namespace kugle_misc
