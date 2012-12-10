/*
 * This file is part of the psciob library.
 *
 * Copyright (c) 2012, Remi Blanc, University of Bordeaux
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file 2DTransformUtils.cpp
 * \author Rémi Blanc 
 * \date 27. February 2012
*/

#include "2DTransformUtils.h"

using namespace psciob;

vnl_matrix<double> psciob::Get2DRotationMatrixFromAngle(double angle) {
	vnl_matrix<double> rotmat(2,2);
	rotmat.put(0,0, cos(angle));	rotmat.put(0,1,-sin(angle));
	rotmat.put(1,0, sin(angle));	rotmat.put(1,1, cos(angle));

	return rotmat;
}



double psciob::GetAngleFrom2DRotationMatrix(const vnl_matrix<double> &mat) {
	return atan2(mat(1,0),mat(0,0));
}



/** template function that is specialized for D=2 and D=3, though not implemented otherwise
* It computes a new versions of the bases, such that they are both direct bases, and they are oriented as similarly as possible (direction can be reversed such that the corresponding scalar product is >0 ; last axis has priority)
* \warning no checks are performed on the validity of the input matrices (dimension, orthonormality of entries, etc...)
*/
void psciob::ReOrient2DCoordinateSystems(const vnl_matrix<double> &U1, const vnl_matrix<double> &U2, vnl_matrix<double> &newU1, vnl_matrix<double> &newU2) {
	newU1.set_size(2,2); newU2.set_size(2,2);
	newU1.set_column(0, U1.get_column(1));
	newU2.set_column(0, U2.get_column(1));
	//now, make sure their axes tend to point in the same direction
	double dp0 = dot_product(newU1.get_column(0), newU2.get_column(0));
	//then, the second axis is obtained such that is is orthogonal, and form a direct basis
	if (dp0<0) newU2.set_column(0, -newU2.get_column(0));
	newU1(0,1) = -newU1(1,0); newU1(1,1) = newU1(0,0);
	newU2(0,1) = -newU2(1,0); newU2(1,1) = newU2(0,0);
}
