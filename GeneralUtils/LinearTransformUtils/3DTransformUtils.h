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
 * \file 3DTransformUtils.h
 * \author R�mi Blanc 
 * \26. September 2011
*/

#ifndef _3DTRANSFORMUTILS_H_
#define _3DTRANSFORMUTILS_H_

#include "GeneralUtils.h"
#include "TransformUtils.h"


namespace psciob {


typedef enum {
	HORIZONTAL, //(x,y) plane - look from down to up (default direction)
	SAGITTAL,	//(y,z) plane - look from right to left
	CORONAL		//(x,z) plane - look from anterior to posterior (front to back)
} ObservationDirectionType;


/** Convention for euler angles is heading, attitude, bank */
vnl_matrix<double> Get3DRotationMatrixFromEulerAngles(float h, float a, float b);

/** Convention for euler angles is heading, attitude, bank */
vnl_matrix<double> Get3DRotationMatrixFromEulerAngles(const vnl_vector<double> &euler_angles);

/** code adapted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm */
vnl_vector<double> GetEulerAnglesFrom3DRotationMatrix(const vnl_matrix<double> &mat);


/** */
vnl_matrix<double> Rotate3DTransformAroundX(const vnl_matrix<double> &R, double angle);
void Rotate3DTransformAroundX_InPlace(vnl_matrix<double> &R, double angle);

/** */
vnl_matrix<double> Rotate3DTransformAroundY(const vnl_matrix<double> &R, double angle);
void Rotate3DTransformAroundY_InPlace(vnl_matrix<double> &R, double angle);

/** */
vnl_matrix<double> Rotate3DTransformAroundZ(const vnl_matrix<double> &R, double angle);
void Rotate3DTransformAroundZ_InPlace(vnl_matrix<double> &R, double angle);


} // namespace psciob


#endif //_3DTRANSFORMUTILS_H_