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
 * \file 2DTransformUtils.h
 * \author R�mi Blanc 
 * \26. September 2011
*/

#ifndef _2DTRANSFORMUTILS_H_
#define _2DTRANSFORMUTILS_H_

#include "GeneralUtils.h"
#include "TransformUtils.h"

namespace psciob {


/** convention: usual trigonometric convention, in radian units, with the 1st axis as origin */
vnl_matrix<double> Get2DRotationMatrixFromAngle(double angle);

/** */
double GetAngleFrom2DRotationMatrix(const vnl_matrix<double> &mat);

/** template function that is specialized for D=2 and D=3, though not implemented otherwise
* It computes a new versions of the bases, such that they are both direct bases, and they are oriented as similarly as possible (direction can be reversed such that the corresponding scalar product is >0 ; last axis has priority)
* \warning no checks are performed on the validity of the input matrices (dimension, orthonormality of entries, etc...)
*/
void ReOrient2DCoordinateSystems(const vnl_matrix<double> &U1, const vnl_matrix<double> &U2, vnl_matrix<double> &newU1, vnl_matrix<double> &newU2);


} // namespace psciob


#endif //_2DTRANSFORMUTILS_H_