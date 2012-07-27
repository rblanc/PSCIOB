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
 * \file ITKUtils.cpp
 * \author Rémi Blanc 
 * \date 29. August 2011
*/

#include "ITKUtils.h"

using namespace psciob;

/**
*/
void psciob::GetImageOriginAndSizeFromPhysicalBoundingBoxAndSpacing(vnl_vector<double> pBB, vnl_vector<double> sp, vnl_vector<double> *origin, vnl_vector<unsigned int> *size) {
	if ( (2*sp.size()!=pBB.size()) ) {throw DeformableModelException("GetImageOriginAndSizeFromPhysicalBoundingBoxAndSpacing: inconsistent parameters ");}
	for (unsigned int i=0 ; i<sp.size() ; i++) {
		(*origin)[i] = floor(pBB(2*i)/sp[i]+TINY+0.5)*sp[i];	//Image Origin is the physical coordinates of the center of the left-most pixel (left/down/front....)
		(*size)[i] = max((unsigned int)1,(unsigned int)(1+ceil(pBB(2*i+1)/sp[i]-TINY-0.5) - floor(pBB(2*i)/sp[i]+TINY+0.5)));
	}
}

