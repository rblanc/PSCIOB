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

/*
* \file TranslationScale3DTransform.cpp
* \author R�mi Blanc 
 * \date 7. August 2012
*/


#include "TranslationScale3DTransform.h"

using namespace psciob;


const std::string TranslationScale3DTransform::m_name = "TranslationScale3DTransform";

TranslationScale3DTransform::TranslationScale3DTransform() : Abstract3DTransform() {SetDefaultParameters();}


vnl_vector<double> TranslationScale3DTransform::GetDefaultParameters() const {
	vnl_vector<double> params(m_nbParams); params.fill(0); params(m_nbParams-1)=1;
	return params;
}

vnl_matrix<double> TranslationScale3DTransform::GetMatrixFromParameters(const vnl_vector<double> &poseParameters) const {
	vnl_matrix<double> output(4,4); 

	output.set_identity();
	for (unsigned i = 0 ; i<3 ; i++) { 
		output(i,i) = poseParameters(3);
		output(i,3) = poseParameters(i); 
	}

	return output;
}


vnl_vector<double> TranslationScale3DTransform::GetParametersFromMatrix(const vnl_matrix<double> &transformMatrix) const {
	vnl_vector<double> params(m_nbParams);

	params(0) = transformMatrix(0,m_nbDimensions);
	params(1) = transformMatrix(1,m_nbDimensions);
	params(2) = transformMatrix(2,m_nbDimensions);
	params(3) = transformMatrix(0,0); //the first n-1 diagonal components should all be equal!
	
	return params;
}
