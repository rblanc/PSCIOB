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
* \file Rigid3DTransform.cpp
* \author Rémi Blanc 
* \date 25. July 2011
*/


#include "Rigid3DTransform.h"

using namespace psciob;


const std::string Rigid3DTransform::m_name = "Rigid3DTransform";

Rigid3DTransform::Rigid3DTransform() : Abstract3DTransform() {SetDefaultParameters();}


vnl_vector<double> Rigid3DTransform::GetDefaultParameters() const {
	vnl_vector<double> params(m_nbParams); params.fill(0);
	return params;
}

vnl_matrix<double> Rigid3DTransform::GetMatrixFromParameters(const vnl_vector<double> &poseParameters) const {
	vnl_matrix<double> output(4,4); 
	output.set_identity();

	vnl_matrix<double> rotmat = Get3DRotationMatrixFromEulerAngles(poseParameters.extract(3,3));
	for (unsigned i = 0 ; i<3 ; i++) {
		output(i,3) = poseParameters(i);
		for (unsigned j = 0 ; j<3 ; j++) {
			output(i,j) = rotmat(i,j);
		}
	}
	return output;
}


vnl_vector<double> Rigid3DTransform::GetParametersFromMatrix(const vnl_matrix<double> &transformMatrix) const {
	vnl_vector<double> params(m_nbParams);

	params(0) = transformMatrix(0,m_nbDimensions);
	params(1) = transformMatrix(1,m_nbDimensions);
	params(2) = transformMatrix(2,m_nbDimensions);

	vnl_vector<double> tmpvect = GetEulerAnglesFrom3DRotationMatrix(transformMatrix.extract(3,3,0,0));

	params(3) = tmpvect(0);
	params(4) = tmpvect(1);
	params(5) = tmpvect(2);

	return params;
}
