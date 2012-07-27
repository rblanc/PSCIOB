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
* \file Similarity3DTransform.cpp
* \author Rémi Blanc 
* \date 25. July 2011
*/


#include "Similarity3DTransform.h"

using namespace psciob;


const std::string Similarity3DTransform::m_name = "Similarity3DTransform";

Similarity3DTransform::Similarity3DTransform() : Abstract3DTransform() {SetDefaultParameters();}


//7 parameters: 0=translation_x, 1=translation_y, 2=translation_z, 3=rotation_y, 4=rotation_z, 5=rotation_x, 6=scale
vnl_vector<double> Similarity3DTransform::GetDefaultParameters() const {
	vnl_vector<double> params(m_nbParams); params.fill(0); params(6)=1;
	return params;
}

vnl_matrix<double> Similarity3DTransform::GetMatrixFromParameters(const vnl_vector<double> &poseParameters) const {
	vnl_matrix<double> output(m_nbDimensions+1,m_nbDimensions+1); 
	output.set_identity();

	vnl_matrix<double> rotmat = Get3DRotationMatrixFromEulerAngles(poseParameters.extract(3,3));
	for (unsigned i = 0 ; i<m_nbDimensions ; i++) {
		output(i,m_nbDimensions) = poseParameters(i);
		for (unsigned j = 0 ; j<m_nbDimensions ; j++) {
			output(i,j) = poseParameters(6)*rotmat(i,j);
		}
	}
	return output;
}


vnl_vector<double> Similarity3DTransform::GetParametersFromMatrix(const vnl_matrix<double> &transformMatrix) const {
	vnl_vector<double> params(m_nbParams);
	vnl_matrix<double> rotmat = transformMatrix.extract(m_nbDimensions,m_nbDimensions,0,0);

	params(6) = pow( vnl_determinant(rotmat), 1.0/3.0);
	rotmat /= params(6);

	params(0) = transformMatrix(0,m_nbDimensions);
	params(1) = transformMatrix(1,m_nbDimensions);
	params(2) = transformMatrix(2,m_nbDimensions);

	vnl_vector<double> tmpvect = GetEulerAnglesFrom3DRotationMatrix(rotmat);

	params(3) = tmpvect(0);	params(4) = tmpvect(1);	params(5) = tmpvect(2);

	return params;
}