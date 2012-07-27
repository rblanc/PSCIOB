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
 * \file PoseTransform.txx
 * \author Rémi Blanc 
 * \date 12. October 2011
*/


#include "PoseTransform.h"

namespace psciob {


//
// Constructor
//
template < unsigned int VDimension, class TAppearance> 
PoseTransform<VDimension, TAppearance>::PoseTransform() : TexturedDeformableModel(), 
m_flagPostMultiply(false), m_interpolationType(NEARESTNEIGHBOR) {
	m_inputShape = NULL;
}

//
// Get Parameters
//
//GetParameters
template < unsigned int VDimension, class TAppearance> 
inline
vnl_vector<double> PoseTransform<VDimension, TAppearance>::GetParameters() {
	return m_parameters;
}


//GetInverseTransformMatrix
template < unsigned int VDimension, class TAppearance> 
inline
vnl_matrix<double> PoseTransform<VDimension, TAppearance>::GetInverseTransformMatrix() {
	return vnl_matrix_inverse<double>(this->GetTransformMatrix());
	//if (!m_inverseMatrixUptodate) { 
	//	m_inverseTransformMatrix = vnl_matrix_inverse<double>(this->GetTransformMatrix()); m_inverseMatrixUptodate = true; 
	//}
	//return m_inverseTransformMatrix; 
} 


//SetTransformationMatrix 
template < unsigned int VDimension, class TAppearance> 
bool PoseTransform<VDimension, TAppearance>::SetTransformationMatrix(const vnl_matrix<double> &mat) {//WARNING: verify that this is an authorized transformation ; this requires to decompose the input transform into the parametric space, reconstruct the matrix, and compare if it is similar
	if ( (mat.rows() != mat.cols()) || (mat.rows() != m_transformMatrix.rows()) ) return false;
	m_parameters = GetParametersFromMatrix(mat);
	//m_transformMatrix = mat; m_transformMatrixUptodate = true;
	//m_inverseMatrixUptodate = false; 
	return true;
}



//Scale
template < unsigned int VDimension, class TAppearance> 
bool PoseTransform<VDimension, TAppearance>::Scale(double scale) {
	throw DeformableModelException("PoseTransform::Scale - not re-implemented... - should add checks whether this is an allowed transformation...");
	vnl_matrix<double> mat(VDimension+1, VDimension+1);
	mat.set_identity();		for (unsigned i=0 ; i<VDimension ; i++) { mat(i,i) = scale; }
	return Compose_Internal(mat);
}


//Scale
template < unsigned int VDimension, class TAppearance> 
bool PoseTransform<VDimension, TAppearance>::Scale(const vnl_vector<double> &scale) {
	throw DeformableModelException("PoseTransform::Scale - not re-implemented... - should add checks whether this is an allowed transformation...");
	if ( scale.size() != VDimension ) { throw DeformableModelException("Error in PoseTransform::Scale : invalid parameter size"); }
	vnl_matrix<double> mat(VDimension+1, VDimension+1);
	mat.set_identity();		for (unsigned i=0 ; i<VDimension ; i++) { mat(i,i) = scale(i); }
	return Compose_Internal(mat);
}


//Compose -- WARNING: no checks are done on the validity of the matrix
template < unsigned int VDimension, class TAppearance> 
inline
bool PoseTransform<VDimension, TAppearance>::Compose(const vnl_matrix<double> &transf) {
	if ( (transf.rows() != transf.cols()) || (transf.rows() != VDimension+1) ) return false;
	//OPTIMIZATION
	//this should check whether the input transform implies any modifications -> compare to the identity matrix...
	//a check should be implementing by transforming the matrix to a parameter representation ; then back in a matrix, and comparing with the original one... ; up to the related numerical precision...
	Compose_Internal(transf);
	return true;
}


//ModifyTransformWithParameters : may be the best way to modify the transform, use post translation and pre-rotation/scaling etc...
template < unsigned int VDimension, class TAppearance> 
bool PoseTransform<VDimension, TAppearance>::ModifyTransformWithParameters(const vnl_vector<double> &poseParameters) {
	if (!CheckParameters(poseParameters)) return false;
	vnl_matrix<double> tmp_mat = GetMatrixFromParameters(poseParameters);
	throw DeformableModelException("ModifyTransformWithParameters: not implemented at the moment, check the source for hints");
	//TODO: check for integer translation ; and whether these pose parameters imply a real modification of the matrix before composing...
	// using ComposeParametersAreNew
	//when composing, pre-compose the matrix part of the transform, and post-apply the translation
	//Compose_Internal(tmp_mat);
	return true;
}
//Compose_Internal -- protected, no validity check are performed ; the method calls Modified without checking if the parameters are really changed <= check this before calling it.
template < unsigned int VDimension, class TAppearance> 
bool PoseTransform<VDimension, TAppearance>::Compose_Internal(const vnl_matrix<double> &transf) {
	unsigned int n = VDimension;
	vnl_matrix<double> transformMatrix = GetTransformMatrix(); //make sure the matrix is uptodate
	vnl_matrix<double> tmp_mat = transformMatrix.extract(n,n,0,0), tmp_vect;
	if (!m_flagPostMultiply) {
		tmp_vect= transf.extract(n,n,0,0)*transformMatrix.extract(n,1,0,n) + transf.extract(n,1,0,n);
		tmp_mat = transf.extract(n,n,0,0)*tmp_mat;
	}
	else {
		tmp_vect= tmp_mat*transf.extract(n,1,0,n) + transformMatrix.extract(n,1,0,n);
		tmp_mat = tmp_mat*transf.extract(n,n,0,0);
	}
	for (unsigned i=0 ; i<n ; i++) {
		transformMatrix(i,n) = tmp_vect(i,0);
		for (unsigned j=0 ; j<n ; j++) { transformMatrix(i,j) = tmp_mat(i,j); }
	}
	
	m_parameters = GetParametersFromMatrix(transformMatrix);
	//m_transformMatrixUptodate = true;
	//m_inverseMatrixUptodate = false; 
	return true;
}


//ApplyGridIntegerTranslationToImage
template < unsigned int VDimension, class TAppearance> 
void PoseTransform<VDimension, TAppearance>::ApplyGridIntegerTranslationToImage(const vnl_vector<double> &trans) {
	if (m_uptodateBinaryImage) {
		BinaryImageType::PointType origin = m_outputBinaryImage->GetOrigin();
		for (unsigned i=0 ; i<VDimension ; i++) { origin[i] += trans(i); }
		m_outputBinaryImage->SetOrigin(origin);
	}
	if (m_uptodateTexturedImage) {
		TexturedImageType::PointType origin = m_outputTexturedImage->GetOrigin();
		for (unsigned i=0 ; i<VDimension ; i++) { origin[i] += trans(i); }
		m_outputTexturedImage->SetOrigin(origin);
	}
	if (m_uptodateLabelMap) {
		LabelMapType::PointType origin = m_outputLabelMap->GetOrigin();
		for (unsigned i=0 ; i<VDimension ; i++) { origin[i] += trans(i); }
		m_outputLabelMap->SetOrigin(origin);
	}
}


} // namespace psciob

