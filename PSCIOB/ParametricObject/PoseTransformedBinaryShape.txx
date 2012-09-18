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
* \file PoseTransformedBinaryShape.txx
* \author Rémi Blanc 
* \date 25. July 2011
*/


#include "PoseTransformedBinaryShape.h"

namespace psciob {


//
// Constructor
//
template <unsigned int VDimension>
PoseTransformedBinaryShape<VDimension>::PoseTransformedBinaryShape() : BinaryDeformableModel(), 
m_inputFlag(false) 
{
	////potentially, allow this parameter to be changed, and use it as constant value for setting the 'textured' image level.
	//m_appearanceParameter.set_size(m_nbAppearanceParams); m_appearanceParameter(0) = 1;
}

//
// Create Another
//

template <unsigned int VDimension>
itk::LightObject::Pointer
PoseTransformedBinaryShape<VDimension>::CreateAnother(void) const {
	Pointer copyPtr = Self::New().GetPointer();	
	copyPtr->SetShapeAndTransform(static_cast<ShapeType*>(m_shape->CreateAnother().GetPointer()), static_cast<TransformType*>(m_transform->CreateAnother().GetPointer()));
	return static_cast<BaseClass*>( copyPtr );
}

//
//SetShapeAndTransform : set associate objects
//
template <unsigned int VDimension>
void PoseTransformedBinaryShape<VDimension>::SetShapeAndTransform(ShapeType *shape, TransformType *transf) {//
	m_shape = shape;
	m_transform = transf; m_transform->SetInput(m_shape);

	m_parameters = CollectParameters();
	m_vtkResolution = m_shape->GetVTKResolution();
	m_imageSpacing = m_shape->GetImageSpacing();
	m_transform->SetImageSpacing( m_shape->GetImageSpacing() );
	m_transform->SetVTKResolution(m_vtkResolution);

	Modified();
	m_inputFlag = true;
}


/** Get Parameters */
template <unsigned int VDimension>
vnl_vector<double> 
PoseTransformedBinaryShape<VDimension>::GetDefaultParameters() const {
	vnl_vector<double> params(GetNumberOfParameters());
	vnl_vector<double> tmp = m_transform->GetDefaultParameters();
	for (unsigned i=0 ; i<tmp.size() ; i++) params(i) = tmp(i);
	tmp = m_shape->GetDefaultParameters();
	for (unsigned i=0 ; i<tmp.size() ; i++) params(i+m_transform->GetNumberOfParameters()-1) = tmp(i);
	return params;
}



//
//Collect parameters from the shape and transform ; the order is fixed : transform - shape - appearance
//
//CollectParameters
template <unsigned int VDimension>
vnl_vector<double> 
PoseTransformedBinaryShape<VDimension>::CollectParameters() {
	vnl_vector<double> poseParams = m_transform->GetParameters(), shapeParams = m_shape->GetParameters();
	unsigned int n1 = poseParams.size(), n2 = shapeParams.size(), i;
	vnl_vector<double> output(n1+n2);
	for (i=0 ; i<n1 ; i++)	{ output(i) = poseParams(i); }
	for (i=0 ; i<n2 ; i++)	{ output(i+n1) = shapeParams(i); }
	return output;
}

//
//Set Parameters - internally check whether the parameters are valid, if not keep the parameters as they were and return false
//
//SetParameters
template <unsigned int VDimension>
bool 
PoseTransformedBinaryShape<VDimension>::SetParameters(const vnl_vector<double> &params) {
	if (!m_inputFlag) throw DeformableModelException("PoseTransformedBinaryShape: the shape and transform must be set before manipulating the parameters!");
	vnl_vector<double> poseParams = params.extract( m_transform->GetNumberOfParameters(), 0 );
	vnl_vector<double> shapeParams = params.extract( m_shape->GetNumberOfParameters(), m_transform->GetNumberOfParameters() );
	if (!m_transform->CheckParameters(poseParams)) { return false; }
	if (!m_shape->CheckParameters(shapeParams)) { return false; }	
	//First, check the shape parameters: if the shape parameters bring novelty, the transform will also need to be updated
	for (unsigned i=0 ; i<m_shape->GetNumberOfParameters() ; i++) { 
		if ( fabs( m_parameters(i+m_transform->GetNumberOfParameters()) - shapeParams(i) )>TINY ) {
			m_transform->_SetParameters_NoCheck( poseParams ); m_shape->_SetParameters_NoCheck( shapeParams ); m_parameters = params; Modified(); 
			return true; 
		}
	}

	//At this stage, the shape is not modified, so concentrate exclusively on the pose
	//First, look at the non-translation pose parameters
	for (unsigned i=Dimension ; i<m_transform->GetNumberOfParameters() ; i++) { 
		if ( fabs(m_parameters(i)-poseParams(i))>TINY ) {
			m_transform->_SetParameters_NoCheck( poseParams ); m_parameters = params; Modified();  
			return true; 
		}
	}

	//Finally, look at the translation parameters and check for a grid-integer translation
	vnl_vector<int> integerTranslation(Dimension); double tmpval;
	for (unsigned i=0 ; i<Dimension ; i++) {
		tmpval = (params(i)-m_parameters(i))/m_imageSpacing[i];
		integerTranslation(i) = round(tmpval); 
		if ( fabs(tmpval - round(tmpval)) > TINY ) { //non-grid integer translation
			m_transform->_SetParameters_NoCheck( poseParams ); m_parameters = params; Modified(); 
			return true; 
		}
	}
//std::cout<<"   PoseTransformedBinaryShape::SetParameters -- input = "<<params<<", integer grid translation: "<<integerTranslation<<std::endl;
	//At this point, nothing but the translation may be different, if it is a non-zero translation, just applied it
	this->_IntegerGridTranslate_NoCheck(integerTranslation);
//std::cout<<"   parametesr after _IntegerGridTranslate_NoCheck: "<<this->GetParameters()<<std::endl;
	return true;
}


/** Set Default Parameters */
template <unsigned int VDimension>
void 
PoseTransformedBinaryShape<VDimension>::SetDefaultParameters() {
	if (!m_inputFlag) throw DeformableModelException("PoseTransformedBinaryShape: the shape and transform must be set before manipulating the parameters!");
	m_transform->SetDefaultParameters();
	m_shape->SetDefaultParameters();
	m_parameters = CollectParameters();
	Modified();
}


/** Position At the specified location
* checks if some updates are necessary or if things boil down to a grid-integer translation
* return false if something goes wrong
*/
template <unsigned int VDimension>
bool 
PoseTransformedBinaryShape<VDimension>::PositionAt(const vnl_vector<double> &pos) {
	if (!m_inputFlag) throw DeformableModelException("PoseTransformedBinaryShape: the shape and transform must be set before manipulating the parameters!");

	if (pos.size()!=Dimension) return false;

	vnl_vector<int> integerTranslation(Dimension); bool zerotranslation = true; double tmpval;
	for (unsigned i=0 ; i<Dimension ; i++) {
		tmpval = ( pos(i)-m_parameters(i) ) / m_imageSpacing[i]; //continuous index
		integerTranslation(i) = round(tmpval); 
		if ( fabs(tmpval - integerTranslation(i)) > TINY ) { 
			//if one of the parameters is not a grid-integer translation, do the translation and exit (idea is to avoid the next divisions, rounding, ...)
			for (unsigned j=0 ; j<Dimension ; j++) m_parameters(j) = pos(j);
			Modified(); m_transform->_PositionAt_NoCheck(pos); return true; 
		}
		else if ( abs(integerTranslation(i)) > TINY ) zerotranslation = false;
	}
	//
	if (!zerotranslation) { //if necessary, update what can be updated both in this object and in the transform
		this->_IntegerGridTranslate_NoCheck(integerTranslation);
		//m_transform->_IntegerGridTranslate_NoCheck(integerTranslation);
	}
	return true;
}

/** Apply Translation
* checks if this is a grid-integer translation
* return false if something goes wrong
*/
template <unsigned int VDimension>
bool 
PoseTransformedBinaryShape<VDimension>::Translate(const vnl_vector<double> &translation) {
	if (translation.size()!=Dimension) return false;

	vnl_vector<int> integerTranslation(Dimension); bool zerotranslation = true; double tmpval;
	for (unsigned i=0 ; i<Dimension ; i++) {
		tmpval = translation(i) / m_imageSpacing[i];
		integerTranslation(i) = round(tmpval);
		if ( fabs(tmpval - integerTranslation(i)) > TINY ) { 
			//if one of the parameters is not a grid-integer translation, do the translation and exit (idea is to avoid the next divisions, rounding, ...)
			for (unsigned j=0 ; j<Dimension ; j++) m_parameters(j) += translation(j);
			Modified(); m_transform->_PositionAt_NoCheck(m_parameters.extract(Dimension,0)+translation); return true; 
		}
		else if ( abs(integerTranslation(i)) > TINY ) zerotranslation = false;
	}
	if (!zerotranslation) { //if necessary, update what can be updated both in this object and in the transform
		this->_IntegerGridTranslate_NoCheck(integerTranslation);
		//m_transform->_IntegerGridTranslate_NoCheck(integerTranslation);
	}
	return true;
}




/** Apply an integer-grid translation, these can be applied very quickly to image representations and avoid recalculating these */
template <unsigned int VDimension>
bool 
PoseTransformedBinaryShape<VDimension>::IntegerGridTranslate(const vnl_vector<int> &translation) {
	if (translation.size()!=Dimension) return false;
	bool zeroTranslation = true;

	double tmp;
	for (unsigned i=0 ; i<Dimension ; i++) {
		if ( translation(i)!=0 ) {
			zeroTranslation = false;
			tmp = translation(i)*m_imageSpacing[i];
			m_parameters(i) += tmp;
			if (m_centerFlag) m_center(i) += tmp;
			if (m_physicalBBoxUpToDate) { m_physicalBoundingBox(2*i) += tmp;m_physicalBoundingBox(2*i+1) += tmp; }
			if (m_imageBBoxUpToDate)    { m_imageOrigin[i] += tmp; m_imageBoundingBox(2*i) += tmp;m_imageBoundingBox(2*i+1) += tmp; }
		}
	}

	if (m_uptodateBinaryImage)   { 
		if (!m_imageBBoxUpToDate) throw DeformableModelException("PoseTransformedBinaryShape::IntegerGridTranslate : m_imageBBoxUpToDate should always be uptodate when m_uptodateBinaryImage is ; this is not the case");; 
		m_outputBinaryImage->SetOrigin(m_imageOrigin); 
	}
	if (m_uptodateLabelMap)      {
		if (!m_imageBBoxUpToDate) throw DeformableModelException("PoseTransformedBinaryShape::IntegerGridTranslate : m_imageBBoxUpToDate should always be uptodate when m_uptodateLabelMap is ; this is not the case");; 
		m_outputLabelMap->SetOrigin(m_imageOrigin);
	}

	if (!zeroTranslation) {
		m_uptodatePolyData = false;
		m_transform->_IntegerGridTranslate_NoCheck(translation);
	}

	return true;
}

template <unsigned int VDimension>
void 
PoseTransformedBinaryShape<VDimension>::ComputeObjectCenter() {
	m_center = m_shape->GetObjectCenter() + m_parameters.extract(VDimension);
	m_centerFlag = true;
}

//template <unsigned int VDimension>
//void 
//PoseTransformedBinaryShape<VDimension>::ComputeObjectInertia() {
//std::cout<<"PoseTransformedBinaryShape::ComputeObjectInertia"<<std::endl;
//	vnl_matrix<double> Rmat = m_transform->GetMatrixFromParameters(m_transform->GetParameters()).extract(VDimension,VDimension);
//	m_inertia =  Rmat * m_shape->GetObjectInertiaMatrix() * Rmat.transpose();
//	m_inertiaFlag = true;
//}

//in the general case of an affine transform, I need to recover the purely rotational component of the matrix...
//template <unsigned int VDimension>
//void 
//PoseTransformedBinaryShape<VDimension>::ComputeObjectInertiaEigenVectors() {
//	vnl_matrix<double> Rmat = m_transform->GetMatrixFromParameters(m_transform->GetParameters()).extract(VDimension,VDimension);
//	m_inertia =  Rmat * m_shape->GetObjectInertiaMatrix() * Rmat.transpose();
//	m_eigVInertiaFlag=true;
//}


} // namespace psciob


