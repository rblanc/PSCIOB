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
* \file BinaryDeformableModel.h
* \author Remi Blanc 
* \date 25. July 2011
* 
*/

#ifndef BINARYDEFORMABLEMODEL_H_
#define BINARYDEFORMABLEMODEL_H_

#include "ParametricObject.h"

namespace psciob {


/** 
 * \class BinaryDeformableModel
 * \brief BinaryDeformableModel is a base class for binary deformable models
 *
 * BinaryDeformableModel is an abstract object, templated against the dimensionality 
 * It assumes the object will be of uniform texture, with default value = 1
 *
 * The textured and binary image format are the same, as well as the TexturedMesh and standard mesh
 */

//ABSTRACT CLASS
template <unsigned int VDimension>
class BinaryDeformableModel : public ParametricObject<VDimension, unsigned char> {
public:
	/** Standard class typedefs. */
	typedef BinaryDeformableModel			Self;
	typedef ParametricObject			    Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(BinaryDeformableModel, ParametricObject);

	/** This function creates a full copy object, with the same parameters, but blank data. */
	virtual BaseClassPointer CreateCopy() {
		Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		clonePtr->SetParameters(GetParameters()); //use set&get parameters methods, in particular because of the PoseTranformedShape, which needs to re-distribute those parameters
		clonePtr->m_imageSpacing  = m_imageSpacing;
		clonePtr->m_vtkResolution = m_vtkResolution;
		return static_cast<BaseClass*>( clonePtr );
	}

	/** This function creates a full copy object, including the vtkPolyData, LabelMap, and itkImage	*/
	virtual BaseClassPointer CreateClone() {
		Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		clonePtr->SetParameters(GetParameters()); 
		clonePtr->m_imageSpacing  = m_imageSpacing;
		clonePtr->m_vtkResolution = m_vtkResolution;

		if (m_physicalBBoxUpToDate) {
			clonePtr->m_physicalBoundingBox = m_physicalBoundingBox; 
			clonePtr->m_physicalBBoxUpToDate = true; 
		}

		if (m_imageBBoxUpToDate) { 
			clonePtr->m_imageBoundingBox = m_imageBoundingBox; 
			clonePtr->m_imageRegion   = m_imageRegion;	
			clonePtr->m_imageOrigin   = m_imageOrigin;
			clonePtr->m_imageBBoxUpToDate = true; 
		}

		if (m_uptodatePolyData) {
			clonePtr->m_outputPolyData = vtkSmartPointer<vtkPolyData>::New();
			clonePtr->m_outputPolyData->DeepCopy( m_outputPolyData );
			clonePtr->m_uptodatePolyData = true; 
		}

		if (m_uptodateBinaryImage) {
			clonePtr->m_outputBinaryImage = BinaryImageType::New();
			CloneITKImage<BinaryImageType>(m_outputBinaryImage, clonePtr->m_outputBinaryImage);
			clonePtr->m_uptodateBinaryImage = true;
		}

		if (m_uptodateLabelMap) {
			clonePtr->m_outputLabelMap = LabelMapType::New();
			clonePtr->m_outputLabelMap->Graft(m_outputLabelMap);
			clonePtr->m_uptodateLabelMap = true;
		}

		if (m_centerFlag)      { clonePtr->m_centerFlag = true; clonePtr->m_center = m_center; }
		if (m_inertiaFlag)     { clonePtr->m_inertiaFlag = true; clonePtr->m_inertia = m_inertia; }
		if (m_eigVInertiaFlag) { clonePtr->m_eigVInertiaFlag = true; clonePtr->m_eigVInertia = m_eigVInertia; }

		return static_cast<BaseClass*>( clonePtr );
	}

	/** For BinaryDeformableModel, the textured outputs are the same as binary outputs */
	inline vtkPolyData*       GetObjectAsTexturedVTKPolyData()   { return GetObjectAsVTKPolyData(); }
	inline TexturedImageType* GetObjectAsTexturedImage()         { return GetObjectAsBinaryImage(); }
	
	inline bool               IsObjectTexturedPolyDataUpToDate() { return IsObjectPolyDataUpToDate(); }
	inline bool               IsObjectTexturedImageUpToDate()    { return IsObjectBinaryImageUpToDate(); }

	/** Apply an integer-grid translation, these can be applied very quickly to image representations and avoid recalculating these */
	virtual bool IntegerGridTranslate(const vnl_vector<int> &translation) {
		if (translation.size()!=Dimension) return false;
		double tmp;
		for (unsigned i=0 ; i<Dimension ; i++) {
			if ( translation(i)!=0 ) {
				m_uptodatePolyData = false;
				tmp = translation(i)*m_imageSpacing[i];
				m_parameters(i) += tmp;
				if (m_centerFlag) m_center(i) += tmp;
				if (m_imageBBoxUpToDate)    {m_imageOrigin[i] += tmp; m_imageBoundingBox(2*i) += tmp;m_imageBoundingBox(2*i+1) += tmp; }
				if (m_physicalBBoxUpToDate) {m_physicalBoundingBox(2*i)+= tmp;m_physicalBoundingBox(2*i+1)+= tmp; }
			}
		}

		if (m_uptodateBinaryImage)   { 
			if (!m_imageBBoxUpToDate) throw DeformableModelException("BinaryDeformableModel::IntegerGridTranslate : m_imageBBoxUpToDate should always be uptodate when m_uptodateBinaryImage is ; this is not the case");; 
			m_outputBinaryImage->SetOrigin(m_imageOrigin); 
		}
		if (m_uptodateLabelMap)      {
			if (!m_imageBBoxUpToDate) throw DeformableModelException("BinaryDeformableModel::IntegerGridTranslate : m_imageBBoxUpToDate should always be uptodate when m_uptodateLabelMap is ; this is not the case");; 
			m_outputLabelMap->SetOrigin(m_imageOrigin);
		}

		return true;
	}


protected:
	BinaryDeformableModel() : ParametricObject() {};
	virtual ~BinaryDeformableModel() {};

private:
	BinaryDeformableModel(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* BINARYDEFORMABLEMODEL_H_ */



