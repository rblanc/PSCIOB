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
* \file TexturedDeformableModel.h
* \author Remi Blanc 
* \date 3. March 2011
* 
*/

#ifndef TEXTUREDDEFORMABLEMODEL_H_
#define TEXTUREDDEFORMABLEMODEL_H_

#include "ParametricObject.h"

namespace psciob {

/** 
* \class TexturedDeformableModel
* \brief TexturedDeformableModel is a base class for textured deformable models
*
* TexturedDeformableModel is an abstract object, templated against the dimensionality 
*
* \attention MOSTLY NOT IMPLEMENTED YET
*/


//ABSTRACT CLASS
template <unsigned int VDimension, class TAppearance = unsigned char>
class TexturedDeformableModel : public ParametricObject<VDimension, TAppearance> {
public:
	/** Standard class typedefs. */
	typedef TexturedDeformableModel       Self;
	typedef ParametricObject              Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(TexturedDeformableModel, ParametricObject);


	/** This function creates a full copy object, including the vtkPolyData, LabelMap, and itkImage	*/
	virtual BaseClassPointer CreateCopy() {
		Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		clonePtr->SetParameters(GetParameters()); //use set&get parameters methods, in particular because of the PoseTranformedShape, which needs to re-distribute those parameters
		clonePtr->m_imageSpacing  = m_imageSpacing;
		clonePtr->m_vtkResolution = m_vtkResolution;
		clonePtr->m_imageRegion   = m_imageRegion;	
		clonePtr->m_imageOrigin   = m_imageOrigin;
		return static_cast<BaseClass*>( clonePtr );
	}

	/** This function creates a full copy object, including the vtkPolyData, LabelMap, and itkImage	*/
	virtual BaseClassPointer CreateClone() {
		Pointer clonePtr = static_cast<Self*>(this->CreateAnother().GetPointer());
		clonePtr->SetParameters(GetParameters()); //use set&get parameters methods, in particular because of the PoseTranformedShape, which needs to re-distribute those parameters
		clonePtr->m_imageSpacing  = m_imageSpacing;
		clonePtr->m_vtkResolution = m_vtkResolution;
		clonePtr->m_imageRegion   = m_imageRegion;	
		clonePtr->m_imageOrigin   = m_imageOrigin;

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

		if (m_uptodateTexturedPolyData) {
			clonePtr->m_texturedPolyData = vtkSmartPointer<vtkPolyData>::New();
			clonePtr->m_texturedPolyData->DeepCopy( m_texturedPolyData );
			clonePtr->m_uptodateTexturedPolyData = true; 
		}

		if (m_uptodateBinaryImage) {
			clonePtr->m_outputBinaryImage = BinaryImageType::New();
			CloneITKImage<BinaryImageType>(m_outputBinaryImage, clonePtr->m_outputBinaryImage);
			clonePtr->m_uptodateBinaryImage = true;
		}

		if (m_uptodateTexturedImage) {
			clonePtr->m_outputTexturedImage = TexturedImageType::New();
			CloneITKImage<TexturedImageType>(m_outputTexturedImage, clonePtr->m_outputTexturedImage);
			clonePtr->m_uptodateTexturedImage = true;
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

	/** Indicate that the output should be recomputed */
	inline void Modified() {
		ParametricObject<VDimension, TexturedPixelType>::Modified();
		m_uptodateTexturedPolyData = false; m_uptodateTexturedImage = false;
	}

	/** Set the image spacing 
	* Invalidates the current pixel-based representations if the spacing changes
	*/
	virtual void SetImageSpacing(const vnl_vector<double> &sp) {
		if (sp.size()!=VDimension) throw DeformableModelException("SetImageSpacing : unexpected dimension");
		for (unsigned i = 0 ; i<VDimension ; i++) { 
			if (sp[i]<=0) throw DeformableModelException("SetImageSpacing : spacing must be strictly positive");
			if ( fabs(m_imageSpacing[i]-sp[i])>TINY ) { m_uptodateBinaryImage = false; m_uptodateLabelMap = false; m_uptodateTexturedImage = false; m_imageBBoxUpToDate = false; }			
		} 
		m_imageSpacing.SetVnlVector( sp );
	}
	virtual void SetImageSpacing(typename BinaryImageType::SpacingType sp) {
		for (unsigned i = 0 ; i<VDimension ; i++) { 
			if (sp[i]<=0) throw DeformableModelException("SetImageSpacing : spacing must be strictly positive");
			if ( fabs(m_imageSpacing[i]-sp[i])>TINY ) { m_uptodateBinaryImage = false; m_uptodateLabelMap = false; m_uptodateTexturedImage = false; m_imageBBoxUpToDate = false; }
		} 
		m_imageSpacing = sp;
	}
	virtual void SetImageSpacing(double sp)	{ 
		if (sp<=0) throw DeformableModelException("SetImageSpacing : spacing must be strictly positive");
		for (unsigned i = 0 ; i<VDimension ; i++) { 
			if ( fabs(m_imageSpacing[i]-sp)>TINY ) { m_uptodateBinaryImage = false; m_uptodateLabelMap = false; m_uptodateTexturedImage = false; m_imageBBoxUpToDate = false; }
			m_imageSpacing[i] = sp;
		} 
	}

	/** Apply an integer-grid translation, these can be applied very quickly to image representations and avoid recalculating these */
	virtual bool IntegerGridTranslate(const vnl_vector<int> &translation) {
		if (translation.size()!=Dimension) return false;

		double tmp;
		for (unsigned i=0 ; i<Dimension ; i++) {
			if ( translation(i)!=0 ) {
				m_uptodatePolyData = false;
				m_uptodateTexturedPolyData = false;
				tmp = translation(i)*m_imageSpacing[i];
				m_parameters(i) += tmp;
				if (m_centerFlag) m_center(i) += tmp;
				if (m_physicalBBoxUpToDate) { m_physicalBoundingBox(2*i) += tmp;m_physicalBoundingBox(2*i+1)+= tmp; }
				if (m_imageBBoxUpToDate)    { m_imageOrigin[i] += tmp; m_imageBoundingBox(2*i) += tmp;m_imageBoundingBox(2*i+1) += tmp; }
			}
		}
		
		if (m_uptodateBinaryImage)   { 
			if (!m_imageBBoxUpToDate) throw DeformableModelException("TexturedDeformableModel::IntegerGridTranslate : m_imageBBoxUpToDate should always be uptodate when m_uptodateBinaryImage is ; this is not the case");; 
			m_outputBinaryImage->SetOrigin(m_imageOrigin); 
		}
		if (m_uptodateTexturedImage) {
			if (!m_imageBBoxUpToDate) throw DeformableModelException("TexturedDeformableModel::IntegerGridTranslate : m_imageBBoxUpToDate should always be uptodate when m_uptodateTexturedImage is ; this is not the case");; 
			m_outputTexturedImage->SetOrigin(m_imageOrigin);
		}
		if (m_uptodateLabelMap)      {
			if (!m_imageBBoxUpToDate) throw DeformableModelException("TexturedDeformableModel::IntegerGridTranslate : m_imageBBoxUpToDate should always be uptodate when m_uptodateLabelMap is ; this is not the case");; 
			m_outputLabelMap->SetOrigin(m_imageOrigin);
		}

		return true;
	}

	/** Ask if the corresponding object representation is uptodate */
	inline bool IsObjectTexturedPolyDataUpToDate() { return m_uptodateTexturedPolyData; }
	inline bool IsObjectTexturedImageUpToDate()    { return m_uptodateTexturedImage; }


protected:
	TexturedDeformableModel() : ParametricObject(), 
		m_uptodateTexturedPolyData(false), m_uptodateTexturedImage(false),
		m_outputTexturedImage(NULL), m_texturedPolyData(NULL)
	{};
	virtual ~TexturedDeformableModel() {};

	vtkSmartPointer<vtkPolyData>        m_texturedPolyData;    bool m_uptodateTexturedPolyData;
	typename TexturedImageType::Pointer m_outputTexturedImage; bool m_uptodateTexturedImage;


	// Apply an integer-grid translation, these can be applied very quickly to image representations and avoid recalculating these 
	bool _IntegerGridTranslate_NoCheck(const vnl_vector<int> &translation) {
		bool zeroTranslation = true;
		double tmp;
		for (unsigned i=0 ; i<Dimension ; i++) {
			if ( translation(i)!=0 ) {
				zeroTranslation = false;
				tmp = translation(i)*m_imageSpacing[i];
				m_parameters(i) += tmp;
				if (m_centerFlag) m_center(i) += tmp;
				if (m_imageBBoxUpToDate)    { m_imageOrigin[i] += tmp; m_imageBoundingBox(2*i) += tmp; m_imageBoundingBox(2*i+1) += tmp; }
				if (m_physicalBBoxUpToDate) { m_physicalBoundingBox(2*i) += tmp; m_physicalBoundingBox(2*i+1) += tmp; }
			}
		}

		if (m_uptodateBinaryImage)   { 
			if (!m_imageBBoxUpToDate) throw DeformableModelException("TexturedDeformableModel::_IntegerGridTranslate_NoCheck : m_imageBBoxUpToDate should always be uptodate when m_uptodateBinaryImage is ; this is not the case");; 
			m_outputBinaryImage->SetOrigin(m_imageOrigin); 
		}
		if (m_uptodateTexturedImage) {
			if (!m_imageBBoxUpToDate) throw DeformableModelException("TexturedDeformableModel::_IntegerGridTranslate_NoCheck : m_imageBBoxUpToDate should always be uptodate when m_uptodateTexturedImage is ; this is not the case");; 
			m_outputTexturedImage->SetOrigin(m_imageOrigin);
		}
		if (m_uptodateLabelMap)      {
			if (!m_imageBBoxUpToDate) throw DeformableModelException("TexturedDeformableModel::_IntegerGridTranslate_NoCheck : m_imageBBoxUpToDate should always be uptodate when m_uptodateLabelMap is ; this is not the case");; 
			m_outputLabelMap->SetOrigin(m_imageOrigin);
		}

		if (!zeroTranslation) {
			m_uptodatePolyData = false;
			m_uptodateTexturedPolyData = false;
		}

		return true;
	}

private:
	TexturedDeformableModel(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* TEXTUREDDEFORMABLEMODEL_H_ */



