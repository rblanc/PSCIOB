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
* \file PoseTransform.h
* \author Rémi Blanc 
* \date 25. July 2011
*/


#ifndef POSETRANSFORM_H_
#define POSETRANSFORM_H_

#include "TexturedDeformableModel.h"
#include "TransformUtils.h"

namespace psciob {

/** 
 * \class PoseTransform
 * \brief Abstract class, managing 2D or 3D linear transformation (from simple translation up to affine transform) 
 *
*/


//ABSTRACT CLASS inheriting from TexturedDeformableModel so that the textured data may be processed as well...
template < unsigned int VDimension, class TAppearance = unsigned char > 
class PoseTransform : public TexturedDeformableModel<VDimension, TAppearance> {
template<unsigned int D> friend class PoseTransformedBinaryShape;
public:
	/** Standard class typedefs. */
	typedef PoseTransform           Self;
	typedef TexturedDeformableModel Superclass;
	typedef itk::SmartPointer<Self> Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PoseTransform, TexturedDeformableModel);


	/** Set Input Shape */
	void SetInput(ParametricObject<VDimension, TAppearance> *input) { m_inputShape = input;  }

	/** Set Image-related interpolation method */
	void SetImageInterpolationMethod(InterpolationType code) {
		if (code != m_interpolationType) m_uptodateTexturedImage = false;
		m_interpolationType = code;
	}

	/** Get the transform matrix (homogeneous coordinates) */
	inline vnl_matrix<double> GetTransformMatrix() { 
		return GetMatrixFromParameters(m_parameters);
		//if (!m_transformMatrixUptodate) { m_transformMatrix = GetMatrixFromParameters(m_parameters); m_transformMatrixUptodate=true; }
		//return m_transformMatrix; 
	}
	
	/** Set Default Parameters */
	inline void SetDefaultParameters() { m_parameters = GetDefaultParameters(); Modified(); }

	/** Set Parameters - internally check whether the parameters are valid and returns false if not
	* Look for pure integer grid translation
	* If a single parameters is different, then the whole shape needs to be update
	*/
	bool SetParameters(const vnl_vector<double> &params) {
		if (!CheckParameters(params)) { return false;}
		//First, look at the non-translation parameters
		for (unsigned i=Dimension ; i<params.size() ; i++) { 
			if ( fabs(m_parameters(i)-params(i))>TINY ) { 
				m_parameters = params; Modified(); return true; 
			}
		}
		//Then, look at the translation parameters and look for a grid-integer translation
		vnl_vector<int> integerTranslation(Dimension); bool zerotranslation = true; double tmpval;
		for (unsigned i=0 ; i<Dimension ; i++) {
			tmpval = (params(i)-m_parameters(i))/m_imageSpacing[i];
			integerTranslation(i) = round(tmpval);
			if ( fabs(tmpval - integerTranslation(i)) > TINY ) { m_parameters = params; Modified(); return true; } //TODO: can update the transformMatrix easily!!!
			else if ( abs(integerTranslation(i)) > TINY ) zerotranslation = false;
		}
		//At this point, nothing but the translation may be different, if it is a non-zero translation, just applied it
		if (!zerotranslation) _IntegerGridTranslate_NoCheck(integerTranslation);
		return true;
	}

	/** Get the vector of parameters 
	* not const, because the parameters may be updated with respect to the matrix
	*/
	inline vnl_vector<double> GetParameters();
	
	/** Get the inverse transform matrix */
	inline vnl_matrix<double> GetInverseTransformMatrix();
	
	/** Conversions between matrix and parametric representations - not modifying the object */
	virtual vnl_matrix<double> GetMatrixFromParameters(const vnl_vector<double> &poseParameters)  const = 0;
	
	/** Conversions between matrix and parametric representations - not modifying the object */
	virtual vnl_vector<double> GetParametersFromMatrix(const vnl_matrix<double> &transformMatrix) const = 0;


	/** Set Parameters - internally check whether the parameters are valid and returns false if not
	* Look for pure integer grid translation
	* If a single parameters is different, then the whole shape needs to be update
	*/
	bool SetTransformationMatrix(const vnl_matrix<double> &mat); 

	//find a better way to offer such an interface , perhaps traits (inherit the interface?)
	bool Scale(double scale);
	bool Scale(const vnl_vector<double> &scale);
	bool Compose(const vnl_matrix<double> &transf); //should be improved 
	bool ModifyTransformWithParameters(const vnl_vector<double> &poseParameters);
protected: 
	bool Compose_Internal(const vnl_matrix<double> &transf);
	void ApplyGridIntegerTranslationToImage(const vnl_vector<double> &trans);
public:
	void SetCompositionToPreCompose()  { m_flagPostMultiply=false; } //normal (default) setting, the new part of the transform is applied after the first
	void SetCompositionToPostCompose() { m_flagPostMultiply=true; }  //inverse, the newly set transform is applied before the existing part of the transform


	/** Get physical, axis-aligned bounding box */
	vnl_vector<double> GetPhysicalBoundingBox() {
		if (!m_inputShape) throw DeformableModelException("PoseTransform: no shape to define a bounding box! use SetInput() ");
		if ( (!m_inputShape->IsObjectPhysicalBoundingBoxUpToDate()) || (!m_physicalBBoxUpToDate) ) {
			m_physicalBoundingBox = TransformBoundingBox(m_inputShape->GetPhysicalBoundingBox(), GetTransformMatrix());
			m_physicalBBoxUpToDate = true;
		}
		return m_physicalBoundingBox; 
	}


	/** Get the shape in the requested format */
	vtkPolyData* GetObjectAsVTKPolyData() {
		if (!m_inputShape) throw DeformableModelException("PoseTransform: no shape to transform! use SetInput() ");
		if ( (!m_inputShape->IsObjectPolyDataUpToDate()) || (!m_uptodatePolyData) ) {
			//instanciate the vtkpolydata if necessary
			if(!m_outputPolyData) m_outputPolyData = vtkSmartPointer<vtkPolyData>::New();
			//transform the input shape
			TransformVTKPolyData(m_inputShape->GetObjectAsVTKPolyData(), m_outputPolyData);
			m_uptodatePolyData = true;
		}
		return m_outputPolyData.GetPointer();
	}

	/** NOT TESTED */
	vtkPolyData* GetObjectAsTexturedVTKPolyData() { 
		if (!m_inputShape) throw DeformableModelException("PoseTransform: no shape to transform! use SetInput() ");
		if ( (!m_inputShape->IsObjectTexturedPolyDataUpToDate()) || (!m_uptodateTexturedPolyData) ) {
			//instanciate the vtkpolydata if necessary
			if(!m_texturedPolyData) m_texturedPolyData = vtkSmartPointer<vtkPolyData>::New();
			TransformVTKPolyData(m_inputShape->GetObjectAsVTKPolyData(), m_texturedPolyData);
			m_uptodateTexturedPolyData = true;
		}
		return m_texturedPolyData.GetPointer();
	}

	/** NOT TESTED */
	TexturedImageType* GetObjectAsTexturedImage() { 
		if (!m_inputShape) throw DeformableModelException("PoseTransform: no shape to transform! use SetInput() ");
		if ( (!m_inputShape->IsObjectTexturedImageUpToDate()) || (!m_uptodateTexturedImage) ) {
			//instanciated the textured image if necessary
			if (!m_outputTexturedImage) m_outputTexturedImage = TexturedImageType::New();
			TransformITKImage<TexturedPixelType>(m_inputShape->GetObjectAsTexturedImage(), m_outputTexturedImage);
			m_uptodateTexturedImage = true;
		}
		return m_outputTexturedImage.GetPointer();
	}



	/** Set Resolution parameters for the VTK mesh representation */
	virtual bool SetVTKResolution(const vnl_vector<double> &res) {
		if (!m_inputShape) throw DeformableModelException("PoseTransform: no shape to transform! use SetInput() ");
		return m_inputShape->SetVTKResolution(res);
	}

	/** Applies the transform to a vtkPolyData */
	inline void TransformVTKPolyData(vtkPolyData* inputPolyData, vtkPolyData* outputPolyData) {
		AffineTransformVTKPolyData(inputPolyData, GetTransformMatrix(), outputPolyData);
	}

	/** Applies the transform to a textured itkImage
	* and the interpolation method corresponding to the specified code (see InterpolationType)
	* \warning for binary images, prefer the NearestNeighbor interpolation
	*/
	template <class TPixel>
	void TransformITKImage(typename itk::Image<TPixel, VDimension>* inputImage, typename itk::Image<TPixel, VDimension>* outputImage) {
		return AffineTransformITKImage< itk::Image<TPixel, VDimension> >(inputImage, GetTransformMatrix(), m_interpolationType, outputImage);
	}

protected:
	PoseTransform();
	virtual ~PoseTransform() {};

	typename ParametricObject<VDimension, TAppearance>::Pointer m_inputShape;

	bool m_flagPostMultiply;
	//bool m_transformMatrixUptodate, m_inverseMatrixUptodate;
	//vnl_vector<double> m_parameters;
	//vnl_matrix<double> m_transformMatrix, m_inverseTransformMatrix; //3*3 or 4*4 matrix

	InterpolationType m_interpolationType;

	//// protected internal mechanism for setting the parameters, without checking for their validity or their non-novelty
	//void _SetParameters_NoCheck(const vnl_vector<double> &params) { 
	//	m_parameters = params; 
	//	Modified(); 
	//}

	///** Apply an integer-grid translation, these can be applied very quickly to image representations and avoid recalculating these */
	//virtual bool _IntegerGridTranslate_NoCheck(const vnl_vector<int> &translation) {
	//	//always invalidate the polydata 
	//	m_uptodatePolyData = false;
	//	m_uptodateTexturedPolyData = false;

	//	double tmp;
	//	for (unsigned i=0 ; i<Dimension ; i++) {
	//		tmp = translation(i)*m_imageSpacing[i];
	//		m_parameters(i) += tmp;
	//		m_imageOrigin[i] += tmp;
	//		if (m_physicalBBoxUpToDate) m_physicalBoundingBox(i) += tmp;
	//		if (m_imageBBoxUpToDate)    m_imageBoundingBox(i)    += tmp;			
	//	}
	//	if (m_uptodateBinaryImage)   m_outputBinaryImage->SetOrigin(m_imageOrigin);
	//	if (m_uptodateTexturedImage) m_outputTexturedImage->SetOrigin(m_imageOrigin);
	//	if (m_uptodatePixelSet)      m_outputPixelSet->SetOrigin(m_imageOrigin);

	//	Modified();

	//	return true;
	//}

	/** Get the shape in the requested format */
	vtkPolyData* _GetObjectAsVTKPolyData_NoCheck() {
		TransformVTKPolyData( m_inputShape->GetObjectAsVTKPolyData(), m_outputPolyData );
		m_uptodatePolyData = true;
		return m_outputPolyData.GetPointer();
	}

	// this is known to modify the object -> just set the parameters and Modify.
	bool _PositionAt_NoCheck(const vnl_vector<double> &pos) {
		for (unsigned i=0 ; i<Dimension ; i++) m_parameters(i) = pos(i);
		Modified();
		return true;
	}

private:
	PoseTransform(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#include "PoseTransform.txx"

#endif /* POSETRANSFORM_H_ */
